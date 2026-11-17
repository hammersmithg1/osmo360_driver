"""Finite camera RAM data loader. Importing never writes or launches anything."""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import zlib

NAMES = frozenset(('apply.sh', 'rollback.sh', 'watchdog.sh', 'lifecycle.py',
                   'netdev.py', 'lease.py', 'ecm_dhcp.py'))
MAX_COMPRESSED, MAX_PLAIN = 32768, 65536


def digest(data):
    return hashlib.sha256(data).hexdigest()


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate bundle key')
        result[key] = value
    return result


def decode_bundle(packed, compressed_size, compressed_sha, plain_size, plain_sha):
    if (type(compressed_size) is not int or not 0 < compressed_size <= MAX_COMPRESSED
            or type(plain_size) is not int or not 0 < plain_size <= MAX_PLAIN
            or any(not isinstance(s, str) or not re.fullmatch('[0-9a-f]{64}', s)
                   for s in (compressed_sha, plain_sha))):
        raise ValueError('Bundle bounds/hash arguments')
    if type(packed) is not bytes or len(packed) != compressed_size or digest(packed) != compressed_sha:
        raise ValueError('Compressed bundle size/hash mismatch')
    decoder = zlib.decompressobj()
    plain = decoder.decompress(packed, plain_size + 1)
    if (len(plain) != plain_size or not decoder.eof or decoder.unused_data
            or decoder.unconsumed_tail or digest(plain) != plain_sha):
        raise ValueError('Incomplete, trailing, oversized or corrupt plain bundle')
    # Do not flush an unbounded decompressor. EOF and exact hash are mandatory.
    value = json.loads(plain.decode('ascii'), object_pairs_hook=unique)
    if (type(value) is not dict or set(value) != {'schema', 'files'}
            or type(value['schema']) is not int or value['schema'] != 1
            or type(value['files']) is not dict or set(value['files']) != NAMES):
        raise ValueError('Bundle schema/file allowlist')
    result = {}
    for name, item in value['files'].items():
        if type(item) is not dict or set(item) != {'source', 'bytes', 'sha256'}:
            raise ValueError('Bundle file record')
        source = item['source']
        if not isinstance(source, str) or not source.isascii() or '\0' in source:
            raise ValueError('Bundle source must be NUL-free ASCII')
        data = source.encode('ascii')
        if (type(item['bytes']) is not int or not 0 < item['bytes'] <= MAX_PLAIN
                or len(data) != item['bytes'] or digest(data) != item['sha256']):
            raise ValueError('Bundle file size/hash mismatch')
        result[name] = data
    return result


def publish(work, files, plain_sha):
    """Only fixed, absent outputs; no cleanup/racing retry on failure."""
    if set(files) != NAMES or not re.fullmatch('[0-9a-f]{64}', plain_sha):
        raise ValueError('Publication allowlist/hash')
    if any(type(data) is not bytes or not data or len(data) > MAX_PLAIN for data in files.values()):
        raise ValueError('Publication data bounds')
    for name in (*NAMES, 'bundle.ready', 'bundle.ready.tmp'):
        if os.path.lexists(work / name):
            raise ValueError('Publication target already exists')
    for name in sorted(NAMES):
        data = files[name]
        path = work / name
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        with os.fdopen(fd, 'wb') as stream:
            created = os.fstat(stream.fileno())
            stream.write(data)
            stream.flush()
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        with os.fdopen(fd, 'rb') as stream:
            current = os.fstat(stream.fileno())
            if ((created.st_dev, created.st_ino) != (current.st_dev, current.st_ino)
                    or not stat.S_ISREG(current.st_mode) or current.st_size != len(data)
                    or stream.read(len(data) + 1) != data):
                raise ValueError('Published file readback mismatch')
    # Atomic marker last; a partial or uncertain unpack is never an apply gate.
    temporary = work / 'bundle.ready.tmp'
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write((plain_sha + '\n').encode('ascii'))
        stream.flush()
    os.link(temporary, work / 'bundle.ready', follow_symlinks=False)
    temporary.unlink()


def camera_main(argv):
    if len(argv) != 5:
        raise ValueError('Expected work, compressed size/SHA and plain size/SHA')
    work = Path(argv[0])
    script = Path(__file__)
    if (os.getuid() != 0 or sys.platform != 'linux'
            or not re.fullmatch(r'/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}', str(work))
            or script != work / 'unpack.py' or work.resolve() != work):
        raise ValueError('Expected camera root and exact private RAM paths')
    for path in (work, script):
        info = path.lstat()
        if path.is_symlink() or info.st_uid != 0 or info.st_mode & 0o022:
            raise ValueError('Private root-owned work/script required')
    if not work.is_dir() or not script.is_file() or Path('/dev/shm').resolve() != Path('/dev/shm'):
        raise ValueError('Unexpected RAM paths')
    rows = [line.split() for line in Path('/proc/mounts').read_text().splitlines()]
    if [r[2] for r in rows if r[1] == '/dev/shm'] != ['tmpfs']:
        raise ValueError('Canonical tmpfs required')
    compressed_size, plain_size = int(argv[1]), int(argv[3])
    if not 0 < compressed_size <= MAX_COMPRESSED or not 0 < plain_size <= MAX_PLAIN:
        raise ValueError('Bounded input/output required')
    fd = os.open(work / 'scripts.z', os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022
                or info.st_size != compressed_size):
            raise ValueError('Unexpected bundle input')
        packed = stream.read(compressed_size + 1)
    files = decode_bundle(packed, compressed_size, argv[2], plain_size, argv[4])
    publish(work, files, argv[4])


if __name__ == '__main__':
    camera_main(sys.argv[1:])
