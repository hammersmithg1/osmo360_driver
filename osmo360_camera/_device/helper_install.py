"""Source-pinned helper cache installer for OQ101 10.00.25.29.

This file never applies an ECM plan or starts a standby owner. The optional
post-boot hook only copies immutable source bytes into private tmpfs. Removal
restores the exact original hook; an explicit camera reboot is still required
to clear any already-running SDK owner and its retained role mount.
"""
import argparse
import contextlib
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import stat
import sys
import time
import zlib

ROOT = Path('/data/osmo360-sdk')
RAM = Path('/dev/shm')
EXPORT = Path('/blackbox/export_tmp')
HOOK = Path('/etc/init.post_boot_product_post.sh')
PARENT = Path('/etc/init.post_boot.sh')
UPPER_HOOK = Path('/overlay/etc/init.post_boot_product_post.sh')
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')
HOOK_SHA = '483881c9dd76d249f82fb1a7264aa0aa3249b23bf75b9a5241c797eb65a2042d'
PARENT_SHA = '6cc12ac2c4e78c6d6d582aae49b03213c2867732b3e2703a3ec321ac9060c682'
SYS_SHA = 'f2fc2a40b8c10b0d1a8289089bb0ab6700372fa062404f44c3dcac00ad513029'
FILES = {
    'usb_standby_owner.py': '9529f3ee74274b5458a4dcda670a1efc94e6c607b7e65971629d03de7058aeb8',
    'gui_qs_exit_forward_probe.py': '9db827623c2e7f9767fc8a52ba5dfc601a768fafdbc1687666b1cbe7cf26e959',
    'prepared_preview_wake_047.py': '246c4e253c07c675f4992b6b4ec494fda3c2d973ad7ad91801134203e569d3a5',
    'prepared_preview_wake_045.py': '33c8a2a1af90fe94dd7c93b58d4f15234b5c5dc0f35e088420c16c1118a654fc',
    'retained_sleep_060.py': '126bc7ea2606f9e5160fec35466414393a420065a9a45de7c10c269df2877960',
}
GROUPS = {'owner': tuple(FILES)[:2], 'prepared': tuple(FILES)[2:4],
          'retained': tuple(FILES)[2:]}
MARKER = {'schema': 1, 'application': 'osmo360-sdk-helpers'}
MAX_PACKED, MAX_PLAIN, MAX_REPORT = 65536, 262144, 16384
O_FLAGS = os.O_NOFOLLOW | os.O_CLOEXEC
UID = 0
GID = 0


def sha(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode('ascii')


def unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('Duplicate JSON key')
        value[key] = item
    return value


def parse(data):
    return json.loads(data, object_pairs_hook=unique,
                      parse_constant=lambda text: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def identity(info):
    return info.st_dev, info.st_ino


def private(path, *, directory=False, mode=None):
    path = Path(path)
    info = path.lstat()
    if (path.resolve() != path or info.st_uid != UID or info.st_mode & 0o022
            or not (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode))
            or mode is not None and stat.S_IMODE(info.st_mode) != mode):
        raise ValueError('Unexpected private path: ' + str(path))
    return info


def ram_root():
    # /dev/shm is a shared tmpfs, commonly01777. Only our child directories
    # must be0700; a writable shared parent additionally needs the sticky bit.
    info = RAM.lstat()
    if (RAM.resolve() != RAM or not stat.S_ISDIR(info.st_mode) or info.st_uid != UID
            or info.st_mode & 0o022 and not info.st_mode & stat.S_ISVTX):
        raise ValueError('Canonical root-owned sticky/private RAM parent required')


def read(path, maximum=MAX_PLAIN, *, mode=None):
    path = Path(path)
    before = private(path, mode=mode)
    fd = os.open(path, os.O_RDONLY | O_FLAGS)
    with os.fdopen(fd, 'rb') as stream:
        now = os.fstat(stream.fileno())
        if identity(now) != identity(before) or now.st_size > maximum:
            raise ValueError('Changed/oversized input')
        data = stream.read(maximum + 1)
        if len(data) != now.st_size or len(data) > maximum:
            raise ValueError('Changed/oversized read')
        return data


def fsync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | O_FLAGS)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def create(path, data, mode=0o400):
    """Exclusive output with readback. A failed partial file is never adopted."""
    path = Path(path)
    private(path.parent, directory=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | O_FLAGS, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        made = os.fstat(stream.fileno())
        stream.write(data)
        stream.flush()
        os.fchmod(stream.fileno(), mode)
        os.fsync(stream.fileno())
    if read(path, max(len(data), 1), mode=mode) != data or identity(path.lstat()) != identity(made):
        raise ValueError('Exclusive output readback mismatch')
    fsync_dir(path.parent)


def replace(path, data, *, expected, nonce, mode=0o600):
    """Replace only exact known bytes. Temporary name itself is single-use."""
    path = Path(path)
    if read(path) != expected:
        raise ValueError('Foreign file drift before replace')
    temporary = path.with_name(path.name + '.osmo-' + nonce)
    create(temporary, data, mode)
    if read(path) != expected:
        raise ValueError('Foreign file drift during replace')
    os.replace(temporary, path)
    fsync_dir(path.parent)
    if read(path, mode=mode) != data:
        raise ValueError('Atomic output mismatch')


def json_file(path, value, *, nonce):
    data = canonical(value)
    if Path(path).exists():
        replace(path, data, expected=read(path), nonce=nonce)
    else:
        create(path, data, 0o600)


def profile_manifest(value):
    if type(value) is not dict or set(value) != {'schema', 'profile', 'installer_sha256', 'files'}:
        raise ValueError('Exact profile manifest required')
    if (type(value['schema']) is not int or value['schema'] != 1
            or not re.fullmatch('[0-9a-f]{64}', value.get('installer_sha256', ''))
            or type(value['files']) is not dict or set(value['files']) != set(FILES)):
        raise ValueError('Profile source allowlist')
    for name, record in value['files'].items():
        if (type(record) is not dict or set(record) != {'sha256', 'size'}
                or record['sha256'] != FILES[name] or type(record['size']) is not int
                or not 0 < record['size'] <= 100000):
            raise ValueError('Source hash/size record mismatch')
    payload = {key: value[key] for key in ('schema', 'installer_sha256', 'files')}
    if value['profile'] != sha(canonical(payload)):
        raise ValueError('Canonical profile hash mismatch')
    return value


def decode_bundle(packed, request):
    if type(request) is not dict or set(request) != {
            'schema', 'profile', 'installer_sha256', 'files', 'packed', 'autostart'}:
        raise ValueError('Exact install request required')
    manifest = profile_manifest({k: request[k] for k in ('schema', 'profile', 'installer_sha256', 'files')})
    info = request['packed']
    if (type(request['autostart']) is not bool or type(info) is not dict
            or set(info) != {'bytes', 'sha256', 'plain_bytes', 'plain_sha256'}
            or type(info['bytes']) is not int or not 0 < info['bytes'] <= MAX_PACKED
            or type(info['plain_bytes']) is not int or not 0 < info['plain_bytes'] <= MAX_PLAIN
            or type(packed) is not bytes or len(packed) != info['bytes'] or sha(packed) != info['sha256']):
        raise ValueError('Compressed source bundle bounds/hash')
    decoder = zlib.decompressobj()
    plain = decoder.decompress(packed, info['plain_bytes'] + 1)
    if (len(plain) != info['plain_bytes'] or sha(plain) != info['plain_sha256'] or not decoder.eof
            or decoder.unused_data or decoder.unconsumed_tail):
        raise ValueError('Corrupt/trailing/oversized source bundle')
    value = parse(plain)
    if (type(value) is not dict or set(value) != {'schema', 'files'} or type(value['schema']) is not int
            or value['schema'] != 1 or type(value['files']) is not dict or set(value['files']) != set(FILES)):
        raise ValueError('Exact bundled source names required')
    files = {}
    for name, source in value['files'].items():
        if not isinstance(source, str) or not source.isascii() or '\0' in source:
            raise ValueError('Expected NUL-free ASCII Python source')
        data = source.encode('ascii')
        if len(data) != manifest['files'][name]['size'] or sha(data) != FILES[name]:
            raise ValueError('Bundled source size/hash mismatch')
        files[name] = data
    return manifest, files


def environment():
    if os.getuid() != 0 or sys.platform != 'linux':
        raise ValueError('Camera root on Linux required')
    private(ROOT.parent, directory=True)
    ram_root()
    rows = [line.split() for line in Path('/proc/mounts').read_text().splitlines()]
    if [row[2] for row in rows if row[1] == str(RAM)] != ['tmpfs']:
        raise ValueError('Canonical /dev/shm tmpfs required')
    if identity(Path('/proc/self/ns/mnt').stat()) != identity(Path('/proc/1/ns/mnt').stat()):
        raise ValueError('Installer must share PID1 mount namespace')
    for target in ('/data', '/etc'):
        matches = [row for row in rows if row[1] == target]
        if (len(matches) != 1 or matches[0][2] != 'overlay'
                or not {'rw', 'upperdir=/overlay' + target} <= set(matches[0][3].split(','))):
            raise ValueError('Verified persistent rw overlay required: ' + target)
    overlays = [row for row in rows if row[1] == '/overlay']
    if len(overlays) != 1 or overlays[0][2] != 'ext4' or 'rw' not in overlays[0][3].split(','):
        raise ValueError('Persistent userdata ext4 mount required')
    if sha(read('/usr/bin/dji_sys', 4 * 1024 * 1024)) != SYS_SHA:
        raise ValueError('Matching retail firmware executable required')
    # The tested firmware boots Android init and calls this stock post hook.
    # A new systemd unit is not a substitute for this verified runtime branch.
    if Path('/proc/1/comm').read_text().strip() != 'aosp_init' or 'aosp' not in Path('/proc/cmdline').read_text():
        raise ValueError('Verified AOSP boot branch required')
    return BOOT_ID.read_text().strip()


@contextlib.contextmanager
def locked():
    """Embedded Python lacks fcntl; Linux libc flock closes on process exit."""
    fd = os.open(ROOT / '.lock', os.O_RDWR | os.O_CREAT | O_FLAGS, 0o600)
    libc = ctypes.CDLL(None, use_errno=True)
    libc.flock.argtypes, libc.flock.restype = [ctypes.c_int, ctypes.c_int], ctypes.c_int
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != UID
                or stat.S_IMODE(info.st_mode) != 0o600):
            raise ValueError('Private installer lock required')
        while libc.flock(fd, 6) != 0:  # LOCK_EX | LOCK_NB
            code = ctypes.get_errno()
            if code != errno.EINTR:
                raise OSError(code, 'Another helper installer owns the lock')
        yield
    finally:
        os.close(fd)


def ensure_root():
    if not os.path.lexists(ROOT):
        ROOT.mkdir(mode=0o700)
        create(ROOT / 'managed.json', canonical(MARKER))
        fsync_dir(ROOT.parent)
    private(ROOT, directory=True, mode=0o700)
    if parse(read(ROOT / 'managed.json', 1024, mode=0o400)) != MARKER:
        raise ValueError('Foreign managed root')


def installed_hook(original, profile):
    command = ('\n# BEGIN OSMO360-SDK-HELPERS v1 ' + profile + '\n'
        '(/usr/bin/python3 -B /data/osmo360-sdk/versions/' + profile
        + '/helper_install.py boot --profile ' + profile
        + ' </dev/null >/dev/null 2>&1) &\n# END OSMO360-SDK-HELPERS v1\n')
    return original + command.encode('ascii')


def validate_version(profile):
    if not isinstance(profile, str) or not re.fullmatch('[0-9a-f]{64}', profile):
        raise ValueError('Exact version hash required')
    version = ROOT / 'versions' / profile
    private(version, directory=True, mode=0o500)
    if {p.name for p in version.iterdir()} != set(FILES) | {'helper_install.py', 'manifest.json'}:
        raise ValueError('Unexpected immutable version inventory')
    manifest = profile_manifest(parse(read(version / 'manifest.json', 8192, mode=0o400)))
    if manifest['profile'] != profile:
        raise ValueError('Version path/manifest mismatch')
    for name, record in manifest['files'].items():
        data = read(version / name, 100000, mode=0o400)
        if sha(data) != record['sha256'] or len(data) != record['size']:
            raise ValueError('Immutable source mismatch')
    if sha(read(version / 'helper_install.py', 100000, mode=0o400)) != manifest['installer_sha256']:
        raise ValueError('Immutable installer mismatch')
    return version, manifest


def state():
    value = parse(read(ROOT / 'install.json', 8192, mode=0o600))
    if (type(value) is not dict or value.get('schema') != 1 or value.get('phase') not in
            ('VERSION_READY', 'HOOK_INTENT', 'INSTALLED', 'REMOVE_INTENT', 'HOOK_RESTORED')
            or type(value.get('autostart')) is not bool
            or not re.fullmatch('[0-9a-f]{64}', value.get('profile', ''))
            or value.get('original_sha256') != HOOK_SHA
            or not re.fullmatch('[0-9a-f]{12}', value.get('nonce', ''))
            or type(value.get('original_upper_present')) is not bool
            or type(value.get('hook_mode')) is not int or value['hook_mode'] & 0o022):
        raise ValueError('Install journal schema')
    return value


def status():
    hook = read(HOOK, 65536)
    boot_id = BOOT_ID.read_text().strip()
    base = dict(installed=False, profile=None, manifest=None, partial=False,
        firmware_sys_sha256=SYS_SHA, autostart=dict(verified=False, hook=str(HOOK),
            backup_sha256=HOOK_SHA, current_sha256=sha(hook)), boot_mode='hydrate-only', boot_id=boot_id)
    if not os.path.lexists(ROOT):
        if sha(hook) != HOOK_SHA:
            raise ValueError('Stock post-boot hook differs')
        return base
    ensure_root()
    journal = state()
    if journal['phase'] != 'INSTALLED':
        raise ValueError('Partial installation: ' + journal['phase'])
    version, manifest = validate_version(journal['profile'])
    original = read(ROOT / 'hook.original', 65536, mode=0o400)
    expected = installed_hook(original, journal['profile']) if journal['autostart'] else original
    if sha(original) != HOOK_SHA or hook != expected or sha(hook) != journal['installed_sha256']:
        raise ValueError('Hook ownership/hash mismatch')
    base.update(installed=True, profile=manifest['profile'], manifest=manifest)
    base['autostart'].update(verified=True, enabled=journal['autostart'], current_sha256=sha(hook))
    base['autostart']['original_upper_present'] = journal['original_upper_present']
    cache = cache_path(manifest['profile'])
    base['boot_cache'] = dict(hydrated=False, boot_id=boot_id, profile=manifest['profile'])
    if os.path.lexists(cache):
        if cache_pending(cache):
            base['boot_cache'].update(phase='pending', work=str(cache))
        else:
            validate_cache(manifest['profile'], boot_id)
            base['boot_cache'].update(hydrated=True, work=str(cache), mode='bytes-only')
    return base


def install(work, nonce):
    work = Path(work)
    if work != RAM / ('osmo-helper-install-' + nonce):
        raise ValueError('Exact fresh bootstrap work required')
    private(work, directory=True, mode=0o700)
    if {p.name for p in work.iterdir()} != {'helper_install.py', 'request.json', 'bundle.z'}:
        raise ValueError('Unexpected bootstrap inventory')
    source = read(work / 'helper_install.py', 100000)
    request = parse(read(work / 'request.json', 8192))
    manifest, files = decode_bundle(read(work / 'bundle.z', MAX_PACKED), request)
    if sha(source) != manifest['installer_sha256'] or Path(__file__).resolve() != work / 'helper_install.py':
        raise ValueError('Running exact uploaded installer required')
    original = read(HOOK, 65536)
    hook_info = private(HOOK)
    if (stat.S_IMODE(hook_info.st_mode) != 0o755 or hook_info.st_gid != GID
            or sha(original) != HOOK_SHA or sha(read(PARENT, 65536)) != PARENT_SHA):
        raise ValueError('Exact stock boot chain required before install')
    upper_present = os.path.lexists(UPPER_HOOK)
    if upper_present and read(UPPER_HOOK, 65536) != original:
        raise ValueError('Existing upper hook differs from merged stock hook')
    ensure_root()
    with locked():
        if (ROOT / 'install.json').exists() or (ROOT / 'versions').exists():
            raise ValueError('Install is single-use; status/remove existing state first')
        versions = ROOT / 'versions'
        versions.mkdir(mode=0o700)
        version = versions / manifest['profile']
        version.mkdir(mode=0o700)
        # No hook is touched until all immutable bytes have been verified.
        for name, data in files.items():
            create(version / name, data)
        create(version / 'helper_install.py', source)
        create(version / 'manifest.json', canonical(manifest))
        os.chmod(version, 0o500)
        fsync_dir(versions)
        validate_version(manifest['profile'])
        create(ROOT / 'hook.original', original)
        target = installed_hook(original, manifest['profile']) if request['autostart'] else original
        journal = dict(schema=1, phase='VERSION_READY', profile=manifest['profile'],
            autostart=request['autostart'], original_sha256=HOOK_SHA,
            installed_sha256=sha(target), hook_mode=stat.S_IMODE(hook_info.st_mode), nonce=nonce,
            original_upper_present=upper_present)
        json_file(ROOT / 'install.json', journal, nonce=nonce)
        if request['autostart']:
            journal['phase'] = 'HOOK_INTENT'
            json_file(ROOT / 'install.json', journal, nonce=nonce)
            replace(HOOK, target, expected=original, nonce=nonce, mode=journal['hook_mode'])
        journal['phase'] = 'INSTALLED'
        json_file(ROOT / 'install.json', journal, nonce=nonce)
        result = status()
    return result


def cache_path(profile):
    return RAM / ('osmo360-sdk-cache-' + profile[:12])


def cache_pending(cache):
    """Incomplete bytes are never adopted; verified disk bytes remain usable."""
    private(cache, directory=True, mode=0o700)
    names = {path.name for path in cache.iterdir()}
    if not names <= set(FILES) | {'cache.json'}:
        raise ValueError('Foreign boot cache content')
    if 'cache.json' not in names:
        for name in names:
            private(cache / name)
        return True
    return False


def validate_cache(profile, boot_id):
    cache = cache_path(profile)
    private(cache, directory=True, mode=0o700)
    if {p.name for p in cache.iterdir()} != set(FILES) | {'cache.json'}:
        raise ValueError('Unexpected boot cache inventory')
    metadata = parse(read(cache / 'cache.json', 4096, mode=0o400))
    if metadata != dict(schema=1, profile=profile, boot_id=boot_id, files=FILES,
                        hydrated=True, mode='bytes-only'):
        raise ValueError('Boot cache identity mismatch')
    for name, digest in FILES.items():
        if sha(read(cache / name, 100000, mode=0o400)) != digest:
            raise ValueError('Boot cache source mismatch')
    return cache


def hydrate(profile, boot_id):
    version, _ = validate_version(profile)
    cache = cache_path(profile)
    if os.path.lexists(cache):
        return validate_cache(profile, boot_id)
    # O_EXCL mkdir reserves the exact cache. The verified metadata marker is
    # written last, so interrupted boot copies are never executable/adopted.
    cache.mkdir(mode=0o700)
    for name in FILES:
        create(cache / name, read(version / name, 100000, mode=0o400))
    create(cache / 'cache.json', canonical(dict(schema=1, profile=profile, boot_id=boot_id,
                                             files=FILES, hydrated=True, mode='bytes-only')))
    fsync_dir(RAM)
    return validate_cache(profile, boot_id)


def stage(profile, group, destination, boot_id):
    if group not in GROUPS:
        raise ValueError('Exact helper group required')
    destination = Path(destination)
    patterns = {
        'owner': re.escape(str(RAM)) + r'/osmo-usb-standby-[0-9a-f]{12}',
        'prepared': re.escape(str(RAM)) + r'/osmo-prepared-assets-[0-9a-f]{12}',
        'retained': re.escape(str(RAM)) + r'/osmo-usb-standby-[0-9a-f]{12}/retained-assets-126bc7ea2606',
    }
    if not re.fullmatch(patterns[group], str(destination)):
        raise ValueError('Exact existing RAM destination required')
    destination_info = private(destination, directory=True, mode=0o700)
    if list(destination.iterdir()):
        raise ValueError('Source destination is not empty')
    current = status()
    if current['profile'] != profile:
        raise ValueError('Installed profile differs')
    version, manifest = validate_version(profile)
    cache = cache_path(profile)
    source = version
    if os.path.lexists(cache) and not cache_pending(cache):
        source = validate_cache(profile, boot_id)
    result = {}
    for name in GROUPS[group]:
        if identity(private(destination, directory=True, mode=0o700)) != identity(destination_info):
            raise ValueError('RAM destination replaced')
        data = read(source / name, 100000, mode=0o400)
        create(destination / name, data)
        result[name] = manifest['files'][name]
    if {p.name for p in destination.iterdir()} != set(GROUPS[group]):
        raise ValueError('Source destination inventory changed')
    return dict(current, staged=True, group=group, dest=str(destination), files=result, profile=profile)


def remove(nonce):
    stock_overlay_copy_remaining = False
    if not os.path.lexists(ROOT):
        if sha(read(HOOK, 65536)) != HOOK_SHA:
            raise ValueError('Cannot claim removal with changed hook')
        return dict(removed=True, installed=False, profile=None, reboot_required=True,
                    stock_overlay_copy_remaining=os.path.lexists(UPPER_HOOK))
    private(ROOT, directory=True, mode=0o700)
    # mkdir/marker creation is the first operation. Power loss at that point
    # leaves no boot change and must not make a zero-byte marker irremovable.
    early_names = {p.name for p in ROOT.iterdir()}
    if early_names <= {'managed.json'} and sha(read(HOOK, 65536)) == HOOK_SHA:
        marker = ROOT / 'managed.json'
        if marker.exists():
            if not canonical(MARKER).startswith(read(marker, 1024)):
                raise ValueError('Foreign incomplete root marker')
            marker.unlink()
        ROOT.rmdir()
        fsync_dir(ROOT.parent)
        return dict(removed=True, installed=False, profile=None, reboot_required=True,
                    stock_overlay_copy_remaining=os.path.lexists(UPPER_HOOK))
    ensure_root()
    with locked():
        journal_path = ROOT / 'install.json'
        journal = None
        if journal_path.exists():
            try:
                journal = state()
            except (ValueError, KeyError):
                # An interrupted FIRST journal write precedes all hook writes.
                # Corruption after a hook change cannot use this escape hatch.
                if sha(read(HOOK, 65536)) != HOOK_SHA:
                    raise
        if journal is not None:
            original = read(ROOT / 'hook.original', 65536, mode=0o400)
            if sha(original) != HOOK_SHA:
                raise ValueError('Original hook backup mismatch')
            target = installed_hook(original, journal['profile']) if journal['autostart'] else original
            current = read(HOOK, 65536)
            if current not in (original, target):
                raise ValueError('Foreign hook change; refusing removal overwrite')
            temporary = HOOK.with_name(HOOK.name + '.osmo-' + journal['nonce'])
            if os.path.lexists(temporary):
                content = read(temporary, 65536)
                if not target.startswith(content) and not original.startswith(content):
                    raise ValueError('Foreign hook temporary content')
                temporary.unlink()
                fsync_dir(HOOK.parent)
            journal['phase'] = 'REMOVE_INTENT'
            json_file(journal_path, journal, nonce=nonce)
            if current == target and current != original:
                replace(HOOK, original, expected=target, nonce=journal['nonce'], mode=journal['hook_mode'])
            if read(HOOK, 65536) != original:
                raise ValueError('Original hook restoration not verified')
            if os.path.lexists(UPPER_HOOK):
                if read(UPPER_HOOK, 65536) != original:
                    raise ValueError('Upper hook restoration not verified')
                stock_overlay_copy_remaining = not journal['original_upper_present']
            # A crash during atomic hook preparation can leave its exclusive
            # temporary file. Its journal nonce and expected byte prefix prove
            # ownership; never sweep arbitrary files from /etc.
            if os.path.lexists(temporary):
                content = read(temporary, 65536)
                if not target.startswith(content) and not original.startswith(content):
                    raise ValueError('Foreign hook temporary content')
                temporary.unlink()
                fsync_dir(HOOK.parent)
            journal['phase'] = 'HOOK_RESTORED'
            json_file(journal_path, journal, nonce=nonce)
        elif sha(read(HOOK, 65536)) != HOOK_SHA:
            raise ValueError('No journal proving changed hook ownership')
        # Delete only fixed managed inventory. Unknown content is never traversed.
        allowed = {'managed.json', '.lock', 'install.json', 'hook.original', 'versions'}
        names = {p.name for p in ROOT.iterdir()}
        temporaries = {name for name in names if re.fullmatch(r'install\.json\.osmo-[0-9a-f]{12}', name)}
        if not names <= allowed | temporaries:
            raise ValueError('Foreign managed-root file; refusing recursive removal')
        for name in temporaries:
            private(ROOT / name)
            (ROOT / name).unlink()
        versions = ROOT / 'versions'
        if versions.exists():
            private(versions, directory=True, mode=0o700)
            for version in versions.iterdir():
                if not re.fullmatch('[0-9a-f]{64}', version.name):
                    raise ValueError('Foreign version directory')
                private(version, directory=True)
                if not {p.name for p in version.iterdir()} <= set(FILES) | {'helper_install.py', 'manifest.json'}:
                    raise ValueError('Foreign version content')
                # Partial publication before hook intent is removable too.
                os.chmod(version, 0o700)
                for path in version.iterdir():
                    private(path)
                    path.unlink()
                version.rmdir()
            versions.rmdir()
        for name in ('hook.original', 'install.json', 'managed.json'):
            path = ROOT / name
            if path.exists():
                private(path)
                path.unlink()
        private(ROOT / '.lock')
        (ROOT / '.lock').unlink()
        ROOT.rmdir()
        fsync_dir(ROOT.parent)
    # Running RAM users are deliberately retained until the host's explicit
    # stock reboot. Never unmount a role shadow, kill an owner, or replay USB.
    return dict(removed=True, installed=False, profile=None, reboot_required=True,
                stock_overlay_copy_remaining=stock_overlay_copy_remaining)


def export(nonce, operation, value):
    if not re.fullmatch('[0-9a-f]{12}', nonce):
        raise ValueError('Exact diagnostic nonce required')
    private(EXPORT.parent, directory=True)
    if not os.path.lexists(EXPORT):
        EXPORT.mkdir(mode=0o700)
    private(EXPORT, directory=True)
    path = EXPORT / ('osmo_helpers_' + nonce + '.json')
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | O_FLAGS, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        info = os.fstat(stream.fileno())
        result = dict(value, schema=1, operation=operation, nonce=nonce,
                      export_identity=dict(device=info.st_dev, inode=info.st_ino))
        data = canonical(result)
        if len(data) > MAX_REPORT:
            raise ValueError('Diagnostic report exceeds16KiB')
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('status', 'install', 'stage', 'boot', 'remove'))
    parser.add_argument('--nonce')
    parser.add_argument('--work')
    parser.add_argument('--profile')
    parser.add_argument('--group', choices=tuple(GROUPS))
    parser.add_argument('--dest')
    args = parser.parse_args(argv)
    # Boot does no IPC/native calls and has an absolute10s process lifetime.
    if args.operation == 'boot':
        signal.signal(signal.SIGALRM, lambda *_: os._exit(124))
        signal.setitimer(signal.ITIMER_REAL, 10)
    elif not args.nonce or not re.fullmatch('[0-9a-f]{12}', args.nonce):
        raise ValueError('Fresh diagnostic nonce required')
    boot_id = environment()
    try:
        if args.operation == 'status':
            result = status()
        elif args.operation == 'install':
            result = install(args.work, args.nonce)
        elif args.operation == 'stage':
            result = stage(args.profile, args.group, args.dest, boot_id)
        elif args.operation == 'remove':
            result = remove(args.nonce)
        else:
            current = status()
            if current['profile'] != args.profile or not current['autostart'].get('enabled'):
                raise ValueError('Boot profile/hook is not installed')
            hydrate(args.profile, boot_id)
            return 0
        result['boot_id'] = boot_id
        export(args.nonce, args.operation, result)
        return 0
    except Exception as error:
        if args.operation != 'boot':
            export(args.nonce, args.operation, dict(installed=False, partial=os.path.lexists(ROOT), boot_id=boot_id,
                   error=repr(error)[:1024]))
        raise


if __name__ == '__main__':
    raise SystemExit(main())
