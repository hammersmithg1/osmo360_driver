"""Pure compressed transport representation of exact generated USB scripts."""

import base64
import hashlib
import json
from pathlib import Path
import re
import shlex
import zlib

NAMES = frozenset(
    (
        "apply.sh",
        "rollback.sh",
        "watchdog.sh",
        "lifecycle.py",
        "netdev.py",
        "lease.py",
        "ecm_dhcp.py",
    )
)


def build_bundle(scripts, work):
    if set(scripts) != NAMES or not re.fullmatch(
        r"/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}", work
    ):
        raise ValueError("Exact script allowlist and fresh RAM path required")
    files = {}
    for name, source in scripts.items():
        if (
            not isinstance(source, str)
            or not source.isascii()
            or "\0" in source
            or not source
        ):
            raise ValueError("NUL-free ASCII generated source required")
        data = source.encode("ascii")
        files[name] = dict(
            source=source, bytes=len(data), sha256=hashlib.sha256(data).hexdigest()
        )
    plain = json.dumps(
        dict(schema=1, files=files),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("ascii")
    packed = zlib.compress(plain, 9)
    if len(plain) > 65536 or len(packed) > 32768:
        raise ValueError("Script bundle exceeds reviewed bounds")
    plain_sha, packed_sha = (
        hashlib.sha256(plain).hexdigest(),
        hashlib.sha256(packed).hexdigest(),
    )
    loader = (Path(__file__).with_name("_device") / "script_bundle.py").read_bytes()
    loader.decode("ascii")
    compile(loader, "unpack.py", "exec")
    q = shlex.quote
    commands = [
        '[ "$(readlink -f /dev/shm)" = /dev/shm ] && '
        '[ "$(awk \'$2=="/dev/shm" {print $3}\' /proc/mounts)" = tmpfs ] && '
        f"umask 077 && mkdir {q(work)} && set -C && : > {q(work + '/unpack.py')} "
        f"&& : > {q(work + '/scripts.z')}"
    ]
    for name, data in (("unpack.py", loader), ("scripts.z", packed)):
        encoded = base64.b64encode(data).decode("ascii")
        for offset in range(0, len(encoded), 320):
            commands.append(
                f"printf %s {encoded[offset : offset + 320]} | base64 -d >> {q(work + '/' + name)}"
            )
    loader_sha = hashlib.sha256(loader).hexdigest()
    commands += [
        f'[ "$(sha256sum {q(work + "/unpack.py")} | cut -d" " -f1)" = {loader_sha} ]',
        f"python3 {q(work + '/unpack.py')} {q(work)} {len(packed)} {packed_sha} {len(plain)} {plain_sha}",
        f'[ "$(cat {q(work + "/bundle.ready")})" = {plain_sha} ]',
    ]
    if any(
        not command.isascii() or len(command.encode("ascii")) > 500
        for command in commands
    ):
        raise ValueError("Bundle command exceeds500-byte worker limit")
    return dict(
        commands=commands,
        compressed_bytes=len(packed),
        plain_bytes=len(plain),
        compressed_sha256=packed_sha,
        plain_sha256=plain_sha,
        loader_sha256=loader_sha,
        command_count=len(commands),
        files={"unpack.py": loader, "scripts.z": packed},
    )
