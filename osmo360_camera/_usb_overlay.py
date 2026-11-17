#!/usr/bin/env python3
"""Generate, never execute, a single-use RAM-only native NCM/ECM USB overlay.

Requires runtime values read by the camera's sole USB owner after authorized
shell access. No firmware/config files are modified: generated actions only
append configfs f6, rebind the saved UDC, and add a temporary netdev address.
The watchdog always rolls back after 60..120s; there is no permanent commit.
"""

from dataclasses import dataclass
import base64
import hashlib
import ipaddress
import posixpath
import re
import shlex
from pathlib import Path
import uuid

GADGET = "/sys/kernel/config/usb_gadget/g1"
FUNCTIONS = ("ffs.diag", "ffs.bulk", "ffs.bulk1", "ffs.bulk5", "ffs.bulk4")
Q = shlex.quote


@dataclass(frozen=True)
class OverlaySpec:
    udc: str
    net_if: str
    original_flags: str
    ram_dir: str = "/dev/shm"  # Must itself be a canonical tmpfs mountpoint.
    vendor: str = "0x2ca3"
    product: str = "0x0025"
    configuration: str = "BULK"
    original_links: tuple = tuple("../../functions/" + name for name in FUNCTIONS)
    original_ipv4: tuple = ()
    address: str = "192.168.253.1/30"
    watchdog_seconds: int = 90
    tag: str = ""
    function_name: str = "ncm.0"
    original_netdev_absent: bool = False
    initial_ifname: str = ""
    dev_addr: str = ""
    host_addr: str = ""

    def __post_init__(self):
        for name, maximum in [("udc", 100), ("net_if", 15)]:
            value = getattr(self, name)
            if name == "net_if" and self.original_netdev_absent and value is None:
                continue
            if not isinstance(value, str) or not re.fullmatch(
                r"[A-Za-z0-9_.:-]{1,%d}" % maximum, value
            ):
                raise ValueError(f"Invalid {name}")
        if self.function_name not in ("ncm.0", "ecm.ecm"):
            raise ValueError(
                "Only the exact existing ncm.0 or ecm.ecm function is supported"
            )
        if type(self.original_netdev_absent) is not bool:
            raise ValueError("original_netdev_absent must be boolean")
        if self.original_netdev_absent:
            if (
                self.function_name != "ecm.ecm"
                or self.net_if is not None
                or self.original_flags is not None
            ):
                raise ValueError(
                    "Absent-netdev mode requires existing ECM and explicit null interface/flags"
                )
            if self.initial_ifname != "(unnamed net_device)":
                raise ValueError("Exact observed ECM ifname placeholder is required")
            for field in ("dev_addr", "host_addr"):
                value = getattr(self, field)
                if not isinstance(value, str) or not re.fullmatch(
                    r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", value
                ):
                    raise ValueError("Exact normalized runtime MAC required: " + field)
                if int(value[:2], 16) & 1 or value == "00:00:00:00:00:00":
                    raise ValueError("Runtime MAC must be nonzero unicast: " + field)
            if self.dev_addr == self.host_addr:
                raise ValueError("ECM device/host MACs must be distinct")
        elif not isinstance(self.original_flags, str) or not re.fullmatch(
            r"0x[0-9a-fA-F]+", self.original_flags
        ):
            raise ValueError("original_flags must be the exact sysfs hexadecimal value")
        if (
            int(self.vendor, 16) != 0x2CA3
            or int(self.product, 16) != 0x25
            or self.configuration != "BULK"
        ):
            raise ValueError(
                "Only the native shooting 2ca3:0025 BULK baseline is supported"
            )
        if (
            not re.fullmatch(r"/[A-Za-z0-9_./-]+", self.ram_dir)
            or posixpath.normpath(self.ram_dir) != self.ram_dir
            or self.ram_dir == "/"
        ):
            raise ValueError("ram_dir must be a canonical absolute tmpfs mountpoint")
        if len(self.original_links) != 5:
            raise ValueError(
                "Exactly the native five original function links are required"
            )
        for target, function in zip(self.original_links, FUNCTIONS):
            if (
                not isinstance(target, str)
                or "\n" in target
                or posixpath.normpath(posixpath.join(GADGET, "configs/c.1", target))
                != GADGET + "/functions/" + function
            ):
                raise ValueError(
                    "Original link does not match the native DIAG/BULK function order"
                )
        if self.original_ipv4:
            raise ValueError("Refuse to replace any existing NCM IPv4 configuration")
        address = ipaddress.ip_interface(self.address)
        if address.version != 4 or address.network.prefixlen != 30:
            raise ValueError("Use a previously checked unused IPv4 /30 test subnet")
        if address.ip in (
            address.network.network_address,
            address.network.broadcast_address,
        ):
            raise ValueError("Camera address must be a usable host address")
        if (
            type(self.watchdog_seconds) is not int
            or not 60 <= self.watchdog_seconds <= 120
        ):
            raise ValueError("watchdog_seconds must be 60..120")
        if self.tag and not re.fullmatch(r"[a-zA-Z0-9_-]{1,32}", self.tag):
            raise ValueError("Invalid single-use run tag")


def _preamble(spec, work):
    checks = "\n".join(
        f'  [ "$(readlink "$C/f{i}")" = {Q(target)} ] || return 1'
        for i, target in enumerate(spec.original_links, 1)
    )
    return f"""#!/bin/sh
set -eu
G={Q(GADGET)}
C="$G/configs/c.1"
F="$G/functions/{spec.function_name}"
W={Q(work)}
U={Q(spec.udc)}
I={Q(spec.net_if or "")}
A={Q(spec.address)}
check_base() {{
  [ "$(id -u)" = 0 ] || return 1
  [ "$(cat "$G/idVendor")" = {Q(spec.vendor)} ] || return 1
  [ "$(cat "$G/idProduct")" = {Q(spec.product)} ] || return 1
  [ "$(cat "$C/strings/0x409/configuration")" = {Q(spec.configuration)} ] || return 1
{checks}
  for p in "$C"/*; do
    [ -L "$p" ] || continue
    case "${{p##*/}}" in f1|f2|f3|f4|f5|f6) ;; *) return 1 ;; esac
  done
}}
check_ram() {{
  [ "$(readlink -f {Q(spec.ram_dir)})" = {Q(spec.ram_dir)} ] || return 1
  [ "$(awk -v p={Q(spec.ram_dir)} '$2==p {{print $3}}' /proc/mounts)" = tmpfs ] || return 1
  [ -d "$W" ] && [ ! -L "$W" ]
}}
"""


def generate(spec: OverlaySpec, *, command_limit=500):
    # Native D1 worker appends a space and NUL in a 512-byte sprintf buffer.
    # Keep the same conservative ASCII cap as the owning D1 session.
    if not 256 <= command_limit <= 500:
        raise ValueError("D1 command_limit must be 256..500 ASCII bytes")
    tag = spec.tag or uuid.uuid4().hex[:16]
    work = spec.ram_dir + "/osmo-ncm-" + tag
    head = _preamble(spec, work)
    original_up = bool(int(spec.original_flags or "0x0", 16) & 1)
    if spec.function_name == "ecm.ecm":
        net_preflight = """command -v python3 >/dev/null || exit 82
[ -d "$F" ] && [ -f "$W/netdev.py" ] && [ ! -L "$W/netdev.py" ] || exit 44
python3 "$W/netdev.py" preflight || exit 83"""
        net_recheck = 'python3 "$W/netdev.py" preflight || exit 83'
        net_discover = 'I=$(python3 "$W/netdev.py" discover) || exit 55'
        net_cleanup = """python3 "$W/netdev.py" stop-dhcp || failed=80
python3 "$W/netdev.py" cleanup || failed=34"""
        net_verify = 'python3 "$W/netdev.py" verify || failed=64'
    else:
        net_preflight = f"""[ -d "$F" ] && [ "$(cat "$F/ifname")" = "$I" ] || exit 44
[ -d "/sys/class/net/$I" ] || exit 45
[ "$(cat "/sys/class/net/$I/flags")" = {Q(spec.original_flags)} ] || exit 46
addresses=$(ip -o -4 addr show dev "$I") || exit 48
[ -z "$addresses" ] || exit 49"""
        net_recheck = ""
        net_discover = '[ "$(cat "$F/ifname")" = "$I" ] || exit 55'
        net_cleanup = f"""ip addr del "$A" dev "$I" 2>/dev/null || :
ip link set "$I" {"up" if original_up else "down"} || failed=34"""
        net_verify = f"""[ "$(cat "$F/ifname")" = "$I" ] || failed=60
addresses=$(ip -o -4 addr show dev "$I") || failed=61
[ -z "$addresses" ] || failed=62
flags=$(cat "/sys/class/net/$I/flags") || failed=63
[ "$((flags & 1))" = {1 if original_up else 0} ] || failed=64"""
    rollback = (
        head
        + f"""
check_ram || exit 30
[ -f "$W/armed" ] || exit 0
mkdir "$W/rollback.lock" 2>/dev/null || exit 76
trap 'rmdir "$W/rollback.lock" 2>/dev/null || :' 0
# Stop the exact owned apply process group before restoring any USB/IP state.
# Its own EXIT trap is the sole exemption, verified by parent ancestry.
if [ "$#" -eq 1 ]; then
  python3 "$W/lifecycle.py" quiesce "$1" || exit 81
else
  python3 "$W/lifecycle.py" quiesce || exit 81
fi
check_base || exit 31
# Refuse to alter a different actor's replacement configuration.
if [ -e "$C/f6" ] || [ -L "$C/f6" ]; then
  [ -L "$C/f6" ] && [ "$(readlink -f "$C/f6")" = "$F" ] || exit 32
fi
current_udc=$(cat "$G/UDC")
[ -z "$current_udc" ] || [ "$current_udc" = "$U" ] || exit 33
# A preflight/guardian failure must not itself disconnect an untouched USB.
if [ ! -f "$W/usb.touched" ]; then
  [ "$current_udc" = "$U" ] || exit 33
  [ ! -e "$C/f6" ] && [ ! -L "$C/f6" ] || exit 32
  printf 'ROLLED_BACK\\n' > "$W/status"
  rm "$W/armed"
  exit 0
fi
# Network errors must not skip USB restoration.
failed=0
{net_cleanup}
printf '\\n' > "$G/UDC" || failed=35
if [ -L "$C/f6" ]; then
  if [ -f "$W/f6.owned" ]; then rm "$C/f6" || failed=36; else failed=77; fi
fi
bound=0
attempt=0
while [ "$attempt" -lt 3 ]; do
  if printf '%s' "$U" > "$G/UDC" && [ "$(cat "$G/UDC")" = "$U" ]; then
    bound=1
    break
  fi
  attempt=$((attempt + 1))
  sleep 1
done
[ "$bound" = 1 ] || failed=37
check_base || failed=38
[ ! -e "$C/f6" ] && [ ! -L "$C/f6" ] || failed=39
{net_verify}
if [ "$failed" -ne 0 ]; then
  printf 'ROLLBACK_FAILED:%s\\n' "$failed" > "$W/status"
  exit "$failed"
fi
printf 'ROLLED_BACK\\n' > "$W/status"
rm "$W/armed"
"""
    )
    watchdog = (
        head
        + f"""
# Survive RPC hangups; the launcher also requires setsid.
trap '' 1
check_ram || exit 70
[ -f "$W/armed" ] || exit 0
[ -f "$W/rollback.sh" ] && [ ! -L "$W/rollback.sh" ] || exit 71
printf '%s\\n' "$$" > "$W/watchdog.ready"
sleep {spec.watchdog_seconds}
/bin/sh "$W/rollback.sh" >>"$W/rollback.log" 2>&1
"""
    )
    apply = (
        head
        + f"""
check_ram || exit 40
check_base || exit 41
[ "$(cat "$G/UDC")" = "$U" ] || exit 42
[ ! -e "$C/f6" ] && [ ! -L "$C/f6" ] || exit 43
command -v ip >/dev/null || exit 47
command -v setsid >/dev/null || exit 59
command -v python3 >/dev/null || exit 82
{net_preflight}
[ ! -e "$W/used" ] || exit 50
# One apply per fresh RAM directory, so an old watchdog cannot affect a retry.
: > "$W/used"
python3 "$W/lifecycle.py" claim "$$" || exit 84
: > "$W/armed"
trap 'rc=$?; trap - 0; if [ "$rc" -ne 0 ]; then /bin/sh "$W/rollback.sh" "$$" >>"$W/rollback.log" 2>&1 || :; fi; exit "$rc"' 0
trap 'exit 51' 1 2 15
setsid /bin/sh "$W/watchdog.sh" </dev/null >/dev/null 2>&1 &
printf '%s\\n' "$!" > "$W/watchdog.launcher.pid"
attempt=0
while [ ! -s "$W/watchdog.ready" ] && [ "$attempt" -lt 5 ]; do
  sleep 1
  attempt=$((attempt + 1))
done
watchdog_pid=$(cat "$W/watchdog.ready") || exit 72
case "$watchdog_pid" in ''|*[!0-9]*) exit 73 ;; esac
kill -0 "$watchdog_pid" || exit 74
printf '%s\\n' "$watchdog_pid" > "$W/watchdog.pid"
# The calling AMT RPC must finish before its own USB link is rebound.
sleep 2
check_base || exit 52
[ "$(cat "$G/UDC")" = "$U" ] || exit 53
[ ! -e "$C/f6" ] && [ ! -L "$C/f6" ] || exit 78
{net_recheck}
kill -0 "$watchdog_pid" || exit 75
printf 'UNBINDING\\n' > "$W/status"
: > "$W/usb.touched"
printf '\\n' > "$G/UDC"
# Record ownership intent before link creation; preflight proved f6 absent.
: > "$W/f6.owned"
ln -s "$F" "$C/f6"
printf '%s' "$U" > "$G/UDC"
[ "$(cat "$G/UDC")" = "$U" ] || exit 54
{net_discover}
ip addr add "$A" dev "$I"
ip link set "$I" up
check_base || exit 56
[ "$(readlink -f "$C/f6")" = "$F" ] || exit 57
ip -o -4 addr show dev "$I" | awk '{{print $4}}' | grep -Fx "$A" >/dev/null || exit 58
# OPTIONAL_RAM_SERVICE_AFTER_IP
printf 'APPLIED\\n' > "$W/status"
"""
    )
    files = {"rollback.sh": rollback, "watchdog.sh": watchdog, "apply.sh": apply}
    lifecycle = (Path(__file__).with_name("_device") / "lifecycle.py").read_text()
    files["lifecycle.py"] = lifecycle.replace(
        "CONFIG = None", "CONFIG = " + repr({"work": work, "proc": "/proc"}), 1
    )
    if spec.function_name == "ecm.ecm":
        config = {
            "function": GADGET + "/functions/" + spec.function_name,
            "work": work,
            "sys_net": "/sys/class/net",
            "dev_mac": spec.dev_addr,
            "host_mac": spec.host_addr,
            "initial_ifname": spec.initial_ifname,
            "address": spec.address,
            "original_netdev_absent": spec.original_netdev_absent,
            "original_net_if": spec.net_if,
            "original_flags": spec.original_flags,
        }
        template = (Path(__file__).with_name("_device") / "netdev.py").read_text()
        files["netdev.py"] = template.replace(
            "CONFIG = None", "CONFIG = " + repr(config), 1
        )
    commands = []
    bootstrap = (
        f'[ "$(readlink -f {Q(spec.ram_dir)})" = {Q(spec.ram_dir)} ] && '
        f"[ \"$(awk -v p={Q(spec.ram_dir)} '$2==p {{print $3}}' /proc/mounts)\" = tmpfs ] && "
        f"umask 077 && mkdir {Q(work)} && "
        f": > {Q(work + '/rollback.sh')} && : > {Q(work + '/watchdog.sh')} && "
        f": > {Q(work + '/apply.sh')} && printf NCM_BOOT_OK"
    )
    commands.append({"command": bootstrap, "expect": "NCM_BOOT_OK"})
    for name, source in files.items():
        data, offset = source.encode(), 0
        while offset < len(data):
            token = f"NCM_UPLOAD_{len(commands):04d}_OK"
            template = (
                f"test -d {Q(work)} && test ! -L {Q(work + '/' + name)} && "
                f"printf '%s' '{{data}}' | base64 -d >> {Q(work + '/' + name)} && printf {token}"
            )
            capacity = (
                (command_limit - len(template.format(data="").encode())) // 4
            ) * 3
            if capacity < 3:
                raise ValueError(
                    "Runtime paths are too long for bounded AMT upload commands"
                )
            piece = data[offset : offset + capacity]
            encoded = base64.b64encode(piece).decode()
            command = template.format(data=encoded)
            commands.append(
                {
                    "command": command,
                    "expect": token,
                    "file": name,
                    "offset": offset,
                    "decoded_bytes": len(piece),
                }
            )
            offset += len(piece)
    result = {
        "work_dir": work,
        "address": spec.address,
        "function_name": spec.function_name,
        "original_netdev_absent": spec.original_netdev_absent,
        "command_limit": command_limit,
        "watchdog_seconds": spec.watchdog_seconds,
        "scripts": files,
        "upload_commands": commands,
        "sha256": {
            name: hashlib.sha256(source.encode()).hexdigest()
            for name, source in files.items()
        },
        "byte_lengths": {name: len(source.encode()) for name, source in files.items()},
        "verify_command": "sha256sum "
        + " ".join(Q(work + "/" + name) for name in files),
        "apply_command": (
            f"setsid /bin/sh {Q(work + '/apply.sh')} </dev/null "
            f'>{Q(work + "/apply.log")} 2>&1 & printf "NCM_SCHEDULED:%s" "$!"'
        ),
        "rollback_command": (
            f"setsid /bin/sh {Q(work + '/rollback.sh')} </dev/null "
            f'>{Q(work + "/manual-rollback.log")} 2>&1 & printf "NCM_ROLLBACK:%s" "$!"'
        ),
        "status_command": f"cat {Q(work + '/status')} 2>/dev/null",
        "note": "Compare verify_command output to sha256 before running apply_command. "
        "All commands are generated text; this module never contacts the camera. "
        "The /30 must already be checked unused on both hosts.",
    }
    for entry in commands:
        if (
            not entry["command"].isascii()
            or len(entry["command"].encode()) > command_limit
        ):
            raise ValueError("An AMT upload command exceeds its byte limit")
    for key in (
        "verify_command",
        "apply_command",
        "rollback_command",
        "status_command",
    ):
        if not result[key].isascii() or len(result[key].encode()) > command_limit:
            raise ValueError(f"{key} exceeds the AMT byte limit")
    return result
