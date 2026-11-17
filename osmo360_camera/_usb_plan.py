"""Pure builder for the reviewed, renewable USB2 ECM overlay.

Only generated camera-side RAM scripts change the gadget, after explicit caller
authorization. Importing or building a plan never opens USB, runs a command, or
changes host networking. Runtime values come from the owned USB bootstrap.
"""

import hashlib
import ipaddress
from pathlib import Path
import re
import secrets
import shlex

from ._usb_bootstrap import USBRuntime, _mac
from ._usb_bundle import build_bundle
from ._usb_overlay import OverlaySpec, generate


DEVICE = Path(__file__).with_name("_device")
Q = shlex.quote


def _replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError("Reviewed overlay template marker changed: " + old)
    return source.replace(old, new, 1)


def _validate_runtime(runtime):
    if not isinstance(runtime, USBRuntime):
        raise TypeError("runtime must be the validated USBRuntime report")
    if (
        runtime.original_max_speed != "super-speed-plus"
        or runtime.ram_dir != "/dev/shm"
    ):
        raise ValueError("Expected the native SSP baseline and canonical /dev/shm")
    if runtime.function_name != "ecm.ecm":
        raise ValueError("Only the existing native ECM function is supported")
    if _mac(runtime.dev_addr) == _mac(runtime.host_addr):
        raise ValueError("Camera and host USB MACs must differ")
    if runtime.original_netdev_absent:
        if (
            runtime.net_if is not None
            or runtime.original_flags is not None
            or runtime.initial_ifname != "(unnamed net_device)"
        ):
            raise ValueError("Absent ECM requires its exact native placeholder")
    elif (
        runtime.original_flags != "0x1002"
        or runtime.initial_ifname != runtime.net_if
        or not isinstance(runtime.net_if, str)
        or not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", runtime.net_if)
    ):
        raise ValueError("Retained ECM must be the exact observed DOWN interface")
    for name, cidr in runtime.camera_ipv4:
        if name == runtime.net_if or ipaddress.IPv4Interface(cidr).network.overlaps(
            ipaddress.IPv4Network("192.168.253.0/30")
        ):
            raise ValueError("USB alias would replace or overlap existing camera IPv4")


def build_usb_plan(runtime: USBRuntime, tag=None, nonce=None, port=None):
    """Return scripts, bounded upload commands, hashes and the exact lease wire API."""
    _validate_runtime(runtime)
    tag = secrets.token_hex(8) if tag is None else tag
    nonce = secrets.token_hex(16) if nonce is None else nonce
    port = 49152 + secrets.randbelow(12000) if port is None else port
    if not isinstance(tag, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", tag):
        raise ValueError("tag must contain 1..32 safe single-use characters")
    if not isinstance(nonce, str) or not re.fullmatch(r"[0-9a-f]{32}", nonce):
        raise ValueError("nonce must be exactly 16 bytes encoded as lowercase hex")
    if type(port) is not int or not 49152 <= port <= 65535:
        raise ValueError("port must be a dedicated high UDP port")
    spec = OverlaySpec(**runtime.overlay_values(), watchdog_seconds=90, tag=tag)
    plan = generate(spec)
    files, work = plan["scripts"], plan["work_dir"]
    original = Q(runtime.original_max_speed)
    apply = _replace_once(
        files["apply.sh"],
        "check_base || exit 41",
        f'check_base || exit 41\n[ "$(cat "$G/max_speed")" = {original} ] || exit 88',
    )
    unbind = "printf '\\n' > \"$G/UDC\""
    apply = _replace_once(
        apply,
        unbind,
        unbind + "\n"
        'printf high-speed > "$G/max_speed"\n'
        '[ "$(cat "$G/max_speed")" = high-speed ] || exit 91',
    )
    rollback = _replace_once(
        files["rollback.sh"],
        unbind + " || failed=35",
        unbind + " || failed=35\n"
        f'printf %s {original} > "$G/max_speed" || failed=88\n'
        f'[ "$(cat "$G/max_speed")" = {original} ] || failed=90',
    )
    rollback = _replace_once(
        rollback,
        "check_base || failed=38",
        "check_base || failed=38\n"
        f'[ "$(cat "$G/max_speed")" = {original} ] || failed=89',
    )
    config = dict(
        work=work,
        nonce=nonce,
        port=port,
        camera_ip="192.168.253.1",
        host_ip="192.168.253.2",
        dev_mac=runtime.dev_addr,
        lease_seconds=10,
        setup_seconds=90,
    )
    files["lease.py"] = _replace_once(
        (DEVICE / "ecm_lease.py").read_text(),
        "CONFIG = None",
        "CONFIG = " + repr(config),
    )
    marker = 'printf \'%s\\n\' "$$" > "$W/watchdog.ready"'
    watchdog = files["watchdog.sh"]
    if watchdog.count(marker) != 1:
        raise RuntimeError("Reviewed watchdog ready marker changed")
    files["watchdog.sh"] = (
        watchdog.split(marker)[0] + 'exec python3 "$W/lease.py" watchdog\n'
    )
    service = (
        """# Start the lease receiver after the verified USB IPv4 alias exists.
setsid python3 "$W/lease.py" receiver </dev/null >"$W/lease.receiver.log" 2>&1 &
printf '%s\\n' "$!" > "$W/lease.receiver.launcher.pid"
attempt=0
while [ ! -s "$W/lease.receiver.ready" ] && [ "$attempt" -lt 5 ]; do
  sleep 1
  attempt=$((attempt + 1))
done
[ -s "$W/lease.receiver.ready" ] || exit 93
# Lease-bounded DHCP; renewals never extend application liveness.
[ ! -e "$W/dhcp.pid" ] && [ ! -L "$W/dhcp.pid" ] || exit 84
"""
        + f'setsid python3 "$W/ecm_dhcp.py" "$W" {Q(runtime.host_addr)} </dev/null >"$W/dhcp.log" 2>&1 &\n'
        + """printf '%s\\n' "$!" > "$W/dhcp.launcher.pid"
attempt=0
while [ ! -s "$W/dhcp.ready" ] && [ "$attempt" -lt 5 ]; do
  sleep 1
  attempt=$((attempt + 1))
done
[ -s "$W/dhcp.ready" ] && [ -s "$W/dhcp.pid" ] || exit 85
dhcp_pid=$(cat "$W/dhcp.pid")
case "$dhcp_pid" in ''|*[!0-9]*) exit 86 ;; esac
kill -0 "$dhcp_pid" || exit 87"""
    )
    files["apply.sh"] = _replace_once(apply, "# OPTIONAL_RAM_SERVICE_AFTER_IP", service)
    files["rollback.sh"] = _replace_once(
        rollback,
        'python3 "$W/netdev.py" stop-dhcp || failed=80',
        'python3 "$W/lease.py" close || failed=92\n'
        'python3 "$W/netdev.py" stop-dhcp || failed=80',
    )
    dhcp = (DEVICE / "ecm_dhcp.py").read_text()
    dhcp = _replace_once(dhcp, "    deadline = time.monotonic() + 110\n", "")
    files["ecm_dhcp.py"] = _replace_once(
        dhcp,
        "while alive and time.monotonic() < deadline:",
        "while alive and not (work / 'lease.closed').exists():",
    )
    for name, source in files.items():
        if name.endswith(".py"):
            compile(source, name, "exec")
    bundle = build_bundle(files, work)
    commands = bundle.pop("commands")
    bundle.pop("files")
    plan.pop("upload_commands")
    plan.update(
        commands=commands,
        bundle=bundle,
        sha256={
            name: hashlib.sha256(source.encode()).hexdigest()
            for name, source in files.items()
        },
        byte_lengths={name: len(source.encode()) for name, source in files.items()},
        verify_command="sha256sum " + " ".join(Q(work + "/" + name) for name in files),
        original_netdev_absent=runtime.original_netdev_absent,
        original_flags=runtime.original_flags,
        original_max_speed=runtime.original_max_speed,
        trial_max_speed="high-speed",
        watchdog_seconds=None,
        watchdog_mode="renewable-application-lease",
        lease=dict(
            config,
            request_format="!8s16sQB",
            request_bytes=33,
            reply_format="!8s16sQddB",
            reply_bytes=49,
            magic="O360LSE1",
            renew_opcode=0,
            close_opcode=1,
            heartbeat_interval_seconds=1,
            sequence="unsigned64, strictly increasing from 1",
            dhcp="No router/DNS; exits on close and owned PID cleanup",
            rollback="Host loss, receiver death, expiry or close restores native five functions and SSP",
        ),
    )
    checked = [
        *commands,
        *(
            plan[key]
            for key in (
                "verify_command",
                "apply_command",
                "rollback_command",
                "status_command",
            )
        ),
    ]
    if any(not item.isascii() or len(item.encode("ascii")) > 500 for item in checked):
        raise ValueError("Generated command exceeds the reviewed 500-byte worker limit")
    return plan
