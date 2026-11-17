"""Platform host checks and fresh per-preview network ownership."""

import socket
import subprocess
import sys
import time

from ._mac_network import evaluate


class NetworkGuardError(RuntimeError):
    pass


def preview_guard():
    """Return a fresh Linux session owner, or the passive platform guard.

    This only constructs an object. The Linux owner can configure exactly its
    new unmanaged USB link later; these module-level checks remain read-only.
    """
    if sys.platform == "linux":
        from ._linux_usb_network import LinuxUSBNetworkGuard

        return LinuxUSBNetworkGuard()
    return sys.modules[__name__]


def capture(camera_ip=None):
    if sys.platform == "linux":
        from ._linux_network import capture as linux_capture

        return linux_capture(camera_ip)
    if sys.platform != "darwin":
        raise NetworkGuardError("USB preview supports Linux and macOS hosts")
    commands = [
        ["ifconfig", "-a"],
        ["netstat", "-rn", "-f", "inet"],
        ["route", "-n", "get", "default"],
        ["scutil", "--dns"],
    ]
    if camera_ip:
        commands.append(["route", "-n", "get", camera_ip])
    snapshot = {"commands": []}
    for argv in commands:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=5)
        snapshot["commands"].append(
            dict(
                argv=argv,
                returncode=result.returncode,
                stdout=result.stdout,
                stderr=result.stderr,
            )
        )
    return snapshot


def preflight():
    if sys.platform == "linux":
        from ._linux_network import preflight as linux_preflight

        return linux_preflight()
    baseline = capture()
    guard = evaluate(baseline)
    if not guard["ok"]:
        raise NetworkGuardError("; ".join(guard["errors"]))
    return baseline


def wait_ready(baseline, host_mac, *, timeout=45):
    if sys.platform == "linux":
        from ._linux_network import wait_ready as linux_wait_ready

        return linux_wait_ready(baseline, host_mac, timeout=timeout)
    deadline = time.monotonic() + timeout
    while True:
        after = capture("192.168.253.1")
        guard = evaluate(
            baseline,
            after=after,
            host_mac=host_mac,
            host_ip="192.168.253.2",
            camera_ip="192.168.253.1",
        )
        if guard["ok"]:
            # This source check precedes all preview pairing/registration.
            with socket.create_connection(
                ("192.168.253.1", 7001), timeout=1, source_address=("192.168.253.2", 0)
            ) as link:
                if link.getsockname()[0] != "192.168.253.2":
                    raise NetworkGuardError(
                        "Camera TCP connection did not use the USB source address"
                    )
            return guard
        if any(
            e
            in (
                "Default route configuration changed",
                "DNS resolver configuration changed",
            )
            for e in guard["errors"]
        ):
            raise NetworkGuardError("; ".join(guard["errors"]))
        if time.monotonic() >= deadline:
            raise NetworkGuardError(
                "USB DHCP/route did not become ready: " + "; ".join(guard["errors"])
            )
        time.sleep(0.25)


def check_restored(baseline):
    if sys.platform == "linux":
        from ._linux_network import check_restored as linux_check_restored

        return linux_check_restored(baseline)
    guard = evaluate(baseline, after=capture(), cleanup=True)
    if not guard["ok"]:
        raise NetworkGuardError("; ".join(guard["errors"]))
    return guard
