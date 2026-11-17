"""Read-only Linux IPv4 guard for the camera's temporary USB Ethernet link.

Reads iproute2 JSON and /etc/resolv.conf. Never configures an interface, DHCP,
route, DNS, NetworkManager or privileges. Live USB is validated on the Pi5 setup.
Preview sockets must also bind their source and SO_BINDTODEVICE (the existing
vendor utility does this). If denied by the OS, CAP_NET_RAW may be required;
this module never grants it or silently removes interface binding.
"""

import ipaddress
import json
import math
from pathlib import Path
import re
import subprocess
import time

NETWORK = ipaddress.IPv4Network("192.168.253.0/30")
HOST = "192.168.253.2"
CAMERA = "192.168.253.1"
# A family filter hides link MAC fields and omits interfaces without IPv4.
# Keep the complete link inventory; _parse selects inet addresses explicitly.
ADDR_COMMAND = ["ip", "-j", "address", "show"]
ROUTE_COMMAND = ["ip", "-j", "-4", "route", "show", "table", "all"]


class NetworkGuardError(RuntimeError):
    pass


def capture(camera_ip=None):
    commands = [ADDR_COMMAND, ROUTE_COMMAND]
    if camera_ip is not None:
        camera_ip = str(ipaddress.IPv4Address(camera_ip))
        commands.append(["ip", "-j", "-4", "route", "get", camera_ip])
    snapshot = {"commands": []}
    try:
        for argv in commands:
            result = subprocess.run(argv, capture_output=True, text=True, timeout=5)
            snapshot["commands"].append(
                dict(
                    argv=list(argv),
                    returncode=result.returncode,
                    stdout=result.stdout,
                    stderr=result.stderr,
                )
            )
        snapshot["resolv_conf"] = Path("/etc/resolv.conf").read_text()
    except (OSError, subprocess.SubprocessError) as error:
        raise NetworkGuardError(
            "Cannot read Linux iproute2/DNS state: " + str(error)
        ) from error
    return snapshot


def _json(snapshot, argv, *, optional=False):
    rows = [row for row in snapshot["commands"] if row["argv"] == argv]
    if len(rows) != 1:
        raise ValueError("Missing or duplicate command: " + " ".join(argv))
    if rows[0]["returncode"]:
        if optional:
            return None
        raise ValueError("Read command failed: " + " ".join(argv))
    result = json.loads(rows[0]["stdout"])
    if not isinstance(result, list) or any(not isinstance(row, dict) for row in result):
        raise ValueError("Expected iproute2 JSON object array")
    return result


def _mac(value):
    if not isinstance(value, str) or not re.fullmatch(
        r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", value
    ):
        raise ValueError("Invalid MAC address")
    return value.lower()


def _network(value):
    if value == "default":
        return ipaddress.IPv4Network("0.0.0.0/0")
    return ipaddress.IPv4Network(value, strict=False)


def _parse(snapshot, camera_ip=None):
    interfaces = {}
    for row in _json(snapshot, ADDR_COMMAND):
        name = row["ifname"]
        if not isinstance(name, str) or name in interfaces:
            raise ValueError("Invalid or duplicate interface name")
        addresses = []
        for binding in row["addr_info"]:
            if binding["family"] != "inet":
                continue
            address = ipaddress.IPv4Interface(
                str(binding["local"]) + "/" + str(binding["prefixlen"])
            )
            addresses.append(
                {
                    "address": str(address.ip),
                    "network": str(address.network),
                    "prefix": address.network.prefixlen,
                }
            )
        flags = row["flags"]
        if not isinstance(flags, list) or any(
            not isinstance(flag, str) for flag in flags
        ):
            raise ValueError("Invalid interface flags")
        link_address = row.get("address")
        ethernet = row.get("link_type") in ("ether", "loopback")
        looks_like_mac = isinstance(link_address, str) and re.fullmatch(
            r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", link_address
        )
        mac = _mac(link_address) if ethernet or looks_like_mac else None
        interfaces[name] = {
            "mac": mac,
            "link_address": link_address,
            "ipv4": sorted(addresses, key=lambda v: (v["address"], v["prefix"])),
            "flags": flags,
            "operstate": row.get("operstate"),
        }
    routes = _json(snapshot, ROUTE_COMMAND)
    for row in routes:
        _network(row["dst"])  # Reject ambiguous or malformed IPv4 destinations.
    dns = snapshot["resolv_conf"]
    if not isinstance(dns, str):
        raise ValueError("Missing readable /etc/resolv.conf")
    selected = None
    if camera_ip is not None:
        selected = _json(
            snapshot, ["ip", "-j", "-4", "route", "get", camera_ip], optional=True
        )
    return {
        "interfaces": interfaces,
        "routes": routes,
        "dns": dns,
        "camera_route": selected,
    }


def _defaults(state):
    # Lifetime/cache counters change without a routing change. Keep gateway,
    # table, metrics, source, flags, multipath and every other semantic field.
    volatile = {"expires", "cache", "used", "lastuse", "age", "uid"}
    return sorted(
        json.dumps({k: v for k, v in row.items() if k not in volatile}, sort_keys=True)
        for row in state["routes"]
        if _network(row["dst"]).prefixlen == 0
    )


def _plain(row):
    return not any(
        key in row for key in ("gateway", "via", "nexthops", "nhid", "encap", "newdst")
    ) and not row.get("flags")


def _table(row):
    return {"main": 254, "local": 255, "default": 253}.get(
        row.get("table"), row.get("table", 254)
    )


def _own_route(row, interface):
    if (
        row.get("dev") != interface
        or not _plain(row)
        or row.get("protocol") not in ("kernel", 2)
    ):
        return False
    if row.get("prefsrc", HOST) != HOST or row.get("src", HOST) != HOST:
        return False
    destination = _network(row["dst"])
    kind = row.get("type", "unicast")
    if kind == "unicast":
        return (
            destination == NETWORK
            and _table(row) == 254
            and row.get("scope") in ("link", 253)
        )
    if kind == "local":
        return (
            destination == ipaddress.IPv4Network(HOST + "/32")
            and _table(row) == 255
            and row.get("scope") in ("host", 254)
        )
    if kind == "broadcast":
        return (
            destination.prefixlen == 32
            and destination.network_address
            in (NETWORK.network_address, NETWORK.broadcast_address)
            and _table(row) == 255
            and row.get("scope") in ("link", 253)
        )
    return False


def _overlaps(state, interface=None):
    found = []
    for name, row in state["interfaces"].items():
        for binding in row["ipv4"]:
            if _network(binding["network"]).overlaps(NETWORK):
                if name != interface or binding != {
                    "address": HOST,
                    "network": str(NETWORK),
                    "prefix": 30,
                }:
                    found.append("IPv4 overlap on " + name + ": " + binding["network"])
    for row in state["routes"]:
        network = _network(row["dst"])
        if (
            network.prefixlen
            and network.overlaps(NETWORK)
            and not (interface and _own_route(row, interface))
        ):
            found.append("Route overlap: " + json.dumps(row, sort_keys=True))
    return found


def evaluate(baseline, *, after=None, host_mac=None, cleanup=False):
    """Pure check of captured Linux state; no shell, socket or filesystem calls."""
    report = {
        "ok": False,
        "errors": [],
        "fatal_errors": [],
        "platform": "linux",
        "hardware_tested": False,
        "actual_udp_source_measured": False,
    }
    errors = report["errors"]
    fatal = report["fatal_errors"]
    try:
        before = _parse(baseline)
        base_overlap = _overlaps(before)
        if base_overlap:
            fatal.extend(base_overlap)
        report["baseline_default_routes"] = [
            json.loads(value) for value in _defaults(before)
        ]
        if after is None:
            errors.extend(fatal)
            report["ok"] = not errors
            return report
        state = _parse(after, None if cleanup else CAMERA)
        if _defaults(before) != _defaults(state):
            fatal.append("Default route configuration changed")
        if before["dns"] != state["dns"]:
            fatal.append("DNS resolver configuration changed")
        for name, row in before["interfaces"].items():
            current = state["interfaces"].get(name)
            if (
                current is None
                or current["link_address"] != row["link_address"]
                or current["ipv4"] != row["ipv4"]
            ):
                fatal.append("Baseline interface IPv4/MAC changed: " + name)
        interface = None
        if not cleanup:
            mac = _mac(host_mac)
            if int(mac[:2], 16) & 1 or mac == "00:00:00:00:00:00":
                raise ValueError("Expected a nonzero unicast USB host MAC")
            if any(row["mac"] == mac for row in before["interfaces"].values()):
                fatal.append("USB host MAC already existed in baseline")
            names = [
                name for name, row in state["interfaces"].items() if row["mac"] == mac
            ]
            if len(names) != 1:
                errors.append(
                    "USB host MAC must identify exactly one new Linux interface"
                )
            else:
                interface = names[0]
                report["ncm_interface"] = interface
                if interface in before["interfaces"] or not re.fullmatch(
                    r"[A-Za-z0-9_.:-]{1,15}", interface
                ):
                    fatal.append(
                        "USB interface is not a safe newly enumerated interface"
                    )
                row = state["interfaces"][interface]
                expected = [{"address": HOST, "network": str(NETWORK), "prefix": 30}]
                if row["ipv4"] != expected:
                    errors.append(
                        "USB interface needs DHCP "
                        + HOST
                        + "/30 with no additional IPv4 address"
                    )
                if "UP" not in row["flags"] or row["operstate"] not in (
                    "UP",
                    "UNKNOWN",
                ):
                    errors.append("USB Ethernet interface is not UP")
                selected = state["camera_route"]
                report["selected_camera_route"] = selected
                if not isinstance(selected, list) or len(selected) != 1:
                    errors.append("Camera route lookup is not ready")
                else:
                    route = selected[0]
                    source = route.get("prefsrc", route.get("src"))
                    if (
                        route.get("dev") != interface
                        or source != HOST
                        or route.get("type", "unicast") != "unicast"
                        or not _plain(route)
                        or _network(route["dst"])
                        != ipaddress.IPv4Network(CAMERA + "/32")
                    ):
                        message = "Camera route must be direct on the new USB interface with its exact source"
                        (fatal if row["ipv4"] == expected else errors).append(message)
        fatal.extend(_overlaps(state, interface))
        errors.extend(fatal)
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        errors.append("Invalid Linux network snapshot: " + str(error))
        fatal.append(errors[-1])
    report["ok"] = not errors
    return report


def preflight():
    baseline = capture()
    guard = evaluate(baseline)
    if not guard["ok"]:
        raise NetworkGuardError("; ".join(guard["errors"]))
    return baseline


def wait_ready(baseline, host_mac, *, timeout=45):
    if (
        not isinstance(timeout, (int, float))
        or not math.isfinite(timeout)
        or timeout < 0
    ):
        raise ValueError("timeout must be a finite nonnegative number")
    deadline = time.monotonic() + timeout
    while True:
        report = evaluate(baseline, after=capture(CAMERA), host_mac=host_mac)
        if report["ok"]:
            return report
        if report["fatal_errors"]:
            raise NetworkGuardError("; ".join(report["errors"]))
        if time.monotonic() >= deadline:
            raise NetworkGuardError(
                "USB DHCP/route did not become ready. Enable DHCP only on the new USB Ethernet interface; "
                "the camera supplies its /30 without router/DNS options. "
                + "; ".join(report["errors"])
            )
        time.sleep(0.25)


def check_restored(baseline):
    report = evaluate(baseline, after=capture(), cleanup=True)
    if not report["ok"]:
        raise NetworkGuardError("; ".join(report["errors"]))
    return report
