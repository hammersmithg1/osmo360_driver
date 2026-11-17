#!/usr/bin/env python3
"""Pure parser and guard for macOS USB network snapshots."""

import ipaddress
from pathlib import Path
import re


class GuardError(ValueError):
    pass


def mac_address(value):
    if not isinstance(value, str) or not re.fullmatch(
        r"(?:[0-9a-fA-F]{1,2}:){5}[0-9a-fA-F]{1,2}", value
    ):
        raise GuardError("Invalid MAC address")
    return ":".join(f"{int(x, 16):02x}" for x in value.split(":"))


def mask_prefix(value):
    try:
        if value.lower().startswith("0x"):
            if not re.fullmatch(r"0x[0-9a-fA-F]{1,8}", value):
                raise ValueError()
            mask = int(value, 16)
        else:
            mask = int(ipaddress.IPv4Address(value))
        inverted = mask ^ 0xFFFFFFFF
        if inverted & (inverted + 1):
            raise ValueError()  # This is a netmask field, never an IPv4 hostmask.
        return mask.bit_count()
    except (ValueError, ipaddress.AddressValueError, ipaddress.NetmaskValueError):
        raise GuardError("Invalid or noncontiguous IPv4 mask: " + str(value))


def parse_ifconfig(text):
    interfaces, current = {}, None
    for line in text.splitlines():
        match = re.match(
            r"^([A-Za-z0-9_.:-]+):\s+flags=([0-9a-fA-F]+)(?:<([^>]*)>)?", line
        )
        if match:
            current = match[1]
            if current in interfaces:
                raise GuardError("Duplicate ifconfig interface header: " + current)
            interfaces[current] = {
                "mac": None,
                "ipv4": [],
                "flags": (match[3] or "").split(","),
            }
            continue
        words = line.split()
        if not words or words[0] not in ("inet", "ether", "lladdr"):
            continue
        if current is None:
            raise GuardError("ifconfig address without an interface")
        item = interfaces[current]
        if words[0] in ("ether", "lladdr"):
            if len(words) != 2 or item["mac"] is not None:
                raise GuardError("Malformed or duplicate interface MAC")
            item["mac"] = mac_address(words[1])
        else:
            try:
                address = ipaddress.IPv4Address(words[1])
                mask_index = words.index("netmask")
                prefix = mask_prefix(words[mask_index + 1])
            except (ValueError, IndexError):
                raise GuardError("Malformed IPv4 ifconfig row: " + line)
            item["ipv4"].append(
                {
                    "address": str(address),
                    "prefix": prefix,
                    "network": str(
                        ipaddress.IPv4Network((address, prefix), strict=False)
                    ),
                }
            )
    if not interfaces:
        raise GuardError("Empty or unrecognized ifconfig output")
    return interfaces


def bsd_destination(value, flags=""):
    """Darwin netstat abbreviates network zero octets, e.g. 100.64/10 or 127."""
    if value == "default":
        return ipaddress.IPv4Network("0.0.0.0/0")
    token, slash, prefix_text = value.partition("/")
    octets = token.split(".")
    if not 1 <= len(octets) <= 4 or any(
        not re.fullmatch(r"[0-9]{1,3}", x) or int(x) > 255 for x in octets
    ):
        raise GuardError("Unrecognized BSD IPv4 destination: " + value)
    if slash:
        if not prefix_text.isdigit() or not 0 <= int(prefix_text) <= 32:
            raise GuardError("Invalid BSD route prefix: " + value)
        prefix = int(prefix_text)
    else:
        if len(octets) < 4 and "H" in flags:
            raise GuardError("Ambiguous abbreviated host route: " + value)
        if len(octets) == 4 and "H" not in flags:
            raise GuardError("IPv4 network route lacks an explicit prefix: " + value)
        prefix = 8 * len(octets)
    address = ".".join(str(int(x)) for x in octets + ["0"] * (4 - len(octets)))
    network = ipaddress.IPv4Network(address + "/" + str(prefix), strict=False)
    # An abbreviated CIDR must name its network rather than hide nonzero host bits.
    if slash and ipaddress.IPv4Address(address) != network.network_address:
        raise GuardError("Noncanonical BSD network destination: " + value)
    return network


def parse_netstat(text):
    header, routes, active = None, [], False
    for line in text.splitlines():
        if line.strip() == "Internet:":
            active = True
            continue
        if line.strip().startswith("Internet6:"):
            active = False
            continue
        words = line.split()
        if not active or not words:
            continue
        if words[0] == "Destination":
            if header is not None or not all(
                x in words for x in ("Gateway", "Flags", "Netif")
            ):
                raise GuardError("Malformed or duplicate IPv4 routing header")
            header = {
                x: words.index(x) for x in ("Destination", "Gateway", "Flags", "Netif")
            }
            continue
        if header is None or len(words) <= max(header.values()):
            raise GuardError("Truncated IPv4 route row: " + line)
        destination, gateway, flags, interface = [
            words[header[k]] for k in ("Destination", "Gateway", "Flags", "Netif")
        ]
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface) or not re.fullmatch(
            r"[A-Za-z0-9]+", flags
        ):
            raise GuardError("Unrecognized route flags/interface: " + line)
        network = bsd_destination(destination, flags)
        routes.append(
            {
                "destination": destination,
                "network": str(network),
                "gateway": gateway,
                "flags": flags,
                "interface": interface,
                "default": network.prefixlen == 0,
            }
        )
    if header is None or not routes:
        raise GuardError("Missing IPv4 routing table")
    return routes


def parse_route_get(text):
    fields = {}
    for line in text.splitlines():
        match = re.match(
            r"^\s*(route to|destination|mask|gateway|interface|flags|source|src)\s*:\s*(.*?)\s*$",
            line,
        )
        if match:
            key = match[1].replace(" ", "_")
            if key in fields:
                raise GuardError("Duplicate route-get field: " + key)
            fields[key] = match[2]
    if any(
        not fields.get(k) for k in ("route_to", "interface", "destination", "flags")
    ):
        raise GuardError("Truncated route-get output")
    route_flags(fields["flags"])
    return fields


def route_flags(value):
    known = set(
        "UP GATEWAY HOST REJECT DYNAMIC MODIFIED DONE DELCLONE CLONING XRESOLVE LLINFO STATIC BLACKHOLE WASCLONED PROTO1 PROTO2 PROTO3 PINNED LOCAL BROADCAST MULTICAST IFSCOPE CONDEMNED IFREF PROXY ROUTER DEAD GLOBAL IFDISABLED SRCADDR IFADDR COMPANION".split()
    )
    if not re.fullmatch(r"<[A-Z0-9_,]+>", value):
        raise GuardError("Malformed route-get flags")
    flags = set(value[1:-1].split(","))
    if flags - known:
        raise GuardError(
            "Unknown route-get semantic flags: " + ",".join(sorted(flags - known))
        )
    return flags


def link_gateway(value):
    if re.fullmatch(r"link#[0-9]+", value):
        return True
    try:
        mac_address(value)
        return True
    except GuardError:
        return False


def parse_dns(text):
    if text.strip() == "No DNS configuration available":
        return []
    section, current, resolvers = None, None, []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("DNS configuration"):
            section = line
            current = None
            continue
        if re.fullmatch(r"resolver #[0-9]+", line):
            if section is None:
                raise GuardError("DNS resolver outside a section")
            current = {"section": section, "fields": []}
            resolvers.append(current)
            continue
        match = re.fullmatch(r"([^:]+?)\s*:\s*(.*)", line)
        if current is None or not match:
            raise GuardError("Unrecognized DNS output row: " + line)
        key, value = match[1].strip(), " ".join(match[2].split())
        if key == "reach":
            continue  # Reachability is volatile, not resolver configuration.
        if not re.fullmatch(
            r"(?:search domain|nameserver)\[[0-9]+\]|domain|options|timeout|flags|order|if_index",
            key,
        ):
            raise GuardError("Unknown DNS semantic field: " + key)
        if any(k == key for k, _ in current["fields"]):
            raise GuardError("Duplicate DNS semantic field: " + key)
        current["fields"].append((key, value))
    if not resolvers:
        raise GuardError("Missing DNS configuration")
    if any(not r["fields"] for r in resolvers):
        raise GuardError("Empty/truncated DNS resolver")
    return resolvers


def command_output(snapshot, name, predicate=lambda args: True):
    matches = [
        x
        for x in snapshot.get("commands", [])
        if isinstance(x, dict)
        and isinstance(x.get("argv"), list)
        and x["argv"]
        and Path(x["argv"][0]).name == name
        and predicate(x["argv"])
    ]
    if len(matches) != 1:
        raise GuardError("Missing/duplicate captured command: " + name)
    entry = matches[0]
    if (
        entry.get("returncode") != 0
        or not isinstance(entry.get("stdout"), str)
        or not entry["stdout"].strip()
    ):
        raise GuardError("Failed/empty captured command: " + name)
    return entry["stdout"]


def parse_snapshot(snapshot):
    return {
        "interfaces": parse_ifconfig(
            command_output(snapshot, "ifconfig", lambda a: "-a" in a)
        ),
        "routes": parse_netstat(
            command_output(snapshot, "netstat", lambda a: "inet" in a)
        ),
        "default": parse_route_get(
            command_output(
                snapshot, "route", lambda a: "get" in a and a[-1] == "default"
            )
        ),
        "dns": parse_dns(command_output(snapshot, "scutil", lambda a: "--dns" in a)),
    }


def overlaps(state, candidate, allowed_interface=None, host_ip=None, camera_ip=None):
    found = []
    for interface, item in state["interfaces"].items():
        for binding in item["ipv4"]:
            if ipaddress.IPv4Network(binding["network"]).overlaps(candidate):
                if (
                    interface == allowed_interface
                    and binding["address"] == host_ip
                    and binding["network"] == str(candidate)
                ):
                    continue
                found.append({"kind": "address", "interface": interface, **binding})
    for route in state["routes"]:
        network = ipaddress.IPv4Network(route["network"])
        if route["default"] or not network.overlaps(candidate):
            continue
        if allowed_interface and network.subnet_of(candidate):
            direct_flags = "U" in route["flags"] and not set("GRB") & set(
                route["flags"]
            )
            connected = network == candidate and re.fullmatch(
                r"link#[0-9]+", route["gateway"]
            )
            neighbor = (
                network.prefixlen == 32
                and str(network.network_address) == camera_ip
                and "H" in route["flags"]
                and link_gateway(route["gateway"])
            )
            own_link = (
                network.prefixlen == 32
                and str(network.network_address) == host_ip
                and re.fullmatch(r"link#[0-9]+", route["gateway"])
                and set(route["flags"]).issubset(set("UCS"))
            )
            own_local = (
                route["interface"] == "lo0"
                and str(network) == host_ip + "/32"
                and "H" in route["flags"]
                and (route["gateway"] == host_ip or link_gateway(route["gateway"]))
            )
            if direct_flags and (
                (
                    route["interface"] == allowed_interface
                    and (connected or neighbor or own_link)
                )
                or own_local
            ):
                continue
        found.append({"kind": "route", **route})
    return found


def default_rows(state):
    return sorted(
        (r["network"], r["gateway"], r["flags"], r["interface"])
        for r in state["routes"]
        if r["default"]
    )


def evaluate(
    baseline,
    candidate="192.168.253.0/30",
    after=None,
    host_mac=None,
    host_ip=None,
    camera_ip=None,
    cleanup=False,
):
    report = {"ok": False, "scope": "captured_mac_snapshots_only", "errors": []}
    try:
        network = ipaddress.IPv4Network(candidate, strict=False)
        if network.prefixlen != 30:
            raise GuardError("Candidate must be an IPv4 /30")
        before = parse_snapshot(baseline)
        report.update(
            candidate_network=str(network),
            baseline_interface_count=len(before["interfaces"]),
            baseline_route_count=len(before["routes"]),
            baseline_default_routes=default_rows(before),
            selected_default=before["default"],
            baseline_overlaps=overlaps(before, network),
        )
        if report["baseline_overlaps"]:
            report["errors"].append("Candidate overlaps baseline addresses/routes")
        if cleanup and after is None:
            raise GuardError("Cleanup comparison requires an after snapshot")
        if after is not None:
            state = parse_snapshot(after)
            report["default_route_unchanged"] = before["default"] == state[
                "default"
            ] and default_rows(before) == default_rows(state)
            report["dns_unchanged"] = before["dns"] == state["dns"]
            if not report["default_route_unchanged"]:
                report["errors"].append("Default route configuration changed")
            if not report["dns_unchanged"]:
                report["errors"].append("DNS resolver configuration changed")
            if cleanup:
                report["stage"] = "cleanup"
                report["after_overlaps"] = overlaps(state, network)
                if report["after_overlaps"]:
                    report["errors"].append(
                        "Candidate address/route remains after cleanup"
                    )
                report["ok"] = not report["errors"]
                return report
            expected_mac = mac_address(host_mac)
            if (
                expected_mac in ("00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff")
                or int(expected_mac[:2], 16) & 1
            ):
                raise GuardError("Expected NCM host MAC must be nonzero unicast")
            matches = [
                name
                for name, item in state["interfaces"].items()
                if item["mac"] == expected_mac
            ]
            if len(matches) != 1:
                raise GuardError("Expected NCM MAC must match exactly one interface")
            interface = matches[0]
            if interface in before["interfaces"]:
                raise GuardError("Expected NCM MAC belongs to a baseline interface")
            report["ncm_interface"] = interface
            report["ncm_flags"] = state["interfaces"][interface]["flags"]
            allowed_interface, allowed_ip, allowed_camera = None, None, None
            if host_ip is not None:
                host = ipaddress.IPv4Address(host_ip)
                camera = ipaddress.IPv4Address(camera_ip or network.network_address + 1)
                if (
                    host not in network.hosts()
                    or camera not in network.hosts()
                    or host == camera
                ):
                    raise GuardError(
                        "Camera/host addresses must be distinct usable /30 hosts"
                    )
                allowed_interface, allowed_ip = interface, str(host)
                allowed_camera = str(camera)
                if not {"UP", "RUNNING"}.issubset(
                    state["interfaces"][interface]["flags"]
                ):
                    report["errors"].append(
                        "Configured NCM interface is not UP/RUNNING"
                    )
                if not any(
                    b["address"] == str(host) and b["network"] == str(network)
                    for b in state["interfaces"][interface]["ipv4"]
                ):
                    report["errors"].append(
                        "Expected host /30 alias is absent from NCM interface"
                    )
                selected = parse_route_get(
                    command_output(
                        after, "route", lambda a: "get" in a and a[-1] == str(camera)
                    )
                )
                report["route_to_camera"] = selected
                selected_flags = route_flags(selected["flags"])
                if selected["route_to"] != str(camera):
                    report["errors"].append(
                        "Camera route-get describes a different target"
                    )
                if "UP" not in selected_flags or selected_flags & {
                    "GATEWAY",
                    "REJECT",
                    "BLACKHOLE",
                    "DEAD",
                    "IFDISABLED",
                    "CONDEMNED",
                }:
                    report["errors"].append(
                        "Camera route is not a direct usable USB route"
                    )
                if selected.get("gateway") and not link_gateway(selected["gateway"]):
                    report["errors"].append("Camera route uses an IP gateway")
                if selected["interface"] != interface:
                    report["errors"].append(
                        "Camera route does not select NCM interface"
                    )
                if selected["destination"] == "default":
                    report["errors"].append(
                        "Camera route resolves through a default route"
                    )
                else:
                    if (
                        selected.get("mask") in (None, "default")
                        and "HOST" not in selected_flags
                    ):
                        raise GuardError(
                            "Camera route lacks a netmask without HOST flag"
                        )
                    prefix = (
                        mask_prefix(selected["mask"])
                        if selected.get("mask") not in (None, "default")
                        else 32
                    )
                    if "HOST" in selected_flags and prefix != 32:
                        report["errors"].append(
                            "HOST camera route has a non-host netmask"
                        )
                    selected_net = ipaddress.IPv4Network(
                        (ipaddress.IPv4Address(selected["destination"]), prefix),
                        strict=False,
                    )
                    if camera not in selected_net or not selected_net.subnet_of(
                        network
                    ):
                        report["errors"].append(
                            "Camera route prefix is outside selected /30"
                        )
                source = selected.get("source", selected.get("src"))
                if source is not None and source != str(host):
                    report["errors"].append("Camera route reports wrong source address")
                report["actual_udp_source_measured"] = False
            report["after_overlaps"] = overlaps(
                state, network, allowed_interface, allowed_ip, allowed_camera
            )
            if report["after_overlaps"]:
                report["errors"].append(
                    "Candidate overlaps another after-snapshot address/route"
                )
        report["ok"] = not report["errors"]
    except (GuardError, ValueError, TypeError, KeyError) as error:
        report["errors"].append(str(error))
    return report
