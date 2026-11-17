"""Linux guard fixtures only: never read or change the host network."""

import copy
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

from osmo360_camera import _linux_network as network

MAC = "96:03:ae:cc:f8:37"
IFACE = "enx9603aeccf837"


def interface(name, mac, bindings=(), flags=("UP", "LOWER_UP"), state="UP"):
    return {
        "ifname": name,
        "address": mac,
        "flags": list(flags),
        "operstate": state,
        "addr_info": [
            {"family": "inet", "local": address, "prefixlen": prefix}
            for address, prefix in bindings
        ],
    }


def snapshot(*, ready=False, enumerated=False):
    addresses = [
        interface("lo", "00:00:00:00:00:00", [("127.0.0.1", 8)], state="UNKNOWN"),
        interface("wlan0", "02:12:34:56:78:9a", [("192.168.1.87", 24)]),
        interface("tailscale0", None, [("100.72.200.71", 32)], state="UNKNOWN"),
    ]
    routes = [
        {
            "dst": "default",
            "gateway": "192.168.1.1",
            "dev": "wlan0",
            "protocol": "dhcp",
            "metric": 600,
            "flags": [],
        },
        {
            "dst": "192.168.1.0/24",
            "dev": "wlan0",
            "protocol": "kernel",
            "scope": "link",
            "prefsrc": "192.168.1.87",
            "flags": [],
        },
        {
            "dst": "127.0.0.0/8",
            "dev": "lo",
            "table": "local",
            "protocol": "kernel",
            "scope": "host",
            "type": "local",
            "flags": [],
        },
        {
            "dst": "100.64.0.0/10",
            "dev": "tailscale0",
            "table": 52,
            "scope": "link",
            "flags": [],
        },
        {
            "dst": "default",
            "dev": "tailscale0",
            "table": 100,
            "metric": 20,
            "flags": [],
        },
    ]
    if enumerated or ready:
        addresses.append(
            interface(IFACE, MAC, [("192.168.253.2", 30)] if ready else [])
        )
    if ready:
        routes.extend(
            [
                {
                    "dst": "192.168.253.0/30",
                    "dev": IFACE,
                    "protocol": "kernel",
                    "scope": "link",
                    "prefsrc": network.HOST,
                    "flags": [],
                },
                {
                    "type": "local",
                    "dst": network.HOST,
                    "dev": IFACE,
                    "table": "local",
                    "protocol": "kernel",
                    "scope": "host",
                    "prefsrc": network.HOST,
                    "flags": [],
                },
                {
                    "type": "broadcast",
                    "dst": "192.168.253.0",
                    "dev": IFACE,
                    "table": "local",
                    "protocol": "kernel",
                    "scope": "link",
                    "prefsrc": network.HOST,
                    "flags": [],
                },
                {
                    "type": "broadcast",
                    "dst": "192.168.253.3",
                    "dev": IFACE,
                    "table": "local",
                    "protocol": "kernel",
                    "scope": "link",
                    "prefsrc": network.HOST,
                    "flags": [],
                },
            ]
        )
    result = {
        "resolv_conf": "# systemd-resolved stub\nnameserver 127.0.0.53\noptions edns0 trust-ad\n",
        "commands": [],
    }
    for argv, value in [
        (network.ADDR_COMMAND, addresses),
        (network.ROUTE_COMMAND, routes),
    ]:
        result["commands"].append(
            {
                "argv": argv[:],
                "returncode": 0,
                "stdout": json.dumps(value),
                "stderr": "",
            }
        )
    if enumerated or ready:
        selected = (
            {
                "dst": network.CAMERA,
                "dev": IFACE,
                "prefsrc": network.HOST,
                "uid": 1000,
                "cache": [],
                "flags": [],
            }
            if ready
            else {
                "dst": network.CAMERA,
                "dev": "wlan0",
                "gateway": "192.168.1.1",
                "prefsrc": "192.168.1.87",
                "flags": [],
            }
        )
        result["commands"].append(
            {
                "argv": ["ip", "-j", "-4", "route", "get", network.CAMERA],
                "returncode": 0,
                "stdout": json.dumps([selected]),
                "stderr": "",
            }
        )
    return result


def rows(snapshot, index):
    return json.loads(snapshot["commands"][index]["stdout"])


def replace_rows(snapshot, index, value):
    snapshot["commands"][index]["stdout"] = json.dumps(value)


class LinuxGuardTests(unittest.TestCase):
    def test_actual_pi_all_family_capture_keeps_mac_empty_nic_and_filters_ipv6(self):
        fixture = json.loads(
            (
                Path(__file__).with_name("fixtures") / "linux_pi_ip_address.json"
            ).read_text()
        )
        stage = ["baseline"]
        routes = [
            {
                "dst": "default",
                "gateway": "192.0.2.1",
                "dev": "wlan0",
                "protocol": "dhcp",
                "metric": 600,
                "flags": [],
            },
            {
                "dst": "192.0.2.0/24",
                "dev": "wlan0",
                "protocol": "kernel",
                "scope": "link",
                "prefsrc": "192.0.2.133",
                "flags": [],
            },
        ]

        def run(argv, **options):
            if argv in (fixture["all_families"]["argv"], fixture["ipv4_only"]["argv"]):
                all_families = argv == fixture["all_families"]["argv"]
                addresses = copy.deepcopy(
                    fixture["all_families" if all_families else "ipv4_only"]["rows"]
                )
                if stage[0] != "baseline":
                    usb = {
                        "ifindex": 7,
                        "ifname": IFACE,
                        "flags": ["BROADCAST", "MULTICAST", "UP", "LOWER_UP"],
                        "mtu": 1500,
                        "operstate": "UP",
                        "link_type": "ether",
                        "address": MAC,
                        "broadcast": "ff:ff:ff:ff:ff:ff",
                        "addr_info": [
                            {
                                "family": "inet6",
                                "local": "fe80::9403:aeff:fecc:f837",
                                "prefixlen": 64,
                                "scope": "link",
                            }
                        ],
                    }
                    if stage[0] == "ready":
                        usb["addr_info"].append(
                            {
                                "family": "inet",
                                "local": network.HOST,
                                "prefixlen": 30,
                                "broadcast": "192.168.253.3",
                                "scope": "global",
                            }
                        )
                    if all_families:
                        addresses.append(usb)
                    elif stage[0] == "ready":
                        # Actual -4 formatting also drops MAC/link_type on an addressed NIC.
                        usb.pop("address")
                        usb.pop("link_type")
                        usb.pop("broadcast")
                        usb["addr_info"] = [
                            item
                            for item in usb["addr_info"]
                            if item["family"] == "inet"
                        ]
                        addresses.append(usb)
                value = addresses
            elif argv == network.ROUTE_COMMAND:
                value = copy.deepcopy(routes)
                if stage[0] == "ready":
                    value += rows(snapshot(ready=True), 1)[-4:]
            elif argv == ["ip", "-j", "-4", "route", "get", network.CAMERA]:
                value = [
                    {
                        "dst": network.CAMERA,
                        "dev": IFACE,
                        "prefsrc": network.HOST,
                        "uid": 1000,
                        "cache": [],
                        "flags": [],
                    }
                ]
                if stage[0] != "ready":
                    value = [
                        {
                            "dst": network.CAMERA,
                            "dev": "wlan0",
                            "gateway": "192.0.2.1",
                            "prefsrc": "192.0.2.133",
                            "uid": 1000,
                            "cache": [],
                            "flags": [],
                        }
                    ]
            else:
                raise AssertionError("Unexpected read command: " + repr(argv))
            return subprocess.CompletedProcess(argv, 0, json.dumps(value), "")

        with (
            patch.object(network.subprocess, "run", side_effect=run),
            patch.object(
                network.Path, "read_text", return_value="nameserver 127.0.0.53\n"
            ),
        ):
            baseline = network.capture()
            state = network._parse(baseline)
            self.assertEqual(state["interfaces"]["eth0"]["mac"], "02:11:22:33:44:50")
            self.assertEqual(state["interfaces"]["eth0"]["ipv4"], [])
            self.assertEqual(len(state["interfaces"]["wlan0"]["ipv4"]), 1)
            self.assertTrue(network.evaluate(baseline)["ok"])
            stage[0] = "pending"
            pending = network.evaluate(
                baseline, after=network.capture(network.CAMERA), host_mac=MAC
            )
            self.assertEqual(pending["ncm_interface"], IFACE)
            self.assertFalse(pending["ok"])
            self.assertFalse(pending["fatal_errors"])
            stage[0] = "ready"
            ready = network.evaluate(
                baseline, after=network.capture(network.CAMERA), host_mac=MAC
            )
            self.assertTrue(ready["ok"], ready)
            self.assertEqual(ready["ncm_interface"], IFACE)

    def test_baseline_loopback_wifi_vpn_and_ready_kernel_routes(self):
        base = snapshot()
        after = snapshot(ready=True)
        self.assertTrue(network.evaluate(base)["ok"])
        result = network.evaluate(base, after=after, host_mac=MAC)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["ncm_interface"], IFACE)
        self.assertEqual(len(result["baseline_default_routes"]), 2)
        self.assertFalse(result["actual_udp_source_measured"])
        # Kernel/iproute2 may print table/scope identifiers as numbers.
        routes = rows(after, 1)
        for row in routes[-4:]:
            row["table"] = 255 if row.get("table") == "local" else 254
            row["scope"] = 254 if row["scope"] == "host" else 253
            row["protocol"] = 2
        replace_rows(after, 1, routes)
        self.assertTrue(network.evaluate(base, after=after, host_mac=MAC)["ok"])

    def test_non_ethernet_tunnel_addresses_are_not_parsed_as_mac(self):
        base = snapshot()
        after = snapshot(ready=True)
        for value in (base, after):
            addresses = rows(value, 0)
            addresses.append(
                {
                    "ifname": "tunl0",
                    "address": "0.0.0.0",
                    "link_type": "ipip",
                    "flags": [],
                    "operstate": "DOWN",
                    "addr_info": [],
                }
            )
            replace_rows(value, 0, addresses)
        self.assertTrue(network.evaluate(base, after=after, host_mac=MAC)["ok"])

    def test_overlap_on_existing_nic_or_vpn_route_rejected(self):
        for where in ("address", "route"):
            base = snapshot()
            if where == "address":
                value = rows(base, 0)
                value[1]["addr_info"].append(
                    {"family": "inet", "local": "192.168.253.50", "prefixlen": 24}
                )
                replace_rows(base, 0, value)
            else:
                value = rows(base, 1)
                value.append(
                    {"dst": "192.168.0.0/16", "dev": "tailscale0", "table": 52}
                )
                replace_rows(base, 1, value)
            self.assertFalse(network.evaluate(base)["ok"])

    def test_wrong_gateway_interface_source_or_kernel_type_rejected(self):
        for field, value in [
            ("gateway", "192.168.1.1"),
            ("dev", "wlan0"),
            ("prefsrc", "192.168.1.87"),
            ("type", "blackhole"),
        ]:
            after = snapshot(ready=True)
            route = rows(after, 2)
            route[0][field] = value
            replace_rows(after, 2, route)
            self.assertFalse(
                network.evaluate(snapshot(), after=after, host_mac=MAC)["ok"], field
            )
        for change in ("wrongdev", "gateway", "foreignip", "badtype", "nexthops"):
            after = snapshot(ready=True)
            value = rows(after, 1)
            if change == "wrongdev":
                value[-1]["dev"] = "wlan0"
            elif change == "gateway":
                value[-4]["gateway"] = "192.168.1.1"
            elif change == "foreignip":
                value.append(dict(value[-3], dst=network.CAMERA))
            elif change == "badtype":
                value[-4]["type"] = "prohibit"
            else:
                value[-4]["nexthops"] = [{"gateway": "192.168.1.1", "dev": "wlan0"}]
            replace_rows(after, 1, value)
            self.assertFalse(
                network.evaluate(snapshot(), after=after, host_mac=MAC)["ok"], change
            )

    def test_defaults_dns_and_existing_interface_must_stay(self):
        for change in ("default", "dns", "wifi"):
            after = snapshot(ready=True)
            if change == "default":
                value = rows(after, 1)
                value[0]["gateway"] = "192.168.253.1"
                replace_rows(after, 1, value)
            elif change == "dns":
                after["resolv_conf"] += "nameserver 192.168.253.1\n"
            else:
                value = rows(after, 0)
                value[1]["addr_info"][0]["local"] = "192.168.1.88"
                replace_rows(after, 0, value)
            result = network.evaluate(snapshot(), after=after, host_mac=MAC)
            self.assertFalse(result["ok"])
            self.assertTrue(result["fatal_errors"])

    def test_mac_unique_and_new_exact_host_prefix(self):
        for change in ("duplicate", "old", "prefix", "alias"):
            base = snapshot()
            after = snapshot(ready=True)
            value = rows(after, 0)
            if change == "duplicate":
                value.append(interface("usb1", MAC))
            elif change == "old":
                old = rows(base, 0)
                old.append(interface(IFACE, MAC))
                replace_rows(base, 0, old)
            elif change == "prefix":
                value[-1]["addr_info"][0]["prefixlen"] = 24
            else:
                value[-1]["addr_info"].append(
                    {"family": "inet", "local": "169.254.1.2", "prefixlen": 16}
                )
            replace_rows(after, 0, value)
            self.assertFalse(
                network.evaluate(base, after=after, host_mac=MAC)["ok"], change
            )

    def test_pending_dhcp_waits_then_reports_clear_error(self):
        pending = snapshot(enumerated=True)
        result = network.evaluate(snapshot(), after=pending, host_mac=MAC)
        self.assertFalse(result["ok"])
        self.assertFalse(result["fatal_errors"], result)
        with patch.object(network, "capture", return_value=pending):
            with self.assertRaisesRegex(
                network.NetworkGuardError, "Enable DHCP only on the new USB"
            ):
                network.wait_ready(snapshot(), MAC, timeout=0)

    def test_wait_and_cleanup_with_all_io_mocked(self):
        with patch.object(network, "capture", return_value=snapshot(ready=True)):
            self.assertEqual(
                network.wait_ready(snapshot(), MAC, timeout=0)["ncm_interface"], IFACE
            )
        restored = snapshot()
        # A kernel-retained USB link without IPv4 and candidate routes is okay.
        value = rows(restored, 0)
        value.append(interface(IFACE, MAC, flags=(), state="DOWN"))
        replace_rows(restored, 0, value)
        with patch.object(network, "capture", return_value=restored):
            self.assertTrue(network.check_restored(snapshot())["ok"])
        with patch.object(network, "capture", return_value=snapshot(ready=True)):
            with self.assertRaises(network.NetworkGuardError):
                network.check_restored(snapshot())

    def test_capture_only_read_commands_and_mock_dns(self):
        fixture = snapshot(ready=True)
        outputs = [
            subprocess.CompletedProcess(row["argv"], 0, row["stdout"], "")
            for row in fixture["commands"]
        ]
        with (
            patch.object(network.subprocess, "run", side_effect=outputs) as run,
            patch.object(
                network.Path, "read_text", return_value=fixture["resolv_conf"]
            ),
        ):
            actual = network.capture(network.CAMERA)
        self.assertEqual(
            [call.args[0] for call in run.call_args_list],
            [row["argv"] for row in fixture["commands"]],
        )
        self.assertTrue(network.evaluate(snapshot(), after=actual, host_mac=MAC)["ok"])

    def test_malformed_or_failed_snapshot_fails_closed(self):
        for mode in ("missing", "duplicate", "json", "failed"):
            base = snapshot()
            if mode == "missing":
                base.pop("resolv_conf")
            elif mode == "duplicate":
                base["commands"].append(copy.deepcopy(base["commands"][0]))
            elif mode == "json":
                base["commands"][0]["stdout"] = "{}"
            else:
                base["commands"][0]["returncode"] = 2
            self.assertFalse(network.evaluate(base)["ok"])


if __name__ == "__main__":
    unittest.main()
