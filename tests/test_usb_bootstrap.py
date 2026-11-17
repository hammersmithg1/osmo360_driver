"""Offline bootstrap tests: synthetic native file packets, no USB or shell run."""

import ast
from collections import deque
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from osmo360_camera import _usb_bootstrap as b
from osmo360_camera.vendor.duml import Message


def runtime_report(*, retained=False):
    return dict(
        schema=1,
        uid=0,
        ram_realpath="/dev/shm",
        ram_filesystem="tmpfs",
        udc="a600000.dwc3",
        vendor="0x2ca3",
        product="0x0025",
        configuration="BULK",
        links={f"f{i}": target for i, target in enumerate(b.NATIVE_LINKS, 1)},
        max_speed="super-speed-plus",
        dev_addr="aa:e4:f8:1c:10:86",
        host_addr="96:03:ae:cc:f8:37",
        ifname="usb0" if retained else b.PLACEHOLDER,
        mac_matches=[dict(name="usb0", flags="0x1002")] if retained else [],
        ipv4=[dict(interface="wlan0", cidr="192.168.1.2/24")],
        tools={
            name: True
            for name in ("python3", "ip", "setsid", "base64", "sha256sum", "readlink")
        },
    )


class FakeTransport:
    """Stock metadata -> window -> sequential data -> MD5, driven by host ACKs."""

    sender, connected = 0x0A, True

    def __init__(
        self, content=b"example", *, corrupt_md5=False, wrong_name=False, silent=False
    ):
        self.content, self.corrupt_md5 = content, corrupt_md5
        self.wrong_name, self.silent = wrong_name, silent
        self.unrelated = (0, Message(1, self.sender, 3, 0x40, 2, 0x80, b"other"))
        self.stale = (0, Message(0x28, self.sender, 4, 0x40, 0, 0x2A, b"old"))
        self.events = deque([self.unrelated, self.stale], maxlen=256)
        self.requests, self.writes = [], []
        self.stage, self.offset = None, 0

    def append(self, payload):
        self.events.append(
            (
                b.time.monotonic_ns(),
                Message(
                    0x28, self.sender, 100 + len(self.writes), 0x40, 0, 0x2A, payload
                ),
            )
        )

    def request(self, command_set, command, payload, *, receiver, timeout):
        self.requests.append((command_set, command, payload, receiver, timeout))
        assert (command_set, command, receiver) == (0, 0x2A, 0x28)
        assert payload[0] == 8 and payload[1] == len(payload) - 2
        if not self.silent:
            name = b"wrong.json" if self.wrong_name else payload[2:].rsplit(b"/", 1)[1]
            self.stage = "metadata"
            self.append(
                b"\x01"
                + struct.pack("<I", len(self.content))
                + bytes([len(name)])
                + name
                + b"\0\1\1"
            )
        return SimpleNamespace(payload=b"\0")

    def next_packet(self):
        if self.offset < len(self.content):
            index = self.offset // 512
            chunk = self.content[self.offset : self.offset + 512]
            self.offset += len(chunk)
            self.stage = "data"
            self.append(b"\x04" + struct.pack("<I", index) + chunk)
        else:
            self.stage = "checksum"
            digest = (
                bytes(16) if self.corrupt_md5 else hashlib.md5(self.content).digest()
            )
            self.append(b"\x03" + digest)

    def _write(self, message):
        assert message.is_reply and (message.sender, message.receiver) == (
            self.sender,
            0x28,
        )
        assert message.command == (0, 0x2A)
        self.writes.append(message)
        if message.payload == b"\xd6":
            return
        if self.stage == "metadata":
            assert message.payload == bytes.fromhex("00000201000100")
            self.next_packet()
        elif self.stage == "data":
            assert message.payload == b"\0" + struct.pack(
                "<I", (self.offset - 1) // 512
            )
            self.next_packet()
        else:
            assert self.stage == "checksum" and message.payload == b"\0"


class FakeSession:
    def __init__(self, transport, *, cleanup_fails=False):
        self.transport = transport
        self.commands = []
        self.cleanup_fails = cleanup_fails

    def run(self, command, *, timeout_ms, pre_send=None):
        if pre_send is not None:
            pre_send()
        self.commands.append((command, timeout_ms))
        if self.cleanup_fails and " cleanup " in command:
            raise RuntimeError("injected cleanup failure")
        return dict(completed=True, success=True, result=0)


class RuntimeCodecTests(unittest.TestCase):
    def test_absent_and_retained_actual_baselines(self):
        absent = b.decode_runtime(runtime_report())
        self.assertEqual(absent.udc, "a600000.dwc3")
        self.assertEqual(absent.host_addr, "96:03:ae:cc:f8:37")
        self.assertTrue(absent.original_netdev_absent)
        self.assertIsNone(absent.net_if)
        self.assertIsNone(absent.original_flags)
        self.assertEqual(absent.original_max_speed, "super-speed-plus")
        self.assertEqual(absent.overlay_values()["original_links"], b.NATIVE_LINKS)
        retained = b.decode_runtime(runtime_report(retained=True))
        self.assertFalse(retained.original_netdev_absent)
        self.assertEqual((retained.net_if, retained.original_flags), ("usb0", "0x1002"))

    def test_actual_configfs_readlink_values_are_preserved(self):
        # Exact readlink stdout from ncm-runtime-camera-2.json; keep this test
        # independent of hardware or external diagnostic captures.
        captured = {
            "f1": "../../../../usb_gadget/g1/functions/ffs.diag",
            "f2": "../../../../usb_gadget/g1/functions/ffs.bulk",
            "f3": "../../../../usb_gadget/g1/functions/ffs.bulk1",
            "f4": "../../../../usb_gadget/g1/functions/ffs.bulk5",
            "f5": "../../../../usb_gadget/g1/functions/ffs.bulk4",
        }
        report = runtime_report()
        report["links"] = captured
        runtime = b.decode_runtime(report)
        expected = tuple(
            "../../../../usb_gadget/g1/functions/" + name for name in b.FUNCTIONS
        )
        self.assertEqual(runtime.original_links, expected)
        self.assertEqual(runtime.overlay_values()["original_links"], expected)

    def test_native_function_order_extra_function_and_udc_rejected(self):
        for mutation in ("swapped", "extra", "udc", "speed"):
            report = runtime_report()
            if mutation == "swapped":
                report["links"]["f1"], report["links"]["f2"] = (
                    report["links"]["f2"],
                    report["links"]["f1"],
                )
            if mutation == "extra":
                report["links"]["f6"] = "../../functions/ecm.ecm"
            if mutation == "udc":
                report["udc"] = "bad;command"
            if mutation == "speed":
                report["max_speed"] = "high-speed"
            with self.subTest(mutation=mutation), self.assertRaises(b.BootstrapError):
                b.decode_runtime(report)

    def test_mac_placeholder_retained_state_and_ipv4_guards(self):
        reports = []
        for mac in ("ff:ff:ff:ff:ff:ff", "00:00:00:00:00:00", "9:03:ae:cc:f8:37"):
            r = runtime_report()
            r["host_addr"] = mac
            reports.append(r)
        r = runtime_report()
        r["host_addr"] = r["dev_addr"]
        reports.append(r)
        r = runtime_report()
        r["ifname"] = "usb0"
        reports.append(r)
        r = runtime_report(retained=True)
        r["mac_matches"][0]["flags"] = "0x1003"
        reports.append(r)
        r = runtime_report(retained=True)
        r["ipv4"].append(dict(interface="usb0", cidr="10.0.0.1/24"))
        reports.append(r)
        r = runtime_report()
        r["ipv4"].append(dict(interface="wlan1", cidr="192.168.253.1/30"))
        reports.append(r)
        r = runtime_report(retained=True)
        r["mac_matches"].append(dict(name="usb1", flags="0x1002"))
        reports.append(r)
        for index, report in enumerate(reports):
            with self.subTest(index=index), self.assertRaises(b.BootstrapError):
                b.decode_runtime(report)

    def test_root_tmpfs_tools_and_schema_are_required(self):
        for key, value in [
            ("schema", True),
            ("uid", 1000),
            ("ram_filesystem", "ext4"),
            ("ram_realpath", "/tmp"),
            ("tools", {}),
        ]:
            report = runtime_report()
            report[key] = value
            with self.subTest(key=key), self.assertRaises(b.BootstrapError):
                b.decode_runtime(report)


class BootstrapTransferTests(unittest.TestCase):
    def adapter(self, transport, **options):
        return b.USBBootstrap(transport, session=FakeSession(transport, **options))

    def test_same_reader_pull_acks_packets_and_retains_unrelated_events(self):
        data = bytes(range(256)) * 5 + b"end"
        transport = FakeTransport(data)
        result = self.adapter(transport).pull_bytes("/blackbox/export_tmp/owned.json")
        self.assertEqual(result, data)
        self.assertEqual(list(transport.events), [transport.unrelated, transport.stale])
        self.assertEqual(len(transport.requests), 1)
        self.assertEqual(len(transport.writes), 5)  # metadata, 3 data, checksum

    def test_bad_md5_wrong_basename_and_size_are_rejected(self):
        for transport, limit in [
            (FakeTransport(b"abc", corrupt_md5=True), 100),
            (FakeTransport(b"abc", wrong_name=True), 100),
            (FakeTransport(b"abc"), 2),
        ]:
            with self.subTest(transport=transport), self.assertRaises(b.BootstrapError):
                self.adapter(transport).pull_bytes(
                    "/blackbox/export_tmp/owned.json", max_bytes=limit
                )
            self.assertEqual(transport.writes[-1].payload, b"\xd6")

    def test_pull_timeout_has_no_second_claim_or_other_command(self):
        transport = FakeTransport(silent=True)
        with self.assertRaisesRegex(b.BootstrapError, "deadline"):
            self.adapter(transport).pull_bytes(
                "/blackbox/export_tmp/owned.json", timeout=0.01
            )
        self.assertEqual(len(transport.requests), 1)
        self.assertEqual(transport.writes, [])

    def test_query_commands_are_bounded_and_cleanup_after_success(self):
        transport = FakeTransport(json.dumps(runtime_report()).encode())
        adapter = self.adapter(transport)
        with patch.object(b.secrets, "token_hex", return_value="012345abcdef"):
            result = adapter.query_runtime()
        self.assertEqual(result.host_addr, "96:03:ae:cc:f8:37")
        commands = [command for command, _ in adapter.session.commands]
        self.assertTrue(
            all(command.isascii() and len(command) <= 500 for command in commands)
        )
        self.assertIn("mkdir /dev/shm/osmo-ncm-query012345abcdef", commands[0])
        self.assertIn("sha256sum", commands[-3])
        self.assertIn(" query ", commands[-2])
        self.assertIn(" cleanup ", commands[-1])
        self.assertTrue(adapter.last_report["cleanup_complete"])
        self.assertEqual(
            transport.requests[0][2][2:].decode(),
            "/blackbox/export_tmp/osmo_probe_012345abcdef.json",
        )
        ast.parse(b.COLLECTOR)

    def test_invalid_report_still_cleans_own_export(self):
        report = runtime_report()
        report["links"]["f6"] = "../../functions/ecm.ecm"
        transport = FakeTransport(json.dumps(report).encode())
        adapter = self.adapter(transport)
        with self.assertRaises(b.BootstrapError):
            adapter.query_runtime()
        self.assertIn(" cleanup ", adapter.session.commands[-1][0])
        self.assertTrue(adapter.last_report["cleanup_complete"])

    def test_cleanup_failure_is_reported_and_not_a_success(self):
        transport = FakeTransport(json.dumps(runtime_report()).encode())
        adapter = self.adapter(transport, cleanup_fails=True)
        with self.assertRaises(b.BootstrapError) as caught:
            adapter.query_runtime()
        self.assertIn("injected cleanup failure", caught.exception.cleanup_error)
        self.assertFalse(adapter.last_report["cleanup_complete"])

    def test_session_must_belong_to_the_same_transport(self):
        with self.assertRaises(ValueError):
            b.USBBootstrap(FakeTransport(), session=FakeSession(FakeTransport()))


class CollectorOwnershipTests(unittest.TestCase):
    """Execute collector Python only against an isolated temporary fake tree."""

    def fixture(self, folder, *, retained=False):
        root = Path(folder).resolve()
        ram, blackbox, net = root / "ram", root / "blackbox", root / "net"
        for directory in (ram, blackbox, net):
            directory.mkdir()
        work = ram / "osmo-ncm-query012345abcdef"
        work.mkdir()
        (work / "query.py").write_text("# isolated fixture\n")
        gadget = root / "g1"
        config = gadget / "configs/c.1"
        (config / "strings/0x409").mkdir(parents=True)
        function = gadget / "functions/ecm.ecm"
        function.mkdir(parents=True)
        for name, value in [
            ("UDC", "a600000.dwc3"),
            ("idVendor", "0x2ca3"),
            ("idProduct", "0x0025"),
            ("max_speed", "super-speed-plus"),
        ]:
            (gadget / name).write_text(value + "\n")
        (config / "strings/0x409/configuration").write_text("BULK\n")
        for index, name in enumerate(b.FUNCTIONS, 1):
            (config / f"f{index}").symlink_to(
                "../../../../usb_gadget/g1/functions/" + name
            )
        for name, value in [
            ("dev_addr", "aa:e4:f8:1c:10:86\0\n"),
            ("host_addr", "96:03:ae:cc:f8:37\0\n"),
            ("ifname", "usb0" if retained else b.PLACEHOLDER),
        ]:
            (function / name).write_text(value)
        wlan = net / "wlan0"
        wlan.mkdir()
        (wlan / "address").write_text("02:11:22:33:44:55\n")
        if retained:
            usb = net / "usb0"
            usb.mkdir()
            (usb / "address").write_text("aa:e4:f8:1c:10:86\n")
            (usb / "flags").write_text("0x1002\n")
        mounts = root / "mounts"
        mounts.write_text(f"tmpfs {ram} tmpfs rw 0 0\n")
        source = (
            b.COLLECTOR.replace("/dev/shm", str(ram))
            .replace("/blackbox", str(blackbox))
            .replace("/proc/mounts", str(mounts))
            .replace("/sys/class/net", str(net))
            .replace(b.GADGET, str(gadget))
        )
        self.assertNotIn("/sys/", source)
        self.assertNotIn("/proc/", source)

        def invoke(mode):
            result = SimpleNamespace(
                stdout="2: wlan0 inet 192.168.1.2/24 brd 192.168.1.255 scope global wlan0\n"
            )
            with (
                patch.object(sys, "argv", ["query.py", mode, str(work)]),
                patch("os.getuid", return_value=0),
                patch("shutil.which", return_value="/mock/tool"),
                patch("subprocess.run", return_value=result),
            ):
                exec(compile(source, "isolated-query.py", "exec"), {})

        return work, blackbox / "export_tmp/osmo_probe_012345abcdef.json", invoke

    def test_collector_absent_retained_and_owned_cleanup(self):
        for retained in (False, True):
            with (
                self.subTest(retained=retained),
                tempfile.TemporaryDirectory() as folder,
            ):
                work, output, invoke = self.fixture(folder, retained=retained)
                invoke("query")
                report = json.loads(output.read_text())
                # Substitute only the fixture's mountpoint for host-side decoding.
                report["ram_realpath"] = "/dev/shm"
                runtime = b.decode_runtime(report)
                self.assertEqual(runtime.original_netdev_absent, not retained)
                self.assertTrue((work / "export.owner").exists())
                invoke("cleanup")
                self.assertFalse(output.exists())
                self.assertFalse(work.exists())

    def test_preexisting_export_is_never_deleted(self):
        with tempfile.TemporaryDirectory() as folder:
            work, output, invoke = self.fixture(folder)
            output.parent.mkdir()
            output.write_text("foreign existing file")
            with self.assertRaises(FileExistsError):
                invoke("query")
            self.assertFalse((work / "export.owner").exists())
            invoke("cleanup")
            self.assertEqual(output.read_text(), "foreign existing file")
            self.assertFalse(work.exists())

    def test_replaced_export_is_never_deleted(self):
        with tempfile.TemporaryDirectory() as folder:
            work, output, invoke = self.fixture(folder)
            invoke("query")
            output.rename(output.with_suffix(".old"))
            output.write_text("foreign replacement")
            with self.assertRaisesRegex(ValueError, "ownership"):
                invoke("cleanup")
            self.assertEqual(output.read_text(), "foreign replacement")
            self.assertTrue(work.exists())


if __name__ == "__main__":
    unittest.main()
