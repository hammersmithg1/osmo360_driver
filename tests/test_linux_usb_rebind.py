"""Pinned Linux ECM generation handover; fake sysfs/ip only, no host mutation."""

from dataclasses import asdict
import json
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from osmo360_camera import _linux_network as network
from osmo360_camera._linux_usb_network import LinuxUSBNetworkGuard, LinuxUSBNetworkError
from linux_network_fixtures import FakeHost, MAC, NAME


class ResetHost(FakeHost):
    def __init__(self, root):
        super().__init__(root)
        self.index, self.name = 7, NAME
        self.generations = []
        for name, index in (("lo", 1), ("wlan0", 3)):
            path = self.net / name
            path.mkdir()
            (path / "ifindex").write_text(str(index))

    def capture(self, camera=None):
        result = super().capture(camera)
        for command in result["commands"]:
            rows = json.loads(command["stdout"])
            for row in rows:
                if row.get("ifname") == NAME:
                    row.update(ifname=self.name, ifindex=self.index)
                if row.get("dev") == NAME:
                    row["dev"] = self.name
            command["stdout"] = json.dumps(rows)
        return result

    def run(self, argv, **kwargs):
        self.generations.append((self.index, list(argv)))
        return super().run(argv, **kwargs)

    def reset(
        self, *, index=9, devnum=6, name=NAME, port="1-1", busnum=1, product="0025"
    ):
        (self.net / self.name).unlink()
        for entry in self.bus.iterdir():
            entry.unlink()
        self.usb.rename(self.usb.with_name("old-generation-" + str(self.index)))
        self.usb = self.physical / "usb1" / port
        self.usb.mkdir(parents=True)
        for key, value in dict(
            idVendor="2ca3",
            idProduct=product,
            bDeviceClass="ef",
            busnum=str(busnum),
            devnum=str(devnum),
        ).items():
            (self.usb / key).write_text(value)
        (self.bus / port).symlink_to(self.usb, target_is_directory=True)
        self.interface = self.usb / (port + ":1.5")
        self.nic = self.interface / "net" / name
        self.nic.mkdir(parents=True)
        (self.net / name).symlink_to(self.nic, target_is_directory=True)
        (self.nic / "device").symlink_to(self.interface, target_is_directory=True)
        (self.nic / "address").write_text(MAC)
        (self.nic / "ifindex").write_text(str(index))
        (self.state / str(index)).write_text("ADMIN_STATE=unmanaged\n")
        self.index, self.name = index, name
        self.present, self.up, self.addresses = True, False, []


class LinuxUSBRebindTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.host = ResetHost(self.tmp.name)
        self.guard = LinuxUSBNetworkGuard(
            capture=self.host.capture,
            run=self.host.run,
            net_root=self.host.net,
            usb_root=self.host.bus,
            physical_root=self.host.physical,
            state_root=self.host.state,
            clock=lambda: self.host.now,
            sleep=lambda seconds: setattr(self.host, "now", self.host.now + seconds),
        )
        self.baseline = self.guard.preflight()
        self.host.present = True
        self.guard.wait_ready(self.baseline, MAC, timeout=1)
        self.original = self.guard.identity
        self.host.commands.clear()
        self.host.generations.clear()

    def assert_old_retained(self):
        self.assertIs(self.guard.baseline, self.baseline)
        self.assertIs(self.guard.identity, self.original)
        self.assertFalse(self.guard.cleaned)
        self.assertFalse(self.guard.report["restored"])
        self.assertTrue(self.guard.report["rebind_attempts"][-1].get("error"))

    def test_new_guard_preserves_original_baseline_and_retires_old_only_after_readiness(
        self,
    ):
        self.host.reset(name="usb9")
        replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
        self.assertIsNot(replacement, self.guard)
        self.assertIs(replacement.baseline, self.baseline)
        self.assertTrue(ready["ok"])
        self.assertEqual(ready["ncm_interface"], "usb9")
        self.assertEqual(
            (replacement.identity.ifindex, replacement.identity.devnum), (9, 6)
        )
        self.assertEqual(replacement.identity.usb_path, self.original.usb_path)
        self.assertTrue(replacement.report["old_index_absence"]["absent"])
        self.assertEqual(
            self.guard.report["rebind_attempts"][-1]["new"],
            asdict(replacement.identity),
        )
        self.assertTrue(self.guard.cleaned)
        count = len(self.host.commands)
        self.guard.cleanup_owned()  # Reused MAC/name must never cause old cleanup.
        self.assertEqual(len(self.host.commands), count)
        self.assertTrue(replacement.check_restored(self.baseline)["ok"])
        self.assertEqual([index for index, _ in self.host.generations], [9, 9, 9, 9])
        self.assertEqual(self.host.addresses, [])

    def test_old_ifindex_present_in_either_complete_inventory_blocks_all_new_writes(
        self,
    ):
        for view in ("ip", "sysfs"):
            with self.subTest(view=view):
                self.host.now = 0
                original_capture = self.host.capture
                if view == "ip":

                    def capture(camera=None):
                        value = original_capture(camera)
                        addresses = network._json(value, network.ADDR_COMMAND)
                        addresses[0]["ifindex"] = self.original.ifindex
                        next(
                            row
                            for row in value["commands"]
                            if row["argv"] == network.ADDR_COMMAND
                        )["stdout"] = json.dumps(addresses)
                        return value

                    self.guard.capture = capture
                    self.host.reset(index=9) if self.host.index == 7 else None
                else:
                    self.guard.capture = self.host.capture
                    path = self.host.net / "old-renamed"
                    path.mkdir()
                    (path / "ifindex").write_text(str(self.original.ifindex))
                with self.assertRaisesRegex(LinuxUSBNetworkError, "did not disappear"):
                    self.guard.rebind_after_usb_reset(timeout=0.2)
                self.assertEqual(self.host.commands, [])
                self.assert_old_retained()

    def test_failed_or_incomplete_inventory_is_not_absence(self):
        self.host.reset()
        real_capture = self.host.capture
        for fault in ("failed", "missing_index", "unreadable_sysfs"):
            with self.subTest(fault=fault):

                def capture(camera=None):
                    value = real_capture(camera)
                    row = next(
                        row
                        for row in value["commands"]
                        if row["argv"] == network.ADDR_COMMAND
                    )
                    if fault == "failed":
                        row["returncode"] = 1
                    elif fault == "missing_index":
                        values = json.loads(row["stdout"])
                        values[0].pop("ifindex")
                        row["stdout"] = json.dumps(values)
                    return value

                self.guard.capture = capture
                if fault == "unreadable_sysfs":
                    (self.host.net / "incomplete").mkdir()
                with self.assertRaises(LinuxUSBNetworkError):
                    self.guard.rebind_after_usb_reset(timeout=1)
                self.assertEqual(self.host.commands, [])
                self.assert_old_retained()

    def test_wrong_port_bus_product_or_unchanged_device_number_never_adds_alias(self):
        for change in (
            {"port": "1-2"},
            {"busnum": 2},
            {"product": "8002"},
            {"devnum": 4},
        ):
            with self.subTest(change=change):
                self.host.reset(index=self.host.index + 2, **change)
                with self.assertRaises(LinuxUSBNetworkError):
                    self.guard.rebind_after_usb_reset(timeout=1)
                self.assertEqual(self.host.commands, [])
                self.assert_old_retained()

    def test_managed_foreign_ipv4_default_dns_or_duplicate_mac_never_adopted(self):
        self.host.reset()
        real_capture = self.host.capture
        for fault in ("managed", "foreign_ipv4", "default", "dns", "duplicate_mac"):
            with self.subTest(fault=fault):
                (self.host.state / "9").write_text(
                    "ADMIN_STATE="
                    + ("configured" if fault == "managed" else "unmanaged")
                    + "\n"
                )
                self.host.addresses = ["169.254.1.2"] if fault == "foreign_ipv4" else []

                def capture(camera=None):
                    value = real_capture(camera)
                    if fault == "dns":
                        value["resolv_conf"] += "nameserver 1.1.1.1\n"
                    for row in value["commands"]:
                        data = json.loads(row["stdout"])
                        if (
                            row["argv"] == network.ADDR_COMMAND
                            and fault == "duplicate_mac"
                        ):
                            data.append(dict(data[-1], ifname="foreign9", ifindex=19))
                        if row["argv"] == network.ROUTE_COMMAND and fault == "default":
                            data[0]["gateway"] = "192.168.1.254"
                        row["stdout"] = json.dumps(data)
                    return value

                self.guard.capture = capture
                with self.assertRaises(LinuxUSBNetworkError):
                    self.guard.rebind_after_usb_reset(timeout=0.2)
                self.assertEqual(self.host.commands, [])
                self.assert_old_retained()

    def test_old_index_reappearing_just_before_alias_write_is_refused(self):
        self.host.reset()
        command = LinuxUSBNetworkGuard._command

        def race(guard, key, argv):
            if guard._reset_origin is not None and key == "address_add":
                path = self.host.net / "old-renamed"
                path.mkdir()
                (path / "ifindex").write_text(str(self.original.ifindex))
            return command(guard, key, argv)

        with patch.object(LinuxUSBNetworkGuard, "_command", race):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "still present"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(self.host.commands, [])
        self.assert_old_retained()

    def test_deadline_before_new_write_or_after_ready_cleans_only_new_owned_state(self):
        self.host.reset()
        original = LinuxUSBNetworkGuard.wait_ready

        def finish_late(guard, *args, **kwargs):
            value = original(guard, *args, **kwargs)
            self.host.now = 2
            return value

        with patch.object(LinuxUSBNetworkGuard, "wait_ready", finish_late):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "deadline"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual([index for index, _ in self.host.generations], [9, 9, 9, 9])
        self.assertEqual(self.host.addresses, [])
        self.assertFalse(self.host.up)
        self.assert_old_retained()
        self.host.commands.clear()
        self.host.generations.clear()
        self.host.now = 0
        command = LinuxUSBNetworkGuard._command

        def late(guard, key, argv):
            if key == "address_add":
                self.host.now = 2
            return command(guard, key, argv)

        with patch.object(LinuxUSBNetworkGuard, "_command", late):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "deadline"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(self.host.commands, [])

    def test_partial_new_add_failure_cleans_once_and_preserves_old_guard_error(self):
        self.host.reset()
        self.host.fail_after = "add"
        with self.assertRaises(subprocess.TimeoutExpired):
            self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual([argv[3] for argv in self.host.commands], ["add", "del"])
        self.assertEqual(self.host.addresses, [])
        self.assertTrue(all(index == 9 for index, _ in self.host.generations))
        self.assert_old_retained()

    def test_foreign_state_after_pin_never_receives_cleanup_writes(self):
        self.host.reset()
        command = LinuxUSBNetworkGuard._command

        def foreign(guard, key, argv):
            if guard._reset_origin is not None and key == "link_up":
                self.host.addresses.append("192.168.253.30")
            return command(guard, key, argv)

        with patch.object(LinuxUSBNetworkGuard, "_command", foreign):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "Foreign IPv4"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual([argv[3] for argv in self.host.commands], ["add"])
        self.assertIn(network.HOST, self.host.addresses)
        self.assertIn("192.168.253.30", self.host.addresses)
        event = self.guard.report["rebind_attempts"][-1]
        self.assertIn("Foreign IPv4", event["new_guard_report"]["cleanup_error"])
        self.assert_old_retained()

    def test_third_generation_during_slow_absence_inventory_never_receives_alias(self):
        self.host.reset()
        inventory = LinuxUSBNetworkGuard._old_index_absent
        changed = []

        def replace_during_inventory(guard, *args, **kwargs):
            result = inventory(guard, *args, **kwargs)
            if guard is not self.guard and not changed:
                changed.append(True)
                self.host.reset(index=11, devnum=8)
            return result

        with patch.object(
            LinuxUSBNetworkGuard, "_old_index_absent", replace_during_inventory
        ):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "identity changed"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(self.host.commands, [])
        self.assertEqual(self.host.generations, [])
        self.assertEqual(self.host.addresses, [])
        self.assertEqual(self.host.index, 11)
        self.assert_old_retained()

    def test_next_reset_uses_returned_guard_and_the_same_original_baseline(self):
        self.host.reset()
        first, _ = self.guard.rebind_after_usb_reset(timeout=1)
        first_identity = first.identity
        self.host.reset(index=11, devnum=8)
        second, ready = first.rebind_after_usb_reset(timeout=1)
        self.assertTrue(ready["ok"])
        self.assertIs(first.baseline, self.baseline)
        self.assertIs(second.baseline, self.baseline)
        self.assertEqual(second.report["rebind_from"], asdict(first_identity))
        self.assertEqual(second.identity.ifindex, 11)
        self.assertTrue(first.cleaned)
        before = len(self.host.commands)
        first.cleanup_owned()
        self.guard.cleanup_owned()
        self.assertEqual(len(self.host.commands), before)
        self.assertTrue(second.check_restored(self.baseline)["ok"])

    def test_invalid_timeout_or_inactive_guard_never_performs_io(self):
        with patch.object(
            self.guard, "capture", side_effect=AssertionError("I/O before validation")
        ):
            for timeout in (0, -1, True, float("nan"), float("inf"), "1"):
                with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                    self.guard.rebind_after_usb_reset(timeout=timeout)
            self.guard.cleaned = True
            with self.assertRaises(LinuxUSBNetworkError):
                self.guard.rebind_after_usb_reset(timeout=1)


if __name__ == "__main__":
    unittest.main()
