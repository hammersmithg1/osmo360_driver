"""Reconcile only a completely vanished intermediate NIC; fake sysfs/ip."""

import json
import errno
import subprocess
import unittest
from unittest.mock import patch

from osmo360_camera import _linux_network as network
from osmo360_camera._linux_usb_network import (
    LinuxUSBNetworkGuard,
    LinuxUSBNetworkError,
    USBGenerationVanished,
)
import test_linux_usb_rebind as fixtures


class VanishedGenerationTests(unittest.TestCase):
    setUp = fixtures.LinuxUSBRebindTests.setUp

    def unplug(self):
        (self.host.net / self.host.name).unlink()
        self.host.present, self.host.up, self.host.addresses = False, False, []

    def vanished(self, phase, after_unplug=lambda self: None):
        self.host.reset()
        original = LinuxUSBNetworkGuard._command
        fired = []

        def race(guard, key, argv):
            if guard._reset_origin is not None and key == phase and not fired:
                fired.append(True)
                self.unplug()
                after_unplug(self)
            return original(guard, key, argv)

        with patch.object(LinuxUSBNetworkGuard, "_command", race):
            with self.assertRaises(LinuxUSBNetworkError) as caught:
                self.guard.rebind_after_usb_reset(timeout=1)
        return caught.exception

    def test_no_write_candidate_disappearance_has_typed_complete_proof(self):
        error = self.vanished("address_add")
        self.assertIsInstance(error, USBGenerationVanished)
        self.assertFalse(error.proof["completed_address_add"])
        self.assertEqual(self.host.commands, [])
        self.assertFalse(self.guard.cleaned)
        self.assertIs(self.guard.identity, self.original)
        for key, index in (("parent", 7), ("candidate", 9)):
            self.assertEqual(error.proof[key]["old_ifindex"], index)
            self.assertTrue(error.proof[key]["complete"])
            self.assertTrue(error.proof[key]["absent"])

    def sysfs_read_race(self, *, after_add=False, unplug=True, interference=None,
                       unrelated=False, error_type=FileNotFoundError):
        self.host.reset()
        original = LinuxUSBNetworkGuard._identity
        fired = []
        test = self

        def identity(guard, name, mac):
            # The first _identity succeeds. Race the repeated resolve/read
            # inside _current after this candidate has already been pinned.
            if (guard._reset_origin is not None and guard.identity is not None
                    and (not after_add or "address_add" in guard.attempts) and not fired):
                fired.append(True)
                if unplug:
                    test.unplug()
                if interference is not None:
                    interference(test)
                missing = test.host.root / "unrelated" if unrelated else test.host.net / name
                code = errno.EACCES if error_type is PermissionError else errno.ENOENT
                raise error_type(code, "injected pinned sysfs race", str(missing))
            return original(guard, name, mac)

        with patch.object(LinuxUSBNetworkGuard, "_identity", identity):
            with self.assertRaises((LinuxUSBNetworkError, OSError)) as caught:
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(len(fired), 1)
        return caught.exception

    def test_pinned_sysfs_disappearing_after_ip_snapshot_requires_complete_absence_before_retry(self):
        error = self.sysfs_read_race()
        self.assertIsInstance(error, USBGenerationVanished)
        self.assertEqual(self.host.commands, [])
        self.assertFalse(error.proof["completed_address_add"])
        for name in ("parent", "candidate"):
            self.assertTrue(error.proof[name]["complete"])
            self.assertTrue(error.proof[name]["absent"])
        self.assertFalse(self.guard.cleaned)
        self.assertIs(self.guard.identity, self.original)

    def test_pinned_sysfs_read_loss_after_successful_add_is_typed_without_replaying_add(self):
        error = self.sysfs_read_race(after_add=True)
        self.assertIsInstance(error, USBGenerationVanished)
        self.assertTrue(error.proof["completed_address_add"])
        self.assertEqual([argv[3] for argv in self.host.commands], ["add"])

    def test_single_missing_read_or_unrelated_enoent_is_not_generation_absence(self):
        for options in (dict(unplug=False), dict(unrelated=True), dict(error_type=PermissionError)):
            with self.subTest(options=options):
                self.setUp()
                error = self.sysfs_read_race(**options)
                self.assertIsInstance(error, options.get('error_type', FileNotFoundError))
                self.assertNotIsInstance(error, USBGenerationVanished)
                self.assertEqual(self.host.commands, [])

    def test_enoent_from_attempted_ip_command_is_never_a_retryable_read_observation(self):
        self.host.reset()
        original = self.host.run

        def uncertain(argv, **options):
            result = original(argv, **options)
            if argv[3] == 'add':
                self.unplug()
                raise FileNotFoundError(errno.ENOENT, 'uncertain command result', str(self.host.net / self.host.name))
            return result

        self.guard.run = uncertain
        with self.assertRaises(FileNotFoundError):
            self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual([argv[3] for argv in self.host.commands], ['add'])
        self.assertNotIn('vanished_generation', self.guard.report['rebind_attempts'][-1])

    def test_missing_owned_read_with_incomplete_inventory_or_renamed_index_is_not_retryable(self):
        def incomplete(case):
            (case.host.net / "incomplete").mkdir()

        def renamed(case):
            name = "renamed"
            (case.host.net / name).symlink_to(case.host.nic, target_is_directory=True)
            case.host.name, case.host.present = name, True

        for interference in (incomplete, renamed):
            with self.subTest(interference=interference):
                self.setUp()
                error = self.sysfs_read_race(interference=interference)
                self.assertNotIsInstance(error, USBGenerationVanished)
                self.assertEqual(self.host.commands, [])

    def test_actual_add_rc0_then_disappear_before_up_can_retry_fresh_strict_generation(
        self,
    ):
        error = self.vanished("link_up")
        self.assertIsInstance(error, USBGenerationVanished)
        self.assertTrue(error.proof["completed_address_add"])
        self.assertEqual([argv[3] for argv in self.host.commands], ["add"])
        self.assertFalse(self.guard.cleaned)
        # A later generation appears. The same last-committed guard repeats
        # its complete absence/physical-port/MAC checks before any new write.
        (self.host.net / self.host.name).symlink_to(
            self.host.nic, target_is_directory=True
        )
        self.host.reset(index=11, devnum=8)
        replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(replacement.identity.ifindex, 11)
        self.assertTrue(ready["ok"])
        self.assertEqual([index for index, _ in self.host.generations], [9, 11, 11])

    def test_uncertain_add_even_if_nic_disappears_never_gets_retry_permission(self):
        self.host.reset()
        original = self.host.run

        def uncertain(argv, **options):
            result = original(argv, **options)
            if argv[3] == "add":
                self.unplug()
                raise subprocess.TimeoutExpired(argv, 5)
            return result

        self.guard.run = uncertain
        with self.assertRaises(subprocess.TimeoutExpired):
            self.guard.rebind_after_usb_reset(timeout=1)
        self.assertNotIn(
            "vanished_generation", self.guard.report["rebind_attempts"][-1]
        )
        self.assertFalse(self.guard.cleaned)

    def test_reappeared_parent_or_incomplete_sysfs_is_not_typed_absence(self):
        for fault in ("parent", "incomplete"):
            with self.subTest(fault=fault):
                # Each case starts with an independent exact baseline.
                self.setUp()

                def interference(case):
                    path = case.host.net / "foreign"
                    path.mkdir()
                    if fault == "parent":
                        (path / "ifindex").write_text("7")

                error = self.vanished("link_up", interference)
                self.assertNotIsInstance(error, USBGenerationVanished)

    def test_changed_dns_after_nic_loss_is_not_a_retryable_generation(self):
        original = self.host.capture
        changed = []

        def capture(camera=None):
            value = original(camera)
            if changed:
                value["resolv_conf"] += "nameserver 1.1.1.1\n"
            return value

        self.guard.capture = capture
        error = self.vanished("link_up", lambda case: changed.append(True))
        self.assertNotIsInstance(error, USBGenerationVanished)

    def test_alias_elsewhere_after_nic_loss_blocks_retry(self):
        original = self.host.capture
        changed = []

        def capture(camera=None):
            value = original(camera)
            if changed:
                row = next(
                    row
                    for row in value["commands"]
                    if row["argv"] == network.ADDR_COMMAND
                )
                rows = json.loads(row["stdout"])
                rows.append(
                    dict(
                        ifname="foreign",
                        ifindex=20,
                        address="02:00:00:00:00:20",
                        flags=["UP"],
                        operstate="UP",
                        addr_info=[
                            dict(family="inet", local=network.HOST, prefixlen=30)
                        ],
                    )
                )
                row["stdout"] = json.dumps(rows)
            return value

        self.guard.capture = capture
        error = self.vanished("link_up", lambda case: changed.append(True))
        self.assertNotIsInstance(error, USBGenerationVanished)


if __name__ == "__main__":
    unittest.main()
