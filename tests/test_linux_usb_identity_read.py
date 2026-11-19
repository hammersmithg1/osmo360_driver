"""Netdev EINVAL before reset ownership; fake sysfs/ip, no host networking."""

from contextlib import contextmanager
import errno
from pathlib import Path
import unittest
from unittest.mock import patch

from osmo360_camera._linux_usb_network import (
    LinuxUSBNetworkGuard,
    LinuxUSBNetworkError,
    _NetdevIdentityReadError,
)
import test_linux_usb_rebind as fixtures


class LinuxUSBIdentityReadTests(unittest.TestCase):
    setUp = fixtures.LinuxUSBRebindTests.setUp

    @contextmanager
    def read_race(self, attribute, *, code=errno.EINVAL, once=True,
                  when=lambda guard: guard._reset_origin is not None,
                  after=lambda guard: None):
        """Fault the real read only inside _identity, not inventory reads."""
        identity, read = LinuxUSBNetworkGuard._identity, Path.read_text
        active, errors, guards, times = [], [], [], []

        def identify(guard, name, mac):
            active.append(guard)
            try:
                return identity(guard, name, mac)
            finally:
                active.pop()

        def racing(path, *args, **kwargs):
            target = (self.host.net / self.host.name / attribute
                      if attribute in ("address", "ifindex") else self.host.usb / attribute)
            if active and path == target and when(active[-1]) and (not once or not errors):
                guard = active[-1]
                guards.append(guard)
                times.append(self.host.now)
                after(guard)
                # A sysfs read() can fail after open(), without a filename.
                error = OSError(code, "injected identity read")
                errors.append(error)
                raise error
            return read(path, *args, **kwargs)

        with patch.object(LinuxUSBNetworkGuard, "_identity", identify), \
                patch.object(Path, "read_text", racing):
            yield dict(errors=errors, guards=guards, times=times)

    def test_transient_address_or_ifindex_einval_recaptures_before_single_add_and_up(self):
        for attribute in ("address", "ifindex"):
            with self.subTest(attribute=attribute):
                self.setUp()
                self.host.reset()
                capture, snapshots = self.guard.capture, []

                def fresh(camera=None):
                    snapshots.append(camera)
                    return capture(camera)

                self.guard.capture = fresh
                with self.read_race(attribute) as observed:
                    replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
                self.assertTrue(ready["ok"])
                self.assertEqual(len(observed["errors"]), 1)
                self.assertGreaterEqual(len(snapshots), 2)
                self.assertEqual(self.host.now, .1)
                self.assertEqual(replacement.report["initial_identity_einval_retries"], 1)
                self.assertEqual([event["operation"] for event in replacement.report["events"]],
                                 ["address_add", "link_up"])
                self.assertTrue(self.guard.cleaned)
                self.assertIs(replacement.baseline, self.baseline)
                row = replacement.report["identity_read_errors"]["errors"][0]
                self.assertEqual(row["path"], str(self.host.net / self.host.name / attribute))
                self.assertEqual(row["attribute"], attribute)
                self.assertTrue(row["preownership"])

    def test_persistent_read_exhausts_same_deadline_without_writes_or_adoption(self):
        for attribute in ("address", "ifindex"):
            with self.subTest(attribute=attribute):
                self.setUp()
                self.host.reset()
                with self.read_race(attribute, once=False) as observed, \
                        self.assertRaisesRegex(LinuxUSBNetworkError, "deadline") as caught:
                    self.guard.rebind_after_usb_reset(timeout=1.2)
                self.assertAlmostEqual(self.host.now, 1.2)
                self.assertTrue(all(at < 1.2 for at in observed["times"]))
                self.assertEqual(self.host.commands, [])
                fixtures.LinuxUSBRebindTests.assert_old_retained(self)
                report = self.guard.report["rebind_attempts"][-1]["new_guard_report"]
                self.assertNotIn("identity", report)
                errors = report["identity_read_errors"]
                self.assertEqual(errors["count"], len(observed["errors"]))
                self.assertEqual(len(errors["errors"]), 8)
                marker = caught.exception.__cause__
                self.assertIsInstance(marker, _NetdevIdentityReadError)
                self.assertEqual(marker.filename, str(self.host.net / self.host.name / attribute))
                self.assertIs(marker.__cause__, observed["errors"][-1])
                self.assertIsNone(marker.__cause__.filename)

    def test_original_reset_deadline_is_not_extended_by_wait_ready_budget(self):
        self.host.reset()
        original = LinuxUSBNetworkGuard.wait_ready

        def extended(guard, baseline, mac, *, timeout):
            # Even a larger caller budget must not renew the reset deadline.
            return original(guard, baseline, mac, timeout=timeout + 10)

        with patch.object(LinuxUSBNetworkGuard, "wait_ready", extended), \
                self.read_race("address", once=False), \
                self.assertRaisesRegex(LinuxUSBNetworkError, "deadline"):
            self.guard.rebind_after_usb_reset(timeout=.35)
        self.assertAlmostEqual(self.host.now, .35)
        self.assertEqual(self.host.commands, [])

    def test_existing_enoent_discovery_retry_is_preserved(self):
        self.host.reset()
        with self.read_race("address", code=errno.ENOENT):
            replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
        self.assertTrue(ready["ok"])
        self.assertEqual(replacement.report["initial_identity_retries"], 1)
        self.assertNotIn("initial_identity_einval_retries", replacement.report)

    def test_einval_without_active_reset_is_terminal_with_original_cause(self):
        self.host.present, self.host.up, self.host.addresses = False, False, []
        old = self.guard
        guard = LinuxUSBNetworkGuard(capture=old.capture, run=old.run,
            net_root=old.net_root, usb_root=old.usb_root, physical_root=old.physical_root,
            state_root=old.state_root, clock=old.clock, sleep=old.sleep)
        baseline = guard.preflight()
        self.host.present = True
        with self.read_race("address", when=lambda guard: True) as observed, \
                self.assertRaises(_NetdevIdentityReadError) as caught:
            guard.wait_ready(baseline, fixtures.MAC, timeout=1)
        self.assertEqual(self.host.now, 0)
        self.assertEqual(len(observed["errors"]), 1)
        self.assertIs(caught.exception.__cause__, observed["errors"][0])
        self.assertNotIn("initial_identity_einval_retries", guard.report)
        self.assertEqual(self.host.commands, [])

    def test_pinned_or_already_attempted_candidate_never_retries_identity(self):
        for phase in ("pinned", "attempted", "event"):
            with self.subTest(phase=phase):
                self.setUp()
                self.host.reset()
                condition = (lambda guard: guard.identity is not None) if phase == "pinned" else (
                    lambda guard: guard._reset_origin is not None and guard.identity is None)

                def contaminate(guard):
                    if phase == "attempted":
                        guard.attempts.add("address_add")
                    elif phase == "event":
                        guard.report["events"].append(dict(operation="address_add", attempted=True))

                with self.read_race("address", when=condition, after=contaminate) as observed, \
                        self.assertRaises(_NetdevIdentityReadError) as caught:
                    self.guard.rebind_after_usb_reset(timeout=1)
                self.assertEqual(self.host.now, 0)
                self.assertEqual(len(observed["errors"]), 1)
                self.assertIs(caught.exception.__cause__, observed["errors"][0])
                self.assertEqual(self.host.commands, [])
                report = self.guard.report["rebind_attempts"][-1]["new_guard_report"]
                self.assertNotIn("initial_identity_einval_retries", report)

    def test_einval_after_successful_add_does_not_replay_add_or_start_up(self):
        self.host.reset()
        with self.read_race("ifindex", when=lambda guard: "address_add" in guard.attempts) as observed, \
                self.assertRaises(_NetdevIdentityReadError):
            self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(len(observed["errors"]), 1)
        self.assertEqual([argv[3] for argv in self.host.commands], ["add", "del"])
        self.assertFalse(self.guard.cleaned)
        self.assertEqual(self.host.now, 0)

    def test_other_attributes_errnos_and_failed_ip_command_are_never_retried(self):
        for attribute, code in (("busnum", errno.EINVAL), ("address", errno.EIO),
                                ("ifindex", errno.EACCES)):
            with self.subTest(attribute=attribute, code=code):
                self.setUp()
                self.host.reset()
                with self.read_race(attribute, code=code) as observed, self.assertRaises(OSError) as caught:
                    self.guard.rebind_after_usb_reset(timeout=1)
                self.assertIs(caught.exception, observed["errors"][0])
                self.assertEqual(self.host.now, 0)
                self.assertEqual(self.host.commands, [])
                report = self.guard.report["rebind_attempts"][-1]["new_guard_report"]
                self.assertNotIn("initial_identity_einval_retries", report)
        self.setUp()
        self.host.reset()
        run = self.guard.run
        error = OSError(errno.EINVAL, "uncertain ip command result")

        def uncertain(argv, **options):
            result = run(argv, **options)
            if argv[3] == "add":
                raise error
            return result

        self.guard.run = uncertain
        with self.assertRaises(OSError) as caught:
            self.guard.rebind_after_usb_reset(timeout=1)
        self.assertIs(caught.exception, error)
        self.assertEqual([argv[3] for argv in self.host.commands], ["add", "del"])


if __name__ == "__main__":
    unittest.main()
