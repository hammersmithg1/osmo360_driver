"""Complete inventories across netdev unregister races; fake sysfs/ip only."""

import errno
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from osmo360_camera import _linux_network as network
from osmo360_camera._linux_usb_network import LinuxUSBNetworkGuard, LinuxUSBNetworkError
import test_linux_usb_rebind as fixtures


class LinuxUSBInventoryTests(unittest.TestCase):
    setUp = fixtures.LinuxUSBRebindTests.setUp

    def departing(self):
        entry = self.host.net / "departing"
        entry.mkdir()
        (entry / "ifindex").write_text("20")
        return entry

    def read_race(self, entry, code, *, once=True, after=lambda: None):
        read = Path.read_text
        calls = []

        def racing(path, *args, **kwargs):
            if path == entry / "ifindex":
                calls.append(path)
                if once:
                    path.unlink()
                    entry.rmdir()
                after()
                raise OSError(code, "injected netdev unregister", str(path))
            return read(path, *args, **kwargs)

        return patch.object(Path, "read_text", racing), calls

    def test_transient_enoent_or_kernel_einval_recaptures_complete_views(self):
        for code in (errno.ENOENT, errno.EINVAL):
            with self.subTest(code=code):
                self.setUp()
                self.host.reset()
                entry = self.departing()
                capture = self.guard.capture
                captures = []

                def observed(camera=None):
                    captures.append(camera)
                    return capture(camera)

                self.guard.capture = observed
                race, reads = self.read_race(entry, code)
                with race:
                    replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
                self.assertTrue(ready["ok"])
                self.assertEqual(len(reads), 1)
                self.assertGreaterEqual(len(captures), 2)
                self.assertAlmostEqual(self.host.now, 0.1)
                retries = self.guard.report["inventory_retries"]
                self.assertEqual(retries["count"], 1)
                self.assertEqual(retries["errors"][0]["errno"], code)
                self.assertTrue(self.guard.cleaned)
                self.assertIs(replacement.baseline, self.baseline)
                self.assertEqual(
                    [event["operation"] for event in replacement.report["events"]],
                    ["address_add", "link_up"],
                )
                self.assertEqual([index for index, _ in self.host.generations], [9, 9])

    def test_retry_discards_supplied_ip_snapshot(self):
        self.host.reset()
        supplied = self.host.capture()
        entry = self.departing()
        capture = self.host.capture
        captures = []

        def reappeared(camera=None):
            captures.append(camera)
            fresh = capture(camera)
            rows = network._json(fresh, network.ADDR_COMMAND)
            rows[0]["ifindex"] = self.original.ifindex
            next(row for row in fresh["commands"] if row["argv"] == network.ADDR_COMMAND)[
                "stdout"
            ] = json.dumps(rows)
            return fresh

        self.guard.capture = reappeared
        race, _ = self.read_race(entry, errno.EINVAL)
        with race:
            absent, proof = self.guard._old_index_absent(
                self.original, supplied, deadline=1
            )
        self.assertEqual(len(captures), 1)
        self.assertFalse(absent)  # Fresh ip view, despite absent old sysfs entry.
        self.assertTrue(proof["complete"])
        self.assertEqual(self.host.commands, [])

    def test_new_sysfs_entries_are_included_after_retry(self):
        self.host.reset()
        entry = self.departing()

        def reappear():
            new = self.host.net / "newly-renamed-old"
            new.mkdir()
            (new / "ifindex").write_text(str(self.original.ifindex))

        race, _ = self.read_race(entry, errno.ENOENT, after=reappear)
        with race:
            absent, proof = self.guard._old_index_absent(self.original, deadline=1)
        self.assertFalse(absent)
        self.assertEqual(proof["sysfs_interfaces"], 4)
        self.assertEqual(self.host.commands, [])

    def test_persistent_unregister_read_exhausts_original_deadline_fail_closed(self):
        for code in (errno.ENOENT, errno.EINVAL):
            with self.subTest(code=code):
                self.setUp()
                self.host.reset()
                entry = self.departing()
                race, reads = self.read_race(entry, code, once=False)
                with race, self.assertRaisesRegex(LinuxUSBNetworkError, "deadline"):
                    self.guard.rebind_after_usb_reset(timeout=1.2)
                self.assertAlmostEqual(self.host.now, 1.2)
                self.assertGreater(len(reads), 8)
                retries = self.guard.report["inventory_retries"]
                self.assertEqual(retries["count"], len(reads))
                self.assertEqual(len(retries["errors"]), 8)
                self.assertEqual(self.host.commands, [])
                fixtures.LinuxUSBRebindTests.assert_old_retained(self)

    def test_permission_other_io_and_malformed_values_are_terminal(self):
        for fault in (errno.EACCES, errno.EIO, "malformed", "nonpositive", "duplicate"):
            with self.subTest(fault=fault):
                self.setUp()
                self.host.reset()
                entry = self.departing()
                if isinstance(fault, int):
                    context, _ = self.read_race(entry, fault, once=False)
                else:
                    (entry / "ifindex").write_text(
                        dict(malformed="bad", nonpositive="0", duplicate="1")[fault]
                    )
                    context = patch.object(self.guard, "sleep", side_effect=AssertionError("retry"))
                with context, self.assertRaises(LinuxUSBNetworkError):
                    self.guard.rebind_after_usb_reset(timeout=1)
                self.assertEqual(self.host.now, 0)
                self.assertNotIn("inventory_retries", self.guard.report)
                self.assertEqual(self.host.commands, [])

    def test_ifindex_unregister_errors_outside_reset_are_terminal(self):
        for code in (errno.ENOENT, errno.EINVAL):
            with self.subTest(code=code):
                self.setUp()
                self.host.reset()
                entry = self.departing()
                race, reads = self.read_race(entry, code, once=False)
                with race, self.assertRaisesRegex(LinuxUSBNetworkError, "complete sysfs"):
                    self.guard._old_index_absent(self.original)
                self.assertEqual(len(reads), 1)
                self.assertEqual(self.host.now, 0)
                self.assertNotIn("inventory_retries", self.guard.report)

    def test_pre_mutation_inventory_retry_never_replays_add_or_up(self):
        self.host.reset()
        original = LinuxUSBNetworkGuard._command
        entries = []

        def before_add(guard, key, argv):
            if guard._reset_origin is not None and key == "address_add":
                entries.append(self.departing())
            return original(guard, key, argv)

        read = Path.read_text

        def unregister(path, *args, **kwargs):
            if entries and path == entries[0] / "ifindex":
                path.unlink()
                entries[0].rmdir()
                raise OSError(errno.EINVAL, "unregister", str(path))
            return read(path, *args, **kwargs)

        with patch.object(LinuxUSBNetworkGuard, "_command", before_add), patch.object(
            Path, "read_text", unregister
        ):
            replacement, ready = self.guard.rebind_after_usb_reset(timeout=1)
        self.assertTrue(ready["ok"])
        self.assertEqual(replacement.report["inventory_retries"]["count"], 1)
        self.assertEqual(
            [event["operation"] for event in replacement.report["events"]],
            ["address_add", "link_up"],
        )
        self.assertEqual(replacement.attempts, {"address_add", "link_up"})

    def test_final_proof_evaluates_fresh_state_after_inventory_retry(self):
        self.host.reset()
        original = LinuxUSBNetworkGuard._interface_inventory
        capture = self.guard.capture
        changed = []

        def inventory(guard, *args, **kwargs):
            if guard._reset_origin is not None and guard.report.get("ready", {}).get("ok"):
                entry = self.departing()
                race, _ = self.read_race(entry, errno.EINVAL, after=lambda: changed.append(True))
                with race:
                    return original(guard, *args, **kwargs)
            return original(guard, *args, **kwargs)

        def state(camera=None):
            snapshot = capture(camera)
            if changed:
                snapshot["resolv_conf"] += "nameserver 1.1.1.1\n"
            return snapshot

        self.guard.capture = state
        with patch.object(LinuxUSBNetworkGuard, "_interface_inventory", inventory):
            with self.assertRaisesRegex(LinuxUSBNetworkError, "readiness/absence"):
                self.guard.rebind_after_usb_reset(timeout=1)
        self.assertEqual(len(changed), 1)
        self.assertEqual(sum("add" in argv for argv in self.host.commands), 1)
        self.assertEqual(sum(argv[-1] == "up" for argv in self.host.commands), 1)
        fixtures.LinuxUSBRebindTests.assert_old_retained(self)


if __name__ == "__main__":
    unittest.main()
