"""Attachment failure provenance with fake USB/network/producer owners only."""

import errno
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _retained_stream as m
import test_retained_stream as fixtures


class AttachmentDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.RetainedTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.value = self.fixture.armed()

    def attach(self):
        return self.value.attach_data_first(
            timeout=13, native_floor_ns=1, wake_host_ns=2
        )

    def test_rebind_failure_freezes_candidate_and_exact_sysfs_error_without_retry(self):
        network, value = self.fixture.network, self.value
        path = "/sys/class/net/usb1/address"
        error = OSError(errno.EINVAL, "netdev unregister", path)
        commands = list(self.fixture.control.commands)

        def fail_rebind(**options):
            network.report["rebind"] = [{"new_guard_report": {
                "initial_identity_retries": 2, "addresses_added": []}}]
            raise error

        network.rebind_after_usb_reset = Mock(side_effect=fail_rebind)
        with self.assertRaises(m.UnknownFutureApply) as caught:
            self.attach()
        failure = value.report["attach_failure"]
        self.assertIs(caught.exception.__cause__, error)
        self.assertEqual(failure["phase"], "rebind")
        detail = failure["errors"][0]
        self.assertEqual((detail["type"], detail["errno"], detail["filename"]),
                         ("OSError", errno.EINVAL, path))
        self.assertEqual(detail["traceback"][-1]["function"], "fail_rebind")
        self.assertTrue(detail["traceback"][-1]["line"] > 0)
        network.report["rebind"][0]["new_guard_report"]["addresses_added"].append("later")
        self.assertEqual(failure["network"]["rebind"][0]["new_guard_report"]["addresses_added"], [])
        network.rebind_after_usb_reset.assert_called_once_with(timeout=13.0)
        self.assertTrue(value.possible_future_apply)
        self.assertTrue(value.report["recovery_required"])
        self.assertFalse(value.lease_proven)
        self.assertEqual(self.fixture.control.commands, commands)
        self.assertNotIn("video_start", self.fixture.events)
        self.assertNotIn("imu_start", self.fixture.events)
        self.assertNotIn("lease.CLOSE", self.fixture.events)

    def test_first_absence_probe_failure_is_distinct_and_never_rebinds(self):
        network = self.fixture.network
        network._old_index_absent = Mock(side_effect=OSError(errno.EINVAL, "inventory"))
        network.rebind_after_usb_reset = Mock()
        with self.assertRaises(m.UnknownFutureApply):
            self.attach()
        self.assertEqual(self.value.report["attach_failure"]["phase"], "old_index_absent")
        network.rebind_after_usb_reset.assert_not_called()

    def test_socket_bind_failure_keeps_local_lease_cleanup_and_new_guard_snapshot(self):
        final = fixtures.Network(self.fixture.events, 1)
        self.fixture.network.next = final
        sock = Mock()
        self.value.socket_factory = Mock(return_value=sock)
        with patch.object(m, "bind_socket", side_effect=OSError(errno.ENODEV, "bind")):
            with self.assertRaises(m.UnknownFutureApply):
                self.attach()
        failure = self.value.report["attach_failure"]
        self.assertEqual(failure["phase"], "socket_bind")
        self.assertEqual(failure["network"]["ready"]["ncm_interface"], "usb1")
        self.assertEqual(self.fixture.events.count("lease.local_stop"), 1)
        self.assertNotIn("lease.CLOSE", self.fixture.events)
        sock.close.assert_called_once_with()

    def test_proven_lease_failure_preserves_original_exception_and_media_phase(self):
        value = self.value
        value.network = fixtures.Network(self.fixture.events, 1)

        def complete(remaining, *, deadline):
            ready = dict(ok=True, ncm_interface="usb1")
            value._start_provisional(ready, remaining)
            return ready

        value._completion = complete
        error = OSError(errno.EIO, "video start")
        value.video_factory = Mock(return_value=Mock(start=Mock(side_effect=error)))
        with self.assertRaises(OSError) as caught:
            self.attach()
        self.assertIs(caught.exception, error)
        self.assertTrue(value.lease_proven)
        self.assertFalse(value.possible_future_apply)
        self.assertEqual(value.report["attach_failure"]["phase"], "video_start")
        self.assertNotIn("recovery_required", value.report)

    def test_exception_details_are_bounded_and_do_not_include_frame_locals(self):
        def nested(depth):
            secret_local = "must not serialize frame locals"
            if depth:
                return nested(depth - 1)
            raise OSError(errno.EINVAL, "x" * 3000, "/" + "p" * 2000)

        try:
            nested(12)
        except OSError as error:
            original = error
        current = original
        for _ in range(6):
            outer = RuntimeError("context" * 500)
            outer.__cause__ = current
            current = outer
        rows = m._attachment_error(current)
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(len(row["message"]) <= 1024 for row in rows))
        row = m._attachment_error(original)[0]
        self.assertEqual(len(row["traceback"]), 8)
        self.assertEqual(len(row["filename"]), 512)
        self.assertNotIn("secret_local", str(row))
        self.assertNotIn("must not serialize", str(row))

    def test_missing_or_uncopyable_guard_report_never_masks_original_error(self):
        class Uncopyable:
            def __deepcopy__(self, memo):
                raise RuntimeError("diagnostic copy failed")

        for report in (None, Uncopyable()):
            with self.subTest(report=type(report).__name__):
                self.setUp()
                if report is None:
                    del self.value.network.report
                else:
                    self.value.network.report = report
                error = OSError(errno.EINVAL, "original failure")
                self.value._completion = Mock(side_effect=error)
                with self.assertRaises(m.UnknownFutureApply) as caught:
                    self.attach()
                self.assertIs(caught.exception.__cause__, error)
                failure = self.value.report["attach_failure"]
                self.assertEqual(failure["errors"][0]["errno"], errno.EINVAL)
                if report is None:
                    self.assertIsNone(failure["network"])
                else:
                    self.assertEqual(failure["network_error"], "RuntimeError")

    def test_exception_formatting_failure_never_masks_original_error(self):
        class BrokenStringError(Exception):
            def __str__(self):
                raise ValueError("formatting failed")

            def __repr__(self):
                raise ValueError("representation failed")

        error = BrokenStringError()
        self.value._completion = Mock(side_effect=error)
        with self.assertRaises(m.UnknownFutureApply) as caught:
            self.attach()
        self.assertIs(caught.exception.__cause__, error)
        self.assertEqual(self.value.report["attach_diagnostics_error"], "ValueError")
        self.assertEqual(self.value.report["errors"][-1], "BrokenStringError")


if __name__ == "__main__":
    unittest.main()
