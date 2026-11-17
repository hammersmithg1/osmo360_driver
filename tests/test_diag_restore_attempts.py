"""Verified restoration distinguishes retry diagnostics from terminal faults."""

import struct
import unittest
from unittest.mock import Mock

from osmo360_camera import _diag_collector as m
import test_diag_rebind as fixtures


def mask_reply(operation, mask=b"\0", last=7, status=0):
    return b"\x73\0\0\0" + struct.pack("<4I", operation, status, 1, last) + mask


class RestoreAttemptTests(unittest.TestCase):
    def report(self, *errors):
        return dict(restored=False, errors=list(errors))

    def test_first_set_ack_timeout_then_second_verified_readback_is_clean(self):
        report = self.report()
        request = Mock(
            side_effect=[RuntimeError("SET ACK timeout"), mask_reply(3), mask_reply(4)]
        )
        fallback = Mock(
            side_effect=AssertionError("No rebind after verified normal restoration")
        )
        m._restore_original_mask(report, request, 7, b"\0", rebind=fallback)
        self.assertTrue(report["restored"])
        self.assertEqual(report["errors"], [])
        self.assertEqual(
            report["restore_attempt_errors"], ["restore attempt 1: SET ACK timeout"]
        )
        self.assertEqual([call.args[1] for call in request.call_args_list], [3, 3, 4])
        self.assertEqual(
            request.call_args_list[0].args[0],
            m.config(3, struct.pack("<II", 1, 7) + b"\0"),
        )
        self.assertEqual(
            request.call_args_list[-1].args[0], m.config(4, struct.pack("<I", 1))
        )
        fallback.assert_not_called()

    def test_get_timeout_then_verified_retry_retains_diagnostic_without_terminal_error(
        self,
    ):
        report = self.report()
        request = Mock(
            side_effect=[
                mask_reply(3),
                OSError("GET timed out"),
                mask_reply(3),
                mask_reply(4),
            ]
        )
        m._restore_original_mask(report, request, 7, b"\0")
        self.assertTrue(report["restored"])
        self.assertEqual(report["errors"], [])
        self.assertEqual(len(report["restore_attempt_errors"]), 1)
        self.assertEqual(
            [call.args[1] for call in request.call_args_list], [3, 4, 3, 4]
        )

    def test_all_normal_attempts_fail_and_are_terminal_without_extra_attempt(self):
        report = self.report()
        request = Mock(side_effect=RuntimeError("No ACK"))
        m._restore_original_mask(report, request, 7, b"\0")
        self.assertFalse(report["restored"])
        self.assertEqual(request.call_count, 3)
        self.assertEqual(len(report["restore_attempt_errors"]), 3)
        self.assertEqual(
            report["errors"],
            ["Original DIAG mask restoration was not verified after 3 attempts"],
        )

    def test_foreign_mask_last_item_or_invalid_get_reply_never_claims_restoration(self):
        for reply in (
            mask_reply(4, b"\x04"),
            mask_reply(4, b"\0\0", last=8),
            mask_reply(4, status=1),
            b"\x73\0",
        ):
            with self.subTest(reply=reply.hex()):
                report = self.report()
                request = Mock(
                    side_effect=lambda raw, operation: (
                        mask_reply(3) if operation == 3 else reply
                    )
                )
                m._restore_original_mask(report, request, 7, b"\0")
                self.assertFalse(report["restored"])
                self.assertTrue(report["errors"])
                self.assertEqual(
                    [call.args[1] for call in request.call_args_list], [3, 4] * 3
                )
                self.assertEqual(len(report["restore_attempt_errors"]), 3)

    def test_first_valid_readback_needs_no_retry_or_fallback(self):
        report = self.report()
        request = Mock(side_effect=[mask_reply(3), mask_reply(4)])
        fallback = Mock()
        m._restore_original_mask(report, request, 7, b"\0", rebind=fallback)
        self.assertEqual(
            report, dict(restored=True, errors=[], restore_attempt_errors=[])
        )
        self.assertEqual(request.call_count, 2)
        fallback.assert_not_called()

    def test_successful_restore_only_rebind_keeps_retry_diagnostics_and_capture_fault(
        self,
    ):
        for existing in ((), ("Fatal DIAG read: stream continuity lost",)):
            with self.subTest(existing=existing):
                report = self.report(*existing)
                request = Mock(side_effect=OSError("Old handle vanished"))
                guard = fixtures.Guard(fixtures.Clock())

                def rebound():
                    return m._restore_only_after_reset(
                        guard,
                        7,
                        b"\0",
                        b"\x04",
                        clock=guard.clock,
                        sleep=guard.clock.sleep,
                    )

                fallback = Mock(side_effect=rebound)
                m._restore_original_mask(report, request, 7, b"\0", rebind=fallback)
                self.assertTrue(report["restored"])
                self.assertEqual(report["errors"], list(existing))
                self.assertEqual(len(report["restore_attempt_errors"]), 3)
                self.assertFalse(report["mask_restore_rebind"]["sample_stream_resumed"])
                fallback.assert_called_once_with()

    def test_foreign_rebind_mask_is_terminal_and_is_never_overwritten(self):
        report = self.report()
        guard = fixtures.Guard(fixtures.Clock())
        guard.mask = (7, b"\x08")

        def rebound():
            return m._restore_only_after_reset(
                guard, 7, b"\0", b"\x04", clock=guard.clock, sleep=guard.clock.sleep
            )

        m._restore_original_mask(
            report, Mock(side_effect=OSError("Gone")), 7, b"\0", rebind=rebound
        )
        self.assertFalse(report["restored"])
        self.assertIn("Foreign DIAG mask", report["errors"][0])
        self.assertFalse(
            any(
                isinstance(event, tuple) and event[0] == "SET" for event in guard.events
            )
        )

    def test_reset_first_observed_during_restore_still_enables_late_rebind(self):
        report = self.report()
        state = dict(fatal_read_status=None)
        guard = fixtures.Guard(fixtures.Clock())

        def request(raw, operation):
            state["fatal_read_status"] = -4
            raise OSError("USB disappeared during restore ACK")

        def rebound():
            if state["fatal_read_status"] is None:
                return None
            return m._restore_only_after_reset(
                guard, 7, b"\0", b"\x04", clock=guard.clock, sleep=guard.clock.sleep
            )

        self.assertIsNone(state["fatal_read_status"])
        m._restore_original_mask(
            report, Mock(side_effect=request), 7, b"\0", rebind=rebound
        )
        self.assertEqual(state["fatal_read_status"], -4)
        self.assertTrue(report["restored"])
        self.assertEqual(report["errors"], [])
        self.assertEqual(len(report["restore_attempt_errors"]), 3)
        self.assertEqual(report["mask_restore_rebind"]["set_attempts"], 1)

    def test_no_late_reset_evidence_keeps_rebind_unavailable_and_failure_terminal(self):
        report = self.report()
        fallback = Mock(return_value=None)
        m._restore_original_mask(
            report, Mock(side_effect=RuntimeError("No ACK")), 7, b"\0", rebind=fallback
        )
        self.assertFalse(report["restored"])
        self.assertEqual(len(report["errors"]), 1)
        fallback.assert_called_once_with()

    def test_verified_rebind_with_close_failure_and_existing_fault_remains_terminal(
        self,
    ):
        report = self.report("Earlier capture fault")
        guard = fixtures.Guard(fixtures.Clock())
        guard.mask = (7, b"\0")
        guard.close_rebound = Mock(side_effect=OSError("Close failed"))

        def rebound():
            return m._restore_only_after_reset(
                guard, 7, b"\0", b"\x04", clock=guard.clock, sleep=guard.clock.sleep
            )

        m._restore_original_mask(
            report, Mock(side_effect=OSError("Gone")), 7, b"\0", rebind=rebound
        )
        self.assertTrue(report["restored"])
        self.assertEqual(report["errors"][0], "Earlier capture fault")
        self.assertIn("Close failed", report["errors"][1])
        self.assertIn("close_error", report["mask_restore_rebind"])


if __name__ == "__main__":
    unittest.main()
