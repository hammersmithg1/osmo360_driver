"""Sleep-side preparation proof/budget fakes; no camera or network access."""

from dataclasses import replace
import unittest
from unittest.mock import Mock

import test_prepared_stream as fixtures
from osmo360_camera import _prepared_stream as m


class PreparedSleepBudgetTests(unittest.TestCase):
    setUp = fixtures.PreparedTests.setUp
    backend = fixtures.PreparedTests.backend

    def asleep(self):
        self.assets.preload_assets()
        self.control.state, self.control.sequence = "SLEEP", 1
        self.control.commands.clear()
        self.events.clear()

    def test_requires_preloaded_immutable_assets_and_exact_existing_sleep_without_new_upload(
        self,
    ):
        self.control.state, self.control.sequence = "SLEEP", 1
        with self.assertRaises(m.PreparedStreamError):
            self.backend().prepare_after_sleep()
        self.assertEqual(self.control.commands, [])
        self.control.bootstrap.query_runtime.assert_not_called()
        self.control.state, self.control.sequence = "AWAKE", 0
        self.asleep()
        for state, sequence in (("AWAKE", 0), ("SLEEP", 3), ("SLEEPING", 1)):
            self.control.state, self.control.sequence = state, sequence
            with (
                self.subTest(state=state, sequence=sequence),
                self.assertRaises(m.PreparedStreamError),
            ):
                self.backend().prepare_after_sleep()
        self.assertEqual(self.control.commands, [])
        self.control.bootstrap.query_runtime.assert_not_called()

    def test_strict_native_runtime_query_receives_budget_and_no_capture_or_apply_starts(
        self,
    ):
        self.asleep()
        guard = Mock()
        value = self.backend().prepare_after_sleep(guard=guard)
        self.control.bootstrap.query_runtime.assert_called_once_with(budget=guard)
        self.assertTrue(guard.called)
        self.assertTrue(value.prepared)
        self.assertFalse(value.possible_future_apply)
        self.assertFalse(value.preview._scheduled)
        self.assertIsNone(self.camera._usb_preview)
        self.assertIsNone(self.camera._imu)
        self.assertFalse(
            any(
                name in self.events
                for name in ("lease_ack", "imu_start", "video_start")
            )
        )
        self.assertFalse(
            any(command.endswith(" launch") for command in self.control.commands)
        )

    def test_runtime_already_in_high_speed_overlay_is_rejected_before_any_plan_write(
        self,
    ):
        self.asleep()
        self.control.bootstrap.query_runtime.return_value = replace(
            fixtures.runtime(), original_max_speed="high-speed"
        )
        value = self.backend()
        with self.assertRaises(ValueError):
            value.prepare_after_sleep(guard=lambda: None)
        self.assertEqual(self.control.commands, [])
        self.assertFalse(value.prepared)
        self.assertIsNone(value.plan)
        self.assertFalse(value.possible_future_apply)

    def test_runtime_returning_after_budget_cannot_start_plan_upload_or_claim_prepared(
        self,
    ):
        self.asleep()
        now = [0.0]

        def guard():
            if now[0] >= 1:
                raise TimeoutError("prepare budget expired")

        def late_query(**options):
            self.assertIs(options["budget"], guard)
            now[0] = 1
            return fixtures.runtime()

        self.control.bootstrap.query_runtime.side_effect = late_query
        value = self.backend()
        with self.assertRaisesRegex(TimeoutError, "budget expired"):
            value.prepare_after_sleep(guard=guard)
        self.assertEqual(self.control.commands, [])
        self.assertFalse(value.prepared)
        self.assertIsNone(value.preview)
        self.assertIsNone(value.imu)

    def test_late_plan_command_stops_all_later_starts_and_does_not_publish_ready_or_retry(
        self,
    ):
        self.asleep()
        expired = [False]

        def guard():
            if expired[0]:
                raise TimeoutError("late plan write")

        def late(command):
            expired[0] = True

        self.control.failure = late
        value = self.backend()
        with self.assertRaisesRegex(TimeoutError, "late plan write"):
            value.prepare_after_sleep(guard=guard)
        self.assertEqual(len(self.control.commands), 1)
        self.assertFalse(value.prepared)
        self.assertIsNone(value.preview)
        self.assertIsNone(value.imu)
        self.assertFalse(value.possible_future_apply)
        attempted = list(self.control.commands)
        with self.assertRaises(m.PreparedStreamError):
            value.prepare_after_sleep(guard=lambda: None)
        self.assertEqual(self.control.commands, attempted)
        self.assertFalse(
            any(
                "manifest.json" in command or command.endswith(" launch")
                for command in attempted
            )
        )

    def test_guard_rechecked_after_pending_work_before_first_possible_plan_start(self):
        self.asleep()
        expired = [False]

        def guard():
            if expired[0]:
                raise TimeoutError("pending work used budget")

        original = self.control._run

        def pending_then_send(command, **options):
            expired[0] = True
            return original(
                command, **options
            )  # Fixture invokes actual pre_send before recording a START.

        self.control._run = pending_then_send
        value = self.backend()
        with self.assertRaisesRegex(TimeoutError, "pending work used budget"):
            value.prepare_after_sleep(guard=guard)
        self.assertEqual(self.control.commands, [])
        self.assertFalse(value.prepared)
        self.assertIsNone(value.preview)
        self.assertFalse(value.possible_future_apply)


if __name__ == "__main__":
    unittest.main()
