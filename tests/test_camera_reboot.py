"""Verified warm-reboot lifecycle with fake USB/time; never touches a camera."""

from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import Camera, RebootError, RebootResult
from osmo360_camera.imu import IMUStopError
from osmo360_camera.transport import CameraDisconnected, RequestTimeout


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class RebootTransport:
    def __init__(self, clock):
        self.clock = clock
        self.connected = False
        self.link_speed = "fakeUSB"
        self.failure = None
        self.operations = []
        self.requests = []
        self.receipt = b"\x01"  # Actual post-update-usb-reboot-1.json receipt.
        self.receipt_error = None
        self.observe_disconnect = True
        self.disconnect_delay = 0.1
        self.available_after = 0.5
        self.disconnected = False
        self.disconnect_evidence = None
        self.power_sent_at = None
        self.recording_state = 0
        self.state_outcomes = deque()
        self.mode_outcomes = deque()

    def open(self):
        self.operations.append(("open", self.clock.now))
        if (
            self.power_sent_at is not None
            and self.clock.now < self.power_sent_at + self.available_after
        ):
            raise CameraDisconnected("camera has not enumerated yet")
        self.connected = True
        self.disconnected = False  # Same as real transport.open().
        self.disconnect_evidence = None
        return self

    def close(self):
        self.operations.append(("close", self.clock.now))
        self.connected = False

    def request(
        self, command_set, command, payload=b"", *, receiver=1, timeout=2, ack_type=2
    ):
        self.requests.append((command_set, command, payload, receiver, ack_type))
        self.operations.append(("request", command_set, command, payload))
        if not self.connected:
            raise CameraDisconnected("fake USB closed")
        if (command_set, command, payload, receiver) == (0, 0x44, b"\x82\x06", 0x28):
            self.power_sent_at = self.clock.now
            if self.receipt_error:
                raise self.receipt_error
            return SimpleNamespace(payload=self.receipt)
        if (command_set, command, receiver) == (2, 0x70, 1):
            value = (
                self.state_outcomes.popleft()
                if self.state_outcomes
                else self.recording_state
            )
            if isinstance(value, Exception):
                raise value
            return SimpleNamespace(
                payload=b"\0" + (2 | value << 6).to_bytes(4, "little") + b"\1"
            )
        if (command_set, command, payload, receiver) == (0, 0x44, b"\x86\0\0", 0x28):
            value = (
                self.mode_outcomes.popleft()
                if self.mode_outcomes
                else "standard_state - liveview"
            )
            if isinstance(value, Exception):
                raise value
            if isinstance(value, bytes):
                return SimpleNamespace(payload=value)
            return SimpleNamespace(
                payload=b"\0" + (value.encode() + b"\0").ljust(64, b"\0")
            )
        raise AssertionError("Unrecognized fake request; no actual USB fallback")

    def wait_disconnected(self, timeout):
        self.operations.append(("wait_disconnect", timeout))
        if self.observe_disconnect and timeout >= self.disconnect_delay:
            self.clock.sleep(self.disconnect_delay)
            self.connected = False
            self.disconnected = True
            self.disconnect_evidence = {
                "kind": "inventory_absent",
                "bus": 2,
                "address": 3,
            }
            self.operations.append(("observed_disconnect", self.clock.now))
            return True
        self.clock.sleep(timeout)
        return False

    @property
    def power_requests(self):
        return [r for r in self.requests if r[:3] == (0, 0x44, b"\x82\x06")]


class OwnedIMU:
    def __init__(self, operations, *, report=None, error=None, started=True):
        self.operations = operations
        self.pid = 123 if started else None
        self.report = (
            report
            if report is not None
            else ({"restored": True, "quiescent": True, "errors": []} if started else None)
        )
        self.error = error
        self.calls = 0

    def stop(self, timeout=None):
        self.calls += 1
        self.operations.append(("imu_stop", timeout))
        if self.error:
            raise self.error
        return self.report


class CameraRebootTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.transport = RebootTransport(self.clock)
        self.time_patch = patch("osmo360_camera.camera.time", self.clock)
        self.time_patch.start()
        self.camera = Camera(transport=self.transport).connect()

    def tearDown(self):
        self.time_patch.stop()

    def assert_sent_once(self):
        self.assertEqual(
            self.transport.power_requests, [(0, 0x44, b"\x82\x06", 0x28, 1)]
        )

    def test_receipt01_requires_disconnect_reconnect_and_idle_liveview(self):
        imu = OwnedIMU(self.transport.operations)
        self.camera._imu = imu
        with patch(
            "osmo360_camera.camera.c.decode_set_ack",
            side_effect=AssertionError("ACK_NOW receipt is not a SET completion"),
        ):
            result = self.camera.reboot(timeout=3)
        self.assertIsInstance(result, RebootResult)
        self.assertEqual(result.receipt, b"\x01")
        self.assertIsNone(result.receipt_error)
        self.assertEqual(
            result.disconnect_evidence,
            {"kind": "inventory_absent", "bus": 2, "address": 3},
        )
        self.assertIsNone(
            self.transport.disconnect_evidence
        )  # Reopen cleared current evidence.
        self.assertEqual(result.state.recording_state, 0)
        self.assertEqual(
            (result.system_mode.state, result.system_mode.scene),
            ("standard_state", "liveview"),
        )
        self.assertGreaterEqual(
            result.reconnect_elapsed, self.transport.available_after
        )
        self.assertTrue(self.camera.connected)
        self.assertIsNone(self.camera._imu)
        self.assertEqual(imu.calls, 1)
        names = [op[0] for op in self.transport.operations]
        self.assertLess(names.index("imu_stop"), names.index("wait_disconnect"))
        self.assertLess(names.index("observed_disconnect"), names.index("close"))
        self.assert_sent_once()

    def test_lost_receipt_can_succeed_only_with_real_disconnect_and_readback(self):
        self.transport.receipt_error = RequestTimeout(
            "receipt lost after delivered reboot"
        )
        result = self.camera.reboot(timeout=3)
        self.assertIsNone(result.receipt)
        self.assertIn("RequestTimeout: receipt lost", result.receipt_error)
        self.assertEqual(result.state.recording_state, 0)
        self.assert_sent_once()

    def test_disconnect_can_arrive_before_receipt(self):
        self.transport.receipt_error = CameraDisconnected(
            "USB disappeared before receipt"
        )
        result = self.camera.reboot(timeout=3)
        self.assertIn("CameraDisconnected:", result.receipt_error)
        self.assert_sent_once()

    def test_receipt_without_observed_disconnect_fails_without_closing_or_resending(
        self,
    ):
        self.transport.observe_disconnect = False
        self.transport.read_failure_status = -5
        self.transport.last_inventory_error = "inventory unavailable"
        with self.assertRaises(RebootError) as caught:
            self.camera.reboot(timeout=1)
        error = caught.exception
        self.assertEqual(error.stage, "disconnect")
        self.assertEqual(error.receipt, b"\x01")
        self.assertFalse(error.disconnect_observed)
        self.assertEqual(error.read_status, -5)
        self.assertEqual(error.last_inventory_error, "inventory unavailable")
        self.assertFalse(any(op[0] == "close" for op in self.transport.operations))
        self.assertEqual(sum(op[0] == "open" for op in self.transport.operations), 1)
        self.assertTrue(self.camera.connected)
        self.assert_sent_once()

    def test_active_record_and_transitions_block_reboot_before_imu_stop(self):
        imu = OwnedIMU(self.transport.operations)
        self.camera._imu = imu
        for record_state in (1, 2, 3):
            self.transport.recording_state = record_state
            with (
                self.subTest(record_state=record_state),
                self.assertRaises(RebootError) as caught,
            ):
                self.camera.reboot(timeout=1)
            self.assertEqual(caught.exception.stage, "preflight")
        self.assertEqual(imu.calls, 0)
        self.assertEqual(self.transport.power_requests, [])

    def test_imu_stop_failure_aborts_and_retains_owned_stream(self):
        cause = IMUStopError("original DIAG mask not restored")
        imu = OwnedIMU(self.transport.operations, error=cause)
        self.camera._imu = imu
        with self.assertRaises(RebootError) as caught:
            self.camera.reboot(timeout=3)
        self.assertEqual(caught.exception.stage, "imu_stop")
        self.assertIs(caught.exception.last_error, cause)
        self.assertIs(caught.exception.__cause__, cause)
        self.assertIs(self.camera._imu, imu)
        self.assertEqual(self.transport.power_requests, [])

    def test_unverified_imu_report_also_blocks_power_command(self):
        imu = OwnedIMU(
            self.transport.operations, report={"restored": False, "errors": []}
        )
        self.camera._imu = imu
        with self.assertRaises(RebootError):
            self.camera.reboot(timeout=3)
        self.assertIs(self.camera._imu, imu)
        self.assertEqual(self.transport.power_requests, [])

    def test_created_but_unstarted_imu_can_be_closed_before_reboot(self):
        self.camera._imu = OwnedIMU(self.transport.operations, started=False)
        self.camera.reboot(timeout=3)
        self.assertIsNone(self.camera._imu)
        self.assert_sent_once()

    def test_reboot_rejects_mask_only_or_false_quiescence_without_power_command(self):
        for proof in (
            {"restored": True, "errors": []},
            {"restored": True, "quiescent": False, "errors": []},
            {"restored": True, "quiescent": 1, "errors": []},
        ):
            with self.subTest(proof=proof):
                imu = self.camera._imu = OwnedIMU(self.transport.operations, report=proof)
                with self.assertRaises(RebootError) as caught:
                    self.camera.reboot(timeout=3)
                self.assertEqual(caught.exception.stage, "imu_stop")
                self.assertIs(self.camera._imu, imu)
                self.assertEqual(self.transport.power_requests, [])

    def test_idle_is_rechecked_after_imu_cleanup(self):
        self.camera._imu = OwnedIMU(self.transport.operations)
        self.transport.state_outcomes.extend([0, 2])
        with self.assertRaises(RebootError) as caught:
            self.camera.reboot(timeout=3)
        self.assertEqual(caught.exception.stage, "preflight")
        self.assertEqual(self.transport.power_requests, [])

    def test_reconnect_timeout_keeps_receipt_cause_and_never_resends_reboot(self):
        self.transport.available_after = 100
        with self.assertRaises(RebootError) as caught:
            self.camera.reboot(timeout=1)
        error = caught.exception
        self.assertEqual(error.stage, "reconnect")
        self.assertTrue(error.disconnect_observed)
        self.assertEqual(error.receipt, b"\x01")
        self.assertIsInstance(error.last_error, CameraDisconnected)
        self.assertFalse(self.camera.connected)
        self.assertGreater(sum(op[0] == "open" for op in self.transport.operations), 2)
        self.assert_sent_once()

    def test_reconnect_is_not_success_without_standard_idle_liveview(self):
        self.transport.mode_outcomes.extend(["standard_state - preparing"] * 10)
        with self.assertRaises(RebootError) as caught:
            self.camera.reboot(timeout=1)
        self.assertEqual(caught.exception.stage, "reconnect")
        self.assertTrue(caught.exception.disconnect_observed)
        self.assertEqual(caught.exception.system_mode.scene, "preparing")
        self.assert_sent_once()

    def test_temporary_readiness_rejection_is_retried_without_power_retry(self):
        self.transport.mode_outcomes.extend(
            [b"\xe0", RequestTimeout("camera daemon booting")]
        )
        result = self.camera.reboot(timeout=3)
        self.assertEqual(result.system_mode.scene, "liveview")
        self.assert_sent_once()

    def test_invalid_timeout_has_no_camera_request(self):
        for timeout in (True, False, 0, -1, float("nan"), float("inf"), "90"):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                self.camera.reboot(timeout=timeout)
        self.assertEqual(self.transport.requests, [])


if __name__ == "__main__":
    unittest.main()
