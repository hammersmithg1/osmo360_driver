"""Boot waiting uses only read-only GETs and fake USB/time."""

from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import Camera, CameraError
from osmo360_camera.controls import CameraReplyError, ControlDecodeError
from osmo360_camera.transport import USBError, CameraDisconnected, RequestTimeout


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class BootTransport:
    def __init__(self, clock):
        self.clock = clock
        self.connected = False
        self.open_outcomes = deque()
        self.state_outcomes = deque()
        self.mode_outcomes = deque()
        self.open_calls = self.close_calls = 0
        self.requests = []
        self.recording_state = 0
        self.mode = "standard_state - liveview"
        self.close_error = None

    def open(self):
        self.open_calls += 1
        if self.open_outcomes:
            raise self.open_outcomes.popleft()
        self.connected = True
        return self

    def close(self):
        self.close_calls += 1
        self.connected = False
        if self.close_error:
            raise self.close_error

    def request(self, command_set, command, payload=b"", *, receiver=1, timeout=2):
        request = (command_set, command, payload, receiver)
        self.requests.append(request)
        if not self.connected:
            raise CameraDisconnected("fake closed")
        if request == (2, 0x70, b"", 1):
            outcomes = self.state_outcomes
            default = (
                b"\0" + (2 | self.recording_state << 6).to_bytes(4, "little") + b"\x38"
            )
        elif request == (0, 0x44, b"\x86\0\0", 0x28):
            outcomes = self.mode_outcomes
            default = b"\0" + (self.mode.encode() + b"\0").ljust(64, b"\0")
        else:
            raise AssertionError("Only state/system-mode GETs are allowed")
        result = outcomes.popleft() if outcomes else default
        if callable(result):
            result = result(timeout)
        if isinstance(result, Exception):
            raise result
        return SimpleNamespace(payload=result)


class CameraConnectTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.transport = BootTransport(self.clock)
        self.camera = Camera(transport=self.transport)
        self.time_patch = patch("osmo360_camera.camera.time", self.clock)
        self.time_patch.start()
        self.usb_patch = patch(
            "osmo360_camera.camera.USBTransport",
            side_effect=AssertionError("Actual USB forbidden"),
        )
        self.usb_patch.start()

    def tearDown(self):
        self.usb_patch.stop()
        self.time_patch.stop()

    def test_default_connect_preserves_immediate_open_without_gets_or_retries(self):
        self.assertIs(self.camera.connect(), self.camera)
        self.assertEqual((self.transport.open_calls, self.transport.requests), (1, []))
        self.transport.open_outcomes.append(CameraDisconnected("missing"))
        with self.assertRaises(CameraDisconnected):
            self.camera.connect()
        self.assertEqual(self.transport.open_calls, 2)
        self.assertEqual(self.clock.now, 0)

    def test_absent_then_enumerating_then_both_services_ready(self):
        self.transport.open_outcomes.extend(
            [CameraDisconnected("not enumerated"), USBError("interface not ready")]
        )
        self.assertIs(self.camera.connect(timeout=600), self.camera)
        self.assertEqual(self.transport.open_calls, 3)
        self.assertEqual(self.transport.close_calls, 0)
        self.assertEqual(
            self.transport.requests, [(2, 0x70, b"", 1), (0, 0x44, b"\x86\0\0", 0x28)]
        )
        self.assertAlmostEqual(self.clock.now, 0.4)

    def test_get_timeouts_retain_the_live_handle_and_retry_only_gets(self):
        self.transport.state_outcomes.append(RequestTimeout("camera service booting"))
        self.transport.mode_outcomes.append(RequestTimeout("system service booting"))
        self.assertIs(self.camera.connect(timeout=5), self.camera)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (1, 0)
        )
        self.assertEqual(
            [r[1] for r in self.transport.requests], [0x70, 0x70, 0x44, 0x70, 0x44]
        )

    def test_boot_e0_then_valid_replies_retry_both_services_on_the_same_handle(self):
        # Actual post-reboot hardware49 returned E0 before initialization.
        self.transport.state_outcomes.append(b"\xe0")
        self.transport.mode_outcomes.append(b"\xe0")
        self.assertIs(self.camera.connect(timeout=5), self.camera)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (1, 0)
        )
        self.assertEqual(
            self.transport.requests,
            [
                (2, 0x70, b"", 1),
                (2, 0x70, b"", 1),
                (0, 0x44, b"\x86\0\0", 0x28),
                (2, 0x70, b"", 1),
                (0, 0x44, b"\x86\0\0", 0x28),
            ],
        )
        self.assertAlmostEqual(self.clock.now, 0.4)

    def test_repeated_boot_e0_obeys_deadline_closes_owned_handle_and_retains_cause(
        self,
    ):
        self.transport.state_outcomes.extend([b"\xe0"] * 20)
        with self.assertRaises(CameraError) as caught:
            self.camera.connect(timeout=0.5)
        error = caught.exception
        self.assertIsInstance(error.last_error, CameraReplyError)
        self.assertEqual(
            (error.last_error.status, error.last_error.payload), (0xE0, b"\xe0")
        )
        self.assertIs(error.__cause__, error.last_error)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (1, 1)
        )
        self.assertFalse(self.transport.connected)
        self.assertEqual(self.transport.requests, [(2, 0x70, b"", 1)] * 3)
        self.assertEqual(self.clock.now, 0.5)

    def test_existing_external_recording_and_nonliveview_are_not_changed(self):
        self.transport.connected = True
        self.transport.recording_state = 2
        self.transport.mode = "other_state - playback"
        self.assertIs(self.camera.connect(timeout=1), self.camera)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (0, 0)
        )
        self.assertEqual(self.transport.recording_state, 2)
        self.assertFalse(self.camera._record_owned)

    def timeout_get(self, timeout):
        self.clock.sleep(timeout)
        return RequestTimeout("boot service did not respond")

    def test_timeout_closes_only_connection_opened_here_and_retains_last_error(self):
        self.transport.state_outcomes.append(self.timeout_get)
        with self.assertRaises(CameraError) as caught:
            self.camera.connect(timeout=1)
        self.assertIsInstance(caught.exception.last_error, RequestTimeout)
        self.assertIs(caught.exception.__cause__, caught.exception.last_error)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (1, 1)
        )
        self.assertFalse(self.transport.connected)
        self.assertEqual(self.transport.requests, [(2, 0x70, b"", 1)])
        self.assertEqual(self.clock.now, 1)

    def test_timeout_preserves_preexisting_connection(self):
        self.transport.connected = True
        self.transport.state_outcomes.append(self.timeout_get)
        with self.assertRaises(CameraError):
            self.camera.connect(timeout=1)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (0, 0)
        )
        self.assertTrue(self.transport.connected)

    def test_absence_timeout_has_no_handle_to_close(self):
        self.transport.open_outcomes.extend(
            CameraDisconnected("absent") for _ in range(20)
        )
        with self.assertRaises(CameraError) as caught:
            self.camera.connect(timeout=0.5)
        self.assertIsInstance(caught.exception.last_error, CameraDisconnected)
        self.assertEqual((self.transport.close_calls, self.transport.requests), (0, []))
        self.assertEqual(self.clock.now, 0.5)

    def test_malformed_or_rejected_reply_is_not_retried(self):
        for payload, error_type in (
            (b"\0\0", ControlDecodeError),
            (b"\xdf", CameraReplyError),
        ):
            with self.subTest(payload=payload):
                transport = BootTransport(self.clock)
                transport.state_outcomes.append(payload)
                with self.assertRaises(error_type):
                    Camera(transport=transport).connect(timeout=10)
                self.assertEqual((transport.open_calls, transport.close_calls), (1, 1))
                self.assertEqual(transport.requests, [(2, 0x70, b"", 1)])
                self.assertEqual(self.clock.now, 0)

    def test_late_wellformed_reply_does_not_establish_readiness(self):
        def late_mode(timeout):
            self.clock.sleep(timeout)
            return b"\0" + b"standard_state - liveview\0".ljust(64, b"\0")

        self.transport.mode_outcomes.append(late_mode)
        with self.assertRaises(CameraError):
            self.camera.connect(timeout=1)
        self.assertEqual(self.transport.close_calls, 1)

    def test_configuration_error_is_not_treated_as_temporary_usb_absence(self):
        error = RuntimeError("libusb not installed")
        self.transport.open_outcomes.append(error)
        with self.assertRaises(RuntimeError) as caught:
            self.camera.connect(timeout=10)
        self.assertIs(caught.exception, error)
        self.assertEqual(
            (self.transport.open_calls, self.transport.close_calls), (1, 0)
        )
        self.assertEqual(self.clock.now, 0)

    def test_failure_cleanup_does_not_mask_protocol_error(self):
        self.transport.mode_outcomes.append(b"\0bad")
        self.transport.close_error = RuntimeError("cleanup failed")
        with self.assertRaises(ControlDecodeError) as caught:
            self.camera.connect(timeout=10)
        self.assertIs(caught.exception.camera_cleanup_error, self.transport.close_error)
        self.assertEqual(self.transport.close_calls, 1)

    def test_invalid_timeout_fails_before_usb_access(self):
        for invalid in (True, False, 0, -1, float("nan"), float("inf"), "10", object()):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.camera.connect(timeout=invalid)
        self.assertEqual((self.transport.open_calls, self.transport.requests), (0, []))


if __name__ == "__main__":
    unittest.main()
