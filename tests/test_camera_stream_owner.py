"""Camera cleanup must respect the standby session's independent ownership."""

import threading
import time
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import Camera, CameraError


class CleanupFailure(RuntimeError):
    def __init__(self):
        super().__init__("prepared apply may still start")
        self.report = {"possible_future_apply": True}


class CameraStreamOwnerTests(unittest.TestCase):
    def camera(self):
        return Camera(transport=Mock())

    def test_session_close_can_finish_an_operation_that_needs_camera_control(self):
        camera = self.camera()
        completed = threading.Event()
        session = Mock(cleanup_safe=True)

        def close_session():
            def pending_control():
                with camera._control_lock:
                    completed.set()

            worker = threading.Thread(target=pending_control, daemon=True)
            worker.start()
            worker.join(1)
            self.assertTrue(
                completed.is_set(),
                "Camera.close inverted the session/control lock order",
            )

        session.close.side_effect = close_session
        camera._usb_stream = session
        camera.close()
        self.assertIsNone(camera._usb_stream)
        camera._transport.close.assert_called_once()

    def test_unknown_future_apply_never_falls_through_to_preview_or_imu_reset(self):
        camera = self.camera()
        original = CleanupFailure()
        session = Mock(cleanup_safe=False)
        session.close.side_effect = original
        camera._usb_stream, camera._usb_preview, camera._imu = session, Mock(), Mock()
        with self.assertRaises(CleanupFailure) as caught:
            camera.close()
        self.assertIs(caught.exception, original)
        self.assertIs(camera._usb_stream, session)
        camera._usb_preview.stop.assert_not_called()
        camera._imu.stop.assert_not_called()
        camera._transport.close.assert_called_once()

    def test_an_unsafe_receipt_without_an_exception_still_retains_the_owner(self):
        camera = self.camera()
        session = Mock(cleanup_safe=False)
        camera._usb_stream, camera._usb_preview = session, Mock()
        with self.assertRaises(CameraError):
            camera.close()
        camera._usb_preview.stop.assert_not_called()
        self.assertIs(camera._usb_stream, session)

    def test_uncertain_worker_holding_control_does_not_extend_outer_close(self):
        camera = self.camera()
        original = CleanupFailure()
        session = Mock(cleanup_safe=False)
        session.close.side_effect = original
        camera._usb_stream, camera._usb_preview = session, Mock()
        entered, release = threading.Event(), threading.Event()

        def pending_control():
            with camera._control_lock:
                entered.set()
                release.wait(1)  # Keep a regression failure finite as well.

        worker = threading.Thread(target=pending_control, daemon=True)
        worker.start()
        self.assertTrue(entered.wait(1))
        began = time.monotonic()
        try:
            with self.assertRaises(CleanupFailure) as caught:
                camera.close()
            self.assertIs(caught.exception, original)
            self.assertLess(time.monotonic() - began, 0.3)
            camera._transport.close.assert_not_called()
            camera._usb_preview.stop.assert_not_called()
            self.assertIs(camera._usb_stream, session)
        finally:
            release.set()
            worker.join(1)

    def test_reboot_cannot_bypass_an_active_or_uncertain_session_owner(self):
        camera = self.camera()
        camera._usb_stream = Mock(owns_camera_resources=True)
        camera._usb_preview = Mock()
        with self.assertRaises(CameraError):
            camera.reboot()
        camera._usb_preview.stop.assert_not_called()
        camera._transport.request.assert_not_called()

    def test_public_factory_is_lazy_and_keeps_one_camera_owner(self):
        camera = self.camera()
        stream = Mock()
        with patch(
            "osmo360_camera._stream_resume.build_resumable_stream", return_value=stream
        ) as build:
            self.assertIs(camera.usb_stream(), stream)
            build.assert_called_once_with(
                camera,
                decode=True,
                queue_size=1,
                imu_options=None,
                timeout=60,
                fast_sleep=False,
                retain_ecm=False,
            )
            camera._transport.open.assert_not_called()
            with self.assertRaises(CameraError):
                camera.usb_stream()

    def test_fast_sleep_optin_still_uses_the_same_lazy_public_factory(self):
        camera = self.camera()
        with patch("osmo360_camera._stream_resume.build_resumable_stream") as build:
            self.assertIs(camera.usb_stream(fast_sleep=True), build.return_value)
            self.assertTrue(build.call_args.kwargs["fast_sleep"])
            camera._transport.open.assert_not_called()

    def test_public_factory_rejects_existing_capture_before_building(self):
        for field in ("_imu", "_usb_preview"):
            camera = self.camera()
            setattr(camera, field, Mock())
            with patch("osmo360_camera._stream_resume.build_resumable_stream") as build:
                with self.assertRaises(CameraError):
                    camera.usb_stream()
                build.assert_not_called()

    def test_retained_sleep_optin_is_lazy_and_keeps_the_same_stream_owner(self):
        camera = self.camera()
        with patch("osmo360_camera._stream_resume.build_resumable_stream") as build:
            stream = camera.usb_stream(fast_sleep=True, retain_ecm=True)
            self.assertIs(stream, build.return_value)
            self.assertTrue(build.call_args.kwargs["fast_sleep"])
            self.assertTrue(build.call_args.kwargs["retain_ecm"])
            self.assertIs(camera._usb_stream, stream)
            camera._transport.open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
