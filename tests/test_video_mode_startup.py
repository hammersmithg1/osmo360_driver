"""Photo-to-video normalization is explicit, verified, idle and pre-capture."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import controls
from osmo360_camera._video_mode import ensure_video_mode


class VideoModeTests(unittest.TestCase):
    def camera(self, *, lens=controls.LensMode.PANORAMIC, profile=56):
        self.lens, self.profile = lens, profile
        camera = SimpleNamespace(_usb_stream=None, _usb_preview=None, _imu=None,
            state=Mock(return_value=SimpleNamespace(recording_state=0)),
            lens_mode=Mock(side_effect=lambda: self.lens),
            mode_profile=Mock(side_effect=lambda: controls.ModeProfileSetting(self.profile)))
        def lens_set(value):
            self.lens, self.profile = value, 5  # Firmware's automatic photo profile.
            return value
        def profile_set(value):
            self.profile = int(value)
            return controls.ModeProfileSetting(self.profile)
        camera.set_lens_mode = Mock(side_effect=lens_set)
        camera.set_mode_profile = Mock(side_effect=profile_set)
        return camera

    def test_photo_panorama_becomes_video_without_unnecessary_lens_switch(self):
        camera = self.camera(profile=5)
        proof = ensure_video_mode(camera)
        self.assertTrue(proof['ready'])
        self.assertEqual(proof['changed'], ['profile'])
        camera.set_lens_mode.assert_not_called()
        camera.set_mode_profile.assert_called_once_with(controls.ModeProfile.VIDEO_PANO)
        self.assertEqual(self.profile, 56)

    def test_single_lens_video_switch_rechecks_auto_photo_profile_before_video_set(self):
        camera = self.camera(lens=controls.LensMode.SINGLE_LENS, profile=1)
        proof = ensure_video_mode(camera)
        self.assertEqual(proof['changed'], ['lens', 'profile'])
        camera.set_lens_mode.assert_called_once_with(controls.LensMode.PANORAMIC)
        camera.set_mode_profile.assert_called_once_with(controls.ModeProfile.VIDEO_PANO)
        self.assertEqual((self.lens, self.profile), (controls.LensMode.PANORAMIC, 56))

    def test_already_panoramic_video_is_read_only_and_verified(self):
        camera = self.camera()
        self.assertEqual(ensure_video_mode(camera)['changed'], [])
        camera.set_lens_mode.assert_not_called()
        camera.set_mode_profile.assert_not_called()
        self.assertGreaterEqual(camera.state.call_count, 2)

    def test_recording_and_existing_capture_block_all_setting_writes(self):
        for resource in ('_usb_preview', '_imu', '_usb_stream', 'recording'):
            camera = self.camera(profile=5)
            if resource == 'recording':
                camera.state.return_value.recording_state = 1
            elif resource == '_usb_stream':
                camera._usb_stream = SimpleNamespace(owns_camera_resources=True)
            else:
                setattr(camera, resource, object())
            with self.subTest(resource=resource), self.assertRaisesRegex(RuntimeError, 'requires'):
                ensure_video_mode(camera)
            camera.set_lens_mode.assert_not_called()
            camera.set_mode_profile.assert_not_called()

    def test_recording_started_between_gets_prevents_profile_write(self):
        camera = self.camera(profile=5)
        camera.state.side_effect = [SimpleNamespace(recording_state=0), SimpleNamespace(recording_state=1)]
        with self.assertRaisesRegex(RuntimeError, 'recording was not stopped'):
            ensure_video_mode(camera)
        camera.set_mode_profile.assert_not_called()

    def test_ack_without_matching_readback_is_not_ready_and_is_not_retried(self):
        camera = self.camera(profile=5)
        camera.set_mode_profile.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'did not confirm'):
            ensure_video_mode(camera)
        camera.set_mode_profile.assert_called_once()

    def test_failed_readonly_profile_query_does_not_authorize_writes(self):
        camera = self.camera(profile=5)
        camera.mode_profile.side_effect = TimeoutError('mode not known')
        with self.assertRaises(TimeoutError):
            ensure_video_mode(camera)
        camera.set_lens_mode.assert_not_called()
        camera.set_mode_profile.assert_not_called()
