"""Driver tests do not open USB or require a camera."""

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import controls
from osmo360_driver.clock import CameraClock
from osmo360_driver.runtime import OsmoCamera, json_value, validate_settings


class ClockTests(unittest.TestCase):
    def test_video_and_imu_use_one_offset_and_keep_native_intervals(self):
        clock = CameraClock()
        first = clock.stamp(1000, 5000, 'imu')
        video = clock.stamp(1100, 9000, 'video')
        next_imu = clock.stamp(1125, 12000, 'imu')
        self.assertEqual(video - first, 100)
        self.assertEqual(next_imu - first, 125)
        self.assertFalse(clock.status()['hardware_synchronized'])
        self.assertNotEqual(CameraClock().epoch, clock.epoch)

    def test_clock_reset_duplicate_and_missing_timestamp_are_errors(self):
        clock = CameraClock()
        clock.stamp(1000, 5000, 'imu')
        for value in (None, -1, 900, 1000):
            with self.subTest(value=value), self.assertRaises(ValueError):
                clock.stamp(value, 9000, 'imu')

    def test_shared_host_base_survives_starting_a_new_epoch(self):
        first, restarted = CameraClock(12345), CameraClock(12345)
        self.assertEqual(first.stamp(100, 2000, 'video'), restarted.stamp(100, 2000, 'video'))
        self.assertNotEqual(first.epoch, restarted.epoch)


class RuntimeTests(unittest.TestCase):
    def settings_owner(self):
        # Replay the hardware GET snapshot, including Program + shutter DF.
        native = json.loads((Path(__file__).parent / 'fixtures/camera_controls_native.json').read_text())['current_gets']
        owner = OsmoCamera.__new__(OsmoCamera)
        getters = {
            'exposure_mode': controls.decode_exposure_mode, 'iso': controls.decode_iso,
            'white_balance': controls.decode_white_balance, 'shutter': controls.decode_shutter,
            'color': controls.decode_color, 'video_format': controls.decode_video_format,
        }
        methods = {('shutter_setting' if key == 'shutter' else key): Mock(side_effect=lambda key=key, decode=decode: decode(bytes.fromhex(native[key])))
                   for key, decode in getters.items()}
        owner.camera = SimpleNamespace(connected=True, usb_stream=Mock(return_value=Mock()), **methods)
        owner.stream, owner.enabled, owner.settings = None, False, {}
        owner.start_readers = Mock()
        return owner

    def test_real_program_snapshot_with_unset_shutter_starts_readers(self):
        owner = self.settings_owner()
        owner.activate()
        owner.stream.start.assert_called_once_with()
        owner.start_readers.assert_called_once_with()
        self.assertEqual(owner.settings['exposure_mode'], int(controls.ExposureMode.PROGRAM))
        self.assertEqual(owner.settings['shutter_denominator'], 0)
        self.assertEqual(owner.settings['iso'], 0)
        self.assertEqual(validate_settings({'exposure_mode': 1}), {'exposure_mode': 1})

    def test_missing_shutter_is_only_tolerated_in_auto_or_program(self):
        for mode, status in ((controls.ExposureMode.MANUAL, 0xDF), (controls.ExposureMode.PROGRAM, 0xE0)):
            with self.subTest(mode=mode, status=status):
                owner = self.settings_owner()
                owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(mode))
                owner.camera.shutter_setting = Mock(side_effect=controls.CameraReplyError(status, bytes([status])))
                with self.assertRaises(controls.CameraReplyError):
                    owner.activate()
                owner.start_readers.assert_not_called()
        owner = self.settings_owner()
        owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(controls.ExposureMode.AUTO))
        owner.activate()
        owner.start_readers.assert_called_once_with()

    def test_manual_shutter_value_is_preserved(self):
        owner = self.settings_owner()
        owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(controls.ExposureMode.MANUAL))
        owner.camera.shutter_setting = Mock(return_value=controls.ShutterSpeed(1, controls.ShutterValue(250, True)))
        owner.activate()
        self.assertEqual(owner.settings['shutter_denominator'], 250)
        self.assertEqual(owner.settings['exposure_mode'], int(controls.ExposureMode.MANUAL))

    def test_initializing_driver_does_not_touch_usb_and_commands_are_asynchronous(self):
        called, release = threading.Event(), threading.Event()

        class Camera:
            connected = False
            def connect(self, timeout):
                called.set()
                release.wait(2)
                raise RuntimeError('No camera')
            def close(self):
                pass

        owner = OsmoCamera(camera_factory=Camera)
        try:
            self.assertFalse(called.is_set())
            state = owner.command('start')
            self.assertEqual(state['busy'], 'start')
            self.assertTrue(called.wait(1))
            with self.assertRaisesRegex(ValueError, 'Дождись'):
                owner.command('reboot')
            release.set()
            deadline = time.monotonic() + 2
            while owner.status()['busy'] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertIn('No camera', owner.status()['error'])
            self.assertFalse(owner.status()['enabled'])
        finally:
            release.set()
            owner.close()

    def test_lifecycle_does_not_run_until_both_readers_have_exited(self):
        owner = OsmoCamera()
        released = threading.Event()
        owner.readers = [threading.Thread(target=lambda: (owner.reader_stop.wait(), released.set()))]
        owner.readers[0].start()
        session = SimpleNamespace(state='AWAKE', stats={}, sleep=lambda: self.assertTrue(released.is_set()))
        owner.stream = session
        try:
            owner.execute('sleep', None)
            self.assertTrue(released.is_set())
            self.assertEqual(owner.state, 'sleeping')
            self.assertFalse(owner.enabled)
        finally:
            owner.stream = None
            owner.close()

    def test_settings_are_validated_before_camera_is_touched(self):
        for values in ({'unknown': 1}, {'iso': True}, {'iso': 123}, {'white_balance': 2030}, {'frame_rate': 999}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_settings(values)
        self.assertEqual(validate_settings({'iso': 400, 'white_balance': 5000}), {'iso': 400, 'white_balance': 5000})
        self.assertEqual(json_value({'calibration': b'\x00\xff'}), {'calibration': {'encoding': 'hex', 'data': '00ff'}})
