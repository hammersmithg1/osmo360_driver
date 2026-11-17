"""Driver tests do not open USB or require a camera."""

import threading
import time
from types import SimpleNamespace
import unittest

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
