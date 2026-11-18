"""Driver tests do not open USB or require a camera."""

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import controls
from osmo360_camera.transport import CameraDisconnected, RequestTimeout
from osmo360_camera._standby_control import StandbyUncertain
from osmo360_camera._stream_session import StreamSessionError
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
        owner.camera = SimpleNamespace(connected=True, usb_stream=Mock(return_value=Mock(state='AWAKE')),
            state=Mock(), system_mode=Mock(), install_helpers=Mock(return_value={
                'installed': True, 'already_installed': True, 'autostart_hydration_verified': True,
                'reboot_attempted': False}), **methods)
        owner.camera.usb_stream.return_value.verify_control.return_value = {'state': 'AWAKE'}
        owner.stream, owner.enabled, owner.settings = None, False, {}
        owner.helpers_ready, owner.preparation = True, {}
        owner.start_readers = Mock()
        return owner

    def test_start_lets_sdk_open_a_sleeping_camera_without_service_wait(self):
        owner = self.settings_owner()
        owner.camera.connected = False
        owner.camera.connect = Mock(side_effect=lambda **kwargs: setattr(owner.camera, 'connected', True))
        owner.camera.state.side_effect = RequestTimeout('ordinary services are asleep')
        owner.activate()
        owner.camera.connect.assert_called_once_with(timeout=None)
        owner.camera.install_helpers.assert_not_called()
        owner.stream.start.assert_called_once_with()
        owner.start_readers.assert_called_once_with()

    def test_preparation_uses_public_idempotent_installer_and_requires_autostart_proof(self):
        owner = self.settings_owner()
        owner.helpers_ready = False
        owner.activate()
        owner.camera.install_helpers.assert_called_once_with(timeout=180)
        self.assertTrue(owner.preparation['already_installed'])
        self.assertTrue(owner.helpers_ready)
        owner = self.settings_owner()
        owner.helpers_ready = False
        owner.camera.install_helpers.return_value = {'installed': True, 'autostart_hydration_verified': False}
        with self.assertRaisesRegex(RuntimeError, 'автозапуск'):
            owner.activate()
        owner.camera.usb_stream.assert_not_called()

    def test_stream_toggle_sleeps_and_wakes_the_same_sdk_session(self):
        owner = self.settings_owner()
        owner.helpers_ready = False
        owner.start_readers.side_effect = lambda: setattr(owner, 'enabled', True)
        owner.stop_readers = Mock(side_effect=lambda: setattr(owner, 'enabled', False))
        session = owner.camera.usb_stream.return_value
        session.sleep.side_effect = lambda: setattr(session, 'state', 'SLEEP')
        session.wake.side_effect = lambda: setattr(session, 'state', 'AWAKE')
        owner.activate()
        owner.execute('stop', None)
        self.assertEqual(owner.state, 'sleeping')
        self.assertFalse(owner.enabled)
        owner.activate()
        owner.camera.usb_stream.assert_called_once()
        owner.camera.install_helpers.assert_called_once()
        session.start.assert_called_once()
        session.sleep.assert_called_once()
        session.wake.assert_called_once()
        session.close.assert_not_called()

    def test_explicit_reboot_clears_stale_preparation_then_only_verifies_known_new_boot(self):
        old, new, profile = '11111111-1111-1111-1111-111111111111', '22222222-2222-2222-2222-222222222222', 'a' * 64
        owner = self.settings_owner()
        owner.helpers_ready = False
        owner.camera.install_helpers.return_value.update(boot_id=old, profile=profile,
                                                        installation=dict(boot_id='older-unverified-boot'))
        owner.prepare_helpers()
        self.assertEqual(owner._verified_helpers, dict(boot_id=old, profile=profile))
        owner.close_stream = Mock()
        owner.read_settings = Mock()
        owner.host_offset_ns = 12345
        owner.camera.reboot = Mock()
        owner.execute('reboot', None)
        self.assertFalse(owner.helpers_ready)
        self.assertFalse(owner.preparation['autostart_hydration_verified'])
        self.assertNotIn('boot_id', owner.preparation)
        self.assertEqual(owner.preparation['boot_id_before'], old)
        owner.camera.install_helpers.return_value.update(boot_id=new, boot_id_before=old,
            hydration_wait=dict(mode='after-known-reboot', attempts=2), verification_only=True)
        owner.activate()
        owner.camera.install_helpers.assert_called_with(timeout=180, previous_boot_id=old)
        self.assertEqual(owner._verified_helpers['boot_id'], new)
        self.assertEqual(owner.preparation['boot_id'], new)
        self.assertFalse(owner.preparation['reboot_attempted'])
        owner.camera.reboot.assert_called_once_with(timeout=90)

    def test_known_reboot_verification_cannot_be_silently_skipped_by_ordinary_readiness_probe(self):
        owner = self.settings_owner()
        owner.helpers_ready = False
        previous = '11111111-1111-1111-1111-111111111111'
        owner._helpers_previous_boot_id = previous
        owner.camera.state.side_effect = RequestTimeout('early ordinary GET')
        owner.prepare_helpers()
        owner.camera.install_helpers.assert_called_once_with(timeout=180, previous_boot_id=previous)
        owner.camera.state.assert_not_called()

    def test_real_program_snapshot_with_unset_shutter_starts_readers(self):
        owner = self.settings_owner()
        owner.activate()
        owner.stream.start.assert_called_once_with()
        owner.start_readers.assert_called_once_with()
        self.assertEqual(owner.settings['exposure_mode'], int(controls.ExposureMode.PROGRAM))
        self.assertEqual(owner.settings['shutter_denominator'], 0)
        self.assertEqual(owner.settings['iso'], 0)
        self.assertEqual(validate_settings({'exposure_mode': 1}), {'exposure_mode': 1})

    def test_initial_and_warm_settings_wait_for_public_control_proof_after_readers(self):
        for warm in (False, True):
            with self.subTest(warm=warm):
                owner, events = self.settings_owner(), []
                session = owner.camera.usb_stream.return_value
                if warm:
                    owner.stream, session.state = session, 'SLEEP'
                    def wake():
                        events.append('wake')
                        session.state = 'AWAKE'
                    session.wake.side_effect = wake
                else:
                    session.start.side_effect = lambda: events.append('start')
                def readers():
                    events.append('readers')
                    owner.enabled = True
                owner.start_readers.side_effect = readers
                # Reproduce a stale C1 generation: settings would raise cached
                # LIBUSB_ERROR_IO until the public SDK barrier verifies it.
                ready, original = [], owner.camera.exposure_mode.side_effect
                def verify(**options):
                    self.assertTrue(owner.enabled)
                    events.append('verified')
                    ready.append(True)
                    return {'state': 'AWAKE'}
                def get():
                    if not ready: raise CameraDisconnected('LIBUSB_ERROR_IO')
                    events.append('get')
                    return original()
                session.verify_control.side_effect = verify
                owner.camera.exposure_mode.side_effect = get
                owner.activate()
                self.assertEqual(events, ['wake' if warm else 'start', 'readers', 'verified', 'get'])
                session.verify_control.assert_called_once_with(timeout=20)
                self.assertEqual(owner.settings_error, '')
                self.assertTrue(owner.enabled)
                self.assertEqual(owner.state, 'streaming')
                session.close.assert_not_called()

    def test_failed_or_false_control_proof_is_metadata_warning_without_get_or_capture_stop(self):
        for outcome in (False, None, {}, {'state': 'SLEEP'},
                        CameraDisconnected('LIBUSB_ERROR_IO'), StandbyUncertain('owned Wake task unknown')):
            with self.subTest(outcome=outcome):
                owner = self.settings_owner()
                session = owner.camera.usb_stream.return_value
                if isinstance(outcome, Exception):
                    session.verify_control.side_effect = outcome
                else:
                    session.verify_control.return_value = outcome
                owner.start_readers.side_effect = lambda: setattr(owner, 'enabled', True)
                owner.activate()
                self.assertTrue(owner.enabled)
                self.assertEqual(owner.state, 'streaming')
                self.assertEqual(owner.settings, {})
                self.assertIn('Не удалось подтвердить USB-управление', owner.settings_error)
                owner.camera.exposure_mode.assert_not_called()
                session.close.assert_not_called()
                session.sleep.assert_not_called()
                session.recover.assert_not_called()
                session.verify_control.assert_called_once_with(timeout=20)
                # A refresh may obtain the real proof later. It must use the
                # same barrier instead of directly reconnecting around it.
                owner.camera.connected = False
                owner.camera.connect = Mock(side_effect=AssertionError('connect bypass'))
                session.verify_control.side_effect = None
                session.verify_control.return_value = {'state': 'AWAKE'}
                owner.execute('refresh', None)
                owner.camera.connect.assert_not_called()
                self.assertEqual(session.verify_control.call_count, 2)
                self.assertEqual(owner.settings_error, '')
                self.assertTrue(owner.settings)
                self.assertTrue(owner.enabled)
                self.assertEqual(owner.state, 'streaming')

    def test_strict_control_warning_preserves_cause_and_sleep_refresh_never_wakes(self):
        owner = self.settings_owner()
        owner.stream = owner.camera.usb_stream.return_value
        failure = StandbyUncertain('running prior task')
        owner.stream.verify_control.side_effect = failure
        with self.assertRaisesRegex(RuntimeError, 'USB-управление') as caught:
            owner.read_settings()
        self.assertIs(caught.exception.__cause__, failure)
        owner.camera.exposure_mode.assert_not_called()
        owner.stream.state, owner.state = 'SLEEP', 'sleeping'
        owner.camera.connected = False
        owner.camera.connect = Mock()
        owner.stream.verify_control.reset_mock()
        for read in (lambda: owner.execute('refresh', None), lambda: owner.read_settings(strict=False)):
            with self.assertRaisesRegex(ValueError, 'разбуди'):
                read()
        owner.stream.verify_control.assert_not_called()
        owner.camera.connect.assert_not_called()
        owner.stream.wake.assert_not_called()

    def test_failed_refresh_control_proof_keeps_capture_and_never_issues_settings_get(self):
        owner = self.settings_owner()
        owner.stream = owner.camera.usb_stream.return_value
        owner.state, owner.enabled = 'streaming', True
        owner.stream.verify_control.side_effect = CameraDisconnected('LIBUSB_ERROR_IO')
        owner.camera.connected = False
        owner.camera.connect = Mock(side_effect=AssertionError('connect bypass'))
        owner.execute('refresh', None)
        self.assertTrue(owner.enabled)
        self.assertEqual(owner.state, 'streaming')
        self.assertEqual(owner.settings, {})
        self.assertIn('USB-управление', owner.settings_error)
        owner.camera.exposure_mode.assert_not_called()
        owner.camera.connect.assert_not_called()
        owner.stream.close.assert_not_called()

    def test_missing_shutter_is_only_tolerated_in_auto_or_program(self):
        for mode, status in ((controls.ExposureMode.MANUAL, 0xDF), (controls.ExposureMode.PROGRAM, 0xE0)):
            with self.subTest(mode=mode, status=status):
                owner = self.settings_owner()
                owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(mode))
                owner.camera.shutter_setting = Mock(side_effect=controls.CameraReplyError(status, bytes([status])))
                with self.assertRaisesRegex(RuntimeError, 'выдержку') as caught:
                    owner.read_settings()
                self.assertIsInstance(caught.exception.__cause__, controls.CameraReplyError)
                owner.activate()
                owner.start_readers.assert_called_once_with()
                self.assertIn('выдержку', owner.settings_error)
                self.assertEqual(owner.settings, {})
        owner = self.settings_owner()
        owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(controls.ExposureMode.AUTO))
        owner.activate()
        owner.start_readers.assert_called_once_with()

    def test_failed_settings_get_does_not_prevent_data_and_refresh_clears_warning(self):
        owner = self.settings_owner()
        def start_readers():
            owner.enabled, owner.state = True, 'streaming'
        owner.start_readers.side_effect = start_readers
        owner.read_settings()
        self.assertTrue(owner.settings)
        native_format = owner.camera.video_format.side_effect

        def rejected_format():
            owner.start_readers.assert_called_once_with()
            raise controls.CameraReplyError(0xDF, b'\xdf')

        owner.camera.video_format.side_effect = rejected_format
        owner.activate()
        self.assertIn('частоту съёмки', owner.settings_error)
        self.assertIn('0xdf', owner.settings_error)
        self.assertEqual(owner.settings, {})
        self.assertTrue(owner.enabled)
        self.assertEqual(owner.state, 'streaming')
        owner.camera.video_format.side_effect = native_format
        owner.execute('refresh', None)
        self.assertEqual(owner.settings_error, '')
        self.assertTrue(owner.enabled)
        self.assertEqual(owner.state, 'streaming')
        self.assertEqual(owner.settings['frame_rate'], int(controls.FrameRate.FPS30))

    def test_unknown_session_is_not_reported_as_asleep_or_retried(self):
        owner = self.settings_owner()
        owner.state = 'error'
        owner.stream = Mock(state='RECOVERY_REQUIRED')
        owner.stop_readers = Mock()
        for action in ('start', 'stop', 'sleep', 'wake', 'reboot', 'refresh', 'configure'):
            with self.subTest(action=action), self.assertRaisesRegex(RuntimeError, 'RECOVERY_REQUIRED'):
                owner.execute(action, {})
        self.assertEqual(owner.state, 'error')
        owner.stream.sleep.assert_not_called()
        owner.stream.wake.assert_not_called()
        owner.stream.close.assert_not_called()
        owner.stop_readers.assert_not_called()
        owner.camera.usb_stream.assert_not_called()

    def test_manual_shutter_value_is_preserved(self):
        owner = self.settings_owner()
        owner.camera.exposure_mode = Mock(return_value=controls.ExposureModeSetting(controls.ExposureMode.MANUAL))
        owner.camera.shutter_setting = Mock(return_value=controls.ShutterSpeed(1, controls.ShutterValue(250, True)))
        owner.activate()
        self.assertEqual(owner.settings['shutter_denominator'], 250)
        self.assertEqual(owner.settings['exposure_mode'], int(controls.ExposureMode.MANUAL))

    def test_optional_get_does_not_hide_original_set_failure(self):
        owner = self.settings_owner()
        owner.close_stream = Mock()
        owner.camera.set_color = Mock(side_effect=RuntimeError('SET color rejected'))
        owner.camera.video_format.side_effect = controls.CameraReplyError(0xDF, b'\xdf')
        with self.assertRaisesRegex(RuntimeError, 'SET color rejected'):
            owner.apply_settings({'color': 0})
        self.assertIn('частоту съёмки', owner.settings_error)

    def test_verified_set_resumes_stream_despite_unrelated_metadata_failure(self):
        owner = self.settings_owner()
        owner.enabled = True
        owner.close_stream = Mock(side_effect=lambda: setattr(owner, 'enabled', False))
        owner.camera.set_color = Mock()
        owner.camera.video_format.side_effect = controls.CameraReplyError(0xDF, b'\xdf')
        owner.activate = Mock()
        owner.apply_settings({'color': 0})
        owner.camera.set_color.assert_called_once()
        owner.activate.assert_called_once()
        self.assertIn('частоту съёмки', owner.settings_error)

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

    def test_failed_start_retains_sdk_report_for_diagnosis(self):
        report = {'report': {'state': 'RECOVERY_REQUIRED', 'errors': ['initial preview failed']}}
        session = SimpleNamespace(start=Mock(side_effect=RuntimeError('preview failed')), stats=report, close=Mock())
        camera = SimpleNamespace(connected=True, state=Mock(side_effect=RequestTimeout('sleeping')),
                                 usb_stream=Mock(return_value=session), close=Mock())
        owner = OsmoCamera(camera_factory=lambda: camera)
        try:
            owner.command('start')
            deadline = time.monotonic() + 2
            while owner.status()['busy'] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(owner.status()['state'], 'error')
            self.assertEqual(owner.status()['stats'], report)
            self.assertIn('preview failed', owner.status()['error'])
        finally:
            owner.close()

    def test_usb_loss_and_failed_cleanup_keep_final_sdk_report_and_ownership(self):
        final = dict(state='RECOVERY_REQUIRED', cleanup_deferred=True, last_imu_stop=dict(
            restored=False, quiescent=False, errors=['DIAG read: LIBUSB_ERROR_NO_DEVICE',
                                                      'rebind restore: KeyboardInterrupt']))
        error = StreamSessionError('Session retains ownership after deferred cleanup', final)
        error.__cause__ = RuntimeError('DIAG mask restoration not proven')

        class Session:
            state = 'AWAKE'
            @property
            def stats(self):
                return dict(state=self.state, preview=dict(healthy=self.state == 'AWAKE'),
                    imu=dict(received=1200217, collector_live=dict(fatal_read_status=-4),
                             cleanup=None if self.state == 'AWAKE' else dict(restored=False, quiescent=False)),
                    report=dict(state='AWAKE', errors=[]))
            def close(self):
                self.state = 'RECOVERY_REQUIRED'
                raise error

        owner = OsmoCamera()
        owner.stream, owner.camera = Session(), SimpleNamespace(close=Mock())
        session, camera = owner.stream, owner.camera
        try:
            owner.failed(RuntimeError('Osmo 360 не передаёт IMU более 3 секунд'))
            deadline = time.monotonic() + 2
            while owner.status()['state'] != 'error' and time.monotonic() < deadline:
                time.sleep(.005)
            status = owner.status()
            self.assertEqual(status['state'], 'error')
            self.assertFalse(status['enabled'])
            self.assertIn('LIBUSB_ERROR_NO_DEVICE', status['error'])
            self.assertIn('cleanup: Session retains ownership', status['error'])
            stats = status['stats']
            self.assertEqual(stats['state'], 'RECOVERY_REQUIRED')
            self.assertFalse(stats['preview']['healthy'])
            self.assertEqual(stats['report'], final)
            self.assertEqual(stats['imu']['cleanup'], dict(restored=False, quiescent=False))
            self.assertIn('3 секунд', stats['failure']['capture']['causes'][0]['message'])
            self.assertEqual(stats['failure']['close'][1]['message'], 'DIAG mask restoration not proven')
            self.assertIs(owner.stream, session)
            camera.close.assert_not_called()
            # The published report is copied, so later mutation cannot erase it.
            final['last_imu_stop']['errors'].clear()
            self.assertEqual(len(stats['report']['last_imu_stop']['errors']), 2)
        finally:
            # Drop the test doubles without attempting any further lifecycle.
            owner.stream = owner.camera = None
            owner.close()

    def test_failed_close_preserves_original_error_even_if_diagnostic_getters_fail(self):
        report = dict(state='RECOVERY_REQUIRED', last_imu_stop=dict(restored=False, quiescent=False))
        error = StreamSessionError('original cleanup failure', report)
        for fails_before_close in (False, True):
            with self.subTest(fails_before_close=fails_before_close):
                class Session:
                    closed = False
                    @property
                    def stats(self):
                        if self.closed or fails_before_close:
                            raise error if fails_before_close else RuntimeError('stats unavailable')
                        return dict(state='AWAKE', imu=dict(received=99))
                    @property
                    def state(self):
                        raise RuntimeError('state unavailable')
                    def close(self):
                        self.closed = True
                        raise error
                owner = OsmoCamera()
                session = owner.stream = Session()
                owner.camera = SimpleNamespace(close=Mock())
                camera = owner.camera
                try:
                    with self.assertRaises(StreamSessionError) as caught:
                        owner.close_stream()
                    self.assertIs(caught.exception, error)
                    self.assertEqual(session.closed, not fails_before_close)
                    self.assertEqual(owner.capture_stats['report'], report)
                    self.assertEqual(owner.capture_stats['state'], 'RECOVERY_REQUIRED')
                    self.assertIn('stats_error', owner.capture_stats)
                    self.assertIn('state_error', owner.capture_stats)
                    camera.close.assert_not_called()
                    self.assertIs(owner.stream, session)
                finally:
                    owner.stream = owner.camera = None
                    owner.close()

    def test_stale_collector_failure_does_not_relabel_current_timeout_as_usb_loss(self):
        for current_stats_unavailable in (False, True):
            with self.subTest(current_stats_unavailable=current_stats_unavailable):
                owner = OsmoCamera()
                # Stop only the control worker so the assertion sees the
                # reader's failure before asynchronous cleanup; no USB opened.
                owner.close()
                owner.capture_stats = dict(imu=dict(collector_live=dict(fatal_read_status=-4)))
                if current_stats_unavailable:
                    class Session:
                        state = 'AWAKE'
                        @property
                        def stats(self):
                            raise RuntimeError('current stats unavailable')
                    owner.stream = Session()
                else:
                    owner.stream = SimpleNamespace(state='AWAKE', stats=dict(
                        imu=dict(collector_live=dict(fatal_read_status=None)),
                        report=dict(last_imu_stop=dict(fatal_read_status=-4))))
                message = 'Osmo 360 не передаёт IMU более 3 секунд'
                owner.failed(RuntimeError(message))
                self.assertEqual(owner.capture_error, message)
                self.assertEqual(owner.capture_stats['failure']['capture']['message'], message)
                self.assertEqual('stats_error' in owner.capture_stats, current_stats_unavailable)
                self.assertTrue(owner.reader_stop.is_set())
                self.assertFalse(owner.enabled)

    def test_new_command_failure_refreshes_stats_after_an_earlier_failed_close(self):
        owner = OsmoCamera()
        owner.capture_stats = dict(failure=dict(close=[dict(message='old cleanup failure')]))
        owner._close_failure_sequence = 1
        owner.stream = SimpleNamespace(stats=dict(state='RECOVERY_REQUIRED', current=True))
        owner.execute = Mock(side_effect=RuntimeError('new command failure'))
        try:
            owner.command('refresh')
            deadline = time.monotonic() + 2
            while owner.status()['busy'] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(owner.status()['stats'], owner.stream.stats)
            self.assertEqual(owner.status()['error'], 'new command failure')
        finally:
            owner.stream = None
            owner.close()

    def test_settings_are_validated_before_camera_is_touched(self):
        for values in ({'unknown': 1}, {'iso': True}, {'iso': 123}, {'white_balance': 2030}, {'frame_rate': 999}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_settings(values)
        self.assertEqual(validate_settings({'iso': 400, 'white_balance': 5000}), {'iso': 400, 'white_balance': 5000})
        self.assertEqual(json_value({'calibration': b'\x00\xff'}), {'calibration': {'encoding': 'hex', 'data': '00ff'}})

    def test_missing_video_clock_keeps_live_view_and_following_native_clock(self):
        received, unstamped = [], []
        owner = OsmoCamera(host_offset_ns=5000)
        frames = iter(SimpleNamespace(timestamp_ns=stamp, host_receive_monotonic_ns=arrival,
                                      metadata={}, bgr=None) for stamp, arrival in
                      ((100, 1000), (None, 1500), (200, 2200)))
        owner.stream = SimpleNamespace(stats={}, read_video=lambda **kw: next(frames))
        owner.enabled = True
        def video(frame, jpeg, stamp):
            self.assertEqual(jpeg, b'full jpeg')
            received.append(stamp)
            if len(received) == 2:
                owner.reader_stop.set()
        owner.on_video = video
        owner.on_unstamped_video = lambda frame, jpeg, stamp: unstamped.append(stamp)
        encoder = SimpleNamespace(for_frame=lambda frame: SimpleNamespace(
            full=lambda: b'full jpeg', preview=lambda: b'preview jpeg'))
        try:
            with patch('osmo360_driver.runtime.PreviewEncoder', return_value=encoder):
                owner.video_loop()
            self.assertEqual(received, [6000, 6100])
            self.assertEqual(unstamped, [6500])
            self.assertEqual(owner.timestamp_missing_frames, 1)
            self.assertEqual(owner.frames, 3)
            self.assertEqual(owner.clock.last['video'], 200)
            self.assertEqual(owner.capture_error, '')
            self.assertTrue(owner.enabled)
            self.assertEqual(owner.latest()[1][0], b'preview jpeg')
        finally:
            owner.stream = None
            owner.close()

    def test_lazy_video_callback_precedes_live_preview_without_eager_full_jpeg(self):
        events = []
        frame = SimpleNamespace(timestamp_ns=100, host_receive_monotonic_ns=1000, metadata={})
        full = Mock(return_value=b'full jpeg')
        jpegs = SimpleNamespace(full=full, preview=lambda: events.append('preview') or b'preview jpeg')
        encoder = Mock()
        encoder.for_frame.return_value = jpegs

        def acquire(actual, factory, stamp):
            self.assertIs(actual, frame)
            self.assertIs(factory, full)
            self.assertEqual(stamp, 6000)
            events.append('acquire')
            owner.reader_stop.set()

        legacy = Mock()
        owner = OsmoCamera(on_video=legacy, on_video_lazy=acquire, host_offset_ns=5000)
        owner.stream = SimpleNamespace(stats={}, read_video=lambda **kw: frame)
        owner.enabled = True
        try:
            with patch('osmo360_driver.runtime.PreviewEncoder', return_value=encoder):
                owner.video_loop()
            self.assertEqual(events, ['acquire', 'preview'])
            full.assert_not_called()
            legacy.assert_not_called()
            self.assertEqual(owner.latest()[1][0], b'preview jpeg')
            self.assertEqual(owner.status()['mode'], {'width': 2048, 'height': 1024, 'fps': 30})
            self.assertEqual(owner.status()['preview'], {'width': 1024, 'height': 512, 'jpeg_quality': 75})
        finally:
            owner.stream = None
            owner.close()

    def test_missing_native_clock_uses_lazy_callback_with_mapped_host_time(self):
        frame = SimpleNamespace(timestamp_ns=None, host_receive_monotonic_ns=1500, metadata={})
        full = Mock(return_value=b'full jpeg')
        jpegs = SimpleNamespace(full=full, preview=lambda: b'preview jpeg')
        callback = Mock(side_effect=lambda *_: owner.reader_stop.set())
        owner = OsmoCamera(on_video_lazy=callback, host_offset_ns=5000)
        owner.stream = SimpleNamespace(stats={}, read_video=lambda **kw: frame)
        owner.enabled = True
        try:
            with patch('osmo360_driver.runtime.PreviewEncoder', return_value=SimpleNamespace(for_frame=lambda _: jpegs)):
                owner.video_loop()
            callback.assert_called_once_with(frame, full, 6500)
            full.assert_not_called()
            self.assertEqual(owner.timestamp_missing_frames, 1)
            self.assertEqual(owner.latest()[1][0], b'preview jpeg')
            self.assertNotIn('video', owner.clock.last)
        finally:
            owner.stream = None
            owner.close()

    def test_default_video_callbacks_do_not_request_full_jpeg(self):
        owner = OsmoCamera(host_offset_ns=5000)
        frame = SimpleNamespace(timestamp_ns=100, host_receive_monotonic_ns=1000, metadata={})
        full = Mock(return_value=b'full jpeg')
        jpegs = SimpleNamespace(full=full, preview=lambda: b'preview jpeg')

        def read(**_):
            owner.reader_stop.set()
            return frame

        owner.stream = SimpleNamespace(stats={}, read_video=read)
        owner.enabled = True
        try:
            with patch('osmo360_driver.runtime.PreviewEncoder', return_value=SimpleNamespace(for_frame=lambda _: jpegs)):
                owner.video_loop()
            full.assert_not_called()
            self.assertEqual(owner.frames, 1)
            self.assertEqual(owner.capture_error, '')
            self.assertEqual(owner.latest()[1][0], b'preview jpeg')
        finally:
            owner.stream = None
            owner.close()
