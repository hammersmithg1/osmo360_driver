"""Recovery uses one SDK owner; recording and first-cause evidence survive."""

import queue
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera._standby_control import StandbyUncertain
from osmo360_driver.runtime import OsmoCamera

BEFORE = '11111111-1111-1111-1111-111111111111'
AFTER = '22222222-2222-2222-2222-222222222222'


def raw_frame(stamp=100, key=True):
    return SimpleNamespace(timestamp_ns=stamp, host_receive_monotonic_ns=stamp+1000,
        raw_h264=b'\0\0\0\1\x65test', metadata=dict(h264_keyframe=key,
            h264_codec_headers_complete=True, h264_codec_headers_hex='0000000167'))


class RuntimeRecoveryTests(unittest.TestCase):
    def owner(self, state='RECOVERY_REQUIRED', *, raw=False):
        # Construct all real host state, without starting a worker or opening USB.
        with patch('osmo360_driver.runtime.threading.Thread') as thread:
            thread.return_value.is_alive.return_value = False
            owner = OsmoCamera(on_error=Mock(), on_video_raw=(lambda *_: None) if raw else None)
        self.events = []
        self.old = SimpleNamespace(state=state, stats=dict(state=state))
        self.new = SimpleNamespace(state='AWAKE', stats=dict(state='AWAKE'), start=Mock())
        self.camera = SimpleNamespace(connected=True, close=Mock(), reboot=Mock(),
            usb_stream=Mock(return_value=self.new), prepare_stream=Mock(return_value=dict(
                ready=True, mode='helpers', installed=True, already_installed=True,
                autostart_hydration_verified=True, boot_id=AFTER, profile='a'*64)))
        owner.camera, owner.stream = self.camera, self.old
        owner.read_settings = Mock()
        owner.start_readers = Mock(side_effect=lambda: setattr(owner, 'enabled', True))
        stop = owner.stop_readers
        def stop_readers():
            self.events.append('readers_joined')
            stop()
        owner.stop_readers = Mock(side_effect=stop_readers)
        epoch = owner.clock
        def recover(**options):
            self.events.append('sdk_recovery')
            self.assertIs(owner.clock, epoch)
            self.assertFalse(owner.enabled)
            self.old.state = 'DETACHED'
            return dict(state='DETACHED', new_boot_verified=True, boot_id_before=BEFORE,
                        boot_id_after=AFTER, reboot_attempted=True, local_retired=True, old_boot_extinct=True)
        self.old.recover_and_wait = Mock(side_effect=recover)
        return owner

    def test_start_and_wake_recover_then_create_one_fresh_stream(self):
        for action, state in [('start', 'RECOVERY_REQUIRED'), ('wake', 'RECOVERY_PENDING')]:
            with self.subTest(action=action):
                owner = self.owner(state)
                epoch = owner.clock.epoch
                owner.execute(action, {})
                self.assertEqual(self.events, ['readers_joined', 'sdk_recovery'])
                self.old.recover_and_wait.assert_called_once_with(timeout=120)
                self.assertIs(owner.stream, self.new)
                self.camera.prepare_stream.assert_called_once_with(timeout=180, previous_boot_id=BEFORE)
                self.new.start.assert_called_once()
                self.camera.reboot.assert_not_called()
                self.camera.close.assert_not_called()
                self.assertNotEqual(owner.clock.epoch, epoch)
                self.assertTrue(owner.enabled)
                self.assertEqual(owner.recovery['state'], 'recovered')

    def test_uncertain_recovery_keeps_owner_clock_and_archive_without_extra_usb(self):
        owner = self.owner(raw=True)
        epoch = owner.clock
        owner.archive.append(raw_frame(), 1000)
        cached = owner.archive.status()
        self.old.recover_and_wait.side_effect = StandbyUncertain(
            'Pending D1 task 73 has no terminal RESULT', command_id='0000000000000001')
        with self.assertRaisesRegex(RuntimeError, 'Pending D1 task 73'):
            owner.activate()
        self.assertIs(owner.stream, self.old)
        self.assertIs(owner.clock, epoch)
        self.assertEqual(owner.archive.status(), cached)
        self.camera.usb_stream.assert_not_called()
        self.camera.reboot.assert_not_called()
        self.camera.close.assert_not_called()
        self.camera.prepare_stream.assert_not_called()
        self.old.recover_and_wait.assert_called_once()
        self.assertIn('73', owner.recovery['causes'][0]['message'])

    def test_charging_rebind_is_only_after_complete_recovery_proof(self):
        owner = self.owner()
        owner.usb_charging = False
        def prepare_power(**_):
            self.assertEqual(owner.recovery['state'], 'recovered')
            self.assertIsNone(owner.stream)
            self.assertTrue(owner._power_rebind_allowed)
            self.events.append('power_rebind')
        owner.prepare_power = Mock(side_effect=prepare_power)
        owner.recheck_power = Mock()
        owner.activate()
        self.assertEqual(self.events, ['readers_joined', 'sdk_recovery', 'power_rebind'])
        owner.prepare_power.assert_called_once()
        owner = self.owner()
        owner.usb_charging = False
        owner.prepare_power = Mock()
        self.old.recover_and_wait.side_effect = RuntimeError('pending task 99')
        with self.assertRaisesRegex(RuntimeError, '99'):
            owner.activate()
        owner.prepare_power.assert_not_called()

    def test_false_recovery_receipt_does_not_release_or_restart(self):
        owner = self.owner()
        self.old.recover_and_wait.side_effect = None
        self.old.recover_and_wait.return_value = dict(state='DETACHED', new_boot_verified=False,
                                                     boot_id_before=BEFORE, boot_id_after=AFTER)
        with self.assertRaisesRegex(RuntimeError, 'не подтвердил'):
            owner.activate()
        self.assertIs(owner.stream, self.old)
        self.camera.usb_stream.assert_not_called()
        self.camera.reboot.assert_not_called()

    def test_explicit_reboot_uses_owned_recovery_without_second_power_command(self):
        owner = self.owner()
        old_epoch = owner.clock.epoch
        owner.usb_charging = False
        owner.prepare_power = Mock()
        owner.execute('reboot', {})
        self.old.recover_and_wait.assert_called_once_with(timeout=120)
        self.camera.reboot.assert_not_called()
        self.camera.close.assert_not_called()
        self.camera.usb_stream.assert_not_called()
        owner.prepare_power.assert_called_once_with(verified_rebind=True)
        self.assertIsNone(owner.stream)
        self.assertEqual(owner.state, 'idle')
        self.assertNotEqual(owner.clock.epoch, old_epoch)

    def first_failure(self, *, wake=False):
        owner = self.owner('SLEEP' if wake else 'NEW')
        def fail():
            self.old.state = 'RECOVERY_REQUIRED'
            raise OSError('first capture transition failed')
        if wake:
            self.old.wake = Mock(side_effect=fail)
        else:
            owner.stream = None
            self.old.start = Mock(side_effect=fail)
            self.camera.usb_stream.side_effect = [self.old, self.new]
        def recover(**_):
            self.old.state = 'DETACHED'
            return dict(state='DETACHED', new_boot_verified=True, boot_id_before=BEFORE,
                        boot_id_after=AFTER, reboot_attempted=True, local_retired=True, old_boot_extinct=True)
        self.old.recover_and_wait.side_effect = recover
        return owner

    def test_first_start_or_wake_failure_recovers_once_then_starts_fresh(self):
        for wake in (False, True):
            with self.subTest(wake=wake):
                owner = self.first_failure(wake=wake)
                owner.activate()
                self.old.recover_and_wait.assert_called_once_with(timeout=120)
                self.new.start.assert_called_once()
                self.camera.reboot.assert_not_called()
                self.assertIs(owner.stream, self.new)
                self.assertTrue(owner.enabled)
                self.assertIn('first capture transition failed', owner.previous_capture_failure['capture']['message'])
                self.assertIsNone(owner._capture_failure)

    def test_second_fresh_start_failure_is_retained_without_a_recovery_loop(self):
        owner = self.first_failure()
        self.new.recover_and_wait = Mock()
        def fail_again():
            self.new.state = 'RECOVERY_REQUIRED'
            raise OSError('replacement start failed')
        self.new.start.side_effect = fail_again
        with self.assertRaisesRegex(OSError, 'replacement start failed'):
            owner.activate()
        self.old.recover_and_wait.assert_called_once()
        self.new.recover_and_wait.assert_not_called()
        self.new.start.assert_called_once()
        self.assertIs(owner.stream, self.new)
        self.assertEqual(owner._capture_failure['message'], 'replacement start failed')
        self.assertIn('first capture transition failed', owner.previous_capture_failure['capture']['message'])
        self.camera.reboot.assert_not_called()

    def test_existing_recovery_consumes_the_activation_recovery_budget(self):
        owner = self.owner()
        self.new.recover_and_wait = Mock()
        def fail():
            self.new.state = 'RECOVERY_REQUIRED'
            raise OSError('fresh stream also failed')
        self.new.start.side_effect = fail
        with self.assertRaisesRegex(OSError, 'fresh stream also failed'):
            owner.activate()
        self.old.recover_and_wait.assert_called_once()
        self.new.recover_and_wait.assert_not_called()

    def test_explicit_reboot_can_recover_a_failure_discovered_during_close(self):
        owner = self.owner('AWAKE')
        def close():
            self.old.state = 'RECOVERY_REQUIRED'
            raise RuntimeError('IMU mask unproven')
        owner.close_stream = Mock(side_effect=close)
        owner.execute('reboot', {})
        owner.close_stream.assert_called_once()
        self.old.recover_and_wait.assert_called_once()
        self.camera.reboot.assert_not_called()

    def test_explicit_reboot_recovers_failed_sleep_wake_without_second_reboot(self):
        owner = self.owner('SLEEP')
        def wake():
            self.old.state = 'RECOVERY_REQUIRED'
            raise RuntimeError('wake failed during reboot')
        self.old.wake = Mock(side_effect=wake)
        owner.close_stream = Mock()
        owner.execute('reboot', {})
        self.old.wake.assert_called_once()
        owner.close_stream.assert_not_called()
        self.old.recover_and_wait.assert_called_once_with(timeout=120)
        self.camera.reboot.assert_not_called()
        self.new.start.assert_not_called()
        self.assertIsNone(owner.stream)
        self.assertEqual(owner.state, 'idle')

    def test_capture_failure_idle_loop_never_starts_automatic_reboot(self):
        owner = self.owner()
        owner.capture_error = 'capture lost'
        owner.close_stream = Mock(side_effect=RuntimeError('owner retained'))
        owner.stopping = Mock()
        owner.stopping.is_set.side_effect = [False, True]
        owner.commands.get = Mock(side_effect=queue.Empty)
        owner._final_cleanup = Mock()
        owner.run()
        self.old.recover_and_wait.assert_not_called()
        self.camera.reboot.assert_not_called()
        self.assertEqual(owner.state, 'error')

    def test_queued_activation_closes_failed_awake_producer_before_new_epoch(self):
        owner = self.owner('AWAKE')
        original_clock = owner.clock
        owner.failed(RuntimeError('runtime IMU timeout'))
        def close():
            self.assertIs(owner.clock, original_clock)
            self.assertEqual(owner._capture_failure['message'], 'runtime IMU timeout')
            self.old.state = 'DETACHED'
            return dict(state='DETACHED', native_restored=True)
        self.old.close = Mock(side_effect=close)
        owner.activate()
        self.old.close.assert_called_once()
        self.camera.close.assert_called_once()
        self.old.recover_and_wait.assert_not_called()
        self.new.start.assert_called_once()
        self.assertIs(owner.stream, self.new)
        self.assertIsNot(owner.clock, original_clock)
        self.assertEqual(owner.previous_capture_failure['capture']['message'], 'runtime IMU timeout')

    def test_failed_awake_close_uses_same_activation_recovery_budget(self):
        owner = self.owner('AWAKE')
        owner.failed(RuntimeError('runtime IMU timeout'))
        def close():
            self.old.state = 'RECOVERY_REQUIRED'
            raise RuntimeError('capture cleanup unproven')
        self.old.close = Mock(side_effect=close)
        owner.activate()
        self.old.close.assert_called_once()
        self.old.recover_and_wait.assert_called_once_with(timeout=120)
        self.camera.close.assert_not_called()
        self.camera.reboot.assert_not_called()
        self.new.start.assert_called_once()
        self.assertEqual(owner.previous_capture_failure['capture']['message'], 'runtime IMU timeout')

    def test_recovery_ends_recording_with_explicit_boundary_and_requires_new_archive(self):
        owner = self.owner(raw=True)
        owner.archive.append(raw_frame(), 1000)
        owner.archive.begin(lambda: None)
        owner.activate()
        self.assertFalse(owner.archive.status()['active'])
        self.assertTrue(owner.archive_boundary['continuation_requires_new_archive'])
        self.assertEqual(owner.archive_boundary['archive']['units'], 1)
        self.assertIn('сегмент записи', owner.previous_capture_failure['capture']['message'])
        self.assertEqual(owner.previous_capture_failure['archive_boundary'], owner.archive_boundary)
        owner.on_error.assert_called_once()
        self.assertIsNone(owner._capture_failure)

    def test_transition_cancels_arming_archive_and_preserves_recording_boundary(self):
        owner = self.owner('AWAKE')
        owner.archive = SimpleNamespace(
            status=lambda: dict(active=False, starting=True, error=''),
            end=Mock(return_value=dict(active=False, starting=False, units=0)))
        owner._interrupt_archive('capture epoch changed while awaiting IDR')
        self.assertTrue(owner.archive_boundary['continuation_requires_new_archive'])
        self.assertEqual(owner.archive_boundary['archive']['units'], 0)
        self.assertEqual(owner.archive_boundary['reason'], 'capture epoch changed while awaiting IDR')
        self.assertGreaterEqual(owner.archive.end.call_count, 1)
        owner.on_error.assert_called_once_with('capture epoch changed while awaiting IDR')

    def test_transition_fences_archive_even_if_snapshot_has_no_waiter(self):
        owner = self.owner('AWAKE')
        owner.archive = SimpleNamespace(status=lambda: dict(active=False, starting=False, error=''),
                                       end=Mock())
        owner._interrupt_archive('new epoch')
        owner.archive.end.assert_called_once()
        owner.on_error.assert_not_called()

    def test_first_raw_cause_survives_later_decoder_or_imu_failure(self):
        owner = self.owner('AWAKE', raw=True)
        owner.archive.append(raw_frame(), 1000)
        owner.archive.begin(lambda: None)
        owner.archive.sink = Mock(side_effect=BufferError('raw queue full'))
        with self.assertRaises(BufferError):
            owner.raw_video(raw_frame(101, False))
        first = owner._capture_failure
        self.old.stats = dict(imu=dict(collector_live=dict(fatal_read_status=-4)))
        owner.failed(RuntimeError('secondary IMU timeout'))
        self.assertIs(owner._capture_failure, first)
        self.assertEqual(first['message'], 'raw queue full')
        self.assertEqual(owner.capture_error, 'raw queue full')
        self.assertIn('secondary IMU timeout',
                      owner.capture_stats['failure']['secondary_capture'][0]['causes'][0]['message'])
        owner.on_error.assert_called_once_with('raw queue full')
        exposed = owner.status()
        exposed['stats']['failure']['capture']['message'] = 'consumer mutated copy'
        self.assertEqual(owner._capture_failure['message'], 'raw queue full')
        self.assertEqual(owner.status()['stats']['failure']['capture']['message'], 'raw queue full')

    def test_first_cause_remains_visible_during_recovery_and_after_idle_reboot(self):
        owner = self.owner()
        owner.failed(RuntimeError('first capture failure'))
        def recover(**_):
            self.assertEqual(owner.status()['stats']['failure']['capture']['message'],
                             'first capture failure')
            self.old.state = 'DETACHED'
            return dict(state='DETACHED', new_boot_verified=True, boot_id_before=BEFORE,
                        boot_id_after=AFTER, reboot_attempted=True, local_retired=True,
                        old_boot_extinct=True)
        self.old.recover_and_wait.side_effect = recover
        owner.execute('reboot', {})
        self.assertEqual(owner.status()['stats']['failure']['capture']['message'],
                         'first capture failure')
        self.assertIsNone(owner.previous_capture_failure)

    def test_raw_and_reader_race_notify_once_and_keep_both_distinct_causes(self):
        owner = self.owner('AWAKE', raw=True)
        barrier = threading.Barrier(2)
        def append(*_):
            barrier.wait(2)
            raise BufferError('producer first cause')
        owner.archive = SimpleNamespace(append=append, status=lambda: dict(active=False, error=''))
        class Session:
            state = 'AWAKE'
            @property
            def stats(self):
                barrier.wait(2)
                return {}
        owner.stream = Session()
        def raw():
            try:
                owner.raw_video(raw_frame())
            except BufferError:
                pass
        threads = [threading.Thread(target=raw), threading.Thread(
            target=lambda: owner.failed(RuntimeError('reader first cause')))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        messages = [owner._capture_failure['message'], owner._capture_secondary[0]['message']]
        self.assertCountEqual(messages, ['producer first cause', 'reader first cause'])
        owner.on_error.assert_called_once_with(owner._capture_failure['message'])

    def test_first_raw_failure_before_python_readers_is_never_reset_by_start_readers(self):
        owner = self.owner('AWAKE', raw=True)
        owner._begin_capture_epoch()
        owner.archive.append = Mock(side_effect=BufferError('early AU failed'))
        with self.assertRaises(BufferError):
            owner.raw_video(raw_frame())
        with patch('osmo360_driver.runtime.threading.Thread') as thread:
            with self.assertRaisesRegex(RuntimeError, 'early AU failed'):
                OsmoCamera.start_readers(owner)
        thread.assert_not_called()
        self.assertEqual(owner._capture_failure['message'], 'early AU failed')


if __name__ == '__main__':
    unittest.main()
