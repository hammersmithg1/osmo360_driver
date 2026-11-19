"""Startup selection is read-only until the camera owner is established."""

from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import Camera
from osmo360_camera._startup import StreamStartup, StartupError
from osmo360_camera._standby_control import ExternallySleepingOwnerError
from osmo360_camera._startup_recovery import StartupRecoveryError
from test_helper_install import BOOT1, BOOT2
from test_startup_recovery import externally_asleep, WORK


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.camera = SimpleNamespace(_control_lock=threading.RLock(), _imu=None,
            _usb_preview=None, _usb_stream=None,
            connect=Mock(side_effect=lambda **_: self.events.append('open')),
            ensure_video_mode=Mock(side_effect=lambda **_: self.events.append('video_mode') or dict(ready=True)),
            install_helpers=Mock(side_effect=lambda **_: self.events.append('install') or
                dict(installed=True, already_installed=True, autostart_hydration_verified=True,
                     boot_id='current', profile='package', reboot_attempted=False)))
        self.control = SimpleNamespace(pending_task=None)
        self.manager = SimpleNamespace(ensure=Mock(
            side_effect=lambda **_: self.events.append('resolve_readonly')))
        self.probe = SimpleNamespace(control=self.control,
                                    idle_probe=SimpleNamespace(__self__=self.manager))
        self.startup = StreamStartup(self.camera)
        self.build = patch('osmo360_camera._startup.build_stream_session', return_value=self.probe).start()
        self.discover = patch('osmo360_camera._startup._discover',
            side_effect=lambda _c, **_: self.events.append('discover') or None).start()
        self.status = patch('osmo360_camera._startup._status_at').start()
        self.addCleanup(patch.stopall)

    def test_fresh_camera_installs_only_after_completed_discovery(self):
        report = self.startup.prepare()
        self.assertEqual(self.events, ['open', 'resolve_readonly', 'discover', 'install', 'video_mode'])
        self.assertTrue(report['ready'])
        self.assertEqual(report['mode'], 'helpers')
        self.assertTrue(report['autostart_hydration_verified'])

    def test_retained_sleep_and_awake_skip_ordinary_gets_and_installer(self):
        for state in ('SLEEP', 'AWAKE'):
            with self.subTest(state=state):
                self.camera.install_helpers.reset_mock()
                self.camera.ensure_video_mode.reset_mock()
                self.discover.side_effect = None
                self.discover.return_value = '/dev/shm/exact-owner'
                self.status.return_value = dict(state=state, owner={'pid': 1}, service={'pid': 2})
                report = self.startup.prepare()
                self.assertTrue(report['ready'])
                self.assertEqual(report['mode'], 'retained_owner')
                self.assertEqual(report['retained_owner']['state'], state)
                self.assertNotIn('installed', report)
                self.camera.install_helpers.assert_not_called()
                self.camera.ensure_video_mode.assert_not_called()
        self.build.assert_called_once()  # Same adapter/task history across calls.

    def test_unknown_discovery_never_installs_or_replaces_diagnostic_adapter(self):
        self.discover.side_effect = RuntimeError('unknown or foreign role')
        for _ in range(2):
            with self.assertRaisesRegex(StartupError, 'foreign role'):
                self.startup.prepare()
        self.build.assert_called_once()
        self.assertEqual(self.manager.ensure.call_count, 2)
        self.camera.install_helpers.assert_not_called()

    def test_pending_task_resolution_failure_blocks_even_discovery(self):
        self.manager.ensure.side_effect = RuntimeError('task 123 still running')
        with self.assertRaisesRegex(StartupError, '123'):
            self.startup.prepare()
        self.discover.assert_not_called()
        self.camera.install_helpers.assert_not_called()

    def test_failed_helper_task_is_resolved_before_any_new_discovery(self):
        control = SimpleNamespace(resolve_pending=Mock(side_effect=RuntimeError('task still unknown')))
        failure = RuntimeError('helper upload timed out')
        failure.helper_control = control
        self.camera.install_helpers.side_effect = failure
        with self.assertRaises(StartupError):
            self.startup.prepare()
        self.discover.reset_mock()
        self.camera.install_helpers.reset_mock()
        with self.assertRaisesRegex(StartupError, 'task still unknown'):
            self.startup.prepare()
        control.resolve_pending.assert_called_once()
        self.discover.assert_not_called()
        self.camera.install_helpers.assert_not_called()

    def test_lost_reboot_reply_can_only_verify_known_new_boot_on_next_prepare(self):
        failure = RuntimeError('reboot reply lost')
        failure.report = dict(reboot_attempted=True, boot_id_before='old')
        self.camera.install_helpers.side_effect = failure
        with self.assertRaises(StartupError):
            self.startup.prepare()
        self.discover.reset_mock()
        self.camera.install_helpers.reset_mock()
        self.camera.install_helpers.side_effect = lambda **_: dict(installed=True,
            autostart_hydration_verified=True, boot_id='new', verification_only=True)
        report = self.startup.prepare()
        self.camera.install_helpers.assert_called_once()
        self.assertEqual(self.camera.install_helpers.call_args.kwargs['previous_boot_id'], 'old')
        self.discover.assert_not_called()
        self.assertTrue(report['verification_only'])

    def test_pending_owner_is_not_mistaken_for_stable_sleep(self):
        self.discover.side_effect = None
        self.discover.return_value = '/dev/shm/exact-owner'
        self.status.return_value = dict(state='SLEEP_PENDING')
        with self.assertRaisesRegex(StartupError, 'pending'):
            self.startup.prepare()
        self.camera.install_helpers.assert_not_called()

    def test_manual_sleep_after_host_crash_recovers_once_and_verifies_new_boot_only(self):
        self.discover.side_effect = None
        self.discover.return_value = WORK
        self.status.side_effect = ExternallySleepingOwnerError(externally_asleep())
        self.camera.install_helpers.side_effect = lambda **_: dict(installed=True,
            autostart_hydration_verified=True, boot_id=BOOT2, reboot_attempted=False,
            verification_only=True)
        with patch('osmo360_camera._startup.ExternalSleepRecovery') as factory:
            factory.return_value.run.return_value = dict(reboot_attempted=True,
                boot_id_before=BOOT1, disconnect_observed=True, new_boot_verified=False)
            report = self.startup.prepare()
        self.camera.install_helpers.assert_called_once()
        self.assertEqual(self.camera.install_helpers.call_args.kwargs['previous_boot_id'], BOOT1)
        self.assertTrue(report['reboot_attempted'])
        self.assertTrue(report['recovery']['new_boot_verified'])
        self.assertEqual(report['recovery']['boot_id_after'], BOOT2)
        factory.return_value.run.assert_called_once()

    def test_failed_recovery_reply_or_disconnect_never_requests_another_reboot(self):
        self.discover.side_effect = None
        self.discover.return_value = WORK
        self.status.side_effect = ExternallySleepingOwnerError(externally_asleep())
        old_control = SimpleNamespace(resolve_pending=Mock())
        recovery_report = dict(reboot_attempted=True, boot_id_before=BOOT1,
                               disconnect_observed=False, new_boot_verified=False)
        with patch('osmo360_camera._startup.ExternalSleepRecovery') as factory:
            factory.return_value.report = recovery_report
            factory.return_value.run.side_effect = StartupRecoveryError('USB deadline',
                                                                         recovery_report, old_control)
            with self.assertRaises(StartupError):
                self.startup.prepare()
            self.camera.install_helpers.assert_not_called()
            self.camera.install_helpers.side_effect = lambda **_: dict(installed=True,
                autostart_hydration_verified=True, boot_id=BOOT2, verification_only=True)
            report = self.startup.prepare()
        factory.assert_called_once()
        factory.return_value.run.assert_called_once()
        old_control.resolve_pending.assert_called_once()
        self.camera.install_helpers.assert_called_once()
        self.assertEqual(self.camera.install_helpers.call_args.kwargs['previous_boot_id'], BOOT1)
        self.assertTrue(report['recovery']['new_boot_verified'])

    def test_recovery_different_boot_proof_precedes_any_video_mode_set(self):
        self.discover.side_effect = None
        self.discover.return_value = WORK
        self.status.side_effect = ExternallySleepingOwnerError(externally_asleep())
        self.camera.install_helpers.side_effect = lambda **_: dict(installed=True,
            autostart_hydration_verified=True, boot_id=BOOT1, verification_only=True)
        with patch('osmo360_camera._startup.ExternalSleepRecovery') as factory:
            factory.return_value.run.return_value = dict(reboot_attempted=True,
                boot_id_before=BOOT1, disconnect_observed=True, new_boot_verified=False)
            factory.return_value.report = factory.return_value.run.return_value
            with self.assertRaisesRegex(StartupError, 'different camera boot') as caught:
                self.startup.prepare()
        self.assertFalse(caught.exception.report['ready'])
        self.camera.ensure_video_mode.assert_not_called()

    def test_known_reboot_is_verify_only_and_cannot_skip_hydration(self):
        report = self.startup.prepare(previous_boot_id='old')
        self.assertTrue(report['ready'])
        self.camera.install_helpers.assert_called_once()
        self.assertEqual(self.camera.install_helpers.call_args.kwargs['previous_boot_id'], 'old')
        self.discover.assert_not_called()
        self.camera.install_helpers.return_value = None
        self.camera.install_helpers.side_effect = lambda **_: dict(installed=True,
            autostart_hydration_verified=False)
        with self.assertRaisesRegex(StartupError, 'not verified'):
            self.startup.prepare(previous_boot_id='old')

    def test_photo_mode_failure_keeps_startup_not_ready_without_implicit_reboot(self):
        self.camera.ensure_video_mode.side_effect = RuntimeError('profile readback differs')
        with self.assertRaisesRegex(StartupError, 'readback differs') as caught:
            self.startup.prepare()
        self.assertFalse(caught.exception.report['ready'])
        self.camera.install_helpers.assert_called_once()
        self.camera.ensure_video_mode.assert_called_once()
        self.assertEqual(self.discover.call_count, 1)

    def test_active_capture_owner_blocks_preparation_before_any_usb(self):
        self.camera._usb_stream = SimpleNamespace(owns_camera_resources=True)
        with self.assertRaisesRegex(StartupError, 'closed capture owners'):
            self.startup.prepare()
        self.camera.connect.assert_not_called()
        self.discover.assert_not_called()

    def test_public_camera_method_keeps_one_startup_controller(self):
        camera = Camera(transport=SimpleNamespace())
        with patch('osmo360_camera._startup.StreamStartup') as factory:
            camera.prepare_stream(timeout=30)
            camera.prepare_stream(timeout=40, previous_boot_id='old')
        factory.assert_called_once_with(camera)
        self.assertEqual(factory.return_value.prepare.call_count, 2)
        factory.return_value.prepare.assert_called_with(timeout=40, previous_boot_id='old')


if __name__ == '__main__':
    unittest.main()
