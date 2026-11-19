"""Serialized policy/shutdown worker commands with fake SDK owners only."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_driver.runtime import OsmoCamera
from test_standby_control import snapshot


class RuntimePowerCommandTests(unittest.TestCase):
    def owner(self, policy=False, live=True):
        self.events = []
        with patch('osmo360_driver.runtime.threading.Thread') as thread:
            thread.return_value.is_alive.return_value = False
            owner = OsmoCamera(usb_charging=policy)
        owner.camera = SimpleNamespace(connected=True, close=Mock(),
            prepare_shutdown=Mock(side_effect=lambda **_: self.events.append('final_sleep_proof') or
                                 dict(ready=True, state='sleeping', local_control_closed=True)))
        owner.stream = SimpleNamespace(state='AWAKE', sleep=Mock(
            side_effect=lambda: self.events.append('sleep') or snapshot('SLEEP', 1)))
        owner.enabled = live
        owner.power = dict(requested=policy, state='prepared', ready=True)
        owner.before_power_restore = lambda: self.events.append('webcam_closed')
        self.lease = SimpleNamespace(receipt={'original': [0, 0, 1000000]},
            restore=Mock(side_effect=lambda **_: self.events.append('old_restore') or
                         dict(ok=True, requested=policy, state='restored', ready=True)))
        owner._power_lease = self.lease if policy is False else None

        def close_stream():
            self.events.append('stream_close')
            owner.stream, owner.enabled = None, False
        owner.close_stream = Mock(side_effect=close_stream)
        owner.stop_readers = Mock(side_effect=lambda: self.events.append('readers_closed'))

        def prepare(**options):
            self.events.append('new_prepare')
            self.assertIsNone(owner._power_lease)
            owner.power = dict(requested=owner.usb_charging, ready=True,
                               state='unmanaged' if owner.usb_charging is None else 'prepared')
        owner.prepare_power = Mock(side_effect=prepare)
        owner.activate = Mock(side_effect=lambda: self.events.append('activate'))
        return owner

    def test_disable_to_keep_restores_original_before_new_policy_and_resumes_live(self):
        owner = self.owner()
        owner.execute('charging_policy', {'policy': 'keep'})
        self.assertIsNone(owner.usb_charging)
        self.assertIsNone(owner._power_lease)
        self.assertEqual(owner.policy_change, dict(state='applied', policy='keep', error=''))
        self.assertLess(self.events.index('webcam_closed'), self.events.index('stream_close'))
        self.assertLess(self.events.index('old_restore'), self.events.index('new_prepare'))
        self.assertLess(self.events.index('new_prepare'), self.events.index('activate'))
        self.lease.restore.assert_called_once_with(verified_rebind=False)
        self.assertEqual(self.lease.receipt, {'original': [0, 0, 1000000]})

    def test_keep_to_disable_closes_webcam_before_policy_preflight_and_does_not_start_idle_camera(self):
        owner = self.owner(policy=None, live=False)
        owner.stream = None
        owner.execute('charging_policy', {'policy': 'disable'})
        self.assertIs(owner.usb_charging, False)
        self.assertLess(self.events.index('webcam_closed'), self.events.index('new_prepare'))
        owner.activate.assert_not_called()
        self.lease.restore.assert_not_called()

    def test_real_unmanaged_prepare_releases_gate_after_keep_policy_without_inventory(self):
        owner = self.owner(live=False)
        owner.prepare_power = OsmoCamera.prepare_power.__get__(owner)
        with patch('osmo360_driver.runtime.attached_camera_count', side_effect=AssertionError('No keep inventory')):
            owner.execute('charging_policy', {'policy': 'keep'})
        self.assertIsNone(owner.power['requested'])
        self.assertTrue(owner.power['ready'])
        self.assertEqual(owner.power['state'], 'unmanaged')

    def test_failed_original_restore_retains_old_lease_policy_and_blocks_new_prepare(self):
        owner = self.owner()
        self.lease.restore.side_effect = RuntimeError('Original policy unknown')
        with self.assertRaisesRegex(RuntimeError, 'Original policy unknown'):
            owner.execute('charging_policy', {'policy': 'keep'})
        self.assertIs(owner._power_lease, self.lease)
        self.assertIs(owner.usb_charging, False)
        owner.prepare_power.assert_not_called()
        owner.activate.assert_not_called()
        self.assertEqual(owner.policy_change['state'], 'error')
        self.assertFalse(owner.power['ready'])

    def test_failed_new_policy_is_visible_unready_and_never_restarts_capture(self):
        owner = self.owner(policy=None)
        owner.prepare_power.side_effect = RuntimeError('New policy readback failed')
        with self.assertRaisesRegex(RuntimeError, 'readback failed'):
            owner.execute('charging_policy', {'policy': 'disable'})
        self.assertEqual(owner.policy_change['state'], 'error')
        self.assertFalse(owner.power['ready'])
        owner.activate.assert_not_called()

    def test_invalid_policy_is_rejected_before_queue_or_any_camera_callback(self):
        owner = self.owner()
        for settings in (None, {}, {'policy': 'enable'}, {'policy': False}, {'policy': 'keep', 'extra': 1}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                owner.command('charging_policy', settings)
        self.assertTrue(owner.commands.empty())
        self.assertEqual(self.events, [])

    def test_shutdown_closes_stream_and_original_policy_before_final_exact_sleep_proof(self):
        owner = self.owner()
        owner.preparation = dict(boot_id='known-boot')
        proof = owner.prepare_shutdown()
        self.assertTrue(proof['ready'])
        self.assertTrue(owner._shutdown_prepared)
        self.assertEqual(owner.state, 'shutdown_ready')
        self.assertLess(self.events.index('sleep'), self.events.index('stream_close'))
        self.assertLess(self.events.index('stream_close'), self.events.index('old_restore'))
        self.assertLess(self.events.index('old_restore'), self.events.index('final_sleep_proof'))
        self.assertEqual(owner.camera.prepare_shutdown.call_args.kwargs['expected_sleep']['command_id'],
                         '0000000000000001')
        self.assertEqual(owner.camera.prepare_shutdown.call_args.kwargs['expected_boot_id'], 'known-boot')
        with self.assertRaisesRegex(ValueError, 'новые команды'):
            owner.command('start')
        owner.stopping.set()
        owner._final_cleanup = Mock()
        owner.run()
        owner._final_cleanup.assert_not_called()

    def test_shutdown_after_streamoff_uses_existing_sleep_receipt_without_new_input(self):
        owner = self.owner(policy=None, live=False)
        previous = snapshot('SLEEP', 1)
        control = SimpleNamespace(last=previous, wait_sleep_snapshot=Mock(return_value=previous))
        owner.stream.state, owner.stream._core = 'SLEEP', SimpleNamespace(control=control)
        stream = owner.stream
        self.assertTrue(owner.prepare_shutdown()['ready'])
        stream.sleep.assert_not_called()
        control.wait_sleep_snapshot.assert_called_once()

    def test_absent_camera_never_opens_factory_and_never_applies_or_restores_policy(self):
        owner = self.owner(policy=None, live=False)
        owner.stream = owner.camera = None
        owner.camera_factory = Mock(side_effect=AssertionError('No USB open for absence'))
        owner.restore_power = Mock(side_effect=AssertionError('No policy writes for absence'))
        with patch('osmo360_driver.runtime.attached_camera_count', return_value=0):
            self.assertTrue(owner.prepare_shutdown()['ready'])
        owner.camera_factory.assert_not_called()
        owner.restore_power.assert_not_called()
        self.assertTrue(owner.shutdown['complete_usb_inventory'])

    def test_missing_camera_with_dirty_receipt_is_not_shutdown_ready(self):
        owner = self.owner(live=False)
        owner.stream = None
        with patch('osmo360_driver.runtime.attached_camera_count', return_value=0):
            with self.assertRaisesRegex(RuntimeError, 'unrestored'):
                owner.prepare_shutdown()
        self.assertFalse(owner.shutdown['ready'])
        owner.camera.prepare_shutdown.assert_not_called()

    def test_unknown_shutdown_never_claims_ready_or_skips_normal_guarded_cleanup(self):
        owner = self.owner()
        owner.camera.prepare_shutdown.return_value = dict(ready=True, local_control_closed=False)
        owner.camera.prepare_shutdown.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'incomplete'):
            owner.prepare_shutdown()
        self.assertFalse(owner._shutdown_prepared)
        self.assertFalse(owner.shutdown['ready'])
        with self.assertRaisesRegex(RuntimeError, 'no input replay'):
            owner.prepare_shutdown()
        owner.stopping.set()
        owner._final_cleanup = Mock()
        owner.run()
        owner._final_cleanup.assert_called_once_with()

    def test_unknown_final_shutdown_task_never_replays_already_restored_charging_lease(self):
        owner = self.owner()
        def unknown(**options):
            owner.camera._shutdown_control = SimpleNamespace(pending_task=77, _unknown_task=True)
            raise RuntimeError('Final Sleep observation task is unknown')
        owner.camera.prepare_shutdown.side_effect = unknown
        with self.assertRaisesRegex(RuntimeError, 'task is unknown'):
            owner.prepare_shutdown()
        self.assertIsNone(owner._power_lease)
        self.assertEqual(owner.shutdown['charging_restore']['state'], 'restored')
        self.assertFalse(owner.shutdown['ready'])
        owner._final_cleanup()
        self.lease.restore.assert_called_once_with(verified_rebind=False)
        owner.camera.close.assert_called_once_with()
        self.assertEqual(owner.camera._shutdown_control.pending_task, 77)


if __name__ == '__main__':
    unittest.main()
