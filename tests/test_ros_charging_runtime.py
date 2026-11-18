"""Runtime power gate/lease ordering; fake camera only, no USB or ROS."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import USBChargingError
from osmo360_driver.runtime import OsmoCamera


class ChargingRuntimeTests(unittest.TestCase):
    def owner(self):
        self.events = []
        self.lease = SimpleNamespace(receipt={'original': 'unchanged'},
            prepare=Mock(return_value=dict(requested=False, ready=True, state='prepared', error='')),
            recheck=Mock(return_value=dict(requested=False, ready=True, state='prepared', error='')),
            restore=Mock(return_value=dict(requested=False, ready=False, state='restored', error='')))
        self.camera = SimpleNamespace(connected=True, close=Mock(),
            usb_charging=Mock(return_value=self.lease), usb_stream=Mock(), reboot=Mock())
        owner = OsmoCamera(camera_factory=lambda: self.camera, usb_charging=False)

        def hold():
            self.assertFalse(owner.status()['power']['ready'])
            self.events.append('webcam_released')

        owner.before_power_restore = hold
        self.addCleanup(owner.close)
        return owner

    def session(self, state='NEW'):
        session = SimpleNamespace(state=state, stats={}, _core=SimpleNamespace(control=object()))

        def start():
            self.events.append('sdk_start')
            session.state = 'AWAKE'

        def close():
            self.events.append('sdk_close')
            session.state = 'DETACHED'
            return {}

        session.start, session.wake, session.close = Mock(side_effect=start), Mock(side_effect=start), Mock(side_effect=close)
        session.verify_control = Mock(side_effect=lambda **_: self.events.append('proof') or {'state': 'AWAKE'})
        self.camera.usb_stream.return_value = session
        return session

    def fake_readers(self, owner):
        def start():
            self.events.append('readers')
            owner.enabled = True
        owner.start_readers = Mock(side_effect=start)
        owner.prepare_helpers = Mock()
        owner.read_settings = Mock(side_effect=lambda **_: self.events.append('settings'))

    def test_default_unmanaged_runtime_does_not_inventory_open_or_change_power(self):
        factory = Mock(side_effect=AssertionError('No implicit camera open'))
        owner = OsmoCamera(camera_factory=factory)
        try:
            with patch('osmo360_driver.runtime.attached_camera_count', side_effect=AssertionError('No inventory')):
                owner.prepare_power()
            self.assertEqual(owner.status()['power']['state'], 'unmanaged')
            self.assertTrue(owner.status()['power']['ready'])
            factory.assert_not_called()
        finally:
            owner.close()

    def test_only_complete_absence_allows_webcam_without_opening_camera(self):
        owner = self.owner()
        owner.camera_factory = Mock(side_effect=AssertionError('Absent camera must not open'))
        with patch('osmo360_driver.runtime.attached_camera_count', return_value=0):
            owner.prepare_power()
        status = owner.status()['power']
        self.assertEqual(status['state'], 'absent')
        self.assertTrue(status['ready'])
        self.assertTrue(status['evidence']['complete_usb_inventory'])
        self.assertEqual(self.events, ['webcam_released'])
        self.camera.usb_charging.assert_not_called()

    def test_failed_or_ambiguous_inventory_never_falls_back_to_ready(self):
        for outcome in (2, RuntimeError('partial inventory')):
            with self.subTest(outcome=outcome):
                owner = self.owner()
                with patch('osmo360_driver.runtime.attached_camera_count', **(
                        {'side_effect': outcome} if isinstance(outcome, Exception) else {'return_value': outcome})):
                    with self.assertRaises(Exception):
                        owner.prepare_power()
                self.assertFalse(owner.status()['power']['ready'])
                self.assertTrue(owner.status()['power']['error'])
                self.camera.usb_charging.assert_not_called()

    def test_callback_failure_precedes_inventory_and_every_policy_mutator(self):
        owner = self.owner()
        owner.before_power_restore = Mock(side_effect=RuntimeError('V4L2 still owned'))
        with patch('osmo360_driver.runtime.attached_camera_count') as inventory:
            with self.assertRaisesRegex(RuntimeError, 'V4L2 still owned'):
                owner.prepare_power()
        inventory.assert_not_called()
        self.camera.usb_charging.assert_not_called()
        self.assertFalse(owner.status()['power']['ready'])

    def test_start_holds_webcam_until_posttransition_readonly_check_while_readers_drain(self):
        owner = self.owner()
        self.session()
        self.fake_readers(owner)
        self.lease.prepare.side_effect = lambda **_: self.events.append('prepare') or dict(
            requested=False, ready=True, state='prepared', error='')

        def check(**_):
            self.assertFalse(owner.power['ready'])
            self.assertTrue(owner.enabled)
            self.events.append('recheck')
            return dict(requested=False, ready=True, state='prepared', error='')

        self.lease.recheck.side_effect = check
        with patch('osmo360_driver.runtime.attached_camera_count', return_value=1):
            owner.activate()
        self.assertEqual(self.events, ['webcam_released', 'prepare', 'webcam_released',
                                      'sdk_start', 'readers', 'proof', 'recheck', 'settings'])
        self.assertTrue(owner.status()['power']['ready'])
        self.lease.prepare.assert_called_once()
        self.lease.recheck.assert_called_once()

    def test_sleep_wake_rechecks_without_reapplying_to_active_native_capture(self):
        owner = self.owner()
        owner.camera = self.camera
        owner._power_lease = self.lease
        owner.power = dict(requested=False, ready=True, state='prepared', error='')
        session = owner.stream = self.session('SLEEP')
        self.fake_readers(owner)
        with patch('osmo360_driver.runtime.attached_camera_count', side_effect=AssertionError('No new policy prepare')):
            owner.activate()
        self.assertEqual(self.events[:4], ['webcam_released', 'sdk_start', 'readers', 'proof'])
        self.lease.prepare.assert_not_called()
        self.lease.recheck.assert_called_once_with(control=session._core.control, verified_rebind=True)
        self.assertTrue(owner.power['ready'])

    def test_sleep_holds_webcam_then_checks_native_sleep_and_charge_without_c1_or_set(self):
        owner = self.owner()
        owner.camera = self.camera
        owner._power_lease = self.lease
        owner.power = dict(requested=False, ready=True, state='prepared', error='')
        session = owner.stream = self.session('AWAKE')
        control = SimpleNamespace(last=dict(state='SLEEP', command_id='0000000000000001'))
        control.wait_sleep_snapshot = Mock(return_value=dict(state='SLEEP'))
        session._core.control = control
        session.sleep = Mock(side_effect=lambda: setattr(session, 'state', 'SLEEP'))
        owner.execute('stop', None)
        self.assertEqual(self.events, ['webcam_released'])
        session.sleep.assert_called_once()
        session.verify_control.assert_not_called()
        control.wait_sleep_snapshot.assert_called_once()
        self.lease.prepare.assert_not_called()
        self.lease.restore.assert_not_called()
        self.lease.recheck.assert_called_once_with(control=control, verified_rebind=True)
        self.assertTrue(owner.power['ready'])
        self.assertEqual(owner.power['evidence']['verified_control_state'], 'SLEEP')

    def test_sleep_charge_check_failure_keeps_gate_closed_without_reapplying(self):
        owner = self.owner()
        owner.camera = self.camera
        owner._power_lease = self.lease
        owner.power = dict(requested=False, ready=True, state='prepared', error='')
        session = owner.stream = self.session('SLEEP')
        session._core.control = SimpleNamespace(last=dict(state='SLEEP', command_id='0000000000000001'),
            wait_sleep_snapshot=Mock(return_value=dict(state='SLEEP')))
        self.lease.recheck.side_effect = USBChargingError('Battery current remains positive', dict(ready=False))
        with self.assertRaisesRegex(USBChargingError, 'positive'):
            owner.execute('stop', None)
        self.assertFalse(owner.power['ready'])
        self.lease.prepare.assert_not_called()
        self.lease.restore.assert_not_called()

    def test_internal_capture_close_keeps_policy_until_runtime_shutdown(self):
        owner = self.owner()
        owner.camera = self.camera
        owner._power_lease = self.lease
        owner.stream = self.session('AWAKE')
        owner.power = dict(requested=False, ready=True, state='prepared', error='')
        owner.close_stream()
        self.lease.restore.assert_not_called()
        owner.close()
        self.lease.restore.assert_called_once_with(verified_rebind=True)
        self.assertEqual(self.events, ['webcam_released', 'sdk_close', 'webcam_released'])
        # addCleanup can close again, but the lease must not restore twice.
        owner._power_lease = None

    def test_reset_policy_after_usb_transition_fails_gate_and_closes_capture_without_reapply(self):
        owner = self.owner()
        session = self.session()
        self.fake_readers(owner)
        self.lease.recheck.side_effect = USBChargingError('Policy reset',
            dict(ready=False, state='error', error='Policy reset'))
        with patch('osmo360_driver.runtime.attached_camera_count', return_value=1):
            with self.assertRaisesRegex(USBChargingError, 'Policy reset'):
                owner.activate()
        self.assertFalse(owner.power['ready'])
        self.assertFalse(owner.enabled)
        session.close.assert_called_once()
        self.lease.prepare.assert_called_once()
        self.lease.restore.assert_not_called()

    def test_callback_ready_false_order_is_preserved_on_shutdown_restore(self):
        owner = self.owner()
        owner._power_lease = self.lease
        owner.power = dict(requested=False, state='prepared', ready=True)
        owner.restore_power()
        self.assertEqual(self.events, ['webcam_released'])
        self.lease.restore.assert_called_once()
        owner._power_lease = None

    def test_native_successful_original_restore_never_reopens_requested_power_gate(self):
        owner = self.owner()
        owner._power_lease = self.lease
        owner.power = dict(requested=False, state='prepared', ready=True)
        self.lease.restore.return_value = dict(requested=False, ready=True, ok=True,
            state='restored', policy=dict(charge_disable=0, restrict_chg=0, restrict_cur=1000000))
        owner.restore_power()
        self.assertEqual(owner.power['state'], 'restored')
        self.assertTrue(owner.power['ok'])
        self.assertFalse(owner.power['ready'])
        owner._power_lease = None

    def test_failed_shutdown_restore_propagates_and_preserves_power_report(self):
        owner = self.owner()
        owner._power_lease = self.lease
        owner.power = dict(requested=False, state='prepared', ready=True)
        self.lease.restore.side_effect = USBChargingError('Original restoration unknown',
            dict(state='unknown', ready=False, errors=['native SET result unavailable']))
        with self.assertRaisesRegex(USBChargingError, 'restoration failed'):
            owner.close()
        self.assertFalse(owner.power['ready'])
        self.assertEqual(owner.power['state'], 'unknown')
        self.assertIn('Original restoration unknown', owner.power['error'])
        self.lease.restore.assert_called_once()
        owner._close_power_error = None  # Do not raise the deliberately handled failure in addCleanup.
        owner._power_lease = None


if __name__ == '__main__':
    unittest.main()
