"""External manual Sleep after a host crash never becomes a fake SDK Sleep."""

from copy import deepcopy
import json
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _startup_recovery as m
from osmo360_camera._standby_control import ExternallySleepingOwnerError, parse_snapshot, StandbyControl
from osmo360_camera._amt_codec import result_payload
from test_standby_control import snapshot, NONCE, Scenario
from test_helper_install import BOOT1

WORK = '/dev/shm/osmo-usb-standby-' + NONCE


def externally_asleep():
    value = snapshot('AWAKE', 2)
    value['requested_readiness']['system_power_level'] = 3
    value['shadow']['value'] = 'none'
    return value


class ExternalSleepParserTests(unittest.TestCase):
    def test_typed_error_retains_awake_receipt_with_current_native_sleep(self):
        value = externally_asleep()
        with self.assertRaises(ExternallySleepingOwnerError) as caught:
            parse_snapshot(json.dumps(value).encode(), nonce=NONCE)
        self.assertEqual(caught.exception.observation, value)
        self.assertEqual(caught.exception.observation['state'], 'AWAKE')
        self.assertEqual(caught.exception.observation['action'], 'wake')

    def test_malformed_role_or_incomplete_receipt_is_not_recoverable_external_sleep(self):
        for mutate in (lambda x: x['shadow'].update(source='/foreign/role.txt'),
                       lambda x: x['last_result'].update(complete=False),
                       lambda x: x.update(owner_alive=False),
                       lambda x: x['requested_readiness'].update(system_power_level=1)):
            value = externally_asleep()
            mutate(value)
            with self.assertRaises(ValueError) as caught:
                parse_snapshot(json.dumps(value).encode(), nonce=NONCE)
            self.assertNotIsInstance(caught.exception, ExternallySleepingOwnerError)


class ExternalSleepRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.now = 0
        self.observed = externally_asleep()
        self.session = SimpleNamespace(quarantined=False)
        self.control = SimpleNamespace(pending_command=None, pending_task=None, _unknown_task=False,
            bootstrap=SimpleNamespace(session=self.session), sources=lambda: {})
        self.camera = SimpleNamespace(_control_lock=threading.RLock(),
            _transport=SimpleNamespace(wait_disconnected=Mock(
                side_effect=lambda _: self.events.append('disconnect') or True),
                close=Mock(side_effect=lambda: self.events.append('close'))),
            connect=Mock(side_effect=lambda **_: self.events.append('connect')))
        self.attached = SimpleNamespace(pending_command=None, pending_task=None, _unknown_task=False,
            bootstrap=self.control.bootstrap, resolve_pending=Mock(
                side_effect=lambda **_: self.events.append('resolve_old_result')))
        def finalize(*, guard):
            guard()
            self.events.append('finalize')
            return dict(reboot_requested=True)
        self.attached.finalize = Mock(side_effect=finalize)
        self.status = patch.object(m, '_status_at', side_effect=lambda *_a, **_k:
            self.events.append('status') or (_ for _ in ()).throw(
                ExternallySleepingOwnerError(deepcopy(self.observed)))).start()
        self.boot = patch.object(m, '_owned_boot',
            side_effect=lambda *_: self.events.append('boot') or BOOT1).start()
        self.attach = patch.object(m, 'StandbyControl', return_value=self.attached).start()
        self.ready = patch.object(m, 'wait_amt_ready', side_effect=lambda *_a, **_k:
            self.events.append('amt_ready') or dict(ready=True)).start()
        self.addCleanup(patch.stopall)
        self.recovery = m.ExternalSleepRecovery(self.camera, self.control, WORK,
            self.observed, clock=lambda: self.now)

    def test_exact_owner_reboots_once_then_requires_separate_new_boot_verification(self):
        report = self.recovery.run(deadline=100)
        self.assertEqual(self.events, ['status', 'boot', 'finalize', 'disconnect',
                                      'close', 'connect', 'amt_ready', 'resolve_old_result'])
        self.assertTrue(report['reboot_attempted'])
        self.assertTrue(report['disconnect_observed'])
        self.assertFalse(report['new_boot_verified'])
        self.assertEqual(report['boot_id_before'], BOOT1)
        self.assertEqual(self.attached.last['state'], 'AWAKE')
        receipt = self.attach.call_args.kwargs['existing']
        self.assertEqual(receipt['owner'], self.observed['owner'])
        self.assertEqual(receipt['service'], self.observed['service'])

    def test_lost_finalize_receipt_does_not_repeat_power_request(self):
        def lost(*, guard):
            guard()
            self.events.append('finalize')
            raise TimeoutError('lost D1 reply')
        self.attached.finalize.side_effect = lost
        report = self.recovery.run(deadline=100)
        self.assertTrue(report['disconnect_observed'])
        self.assertIn('lost D1 reply', report['receipt_error'])
        self.attached.finalize.assert_called_once()

    def test_lost_receipt_new_boot_retires_missing_old_task_using_real_result_protocol(self):
        scenario = Scenario()
        transport = self.camera._transport
        transport.request = scenario.request
        bootstrap = scenario.adapter(transport)
        self.control.bootstrap = bootstrap
        attached = StandbyControl(bootstrap, source_provider=lambda: {},
            control_lock=self.camera._control_lock, adapter_factory=scenario.adapter,
            existing=dict(work=WORK, nonce=NONCE, owner=self.observed['owner'],
                          service=self.observed['service']))
        self.attach.return_value = attached
        def lost(*, guard):
            guard()
            bootstrap.session.quarantined = True
            raise TimeoutError('ACK lost; old camera rebooted')
        attached.finalize = Mock(side_effect=lost)
        report = self.recovery.run(deadline=100)
        self.assertEqual(report['prior_task_result'], dict(task_id=37, status=0xE0,
            result=0xF3, terminal_or_missing=True))
        requests = [event for event in scenario.events if event[0] == 'RESULT']
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0][3], result_payload(37))
        self.assertIsNone(attached.pending_task)
        self.assertFalse(attached.bootstrap.session.quarantined)
        self.assertFalse(any(event[0] == 'START' for event in scenario.events))
        attached.finalize.assert_called_once()

    def test_no_disconnect_fails_closed_and_same_recovery_cannot_replay(self):
        self.camera._transport.wait_disconnected.side_effect = None
        self.camera._transport.wait_disconnected.return_value = False
        with self.assertRaisesRegex(m.StartupRecoveryError, 'disappearance') as caught:
            self.recovery.run(deadline=100)
        self.assertTrue(caught.exception.report['reboot_attempted'])
        self.assertIs(caught.exception.helper_control, self.attached)
        with self.assertRaisesRegex(m.StartupRecoveryError, 'already attempted'):
            self.recovery.run(deadline=100)
        self.attached.finalize.assert_called_once()
        self.camera._transport.close.assert_not_called()
        self.camera.connect.assert_not_called()

    def test_pending_or_unknown_tasks_never_authorize_finalize(self):
        for field, value in (('pending_command', '0000000000000003'), ('pending_task', 7),
                             ('_unknown_task', True), ('quarantined', True)):
            target = self.session if field == 'quarantined' else self.control
            original = getattr(target, field)
            setattr(target, field, value)
            with self.subTest(field=field), self.assertRaises(m.StartupRecoveryError):
                self.recovery.run(deadline=100)
            setattr(target, field, original)
        self.status.assert_not_called()
        self.attach.assert_not_called()

    def test_changed_owned_role_generation_blocks_recovery(self):
        changed = deepcopy(self.observed)
        changed['shadow']['inode'] = [90, 91]
        self.status.side_effect = ExternallySleepingOwnerError(changed)
        with self.assertRaisesRegex(m.StartupRecoveryError, 'owner changed'):
            self.recovery.run(deadline=100)
        self.boot.assert_not_called()
        self.attach.assert_not_called()

    def test_late_observation_cannot_deliver_recovery(self):
        def late(*_a, **_k):
            self.now = 101
            raise ExternallySleepingOwnerError(self.observed)
        self.status.side_effect = late
        with self.assertRaises(m.StartupRecoveryError):
            self.recovery.run(deadline=100)
        self.attach.assert_not_called()


class RecoveryBootExportTests(unittest.TestCase):
    def test_export_is_bounded_and_same_owner_service_boot_is_required(self):
        value = externally_asleep()
        payload = [BOOT1, value['owner'], value['service'], [12, 34]]
        control = SimpleNamespace(_run=Mock(), bootstrap=SimpleNamespace(
            pull_bytes=Mock(return_value=json.dumps(payload).encode())))
        self.assertEqual(m._owned_boot(control, WORK, value, lambda: 20), BOOT1)
        self.assertEqual(control._run.call_count, 2)
        self.assertTrue(all(len(call.args[0]) <= 500 for call in control._run.call_args_list))
        payload[1] = dict(pid=999, start='other')
        control.bootstrap.pull_bytes.return_value = json.dumps(payload).encode()
        with self.assertRaises(ValueError):
            m._owned_boot(control, WORK, value, lambda: 20)

    def test_old_result_consuming_budget_cannot_start_export_or_cleanup_task(self):
        for expired_call in (1, 2):
            now, calls, starts = [0], [0], []
            def remaining():
                if now[0] >= 1:
                    raise TimeoutError('deadline')
                return 1-now[0]
            def run(command, **options):
                calls[0] += 1
                if calls[0] == expired_call:
                    now[0] = 2  # Earlier task RESULT consumed the last budget.
                options['pre_send']()
                starts.append(command)
            payload = [BOOT1, snapshot()['owner'], snapshot()['service'], [12, 34]]
            control = SimpleNamespace(_run=run, bootstrap=SimpleNamespace(
                pull_bytes=Mock(return_value=json.dumps(payload).encode())))
            with self.subTest(expired_call=expired_call), self.assertRaises(TimeoutError):
                m._owned_boot(control, WORK, externally_asleep(), remaining)
            self.assertEqual(len(starts), expired_call-1)


class DiscoveryDeadlineTests(unittest.TestCase):
    def test_old_result_consuming_budget_cannot_start_any_discovery_task(self):
        from osmo360_camera._stream_resume import _discover
        for expired_call in (1, 2, 3):
            now, calls, starts = [0], [0], []
            def run(command, **options):
                calls[0] += 1
                if calls[0] == expired_call:
                    now[0] = 2
                options['pre_send']()
                starts.append(command)
            control = SimpleNamespace(_run=run, bootstrap=SimpleNamespace(
                pull_bytes=Mock(return_value=json.dumps(dict(m=[], w=[], i=[12,34])).encode())))
            with self.subTest(expired_call=expired_call), self.assertRaises(TimeoutError):
                _discover(control, deadline=1, clock=lambda: now[0])
            self.assertEqual(len(starts), expired_call-1)


if __name__ == '__main__':
    unittest.main()
