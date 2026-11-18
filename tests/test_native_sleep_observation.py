"""Transient stock wake locks cannot authorize a Wake key or helper arm."""

from collections import deque
from copy import deepcopy
import json
import unittest

from osmo360_camera import _standby_control as m
from osmo360_camera import _stream_resume as resume
from test_standby_control import snapshot, NONCE, OWNER, SERVICE, proof
from test_standby_status_retry import NativeD1


def observed(level, role, at=100):
    value = snapshot('SLEEP', 1)
    value['requested_readiness']['system_power_level'] = level
    value['shadow']['value'] = role
    value['at']['boottime'] = value['at']['monotonic'] = at
    return value


class SleepObservationTests(unittest.TestCase):
    def setUp(self):
        self.native = NativeD1()
        self.native.value = snapshot('SLEEP', 1)
        self.control = self.native.control()
        self.control.last = deepcopy(self.native.value)

    def values(self, values, *, cost=0):
        pending = deque(deepcopy(values))
        def pull(*args, **options):
            self.native.clock.now += cost
            value = pending.popleft() if len(pending) > 1 else pending[0]
            return json.dumps(value).encode()
        self.control.bootstrap.pull_bytes = pull

    def assert_only_status_inputs(self):
        commands = [row[2] for row in self.native.events if row[0] == 'START']
        self.assertTrue(all(command.endswith(' status') for command in commands))
        self.assertFalse(any(row[0] == 'STOP' for row in self.native.events))
        self.assertIsNone(self.control.pending_command)
        self.assertIsNone(self.control.pending_task)

    def test_complete_receipt_and_owned_shadow_are_required_before_typed_mismatch(self):
        valid = observed(0, 'exit')
        with self.assertRaises(m.NativeSleepObservationMismatch):
            m.parse_snapshot(json.dumps(valid).encode(), nonce=NONCE, owner=OWNER, service=SERVICE)
        for change in ('receipt', 'gui', 'owner', 'source', 'shadow', 'level'):
            value = deepcopy(valid)
            if change == 'receipt': value['last_result']['complete'] = False
            elif change == 'gui': value['last_result']['gui']['done3_after_intent'] = False
            elif change == 'owner': value['owner'] = dict(pid=999, start='1')
            elif change == 'source': value['requested_readiness']['verified_executable'] = '00' * 32
            elif change == 'shadow': value['shadow']['inode'] = [0, 26]
            else: value['requested_readiness']['system_power_level'] = 4
            with self.subTest(change=change), self.assertRaises(ValueError) as got:
                m.parse_snapshot(json.dumps(value).encode(), nonce=NONCE, owner=OWNER, service=SERVICE)
            self.assertNotIsInstance(got.exception, m.NativeSleepObservationMismatch)

    def test_field_update_orders_require_fresh_level2_or3_and_shadow_none(self):
        self.values([observed(0, 'exit'), observed(0, 'none', 100.1),
                     observed(2, 'exit', 100.2), observed(3, 'none', 100.3)])
        report = {}
        value = self.control.wait_sleep_snapshot('0000000000000001', deadline=1, report=report)
        self.assertEqual(value['requested_readiness']['system_power_level'], 3)
        self.assertEqual(report['attempts'], 4)
        self.assertTrue(report['ready'])
        self.assertEqual([row['stable'] for row in report['observations']], [False, False, False, True])
        self.assert_only_status_inputs()

    def test_persistent_native0_expires_under_original_deadline_with_bounded_report(self):
        self.values([observed(0, 'exit')])
        report = {}
        with self.assertRaises(m.StandbyUncertain):
            self.control.wait_sleep_snapshot('0000000000000001', deadline=1, report=report)
        self.assertLessEqual(self.native.clock.now, 1)
        self.assertFalse(report['ready'])
        self.assertGreater(report['attempts'], 8)
        self.assertEqual(len(report['observations']), 8)
        self.assertLess(len(json.dumps(report)), 4096)
        self.assert_only_status_inputs()

    def test_unknown_or_quarantined_tasks_never_start_readiness_poll(self):
        for field in ('unknown', 'quarantine', 'pending'):
            with self.subTest(field=field):
                native = NativeD1()
                native.value = snapshot('SLEEP', 1)
                control = native.control()
                if field == 'unknown': control._unknown_task = True
                elif field == 'pending': control.pending_task = 123
                else: control.bootstrap.session.quarantined = True
                with self.assertRaises(m.StandbyUncertain):
                    control.wait_sleep_snapshot('0000000000000001', deadline=1)
                self.assertEqual(native.events, [])

    def test_receipt_role_or_owner_drift_never_becomes_stable_sleep(self):
        for change in ('receipt', 'role', 'owner', 'clock'):
            value = observed(2, 'none', 101)
            if change == 'receipt': value['last_result']['get_level'] = 2
            elif change == 'role': value['shadow']['inode'] = [23, 99]
            elif change == 'owner': value['owner'] = dict(pid=999, start='1')
            else: value['at']['boottime'] = 99
            self.values([observed(0, 'exit'), value])
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.control.wait_sleep_snapshot('0000000000000001', deadline=self.native.clock.now + 1)
            self.assert_only_status_inputs()

    def test_late_fresh_sleep_never_authorizes_arm(self):
        self.values([observed(2, 'none')], cost=1.1)
        with self.assertRaises(m.StandbyUncertain):
            self.control.wait_sleep_snapshot('0000000000000001', deadline=1)
        self.assert_only_status_inputs()

    def test_native_status_timer_and_file_pull_use_remaining_original_budget(self):
        self.values([observed(2, 'none')])
        self.control.wait_sleep_snapshot('0000000000000001', deadline=.25)
        op = self.control.bootstrap.session.operations[-1]
        self.assertEqual(op['timeout_ms'], 250)
        self.assert_only_status_inputs()

    def test_wait_after_owned_sleep_reobserves_same_completed_command_without_another_key(self):
        self.native.value = snapshot()
        self.control.last = snapshot()
        self.control._submit_sleep(proof())
        self.native.fail_status = 0
        self.values([observed(0, 'none'), observed(2, 'exit', 100.1), observed(2, 'none', 100.2)])
        value = self.control.wait('0000000000000001', 'SLEEP', timeout=1)
        self.assertEqual(value['state'], 'SLEEP')
        commands = [row[2] for row in self.native.events if row[0] == 'START']
        self.assertEqual(sum(' sleep ' in command for command in commands), 1)
        self.assertFalse(any(' wake ' in command for command in commands))
        self.assertTrue(self.control.status_diagnostics['sleep_readiness']['ready'])

    def test_wake_delivery_guard_rechecks_original_deadline_after_slow_status(self):
        self.values([observed(2, 'none')], cost=1.1)
        with self.assertRaises(m.StandbyUncertain):
            self.control.submit_wake(deadline=1)
        self.assert_only_status_inputs()

    def test_wake_delivery_guard_still_prevents_input_when_time_expires_after_snapshot(self):
        self.values([observed(2, 'none')])
        run = self.control._run
        def slow(command, **options):
            if ' wake ' in command:
                self.native.clock.now = 1.1
            return run(command, **options)
        self.control._run = slow
        with self.assertRaises(m.StandbyUncertain):
            self.control.submit_wake(deadline=1)
        self.assert_only_status_inputs()

    def test_discovery_does_not_adopt_transient_native0_but_can_read_same_sleep_after_release(self):
        self.values([observed(0, 'exit'), observed(2, 'none', 100.1)])
        report = {}
        value = resume._status_at(self.control, self.control.work, deadline=1,
                                  clock=self.native.clock, report=report)
        self.assertEqual(value['state'], 'SLEEP')
        self.assertTrue(report['ready'])
        self.assertEqual(report['attempts'], 2)
        self.assertFalse(any(row[0] == 'START' and (' wake ' in row[2] or ' sleep ' in row[2])
                             for row in self.native.events))


if __name__ == '__main__':
    unittest.main()
