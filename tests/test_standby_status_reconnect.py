"""An accepted RO status task survives C1 loss only through original RESULT."""

import json
import struct
from types import SimpleNamespace
import unittest

from osmo360_camera import _standby_control as m
from osmo360_camera.transport import CameraDisconnected
from test_standby_control import proof
from test_standby_status_retry import NativeD1


class BrokenReader(NativeD1):
    def __init__(self):
        super().__init__()
        self.connected, self.read_failure_status = True, None
        self._thread = SimpleNamespace(native_id=101)
        self._device_identity = dict(bus=2, address=38)
        self.status_task = self.failed_task = None
        self.sleep_delivered = False
        self.injected = False
        self.result_mode, self.gate_change = 'terminal', None
        self.reconnect_delay = 0
        self.result_late = False
        self.reconnect_deadlines = []
        self.result_budgets = []
        self.control_ref = None

    def request(self, cmdset, cmdid, payload, **options):
        if not self.connected:
            raise CameraDisconnected('LIBUSB_ERROR_IO')  # No STOP can reach old reader.
        sub = struct.unpack_from('<H', payload)[0]
        if sub == 0:
            command = payload[2:514].split(b'\0', 1)[0].decode('ascii')
            value = super().request(cmdset, cmdid, payload, **options)
            if ' sleep ' in command:
                self.fail_status = 0
                self.sleep_delivered = True
            elif command.endswith(' status') and self.sleep_delivered and not self.injected:
                self.status_task = struct.unpack_from('<I', payload, 518)[0]
            return value
        if sub == 4:
            task = struct.unpack_from('<I', payload, 2)[0]
            if task == self.status_task and not self.injected:
                self.injected, self.failed_task = True, task
                self.connected, self.read_failure_status = False, -1
                op = self.control_ref.bootstrap.session.operations[-1]
                if self.gate_change == 'not-status': op['command'] = 'python3 fixed readonly hash'
                elif self.gate_change == 'not-accepted': op['start_accepted'] = False
                elif self.gate_change == 'unknown': self.control_ref._unknown_task = True
                elif self.gate_change == 'wrong-command': self.control_ref.pending_command = '000000000000ffff'
                elif self.gate_change == 'no-fatal': self.read_failure_status = None
                elif self.gate_change == 'only-timeout': self.read_failure_status = -7
                raise CameraDisconnected('LIBUSB_ERROR_IO')
            if task == self.failed_task:
                self.result_budgets.append(options['timeout'])
                if self.result_late: self.clock.now += 20
                if self.result_mode == 'missing': self.tasks.pop(task, None)
                elif self.result_mode == 'running': self.tasks[task] = 0xF0
        return super().request(cmdset, cmdid, payload, **options)

    def reconnect(self, *, deadline):
        self.reconnect_deadlines.append(deadline)
        self.events.append(('RECONNECT', deadline))
        self.clock.now += self.reconnect_delay
        self.connected, self.read_failure_status = True, None
        self._thread = SimpleNamespace(native_id=102)
        return self.adapter()

    def sleeping(self):
        control = self.control()
        self.control_ref = control
        control.readonly_reconnect = self.reconnect
        control._submit_sleep(proof())
        return control


class StatusReconnectTests(unittest.TestCase):
    def test_terminal_or_missing_original_result_allows_only_same_sleep_status_completion(self):
        for mode in ('terminal', 'missing'):
            with self.subTest(mode=mode):
                native = BrokenReader()
                native.result_mode, native.reconnect_delay = mode, .25
                control = native.sleeping()
                old = control.bootstrap.session
                value = control.wait('0000000000000001', 'SLEEP', timeout=12)
                self.assertEqual(value['state'], 'SLEEP')
                self.assertEqual(value['command_id'], '0000000000000001')
                self.assertIsNone(control.pending_command)
                self.assertIsNone(control.pending_task)
                self.assertFalse(control._unknown_task)
                self.assertTrue(old.quarantined)  # Never clear the failed session flag.
                self.assertIsNot(control.bootstrap.session, old)
                self.assertEqual(native.reconnect_deadlines, [12.0])
                starts = [row for row in native.events if row[0] == 'START']
                self.assertEqual(sum(' sleep ' in row[2] for row in starts), 1)
                self.assertEqual(sum(row[2].endswith(' status') for row in starts), 3)
                self.assertFalse(any(row[0] == 'STOP' for row in native.events))
                reconnect_at = next(i for i, row in enumerate(native.events) if row[0] == 'RECONNECT')
                original_result = next(i for i, row in enumerate(native.events)
                    if i > reconnect_at and row == ('RESULT', native.failed_task))
                fresh_start = next(i for i, row in enumerate(native.events)
                    if i > reconnect_at and row[0] == 'START')
                self.assertLess(original_result, fresh_start)
                self.assertNotEqual(native.events[fresh_start][1], native.failed_task)
                diag = control.status_diagnostics
                self.assertEqual((diag['reconnect_attempts'], diag['reconnect_successes']), (1, 1))
                row = diag['reconnect_events'][0]
                self.assertTrue(row['reader_changed'])
                self.assertTrue(row['resumed'])
                self.assertTrue(row['command_proven'])
                self.assertEqual(row['task_id'], native.failed_task)
                self.assertEqual(row['command_id'], '0000000000000001')
                self.assertEqual(row['generation'], dict(bus=2, address=38, reader_native_id=102))
                self.assertTrue(row['result_proof']['terminal_or_missing'])
                row['generation']['address'] = 99
                self.assertEqual(control.status_diagnostics['reconnect_events'][0]['generation']['address'], 38)
                self.assertLess(len(json.dumps(diag)), 4096)

    def test_unresolved_or_late_original_result_preserves_pending_task_and_blocks_all_new_start(self):
        for mode in ('running', 'late'):
            with self.subTest(mode=mode):
                native = BrokenReader()
                native.result_mode, native.result_late = ('running', False) if mode == 'running' else ('terminal', True)
                control = native.sleeping()
                old = control.bootstrap.session
                with self.assertRaises(m.StandbyUncertain):
                    control.wait('0000000000000001', 'SLEEP', timeout=12)
                self.assertTrue(old.quarantined)
                self.assertEqual(control.pending_task, native.failed_task)
                self.assertEqual(control.pending_command, '0000000000000001')
                self.assertEqual(sum(row[0] == 'START' for row in native.events), 3)
                self.assertEqual(control.status_reconnect_successes, 0)

    def test_nonstatus_unknown_nonaccepted_or_wrong_command_never_reconnects(self):
        for mode in ('not-status', 'unknown', 'not-accepted', 'wrong-command', 'no-fatal', 'only-timeout'):
            with self.subTest(mode=mode):
                native = BrokenReader()
                native.gate_change = mode
                control = native.sleeping()
                with self.assertRaises(CameraDisconnected):
                    control.wait('0000000000000001', 'SLEEP', timeout=12)
                self.assertEqual(native.reconnect_deadlines, [])
                self.assertEqual(control.status_reconnect_attempts, 0)
                self.assertEqual(sum(row[0] == 'START' for row in native.events), 3)
                self.assertIsNotNone(control.pending_task)

    def test_wake_wait_never_uses_sleep_only_reconnect_gate(self):
        native = BrokenReader()
        control = native.sleeping()
        with self.assertRaises(CameraDisconnected):
            control.wait('0000000000000001', 'AWAKE', timeout=12)
        self.assertEqual(native.reconnect_deadlines, [])
        self.assertEqual(control.pending_task, native.failed_task)

    def test_original_result_query_uses_only_remaining_wait_budget(self):
        native = BrokenReader()
        native.reconnect_delay = 11.75
        control = native.sleeping()
        self.assertEqual(control.wait('0000000000000001', 'SLEEP', timeout=12)['state'], 'SLEEP')
        self.assertEqual(native.result_budgets, [.25])

    def test_original_absolute_wait_deadline_is_never_extended_by_reconnect(self):
        native = BrokenReader()
        native.reconnect_delay = 12.1
        control = native.sleeping()
        with self.assertRaises(m.StandbyUncertain):
            control.wait('0000000000000001', 'SLEEP', timeout=12)
        self.assertEqual(native.reconnect_deadlines, [12.0])
        self.assertEqual(control.pending_task, native.failed_task)
        self.assertEqual(control.pending_command, '0000000000000001')
        self.assertEqual(sum(row[0] == 'START' for row in native.events), 3)
        self.assertEqual(control.status_reconnect_successes, 0)

    def test_fresh_reader_never_substitutes_foreign_owner_snapshot(self):
        native = BrokenReader()
        control = native.sleeping()
        reconnect = native.reconnect
        def foreign(**options):
            adapter = reconnect(**options)
            native.value = dict(native.value, nonce='00' * 6)
            return adapter
        control.readonly_reconnect = foreign
        with self.assertRaisesRegex(ValueError, 'Owned live standby snapshot'):
            control.wait('0000000000000001', 'SLEEP', timeout=12)
        self.assertEqual(control.pending_command, '0000000000000001')
        self.assertEqual(sum(row[0] == 'START' and ' sleep ' in row[2] for row in native.events), 1)
        self.assertFalse(control.status_diagnostics['reconnect_events'][0]['command_proven'])

    def test_reconnect_without_new_reader_cannot_query_result_or_start_status(self):
        native = BrokenReader()
        control = native.sleeping()
        before = native._thread
        def unchanged(**options):
            native.connected = True
            native._thread = before
            return native.adapter()
        control.readonly_reconnect = unchanged
        with self.assertRaises(m.StandbyUncertain):
            control.wait('0000000000000001', 'SLEEP', timeout=12)
        self.assertEqual(sum(row[0] == 'START' for row in native.events), 3)
        self.assertEqual(control.pending_task, native.failed_task)
        self.assertEqual(control.status_reconnect_successes, 0)


if __name__ == '__main__':
    unittest.main()
