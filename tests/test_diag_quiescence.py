"""A restored mask cannot authorize USB handoff while DIAG writes remain."""

from contextlib import redirect_stdout
import ctypes as C
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from osmo360_camera import _diag_collector as m
from test_imu_live_status import Clock, DatagramSocket, FakeUSB
from test_ssc_decoder import packet


class Reader:
    """One existing USB reader; late bytes and timeout durations are explicit."""

    def __init__(self, events=(), *, fallback=(-7, 0, .151, 0)):
        self.events, self.fallback = list(events), fallback
        self.now = 0.0
        self.values = dict(usb_reads=0, usb_bytes=0, usb_timeouts=0,
                           last_read_status=0, pending_fragment_bytes=0)

    def clock(self):
        return self.now

    def snapshot(self):
        return dict(self.values)

    def receive(self):
        status, size, seconds, fragment = (
            self.events.pop(0) if self.events else self.fallback)
        self.now += seconds
        self.values['usb_reads'] += 1
        self.values['usb_bytes'] += size
        self.values['usb_timeouts'] += status == -7
        self.values['last_read_status'] = status
        self.values['pending_fragment_bytes'] = fragment
        if status not in (0, -7):
            raise OSError('DIAG read failed: ' + str(status))


class DrainTests(unittest.TestCase):
    def drain(self, reader, **options):
        return m._drain_until_quiet(reader.receive, reader.snapshot,
                                    clock=reader.clock, **options)

    def test_residual_data_after_mask_ack_is_drained_then_two_real_timeouts(self):
        reader = Reader([(0, 1024, .01, 7), (0, 321, .01, 0)])
        value = self.drain(reader)
        self.assertTrue(value['verified'])
        self.assertEqual((value['reads'], value['bytes'], value['timeouts']), (4, 1345, 2))
        self.assertGreaterEqual(value['quiet_s'], .30)
        self.assertEqual(value['pending_fragment_bytes'], 0)

    def test_partial_timeout_with_data_resets_quiet_boundary(self):
        reader = Reader([(-7, 10, .151, 0), (-7, 0, .151, 0),
                         (-7, 5, .151, 0)])
        value = self.drain(reader)
        self.assertTrue(value['verified'])
        self.assertEqual((value['bytes'], value['reads']), (15, 5))
        self.assertEqual(value['consecutive_empty_timeouts'], 2)

    def test_continuing_original_log_traffic_is_bounded_failure(self):
        reader = Reader(fallback=(0, 256, .15, 0))
        value = self.drain(reader)
        self.assertFalse(value['verified'])
        self.assertLessEqual(value['reads'], 21)
        self.assertIn('deadline', value['error'])
        self.assertGreater(value['bytes'], 0)

    def test_unknown_incomplete_packet_is_not_an_idle_boundary(self):
        reader = Reader(fallback=(-7, 0, .151, 12))
        value = self.drain(reader)
        self.assertFalse(value['verified'])
        self.assertEqual(value['pending_fragment_bytes'], 12)

    def test_usb_read_error_preserves_restored_mask_but_forbids_handoff(self):
        reader = Reader([(0, 1024, .01, 0), (-1, 0, .01, 0)])
        report = dict(restored=True, errors=[])
        m._drain_restored_reader(report, reader.receive, reader.snapshot,
                                 original_mask=b'\0', clock=reader.clock)
        self.assertTrue(report['restored'])
        self.assertFalse(report['quiescent'])
        self.assertEqual(report['drain']['last_read_status'], -1)
        self.assertEqual((report['drain']['reads'], report['drain']['bytes']), (2, 1024))
        self.assertIn('DIAG USB handoff blocked', report['errors'][0])

    def test_late_quiet_reply_does_not_authorize_handoff(self):
        reader = Reader([(-7, 0, .2, 0), (-7, 0, 4, 0)])
        self.assertFalse(self.drain(reader)['verified'])

    def test_failed_original_generation_is_not_replaced_for_drain(self):
        reader = Reader()
        report = dict(restored=True, errors=[])
        m._drain_restored_reader(report, reader.receive, reader.snapshot,
                                 original_mask=b'\0', fatal_read_status=-4, clock=reader.clock)
        self.assertFalse(report['quiescent'])
        self.assertEqual(reader.values['usb_reads'], 0)
        self.assertEqual(report['drain']['fatal_read_status'], -4)

    def test_unrestored_mask_does_not_start_another_read_or_change_original_error(self):
        reader = Reader()
        report = dict(restored=False, errors=['Original mask unknown'])
        m._drain_restored_reader(report, reader.receive, reader.snapshot,
                                 original_mask=b'\0', clock=reader.clock)
        self.assertFalse(report['quiescent'])
        self.assertEqual(report['errors'], ['Original mask unknown'])
        self.assertEqual(reader.values['usb_reads'], 0)

    def test_invalid_deadlines_are_rejected_before_reads(self):
        for options in ({'timeout': float('inf')}, {'quiet': float('nan')},
                        {'timeout': 0}, {'quiet': 0}, {'timeout': .3, 'quiet': .3},
                        {'timeout': True}):
            reader = Reader()
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.drain(reader, **options)
            self.assertEqual(reader.values['usb_reads'], 0)

    def test_sparse_original_logs_cannot_pass_a_short_observed_quiet_interval(self):
        reader = Reader()  # A log whose period exceeds the entire drain window.
        report = dict(restored=True, errors=[])
        m._drain_restored_reader(report, reader.receive, reader.snapshot,
                                 original_mask=b'\x01', clock=reader.clock)
        self.assertTrue(report['restored'])
        self.assertTrue(report['drain']['verified'])  # Truthful host observation.
        self.assertEqual(report['drain']['bytes'], 0)
        self.assertFalse(report['quiescent'])
        self.assertIn('equipment1', report['drain']['handoff_blocked_reason'])
        self.assertIn('cannot exclude later traffic', report['errors'][0])


class TailUSB(FakeUSB):
    """Full collector integration: actual framed SSC tail after restore GET."""

    def __init__(self, clock, output, *, continuous=False, sparse=False, read_error=False):
        super().__init__(clock, output)
        self.continuous, self.read_error = continuous, read_error
        if continuous or sparse:
            self.mask = b'\x01' + self.mask[1:]  # A pre-existing log stays enabled.
        self.tail = m.frame(packet('accel', [(19_200_000, (1., 2., 3.))]))
        self.residual = [self.tail[:13], self.tail[13:]]
        self.draining, self.order = False, []

    def libusb_bulk_transfer(self, handle, endpoint, buffer, size, count, timeout):
        if endpoint == 1 or self.replies:
            return super().libusb_bulk_transfer(handle, endpoint, buffer, size, count, timeout)
        self.clock.now += .151
        count._obj.value = 0
        if self.sets < 2:
            raise KeyboardInterrupt('normal owned stop')
        self.draining = True
        self.order.append('tail_read')
        if self.read_error:
            return -1
        raw = self.tail if self.continuous else self.residual.pop(0) if self.residual else b''
        if raw:
            C.memmove(buffer, raw, len(raw))
            count._obj.value = len(raw)
            return 0
        self.order[-1] = 'empty_timeout'
        return -7

    def libusb_release_interface(self, *args):
        self.order.append('release_interface')
        return 0


class CollectorIntegrationTests(unittest.TestCase):
    def run_collector(self, **options):
        output, clock, udp = io.StringIO(), Clock(), DatagramSocket()
        usb = TailUSB(clock, output, **options)
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory) / 'capture'
            with (patch.object(m.osmo_usb, 'load_usb', return_value=usb),
                  patch.object(m, '_DiagRebindGuard', side_effect=RuntimeError('no rebind')),
                  patch.object(m.socket, 'socket', return_value=udp),
                  patch.object(m.time, 'monotonic', side_effect=clock.seconds),
                  patch.object(m.time, 'monotonic_ns', side_effect=clock.ns),
                  patch('sys.argv', ['collector', '--seconds', '0', '--prefix', str(prefix),
                                    '--paired', '--udp-port', '12345', '--udp-format', 'binary',
                                    '--no-raw', '--no-packet-jsonl']), redirect_stdout(output)):
                code = m.main()
            report = json.loads(prefix.with_suffix('.json').read_text())
        return code, report, usb, udp

    def test_same_reader_drains_real_split_frame_before_interface_release(self):
        code, report, usb, udp = self.run_collector()
        self.assertEqual(code, 0)
        self.assertTrue(report['restored'])
        self.assertTrue(report['quiescent'])
        self.assertEqual(report['drain']['bytes'], len(usb.tail))
        self.assertEqual(usb.order[-3:], ['empty_timeout', 'empty_timeout', 'release_interface'])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])  # No extra SET or GET.
        self.assertEqual(udp.sent, [])  # Cleanup tail never becomes captured data.
        self.assertEqual(report['log_codes'], {'19cb': 1})

    def test_nonzero_original_mask_with_continuing_logs_is_not_reported_quiet(self):
        code, report, usb, udp = self.run_collector(continuous=True)
        self.assertEqual(code, 1)
        self.assertTrue(report['restored'])
        self.assertFalse(report['quiescent'])
        self.assertGreater(report['drain']['bytes'], 0)
        self.assertIn('deadline', report['drain']['error'])
        self.assertIn('Original equipment1 DIAG mask', report['errors'][0])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])

    def test_read_error_after_verified_restore_is_an_unsafe_handoff(self):
        code, report, usb, udp = self.run_collector(read_error=True)
        self.assertEqual(code, 1)
        self.assertTrue(report['restored'])
        self.assertFalse(report['quiescent'])
        self.assertEqual(report['fatal_read_status'], -1)
        self.assertIn('DIAG read', report['drain']['error'])

    def test_sparse_original_mask_is_restored_unchanged_but_never_authorizes_handoff(self):
        code, report, usb, udp = self.run_collector(sparse=True)
        self.assertEqual(code, 1)
        self.assertTrue(report['restored'])
        self.assertTrue(report['drain']['verified'])
        self.assertFalse(report['quiescent'])
        self.assertEqual(usb.mask, bytes.fromhex(report['original_mask_hex']))
        self.assertTrue(any(usb.mask))
        self.assertIn('cannot exclude later traffic', report['errors'][0])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])


if __name__ == '__main__':
    unittest.main()
