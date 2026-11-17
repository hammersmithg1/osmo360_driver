"""Live collector evidence through real framing/SSC and the stdout drainer."""

from contextlib import redirect_stdout
import ctypes as C
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _diag_collector as collector
from osmo360_camera import imu
from test_ssc_decoder import packet


class Clock:
    now = 0.

    def seconds(self):
        return self.now

    def ns(self):
        return round(self.now * 10**9)


class FakeUSB:
    """Only LOG_CONFIG plus actual SSC frames; no libusb backend is loaded."""

    def __init__(self, clock, output, *, cleanup_only=False):
        self.clock, self.output, self.cleanup_only = clock, output, cleanup_only
        self.devices = (C.c_void_p * 1)(1)
        self.last, self.mask = 3422, bytes((3422 + 8) // 8)
        self.replies, self.operations = [], []
        self.capture_reads = self.sets = 0
        self.live_before_restore = []
        self.cleanup_io_seen = False

    def libusb_init(self, context):
        context._obj.value = 10
        return 0

    def libusb_get_device_list(self, context, devices):
        C.cast(devices, C.POINTER(C.POINTER(C.c_void_p)))[0] = C.cast(self.devices, C.POINTER(C.c_void_p))
        return 1

    def libusb_get_device_descriptor(self, device, descriptor):
        raw = bytearray(18)
        struct.pack_into('<HH', raw, 8, 0x2CA3, 0x0025)
        C.memmove(descriptor, bytes(raw), len(raw))
        return 0

    def libusb_open(self, device, handle):
        handle._obj.value = 20
        return 0

    def libusb_claim_interface(self, *args):
        return 0

    def libusb_release_interface(self, *args):
        return 0

    def libusb_close(self, *args):
        pass

    def libusb_free_device_list(self, *args):
        pass

    def libusb_exit(self, *args):
        pass

    def libusb_error_name(self, code):
        return b'LIBUSB_ERROR_IO'

    def libusb_bulk_transfer(self, handle, endpoint, buffer, size, count, timeout):
        self.clock.now += .15
        if endpoint == 1:
            request = next(collector.Decoder().feed(C.string_at(buffer, size)))
            operation = struct.unpack_from('<I', request, 4)[0]
            self.operations.append(operation)
            if operation == 3:
                self.sets += 1
                self.mask = request[16:]
                if self.sets == 2:
                    self.live_before_restore = [json.loads(line[len(collector.LIVE_PREFIX):])
                        for line in self.output.getvalue().splitlines() if line.startswith(collector.LIVE_PREFIX)]
            response = b'\x73\0\0\0' + struct.pack('<4I', operation, 0, 1, self.last) + self.mask
            self.replies.append(collector.frame(response))
            count._obj.value = size
            return 0
        count._obj.value = 0
        if self.replies:
            if self.cleanup_only and self.sets == 2 and not self.cleanup_io_seen:
                self.cleanup_io_seen = True
                self.replies.clear()
                return -1
            raw = self.replies.pop(0)
        else:
            self.capture_reads += 1
            if self.cleanup_only:
                raise KeyboardInterrupt('finished test capture')
            if self.capture_reads == 1:
                samples = [(19_200_000 + index * 2400, (1., 2., 3.)) for index in range(1000)]
                raw = collector.frame(packet('accel', samples)) + collector.frame(packet('gyro', samples))
            elif self.capture_reads <= 10:
                return -7
            else:
                return -1
        if len(raw) > size:
            raise AssertionError('Test USB buffer is too small')
        C.memmove(buffer, raw, len(raw))
        count._obj.value = len(raw)
        return 0


class DatagramSocket:
    def __init__(self):
        self.sent = []

    def setblocking(self, enabled):
        pass

    def sendto(self, data, address):
        self.sent.append(data)
        return len(data)

    def close(self):
        pass


class LiveCollectorTests(unittest.TestCase):
    def run_collector(self, directory, *, cleanup_only=False):
        output, clock, udp = io.StringIO(), Clock(), DatagramSocket()
        usb = FakeUSB(clock, output, cleanup_only=cleanup_only)
        prefix = Path(directory) / 'capture'
        arguments = ['--seconds', '0', '--prefix', str(prefix), '--paired',
                     '--udp-port', '12345', '--udp-format', 'binary', '--no-raw', '--no-packet-jsonl']
        with (
            patch.object(collector.osmo_usb, 'load_usb', return_value=usb),
            patch.object(collector, '_DiagRebindGuard', side_effect=RuntimeError('no test rebind')),
            patch.object(collector.socket, 'socket', return_value=udp),
            patch.object(collector.time, 'monotonic', side_effect=clock.seconds),
            patch.object(collector.time, 'monotonic_ns', side_effect=clock.ns),
            patch.object(sys, 'argv', ['collector', *arguments]),
            redirect_stdout(output),
        ):
            result = collector.main()
        return result, usb, udp, output.getvalue(), json.loads(prefix.with_suffix('.json').read_text())

    def test_capture_io_is_visible_before_finally_and_preserves_pair_counters(self):
        with tempfile.TemporaryDirectory() as directory:
            result, usb, udp, text, report = self.run_collector(directory)
        events = [imu._collector_status(line) for line in text.splitlines() if line.startswith(imu.LIVE_PREFIX)]
        self.assertTrue(all(event is not None for event in events))
        labels = [event['event'] for event in events]
        for first, second in (('mask_verified', 'first_ssc'), ('first_ssc', 'first_pair'),
                              ('first_pair', 'fatal_read'), ('fatal_read', 'cleanup_started'),
                              ('cleanup_started', 'collector_finished')):
            self.assertLess(labels.index(first), labels.index(second))
        fatal = next(event for event in events if event['event'] == 'fatal_read')
        self.assertEqual((fatal['phase'], fatal['fatal_read_status']), ('capture', -1))
        self.assertEqual((fatal['ssc_packets'], fatal['paired_samples'], fatal['udp_sent']), (2, 1000, 1000))
        self.assertEqual(len(udp.sent), 1000)
        self.assertEqual(imu.IMUSample.from_datagram(udp.sent[-1]).timestamp_ticks, 19_200_000 + 999 * 2400)
        before_restore = [event['event'] for event in usb.live_before_restore]
        self.assertIn('fatal_read', before_restore)
        self.assertIn('cleanup_started', before_restore)
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])  # No new commands/retries.
        self.assertEqual(result, 1)
        self.assertTrue(report['restored'])
        self.assertEqual(report['collector_live']['fatal_read_phase'], 'capture')
        progress = [event['monotonic_ns'] for event in events if event['event'] == 'progress']
        self.assertTrue(all(b - a >= 10**9 for a, b in zip(progress, progress[1:])))
        self.assertLess(len(events), 15)  # A thousand samples never become stdout logs.

    def test_io_during_restoration_cannot_be_misreported_as_capture_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            result, usb, udp, text, report = self.run_collector(directory, cleanup_only=True)
        events = [imu._collector_status(line) for line in text.splitlines() if line.startswith(imu.LIVE_PREFIX)]
        fatal = next(event for event in events if event['event'] == 'fatal_read')
        self.assertEqual((fatal['phase'], fatal['fatal_read_phase']), ('cleanup', 'cleanup'))
        self.assertEqual(fatal['paired_samples'], 0)
        labels = [event['event'] for event in events]
        self.assertLess(labels.index('cleanup_started'), labels.index('fatal_read'))
        self.assertEqual(result, 0)
        self.assertTrue(report['restored'])
        self.assertEqual(report['errors'], [])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 3, 4])  # Existing restore retry only.

    def status_line(self, **changes):
        snapshot = dict.fromkeys(imu._LIVE_COUNTS, 0)
        snapshot['last_read_status'] = 0
        lines = []
        live = collector._LiveStatus(lambda: snapshot, clock=lambda: 10**9, output=lines.append)
        live.phase = 'capture'
        live.mark('mask_verified')
        value = json.loads(lines[0][len(imu.LIVE_PREFIX):])
        value.update(changes)
        return imu.LIVE_PREFIX + json.dumps(value)

    def test_stdout_drainer_retains_live_evidence_without_final_report_and_bounds_history(self):
        lines = [self.status_line(monotonic_ns=index + 10**9) for index in range(20)]
        lines += [self.status_line(event='fatal_read', monotonic_ns=2 * 10**9,
                    fatal_read_ns=2 * 10**9, fatal_read_status=-1, fatal_read_phase='capture'),
                  self.status_line(monotonic_ns=1), imu.READY_LINE]
        stream = imu.IMUStream()
        stream._process = SimpleNamespace(stdout=io.StringIO('\n'.join(lines)), wait=Mock(), poll=lambda: None)
        stream._read_output()
        stats = stream.stats
        self.assertTrue(stream._ready.is_set())
        self.assertEqual(stats['collector_live']['event'], 'fatal_read')
        self.assertEqual(stats['collector_live']['fatal_read_phase'], 'capture')
        self.assertEqual(len(stats['collector_events']), 12)
        self.assertLess(stats['collector_live_age_s'], 1)
        self.assertIsNone(stream.report_path)  # No report file/finally was needed.

    def test_parser_rejects_unknown_raw_fields_nonfinite_types_and_unbounded_values(self):
        self.assertIsNotNone(imu._collector_status(self.status_line()))
        for changes in ({'schema': True}, {'schema': 2}, {'event': []}, {'phase': {}},
                        {'fatal_read_phase': []}, {'monotonic_ns': float('nan')},
                        {'usb_reads': float('inf')}, {'usb_bytes': -1}, {'paired_samples': True},
                        {'ssc_packets': 2**65}, {'fatal_read_status': 'USB error'}, {'raw_mask': 'private'}):
            with self.subTest(changes=changes):
                self.assertIsNone(imu._collector_status(self.status_line(**changes)))
        self.assertIsNone(imu._collector_status(imu.LIVE_PREFIX + '{broken'))
        self.assertIsNone(imu._collector_status(imu.LIVE_PREFIX + '[' + '0,' * 3000 + '0]'))


if __name__ == '__main__':
    unittest.main()
