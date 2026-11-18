"""Lossless private batching, per-sample queues and unchanged native cleanup."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import queue
import socket
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import _diag_collector as collector
from osmo360_camera._imu_queue import RecentSamples
from osmo360_camera._imu_wire import PACKET, pack_sample
from osmo360_camera.imu import IMUSample, IMUStream, TICK_HZ
from test_imu_api import FakeCollector, wait_for
from test_imu_live_status import Clock, DatagramSocket, FakeUSB
from test_imu_wire import sample_record


def records(count, *, first=2**53, host=123456789):
    return [sample_record(timestamp_ticks=first + index * 2400,
                          host_receive_monotonic_ns=host + index // 64)
            for index in range(count)]


def chunks(values, size):
    return [b''.join(pack_sample(v) for v in values[offset:offset + size])
            for offset in range(0, len(values), size)]


class RecentSamplesTests(unittest.TestCase):
    def test_capacity_is_samples_with_exact_newest_evictions(self):
        target = RecentSamples(3)
        self.assertEqual(target.put_many_recent([1, 2]), 0)
        self.assertEqual(target.put_many_recent([3, 4]), 1)
        self.assertEqual(target.put_many_recent([]), 0)
        self.assertEqual(target.qsize(), 3)
        self.assertEqual([target.get_nowait() for _ in range(3)], [2, 3, 4])
        self.assertEqual(target.put_many_recent(range(10)), 7)
        self.assertEqual([target.get_nowait() for _ in range(3)], [7, 8, 9])
        self.assertTrue(target.empty())
        with self.assertRaises(queue.Empty):
            target.get_nowait()
        with self.assertRaises(queue.Empty):
            target.get(timeout=.001)
        target.put_nowait('first')
        target.put_nowait('second')
        target.put_nowait('third')
        with self.assertRaises(queue.Full):
            target.put_nowait('no space')
        self.assertEqual(target.get(), 'first')

    def test_batch_wakes_waiting_consumers_and_no_external_lock_is_held(self):
        target = RecentSamples(4)
        entered = threading.Barrier(3)
        received, failures = [], []
        def consume():
            entered.wait()
            try:
                value = target.get(timeout=1)
                # Would deadlock if get() returned while retaining condition.
                target.qsize()
                received.append(value)
            except Exception as error:
                failures.append(error)
        workers = [threading.Thread(target=consume) for _ in range(2)]
        for worker in workers:
            worker.start()
        entered.wait()
        self.assertEqual(target.put_many_recent([1, 2]), 0)
        for worker in workers:
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(sorted(received), [1, 2])
        self.assertEqual(failures, [])

    def test_concurrent_consumption_and_eviction_counts_account_for_every_sample(self):
        target, done = RecentSamples(9), threading.Event()
        consumed = []
        def consume():
            while not done.is_set() or not target.empty():
                try:
                    consumed.append(target.get(timeout=.01))
                except queue.Empty:
                    continue
        worker = threading.Thread(target=consume)
        worker.start()
        dropped = 0
        for offset in range(0, 1000, 7):
            dropped += target.put_many_recent(range(offset, min(offset + 7, 1000)))
        done.set()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(consumed) + dropped, 1000)
        self.assertEqual(consumed, sorted(set(consumed)))
        self.assertEqual(consumed[-1], 999)


class IMUOwnedBatchTests(unittest.TestCase):
    def replay(self, datagrams, *, capacity=4096, reports=None):
        reports = reports if reports is not None else [([], 0)] * len(datagrams)
        values = iter((data, ancillary, flags, ('127.0.0.1', 12345))
                      for data, (ancillary, flags) in zip(datagrams, reports))
        class Receiver:
            def recvmsg(self, *args):
                try:
                    return next(values)
                except StopIteration:
                    raise socket.timeout from None
            def close(self):
                pass
        stream = IMUStream(queue_size=capacity)
        stream._socket, stream._rxq_option = Receiver(), 40
        stream._process = SimpleNamespace(poll=lambda: 0)
        stream._receive()
        return stream

    def test_single_full_and_partial_batches_keep_all_fields_float_bits_and_clocks(self):
        expected = records(137)
        stream = self.replay(chunks(expected, 64))
        self.assertEqual(stream.stats['received'], 137)
        self.assertEqual(stream.stats['queue_drops'], 0)
        self.assertEqual(stream.stats['first_host_receive_monotonic_ns'], expected[0]['host_receive_monotonic_ns'])
        self.assertEqual(stream.stats['last_host_receive_monotonic_ns'], expected[-1]['host_receive_monotonic_ns'])
        self.assertEqual(stream.stats['max_native_interval_ticks'], 2400)
        self.assertEqual(stream.stats['gaps_over_5ms'], 0)
        self.assertIsNone(stream.stats['kernel_drops'])
        for value in expected:
            sample = stream._queue.get_nowait()
            self.assertEqual(sample.to_dict(), value)
            self.assertEqual(pack_sample(sample.to_dict()), pack_sample(value))
        # Public single-datagram helpers did not acquire batched semantics.
        with self.assertRaises(ValueError):
            IMUSample.from_datagram(chunks(expected[:2], 2)[0])

    def test_bad_later_record_rejects_whole_datagram_before_admission(self):
        expected = records(6)
        first, bad, last = chunks(expected[:1], 1)[0], chunks(expected[1:4], 3)[0], chunks(expected[4:], 2)[0]
        corrupt = bytearray(bad)
        struct.pack_into('<d', corrupt, PACKET.size + 24, float('nan'))
        stream = self.replay([first, bytes(corrupt), last])
        self.assertEqual(stream.stats['received'], 3)
        self.assertEqual(stream.stats['malformed_datagrams'], 1)
        self.assertEqual([stream._queue.get_nowait().timestamp_ticks for _ in range(3)],
                         [expected[i]['timestamp_ticks'] for i in (0, 4, 5)])
        self.assertEqual(stream.stats['last_timestamp_ticks'], expected[-1]['timestamp_ticks'])

    def test_bounds_unknown_versions_and_truncation_cannot_admit_partial_samples(self):
        expected = records(65)
        one = pack_sample(expected[0])
        malformed = [one + b'x', chunks(expected, 65)[0],
                     one + one[:7] + b'\x02' + one[8:]]
        stream = self.replay(malformed + [one],
            reports=[([], 0)] * 3 + [([(socket.SOL_SOCKET, 40, struct.pack('=I', 99))], socket.MSG_TRUNC)])
        self.assertEqual(stream.stats['received'], 0)
        self.assertEqual(stream.stats['queued'], 0)
        self.assertEqual(stream.stats['malformed_datagrams'], 4)
        self.assertEqual(stream.stats['truncated_datagrams'], 1)
        self.assertEqual(stream.stats['kernel_drops'], 99)
        with self.assertRaises(ValueError):
            IMUStream(queue_size=1)._samples_from_datagram(one + one)

    def test_tick_gaps_and_nonincreasing_ticks_are_counted_inside_and_between_batches(self):
        ticks = [1000, 3400, 3400, 1000, 100000, 102400, 300000]
        values = [sample_record(timestamp_ticks=tick) for tick in ticks]
        stream = self.replay(chunks(values, 4))
        self.assertEqual(stream.stats['received'], len(ticks))
        self.assertEqual(stream.stats['nonincreasing_ticks'], 2)
        self.assertEqual(stream.stats['gaps_over_5ms'], 2)
        self.assertEqual(stream.stats['max_native_interval_ticks'], 197600)

    def test_kernel_reports_stay_datagrams_without_scaling_or_inventing_baselines(self):
        expected = records(8)
        report = lambda value: [(socket.SOL_SOCKET, 40, struct.pack('=I', value))]
        stream = self.replay(chunks(expected, 2), reports=[
            (report(2**32 - 2), 0), ([], 0), (report(3), 0), (report(999), socket.MSG_CTRUNC)])
        self.assertEqual(stream.stats['received'], 8)
        self.assertEqual(stream.stats['kernel_drops'], 3)
        self.assertEqual(stream.stats['kernel_drop_telemetry_errors'], 1)
        self.assertEqual(stream.stats['malformed_datagrams'], 0)

    def test_queue_capacity_and_actual_evictions_are_samples(self):
        expected = records(9)
        stream = self.replay(chunks(expected, 3), capacity=3)
        self.assertEqual(stream.stats['received'], 9)
        self.assertEqual(stream.stats['queued'], 3)
        self.assertEqual(stream.stats['queue_drops'], 6)
        self.assertEqual([stream._queue.get_nowait().timestamp_ticks for _ in range(3)],
                         [v['timestamp_ticks'] for v in expected[-3:]])

    def test_live_callback_queue_drops_are_independent_and_owned_cli_caps_batch(self):
        entered, release = threading.Event(), threading.Event()
        seen, collectors = [], []
        def callback(sample):
            seen.append(sample.timestamp_ticks)
            if len(seen) == 1:
                entered.set()
                release.wait(2)
        def factory(command, **options):
            value = FakeCollector(command)
            collectors.append(value)
            return value
        expected = records(9)
        try:
            with tempfile.TemporaryDirectory() as folder, patch(
                'osmo360_camera.imu.subprocess.Popen', side_effect=factory
            ), IMUStream(queue_size=4, callback=callback, report_dir=folder) as stream:
                command = collectors[-1].command
                self.assertEqual(command[command.index('--udp-batch-size') + 1], '4')
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    sender.sendto(pack_sample(expected[0]), ('127.0.0.1', stream.udp_port))
                    self.assertTrue(entered.wait(1))
                    for block in chunks(expected[1:], 4):
                        sender.sendto(block, ('127.0.0.1', stream.udp_port))
                    wait_for(lambda: stream.stats['received'] == 9)
                    self.assertEqual(stream.stats['queue_drops'], 5)
                    self.assertEqual(stream.stats['callback_drops'], 4)
                    release.set()
                    wait_for(lambda: len(seen) == 5)
                self.assertEqual(seen, [expected[i]['timestamp_ticks'] for i in (0, 5, 6, 7, 8)])
        finally:
            release.set()

    def test_end_to_end_producer_batches_real_udp_receiver_and_public_single_sample_read(self):
        def factory(command, **options):
            return FakeCollector(command)
        expected = records(137)
        with tempfile.TemporaryDirectory() as folder, patch(
            'osmo360_camera.imu.subprocess.Popen', side_effect=factory
        ), IMUStream(queue_size=512, report_dir=folder) as stream:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                emitted = 0
                for offset in range(0, len(expected), 75):
                    batch = collector._UDPSampleBatch(sender, ('127.0.0.1', stream.udp_port), 64)
                    for value in expected[offset:offset + 75]:
                        batch.add(pack_sample(value))
                    batch.flush()
                    emitted += batch.sent
                    self.assertEqual(batch.dropped, 0)
                self.assertEqual(emitted, 137)
                received = [stream.read(timeout=1) for _ in expected]
            self.assertEqual([pack_sample(v.to_dict()) for v in received],
                             [pack_sample(v) for v in expected])
            self.assertEqual(stream.stats['received'], 137)
            self.assertEqual(stream.stats['gaps_over_5ms'], 0)
            self.assertEqual(stream.stats['queue_drops'], 0)
            self.assertEqual(stream.stats['malformed_datagrams'], 0)


class CollectorBatchTests(unittest.TestCase):
    def test_send_and_drop_counts_are_samples_and_tail_flush_does_not_cross_reads(self):
        class Sender(DatagramSocket):
            def sendto(self, data, address):
                super().sendto(data, address)
                if len(self.sent) == 2:
                    raise BlockingIOError('test socket backpressure')
                return len(data)
        sender = Sender()
        expected = records(150)
        sent = dropped = 0
        # Two USB reads: each partial batch must leave in that same read.
        for group in (expected[:75], expected[75:]):
            batch = collector._UDPSampleBatch(sender, ('127.0.0.1', 1234), 64)
            for value in group:
                batch.add(pack_sample(value))
            batch.flush()
            batch.flush()  # Never retransmit a flushed/failed block.
            self.assertEqual(batch.pending, [])
            sent += batch.sent
            dropped += batch.dropped
        self.assertEqual([len(data) // PACKET.size for data in sender.sent], [64, 11, 64, 11])
        self.assertEqual((sent, dropped), (139, 11))
        self.assertEqual(b''.join(sender.sent), b''.join(pack_sample(v) for v in expected))

    def run_collector(self, folder, *, interrupt_after=None, udp=None):
        output, clock = io.StringIO(), Clock()
        udp = udp if udp is not None else DatagramSocket()
        usb = FakeUSB(clock, output)
        prefix = Path(folder) / 'capture'
        arguments = ['--seconds', '0', '--prefix', str(prefix), '--paired',
                     '--udp-port', '12345', '--udp-format', 'binary', '--udp-batch-size', '64',
                     '--no-raw', '--no-packet-jsonl']
        original, calls = collector._sample_outputs, 0
        def outputs(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == interrupt_after:
                raise KeyboardInterrupt('test stop between serialised pairs')
            return original(*args, **kwargs)
        with (
            patch.object(collector.osmo_usb, 'load_usb', return_value=usb),
            patch.object(collector, '_DiagRebindGuard', side_effect=RuntimeError('no test rebind')),
            patch.object(collector.socket, 'socket', return_value=udp),
            patch.object(collector.time, 'monotonic', side_effect=clock.seconds),
            patch.object(collector.time, 'monotonic_ns', side_effect=clock.ns),
            patch.object(collector, '_sample_outputs', side_effect=outputs),
            patch.object(sys, 'argv', ['collector', *arguments]), redirect_stdout(output),
        ):
            result = collector.main()
        return result, usb, udp, json.loads(prefix.with_suffix('.json').read_text())

    def test_real_collector_framing_flushes_partial_and_preserves_native_request_sequence(self):
        with tempfile.TemporaryDirectory() as folder:
            result, usb, udp, report = self.run_collector(folder)
        self.assertEqual([len(data) // PACKET.size for data in udp.sent], [64] * 15 + [40])
        samples = [IMUSample.from_datagram(data[offset:offset + PACKET.size])
                   for data in udp.sent for offset in range(0, len(data), PACKET.size)]
        self.assertEqual([v.timestamp_ticks for v in samples], [TICK_HZ + i * 2400 for i in range(1000)])
        self.assertEqual(len({v.host_receive_monotonic_ns for v in samples}), 1)
        self.assertEqual(report['collector_live']['udp_sent'], 1000)
        self.assertEqual(report['udp_drops'], 0)
        self.assertEqual(report['udp_batch_size'], 64)
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])
        self.assertEqual(result, 1)  # Existing later injected DIAG IO failure.

    def test_pending_tail_flushes_on_capture_interrupt_without_replaying_or_changing_restore(self):
        with tempfile.TemporaryDirectory() as folder:
            result, usb, udp, report = self.run_collector(folder, interrupt_after=67)
        self.assertEqual([len(data) // PACKET.size for data in udp.sent], [64, 2])
        self.assertEqual(report['collector_live']['udp_sent'], 66)
        self.assertEqual(report['udp_drops'], 0)
        self.assertTrue(report['restored'])
        self.assertTrue(report['quiescent'])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])
        self.assertEqual(result, 0)

    def test_uncertain_final_send_is_not_replayed_and_prior_confirmed_counts_survive(self):
        class InterruptedSender(DatagramSocket):
            def sendto(self, data, address):
                super().sendto(data, address)
                if len(self.sent) == 16:
                    raise KeyboardInterrupt('interrupted tail send; delivery unknown')
                return len(data)
        sender = InterruptedSender()
        with tempfile.TemporaryDirectory() as folder:
            result, usb, udp, report = self.run_collector(folder, udp=sender)
        self.assertEqual([len(data) // PACKET.size for data in udp.sent], [64] * 15 + [40])
        self.assertEqual(report['collector_live']['udp_sent'], 960)
        self.assertEqual(report['udp_drops'], 0)  # Uncertain send is not a proven drop.
        self.assertTrue(report['restored'])
        self.assertTrue(report['quiescent'])
        self.assertEqual(usb.operations, [4, 3, 4, 3, 4])
        self.assertEqual(result, 0)

    def test_cli_default_remains_single_and_invalid_batch_fails_before_usb(self):
        original, observed = collector.argparse.ArgumentParser.parse_args, []
        def parse(parser, *args, **kwargs):
            result = original(parser, *args, **kwargs)
            observed.append(result)
            return result
        with tempfile.TemporaryDirectory() as folder:
            base = ['collector', '--prefix', str(Path(folder) / 'capture')]
            with patch.object(collector.argparse.ArgumentParser, 'parse_args', parse), patch.object(
                collector.osmo_usb, 'load_usb', side_effect=RuntimeError('stop before USB')
            ), patch.object(sys, 'argv', base), self.assertRaisesRegex(RuntimeError, 'before USB'):
                collector.main()
            self.assertEqual(observed[-1].udp_batch_size, 1)
            self.assertEqual(observed[-1].udp_format, 'json')
            with patch.object(collector.osmo_usb, 'load_usb') as load:
                for options in (['--udp-batch-size', '0'], ['--udp-batch-size', '65'],
                                ['--udp-batch-size', 'bad'], ['--udp-batch-size', '2'],
                                ['--udp-batch-size', '2', '--paired', '--udp-port', '1234']):
                    with self.subTest(options=options), patch.object(sys, 'argv', base + options), \
                            redirect_stdout(io.StringIO()), patch('sys.stderr', io.StringIO()), \
                            self.assertRaises(SystemExit) as caught:
                        collector.main()
                    self.assertEqual(caught.exception.code, 2)
                load.assert_not_called()


if __name__ == '__main__':
    unittest.main()
