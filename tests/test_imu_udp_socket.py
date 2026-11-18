"""Bounded per-socket buffering and truthful Linux UDP receive-loss reports."""

from contextlib import ExitStack
import errno
import socket
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import imu
from osmo360_camera._imu_wire import pack_sample
from test_imu_wire import sample_record


class OptionSocket:
    def __init__(self, *, force_errno=None, buffer_errno=None, overflow_errno=None,
                 readback_errno=None):
        self.force_errno, self.buffer_errno = force_errno, buffer_errno
        self.overflow_errno, self.readback_errno = overflow_errno, readback_errno
        self.options, self.effective, self.closed = [], None, False

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))
        failure = {33: self.force_errno, 40: self.overflow_errno,
                   socket.SO_RCVBUF: self.buffer_errno}.get(option)
        if failure is not None:
            raise OSError(failure, 'injected socket option failure')
        if option == 33:
            self.effective = value * 2
        elif option == socket.SO_RCVBUF:
            self.effective = 425984  # Example Linux rmem_max-clamped readback.

    def getsockopt(self, level, option):
        if self.readback_errno is not None:
            raise OSError(self.readback_errno, 'injected readback failure')
        return self.effective

    def close(self):
        self.closed = True


class AncillarySocket(OptionSocket):
    def recvmsg(self, *args):
        raise AssertionError('Configuration never receives a datagram')


def linux():
    context = ExitStack()
    context.enter_context(patch.object(imu.sys, 'platform', 'linux'))
    context.enter_context(patch.object(imu.platform, 'machine', return_value='aarch64'))
    context.enter_context(patch.object(imu.socket, 'SO_RCVBUFFORCE', None, create=True))
    context.enter_context(patch.object(imu.socket, 'SO_RXQ_OVFL', None, create=True))
    return context


class IMUSocketConfigurationTests(unittest.TestCase):
    def test_force_uses_same_bounded_request_and_observes_doubled_readback(self):
        stream, sock = imu.IMUStream(), AncillarySocket()
        with linux():
            stream._configure_udp_socket(sock)
        self.assertEqual(sock.options, [(socket.SOL_SOCKET, 33, 4 * 1024 * 1024),
                                        (socket.SOL_SOCKET, 40, 1)])
        report = stream.stats['udp_socket']
        self.assertEqual(report['requested_rcvbuf_bytes'], 4 * 1024 * 1024)
        self.assertEqual(report['effective_rcvbuf_bytes'], 8 * 1024 * 1024)
        self.assertEqual(report['buffer_method'], 'SO_RCVBUFFORCE')
        self.assertTrue(report['force_attempted'])
        self.assertIsNone(report['force_errno'])
        self.assertTrue(report['rxq_overflow'])
        self.assertEqual(report['kernel_drop_source'], 'SO_RXQ_OVFL')
        self.assertEqual(report['kernel_drop_counter_bits'], 32)
        self.assertIsNone(stream.stats['kernel_drops'])  # No report was observed.
        report['effective_rcvbuf_bytes'] = 1
        self.assertEqual(stream.stats['udp_socket']['effective_rcvbuf_bytes'], 8 * 1024 * 1024)

    def test_force_eperm_falls_back_without_hiding_kernel_cap(self):
        stream, sock = imu.IMUStream(), AncillarySocket(force_errno=errno.EPERM)
        with linux():
            stream._configure_udp_socket(sock)
        report = stream.stats['udp_socket']
        self.assertEqual(report['force_errno'], errno.EPERM)
        self.assertEqual(report['buffer_method'], 'SO_RCVBUF')
        self.assertEqual(report['effective_rcvbuf_bytes'], 425984)
        self.assertEqual(sock.options[1], (socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024))

    def test_nonlinux_never_uses_linux_numbers_even_when_names_exist(self):
        for target in ('darwin', 'win32', 'freebsd14'):
            stream, sock = imu.IMUStream(), AncillarySocket()
            with patch.object(imu.sys, 'platform', target), patch.object(
                imu.socket, 'SO_RCVBUFFORCE', 33, create=True
            ), patch.object(imu.socket, 'SO_RXQ_OVFL', 40, create=True):
                stream._configure_udp_socket(sock)
            self.assertEqual(sock.options, [(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)])
            self.assertFalse(stream.stats['udp_socket']['force_attempted'])
            self.assertFalse(stream.stats['udp_socket']['rxq_overflow'])
            self.assertIsNone(stream.stats['kernel_drops'])

    def test_unknown_linux_abi_requires_native_constant_instead_of_guessing(self):
        with linux(), patch.object(imu.platform, 'machine', return_value='unknown-abi'):
            self.assertIsNone(imu._linux_socket_option('SO_RCVBUFFORCE', 33))
            with patch.object(imu.socket, 'SO_RCVBUFFORCE', 123):
                self.assertEqual(imu._linux_socket_option('SO_RCVBUFFORCE', 33), 123)

    def test_required_existing_option_failure_closes_socket_before_collector(self):
        sock = AncillarySocket(force_errno=errno.EPERM, buffer_errno=errno.EACCES)
        with tempfile.TemporaryDirectory() as directory:
            stream = imu.IMUStream(report_dir=directory)
            with linux(), patch.object(imu.socket, 'socket', return_value=sock), patch.object(
                imu.subprocess, 'Popen', side_effect=AssertionError('collector started')
            ), self.assertRaises(OSError) as caught:
                stream.start()
        self.assertEqual(caught.exception.errno, errno.EACCES)
        self.assertTrue(sock.closed)
        self.assertTrue(stream._closed)
        self.assertEqual(stream.stats['udp_socket']['force_errno'], errno.EPERM)
        self.assertEqual(stream.stats['udp_socket']['buffer_errno'], errno.EACCES)

    def test_optional_readback_and_telemetry_failure_remain_observable(self):
        stream = imu.IMUStream()
        sock = AncillarySocket(readback_errno=errno.EIO, overflow_errno=errno.ENOPROTOOPT)
        with linux():
            stream._configure_udp_socket(sock)
        report = stream.stats['udp_socket']
        self.assertEqual(report['readback_errno'], errno.EIO)
        self.assertIsNone(report['effective_rcvbuf_bytes'])
        self.assertEqual(report['rxq_overflow_errno'], errno.ENOPROTOOPT)
        self.assertIsNone(stream._rxq_option)
        self.assertIsNone(stream.stats['kernel_drops'])
        self.assertIsNone(report['kernel_drop_source'])

    def test_no_recvmsg_leaves_telemetry_unavailable_without_option_probe(self):
        stream, sock = imu.IMUStream(), OptionSocket()
        with linux():
            stream._configure_udp_socket(sock)
        self.assertEqual(len(sock.options), 1)
        self.assertIsNone(stream._rxq_option)
        self.assertFalse(stream.stats['udp_socket']['rxq_overflow'])


class IMUKernelDropTests(unittest.TestCase):
    def replay(self, reports):
        records = [sample_record(timestamp_ticks=2**53 + n * 2400,
                                 host_receive_monotonic_ns=123456789 + n)
                   for n in range(len(reports))]
        values = [(pack_sample(record), ancillary, flags, ('127.0.0.1', 12345))
                  for record, (ancillary, flags) in zip(records, reports)]

        class Receiver:
            def __init__(self):
                self.values = iter(values)
                self.closed = False

            def recvmsg(self, size, controls):
                self.last_sizes = size, controls
                try:
                    return next(self.values)
                except StopIteration:
                    raise socket.timeout from None

            def recv(self, size):
                raise AssertionError('Telemetry receiver used recv instead of recvmsg')

            def close(self):
                self.closed = True

        stream = imu.IMUStream()
        stream._socket, stream._rxq_option = Receiver(), 40
        stream._process = SimpleNamespace(poll=lambda: 0)
        stream._receive()
        return stream, records

    @staticmethod
    def report(value):
        return [(socket.SOL_SOCKET, 40, struct.pack('=I', value))]

    def test_absent_control_data_is_not_claimed_zero_kernel_loss(self):
        stream, records = self.replay([([], 0), ([], 0)])
        self.assertIsNone(stream.stats['kernel_drops'])
        self.assertEqual(stream.stats['kernel_drop_telemetry_errors'], 0)
        self.assertEqual(stream.stats['received'], 2)
        self.assertEqual(stream.stats['queue_drops'], 0)
        self.assertEqual([stream._queue.get_nowait().to_dict() for _ in records], records)

    def test_observed_kernel_counter_is_separate_and_all_clock_value_bits_survive(self):
        stream, records = self.replay([(self.report(0), 0), (self.report(99), 0),
                                       (self.report(122), 0), ([], 0)])
        self.assertEqual(stream.stats['kernel_drops'], 122)
        self.assertEqual(stream.stats['received'], 4)
        self.assertEqual(stream.stats['queue_drops'], 0)
        for expected in records:
            actual = stream._queue.get_nowait().to_dict()
            self.assertEqual(actual, expected)
            for axis in ('ax', 'ay', 'az', 'gx', 'gy', 'gz'):
                self.assertEqual(struct.pack('<d', actual[axis]), struct.pack('<d', expected[axis]))
        self.assertTrue(stream._socket.closed)

    def test_bad_or_truncated_ancillary_never_replaces_known_counter_or_loses_payload(self):
        bad = [([(socket.SOL_SOCKET, 40, b'x')], 0),
               (self.report(200) + self.report(201), 0),
               ([(socket.SOL_SOCKET, 40, 'bad')], 0),
               ([(socket.SOL_SOCKET, 40)], 0),
               (self.report(999), socket.MSG_CTRUNC)]
        stream, records = self.replay([(self.report(99), 0), *bad,
                                       ([(socket.SOL_SOCKET + 1, 40, b'ignored')], 0)])
        self.assertEqual(stream.stats['kernel_drops'], 99)
        self.assertEqual(stream.stats['kernel_drop_telemetry_errors'], len(bad))
        self.assertEqual(stream.stats['received'], len(records))
        self.assertEqual(stream.stats['malformed_datagrams'], 0)

    def test_truncated_payload_is_not_admitted_but_kernel_report_is_retained(self):
        stream, _ = self.replay([(self.report(99), socket.MSG_TRUNC)])
        self.assertEqual(stream.stats['received'], 0)
        self.assertEqual(stream.stats['malformed_datagrams'], 1)
        self.assertEqual(stream.stats['truncated_datagrams'], 1)
        self.assertEqual(stream.stats['kernel_drops'], 99)

    def test_uint32_wrap_is_exposed_as_raw_kernel_counter_without_invented_total(self):
        stream, _ = self.replay([(self.report(2**32 - 2), 0), (self.report(3), 0)])
        self.assertEqual(stream.stats['kernel_drops'], 3)
        self.assertEqual(stream.stats['kernel_drop_telemetry_errors'], 0)


if __name__ == '__main__':
    unittest.main()
