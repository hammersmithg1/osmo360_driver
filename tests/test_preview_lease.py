"""Fake-socket lease ACK and lifecycle tests; never sends network packets."""

import socket
import unittest
from unittest.mock import patch

from osmo360_camera import _preview_lease as lease_module


CONFIG = dict(
    camera_ip="192.168.253.1",
    host_ip="192.168.253.2",
    port=55000,
    nonce="0123456789abcdef" * 2,
)


class FakeClock:
    now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSocket:
    def __init__(self, replies=None, *, bind_error=None, calls=None):
        self.replies = list(replies or [])
        self.bind_error, self.closed = bind_error, False
        self.sent, self.bound, self.timeout = [], None, None
        self.calls = calls if calls is not None else []

    def bind(self, address):
        self.calls.append(("bind", address))
        if self.bind_error:
            raise self.bind_error
        self.bound = address

    def settimeout(self, value):
        self.timeout = value

    def sendto(self, data, address):
        self.sent.append((data, address))
        self.calls.append(("send", lease_module.REQUEST.unpack(data)[-1]))

    def recvfrom(self, size):
        if not self.replies:
            raise socket.timeout()
        result = self.replies.pop(0)
        return result(self) if callable(result) else result

    def close(self):
        self.closed = True
        self.calls.append(("close",))


def ack(
    *,
    source=None,
    magic=None,
    nonce=None,
    sequence=None,
    camera_now=8000.0,
    lifetime=None,
    state=None,
):
    def reply(sock):
        request_magic, request_nonce, request_sequence, opcode = (
            lease_module.REQUEST.unpack(sock.sent[-1][0])
        )
        remaining = (0 if opcode else 10) if lifetime is None else lifetime
        payload = lease_module.REPLY.pack(
            request_magic if magic is None else magic,
            request_nonce if nonce is None else nonce,
            request_sequence if sequence is None else sequence,
            camera_now,
            camera_now + remaining,
            (2 if opcode else 1) if state is None else state,
        )
        return payload, source or (CONFIG["camera_ip"], CONFIG["port"])

    return reply


class FakeThread:
    def __init__(self, *, calls=None, **options):
        self.options, self.alive = options, False
        self.calls = calls if calls is not None else []

    def start(self):
        self.alive = True

    def join(self, timeout):
        self.calls.append(("join", timeout))
        self.alive = False

    def is_alive(self):
        return self.alive


class LeaseACKTests(unittest.TestCase):
    def exchange(self, replies, opcode=0):
        lease = lease_module.PreviewLease(CONFIG)
        sock = lease._socket = FakeSocket(replies)
        return lease, sock, lambda: lease._exchange(opcode)

    def test_wire_request_and_correlated_renew_ack(self):
        lease, sock, exchange = self.exchange([ack()])
        report = exchange()
        self.assertEqual(sock.sent[0][1], ("192.168.253.1", 55000))
        self.assertEqual(
            lease_module.REQUEST.unpack(sock.sent[0][0]),
            (b"O360LSE1", bytes.fromhex(CONFIG["nonce"]), 1, 0),
        )
        self.assertEqual(
            report, dict(sequence=1, camera_monotonic=8000.0, expires=8010.0, state=1)
        )
        self.assertEqual(lease.stats["acknowledged"], 1)
        self.assertTrue(lease.healthy)

    def test_wrong_peer_length_magic_nonce_sequence_and_state_are_ignored(self):
        invalid = [
            ack(source=("192.168.1.1", 55000)),
            ack(source=("192.168.253.1", 55001)),
            (b"bad", ("192.168.253.1", 55000)),
            ack(magic=b"WRONGACK"),
            ack(nonce=bytes(16)),
            ack(sequence=0),
            ack(state=2),
            ack(camera_now=float("nan")),
            ack(camera_now=float("inf")),
            ack(lifetime=-1),
            ack(lifetime=11),
        ]
        lease, _, exchange = self.exchange([*invalid, ack()])
        self.assertEqual(exchange()["state"], 1)
        self.assertEqual(lease.stats["invalid_replies"], len(invalid))
        self.assertEqual(lease.stats["acknowledged"], 1)

    def test_active_ack_requires_the_native_ten_second_lifetime(self):
        lease, _, exchange = self.exchange([ack(lifetime=0), ack(lifetime=0.1), ack()])
        self.assertEqual(exchange()["expires"], 8010.0)
        self.assertEqual(lease.stats["invalid_replies"], 2)

    def test_close_ack_requires_closed_state_and_zero_remaining_lifetime(self):
        lease, sock, exchange = self.exchange(
            [ack(state=1), ack(lifetime=10), ack()], opcode=1
        )
        self.assertEqual(exchange()["expires"], 8000.0)
        self.assertEqual(lease_module.REQUEST.unpack(sock.sent[0][0])[-1], 1)
        self.assertEqual(lease.stats["invalid_replies"], 2)

    def test_no_valid_ack_times_out_without_claiming_health(self):
        lease, _, exchange = self.exchange([ack(sequence=42)])
        with self.assertRaises(lease_module.PreviewLeaseError):
            exchange()
        self.assertFalse(lease.healthy)
        self.assertIsNone(lease.last_ack)
        self.assertEqual(lease.stats["timeouts"], 1)

    def test_late_ack_cannot_extend_the_exchange_deadline(self):
        clock = FakeClock()

        def late(sock):
            clock.now += 0.6
            return ack()(sock)

        lease, _, exchange = self.exchange([late])
        with patch.object(lease_module.time, "monotonic", clock.monotonic):
            with self.assertRaises(lease_module.PreviewLeaseError):
                exchange()
        self.assertIsNone(lease.last_ack)

    def test_health_expires_and_stop_always_marks_unhealthy(self):
        clock = FakeClock()
        lease, _, exchange = self.exchange([ack()])
        with patch.object(lease_module.time, "monotonic", clock.monotonic):
            exchange()
            self.assertTrue(lease.healthy)
            clock.now += 4
            self.assertFalse(lease.healthy)
            lease.last_ack_monotonic = clock.now
            lease._stop.set()
            self.assertFalse(lease.healthy)


class LeaseLifecycleTests(unittest.TestCase):
    def test_guard_selected_interface_is_passed_to_strict_binding(self):
        lease = lease_module.PreviewLease(dict(CONFIG, interface="en9"))
        sock, worker = FakeSocket([ack()]), FakeThread()
        with (
            patch.object(lease_module.socket, "socket", return_value=sock),
            patch.object(lease_module.threading, "Thread", return_value=worker),
            patch.object(lease_module, "bind_socket") as bind,
        ):
            lease.start()
            bind.assert_called_once_with(sock, "192.168.253.2", "en9")
            lease.stop(close_camera=False)

    def test_interface_binding_failure_closes_without_fallback_or_heartbeat(self):
        lease = lease_module.PreviewLease(dict(CONFIG, interface="en9"))
        sock = FakeSocket()
        with (
            patch.object(lease_module.socket, "socket", return_value=sock),
            patch.object(
                lease_module, "bind_socket", side_effect=OSError("interface lost")
            ) as bind,
        ):
            with self.assertRaisesRegex(OSError, "interface lost"):
                lease.start()
        bind.assert_called_once_with(sock, "192.168.253.2", "en9")
        self.assertTrue(sock.closed)
        self.assertEqual(sock.sent, [])

    def test_start_binds_source_and_starts_only_after_ack(self):
        lease = lease_module.PreviewLease(CONFIG)
        sock, worker = FakeSocket([ack(), ack()]), FakeThread()
        with (
            patch.object(lease_module.socket, "socket", return_value=sock),
            patch.object(lease_module.threading, "Thread", return_value=worker),
        ):
            self.assertIs(lease.start(), lease)
            self.assertEqual(sock.bound, ("192.168.253.2", 0))
            self.assertTrue(worker.alive)
            self.assertEqual(lease.stats["acknowledged"], 1)
            with self.assertRaises(lease_module.PreviewLeaseError):
                lease.start()
            lease.stop()
        self.assertTrue(sock.closed)
        self.assertFalse(worker.alive)

    def test_start_bind_failure_closes_socket_and_never_starts_thread(self):
        sock = FakeSocket(bind_error=OSError("source interface absent"))
        lease = lease_module.PreviewLease(CONFIG)
        with (
            patch.object(lease_module.socket, "socket", return_value=sock),
            patch.object(lease_module.threading, "Thread") as worker,
        ):
            with self.assertRaises(OSError):
                lease.start()
            worker.assert_not_called()
        self.assertTrue(sock.closed)
        self.assertIsNone(lease._socket)
        self.assertEqual(sock.sent, [])

    def test_start_without_ack_is_finite_and_closes_socket(self):
        sock, clock = FakeSocket(), FakeClock()
        lease = lease_module.PreviewLease(CONFIG)
        with (
            patch.object(lease_module.socket, "socket", return_value=sock),
            patch.object(lease_module.time, "monotonic", clock.monotonic),
            patch.object(lease_module.time, "sleep", clock.sleep),
            patch.object(lease_module.threading, "Thread") as worker,
        ):
            with self.assertRaises(lease_module.PreviewLeaseError):
                lease.start(timeout=0.3)
            worker.assert_not_called()
        self.assertTrue(sock.closed)
        sequences = [lease_module.REQUEST.unpack(data)[2] for data, _ in sock.sent]
        self.assertEqual(sequences, list(range(1, len(sequences) + 1)))
        self.assertFalse(lease.healthy)

    def test_stop_joins_worker_before_sending_close_and_closes_socket(self):
        calls = []
        lease = lease_module.PreviewLease(CONFIG)
        lease._socket = FakeSocket([ack()], calls=calls)
        worker = lease._thread = FakeThread(calls=calls)
        worker.start()
        report = lease.stop()
        self.assertEqual(report["state"], 2)
        self.assertEqual(calls, [("join", 2), ("send", 1), ("close",)])
        self.assertIsNone(lease._socket)
        self.assertFalse(lease.healthy)
        self.assertIsNone(lease.stop())

    def test_stop_without_camera_close_sends_nothing(self):
        lease = lease_module.PreviewLease(CONFIG)
        sock = lease._socket = FakeSocket()
        lease.stop(close_camera=False)
        self.assertTrue(sock.closed)
        self.assertEqual(sock.sent, [])

    def test_bad_close_ack_still_closes_socket_and_reports_failure(self):
        lease = lease_module.PreviewLease(CONFIG)
        sock = lease._socket = FakeSocket([ack(state=1)])
        with self.assertRaises(lease_module.PreviewLeaseError):
            lease.stop()
        self.assertTrue(sock.closed)
        self.assertIsNone(lease._socket)

    def test_stopped_unstarted_instance_cannot_renew_a_new_session(self):
        lease = lease_module.PreviewLease(CONFIG)
        lease.stop()
        with patch.object(lease_module.socket, "socket") as factory:
            with self.assertRaises(lease_module.PreviewLeaseError):
                lease.start()
            factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
