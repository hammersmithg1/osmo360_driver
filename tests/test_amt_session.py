import contextlib
import struct
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import osmo360_camera._amt as d
from osmo360_camera.transport import RequestTimeout


MISSING = bytes.fromhex("0500e0f3")
START = bytes.fromhex("010000")
SUCCESS = bytes.fromhex("05000000")
FAILURE = bytes.fromhex("050000f6")
BUSY = bytes.fromhex("0100e4")
STOP = bytes.fromhex("030000f5")


class FakeTransport:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.calls = []
        self.now = 0.0

    def request(self, cmd_set, cmd, payload, **kwargs):
        self.calls.append((cmd_set, cmd, payload, kwargs))
        value = next(self.replies)
        if callable(value):
            value = value()
        if isinstance(value, BaseException):
            raise value
        return SimpleNamespace(payload=value)

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def subcommands(self):
        return [struct.unpack_from("<H", c[2])[0] for c in self.calls]


@contextlib.contextmanager
def fake_session(fake, *, reuse=False):
    with (
        mock.patch.object(d.time, "monotonic", fake.clock),
        mock.patch.object(d.time, "sleep", fake.sleep),
        mock.patch.object(d.secrets, "randbelow", side_effect=range(100, 200)),
    ):
        yield d.D1Session(fake, reuse_owned_id=reuse)


class D1SessionTests(unittest.TestCase):
    def test_default_preserves_fresh_id_and_preflight_each_run(self):
        fake = FakeTransport([MISSING, START, SUCCESS, MISSING, START, SUCCESS])
        with fake_session(fake) as session:
            first = session.run("id")
            second = session.run("id")
        self.assertNotEqual(first["task_id"], second["task_id"])
        self.assertEqual(fake.subcommands(), [4, 0, 4, 4, 0, 4])
        self.assertTrue(all(op["preflight_performed"] for op in session.operations))
        self.assertIsNone(session.owned_task_id)
        self.assertFalse(session.quarantined)

    def test_reuse_one_missing_preflight_and_natural_failure_is_reusable(self):
        fake = FakeTransport([MISSING, START, SUCCESS, START, FAILURE, START, SUCCESS])
        with fake_session(fake, reuse=True) as session:
            operations = [session.run("id"), session.run("false"), session.run("id")]
        self.assertEqual(len({op["task_id"] for op in operations}), 1)
        self.assertEqual(session.owned_task_id, operations[0]["task_id"])
        self.assertEqual(
            [op["preflight_performed"] for op in operations], [True, False, False]
        )
        self.assertEqual(
            [op["reused_task_id"] for op in operations], [False, True, True]
        )
        self.assertEqual([op["success"] for op in operations], [True, False, True])
        self.assertTrue(all(op["reusable"] for op in operations))
        self.assertFalse(session.quarantined)
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0, 4, 0, 4])
        for cmd_set, cmd, payload, kwargs in fake.calls:
            self.assertEqual((cmd_set, cmd, kwargs["receiver"]), (0, 0xD1, 0x68))
            if payload[:2] == b"\0\0":
                self.assertEqual(len(payload), 528)
                self.assertEqual(
                    struct.unpack_from("<I", payload, 518)[0], session.owned_task_id
                )

    def test_explicit_busy_after_natural_completion_retries_before_any_poll(self):
        fake = FakeTransport([MISSING, START, SUCCESS, BUSY, BUSY, START, SUCCESS])
        with fake_session(fake, reuse=True) as session:
            session.run("id")
            op = session.run("id")
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0, 0, 0, 4])
        self.assertEqual(op["start_attempts"], 3)
        self.assertTrue(op["start_accepted"])
        self.assertTrue(op["reusable"])
        self.assertEqual([c[2] for c in fake.calls[3:6]], [fake.calls[3][2]] * 3)

    def test_busy_wait_is_finite_and_never_stops_a_rejected_new_run(self):
        fake = FakeTransport([MISSING, START, SUCCESS] + [BUSY] * 100)
        with fake_session(fake, reuse=True) as session:
            session.run("id")
            with self.assertRaisesRegex(TimeoutError, "cleanup deadline"):
                session.run("id")
            count = len(fake.calls)
            with self.assertRaisesRegex(RuntimeError, "quarantined"):
                session.run("id")
        self.assertEqual(len(fake.calls), count)
        self.assertNotIn(2, fake.subcommands())
        self.assertLessEqual(fake.now, 1.000001)
        self.assertTrue(session.quarantined)
        self.assertTrue(session.operations[-1]["quarantined"])
        self.assertFalse(session.operations[-1]["start_accepted"])

    def test_lost_start_ack_stops_owned_id_once_and_quarantines(self):
        fake = FakeTransport([MISSING, START, SUCCESS, TimeoutError("lost ACK"), STOP])
        with fake_session(fake, reuse=True) as session:
            session.run("id")
            with self.assertRaisesRegex(TimeoutError, "lost ACK"):
                session.run("id")
            count = len(fake.calls)
            with self.assertRaisesRegex(RuntimeError, "quarantined"):
                session.run("id")
        self.assertEqual(len(fake.calls), count)
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0, 2])
        self.assertEqual(session.operations[-1]["start_attempts"], 1)
        self.assertEqual(session.operations[-1]["cleanup_reply"], STOP.hex())
        self.assertTrue(session.quarantined)

    def test_bad_result_ack_cannot_reuse_even_after_successful_stop(self):
        for bad in (b"\5", b"\3\0\0\0", bytes.fromhex("0500e000")):
            fake = FakeTransport([MISSING, START, bad, STOP])
            with self.subTest(bad=bad), fake_session(fake, reuse=True) as session:
                with self.assertRaises((RuntimeError, ValueError)):
                    session.run("id")
                self.assertTrue(session.quarantined)
                self.assertEqual(fake.subcommands(), [4, 0, 4, 2])
                count = len(fake.calls)
                with self.assertRaisesRegex(RuntimeError, "quarantined"):
                    session.run("id")
                self.assertEqual(len(fake.calls), count)

    def test_native_timeout_stop_or_unknown_result_quarantines_without_resend(self):
        for result in (0xF4, 0xF5, 0x7F):
            fake = FakeTransport([MISSING, START, bytes([5, 0, 0, result])])
            with self.subTest(result=result), fake_session(fake, reuse=True) as session:
                op = session.run("id")
                self.assertEqual(op["result"], result)
                self.assertFalse(op["success"])
                self.assertFalse(op["reusable"])
                self.assertTrue(op["quarantined"])
                self.assertTrue(session.quarantined)
                self.assertEqual(fake.subcommands(), [4, 0, 4])
                with self.assertRaisesRegex(RuntimeError, "quarantined"):
                    session.run("id")

    def test_occupied_preflight_and_initial_busy_do_not_touch_other_tasks(self):
        for replies, subcommands in (([SUCCESS], [4]), ([MISSING, BUSY], [4, 0])):
            fake = FakeTransport(replies)
            with (
                self.subTest(replies=replies),
                fake_session(fake, reuse=True) as session,
            ):
                with self.assertRaises(RuntimeError):
                    session.run("id")
                self.assertEqual(fake.subcommands(), subcommands)
                self.assertTrue(session.quarantined)

    def test_late_natural_reply_is_not_reusable(self):
        fake = FakeTransport([])

        def late_reply():
            fake.now = 8.0
            return SUCCESS

        fake.replies = iter([MISSING, START, late_reply])
        with fake_session(fake, reuse=True) as session:
            with self.assertRaisesRegex(TimeoutError, "after host deadline"):
                session.run("id")
        self.assertTrue(session.quarantined)
        self.assertEqual(fake.subcommands(), [4, 0, 4])
        self.assertFalse(session.operations[-1]["reusable"])

    def test_late_explicit_busy_quarantines_without_stopping_prior_worker(self):
        fake = FakeTransport([])

        def late_busy():
            fake.now = 8.0
            return BUSY

        fake.replies = iter([MISSING, START, SUCCESS, late_busy])
        with fake_session(fake, reuse=True) as session:
            session.run("id")
            with self.assertRaisesRegex(TimeoutError, "after host deadline"):
                session.run("id")
        self.assertTrue(session.quarantined)
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0])
        self.assertNotIn("cleanup_reply", session.operations[-1])

    def test_lock_spans_entire_owned_task_lifecycle(self):
        entered = threading.Event()
        release = threading.Event()
        second_io = threading.Event()
        errors = []

        def blocked_result():
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test release not signaled")
            return SUCCESS

        def second_start():
            second_io.set()
            return START

        fake = FakeTransport([MISSING, START, blocked_result, second_start, SUCCESS])
        with fake_session(fake, reuse=True) as session:

            def run():
                try:
                    session.run("id")
                except BaseException as error:
                    errors.append(error)

            first = threading.Thread(target=run)
            second = threading.Thread(target=run)
            first.start()
            self.assertTrue(entered.wait(1))
            second.start()
            try:
                self.assertFalse(second_io.wait(0.05))
            finally:
                release.set()
            first.join(2)
            second.join(2)
            self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0, 4])
        self.assertEqual(len(session.operations), 2)

    def test_validation_happens_before_any_io(self):
        fake = FakeTransport([])
        with self.assertRaises(ValueError):
            d.D1Session(fake, reuse_owned_id=1)
        with fake_session(fake, reuse=True) as session:
            for command, timeout in (
                ("id\0", 3000),
                ("x" * 501, 3000),
                ("id", 99),
                ("id", True),
            ):
                with (
                    self.subTest(command=command, timeout=timeout),
                    self.assertRaises(ValueError),
                ):
                    session.run(command, timeout_ms=timeout)
        self.assertEqual(fake.calls, [])
        self.assertEqual(session.operations, [])


@contextlib.contextmanager
def boot_clock(fake):
    with (
        mock.patch.object(d.time, "monotonic", fake.clock),
        mock.patch.object(d.time, "sleep", side_effect=fake.sleep) as sleep,
        mock.patch.object(d.secrets, "randbelow", return_value=99),
    ):
        yield sleep


class AMTBootReadinessTests(unittest.TestCase):
    def assert_readonly_result_requests(self, fake):
        for cmd_set, cmd, payload, kwargs in fake.calls:
            self.assertEqual((cmd_set, cmd, kwargs["receiver"]), (0, 0xD1, 0x68))
            self.assertEqual(payload, struct.pack("<HI", 4, 100))
            self.assertGreater(kwargs["timeout"], 0)
            self.assertLessEqual(kwargs["timeout"], 1)

    def test_boot_timeouts_retry_only_one_missing_id_result_until_ready(self):
        fake = FakeTransport(
            [RequestTimeout("booting"), RequestTimeout("still booting"), MISSING]
        )
        fake.connected = True
        with boot_clock(fake) as sleep:
            report = d.wait_ready(fake, timeout=3)
        self.assertEqual(report["ready"], True)
        self.assertEqual(report["attempts"], 3)
        self.assertAlmostEqual(report["elapsed"], 0.2)
        self.assertEqual(sleep.call_count, 2)
        self.assert_readonly_result_requests(fake)
        self.assertEqual(fake.subcommands(), [4, 4, 4])

    def test_boot_wait_is_bounded_and_never_retries_a_disconnected_peer(self):
        fake = FakeTransport([])
        fake.connected = True

        def timeout_response():
            fake.now += fake.calls[-1][3]["timeout"]
            return RequestTimeout("booting")

        fake.replies = iter([timeout_response] * 10)
        with boot_clock(fake):
            with self.assertRaisesRegex(RequestTimeout, "did not become ready"):
                d.wait_ready(fake, timeout=1.25)
        self.assertAlmostEqual(fake.now, 1.25)
        self.assertEqual(len(fake.calls), 2)
        self.assertAlmostEqual(fake.calls[-1][3]["timeout"], 0.15)
        self.assert_readonly_result_requests(fake)

        disconnected = FakeTransport([])
        disconnected.connected = True

        def disconnect():
            disconnected.connected = False
            return RequestTimeout("disconnected during boot")

        disconnected.replies = iter([disconnect, MISSING])
        with boot_clock(disconnected) as sleep:
            with self.assertRaisesRegex(RequestTimeout, "disconnected during boot"):
                d.wait_ready(disconnected, timeout=3)
        self.assertEqual(len(disconnected.calls), 1)
        sleep.assert_not_called()
        self.assert_readonly_result_requests(disconnected)

    def test_malformed_unsupported_or_occupied_reply_fails_without_retry(self):
        for reply in (b"\xe0", b"\3\0\xe0\xf3", SUCCESS, b"\5\0\0\xf0"):
            fake = FakeTransport([reply, MISSING])
            fake.connected = True
            with self.subTest(reply=reply), boot_clock(fake) as sleep:
                with self.assertRaises((ValueError, RuntimeError)):
                    d.wait_ready(fake, timeout=3)
                sleep.assert_not_called()
            self.assertEqual(len(fake.calls), 1)
            self.assert_readonly_result_requests(fake)

    def test_missing_task_ack_at_or_after_deadline_cannot_claim_readiness(self):
        for arrival in (1.0, 1.01):
            fake = FakeTransport([])
            fake.connected = True

            def late_reply():
                fake.now = arrival
                return MISSING

            fake.replies = iter([late_reply, MISSING])
            with self.subTest(arrival=arrival), boot_clock(fake) as sleep:
                with self.assertRaisesRegex(RequestTimeout, "after deadline"):
                    d.wait_ready(fake, timeout=1)
                sleep.assert_not_called()
            self.assertEqual(len(fake.calls), 1)
            self.assert_readonly_result_requests(fake)


if __name__ == "__main__":
    unittest.main()
