"""Natural status exit1 uses REAL D1 quarantine semantics, never hardware."""

import json
import struct
import threading
import types
import unittest

from osmo360_camera import _standby_control as m
from osmo360_camera._amt import D1Session
from osmo360_camera._usb_bootstrap import USBBootstrap, BootstrapError
from osmo360_camera._stream_resume import _wait_native
from test_standby_control import Clock, NONCE, snapshot, proof


class NativeD1:
    """Only D1 wire replies are fake; D1Session.run/quarantine is production."""

    def __init__(self):
        self.clock, self.events, self.tasks, self.results = Clock(), [], {}, {}
        self.value = snapshot()
        self.fail_status = 0
        self.fail_guard = 0
        self.fail_hash = False
        self.failure_result = 1
        self.resolve_running = self.resolve_late = False
        self.lost_status = False
        self.failed_ids = set()
        self.adapters = []

    def request(self, cmdset, cmdid, payload, **options):
        assert (cmdset, cmdid, options["receiver"]) == (0, 0xD1, 0x68)
        sub = struct.unpack_from("<H", payload)[0]
        if sub == 0:
            command = payload[2:514].split(b"\0", 1)[0].decode("ascii")
            tid = struct.unpack_from("<I", payload, 518)[0]
            self.events.append(("START", tid, command))
            result = 0
            if " sleep " in command:
                self.value = snapshot("SLEEP", 1)
                self.fail_status = 1
            elif command.endswith(" status") and self.lost_status:
                raise OSError("lost status START reply")
            elif command.endswith(" status") and self.fail_status:
                self.fail_status -= 1
                result = self.failure_result
                self.clock.now += 2  # Native frontend's acknowledged2s timeout.
            elif "sha256sum" in command and self.fail_hash:
                result = self.failure_result
            elif "max_speed" in command and self.fail_guard:
                self.fail_guard -= 1
                result = self.failure_result
            self.tasks[tid], self.results[tid] = result, 0
            if result:
                self.failed_ids.add(tid)
            raw = struct.pack("<HB", 1, 0)
        elif sub == 4:
            tid = struct.unpack_from("<I", payload, 2)[0]
            self.events.append(("RESULT", tid))
            if tid not in self.tasks:
                raw = struct.pack("<HBB", 5, 0xE0, 0xF3)
            else:
                self.results[tid] += 1
                # First terminal reply belongs to D1Session.run. A second
                # RESULT is the controller's independent RO recovery proof.
                resolving = tid in self.failed_ids and self.results[tid] > 1
                if resolving and self.resolve_late:
                    self.clock.now += 50
                result = 0xF0 if resolving and self.resolve_running else self.tasks[tid]
                raw = struct.pack("<HBB", 5, 0, result)
        elif sub == 2:
            self.events.append(("STOP", struct.unpack_from("<I", payload, 2)[0]))
            raw = struct.pack("<HBB", 3, 0, 0xF6)
        else:
            raise AssertionError("Unexpected native subcommand")
        return types.SimpleNamespace(payload=raw)

    def adapter(self, transport=None):
        transport = transport or self
        value = USBBootstrap(
            transport, session=D1Session(transport, reuse_owned_id=True)
        )
        value.pull_bytes = lambda *a, **kw: json.dumps(self.value).encode()
        self.adapters.append(value)
        return value

    def control(self):
        value = m.StandbyControl(
            self.adapter(),
            source_provider=lambda: {},
            control_lock=threading.RLock(),
            nonce=NONCE,
            clock=self.clock,
            sleep=self.clock.sleep,
            adapter_factory=self.adapter,
        )
        value.launch_attempted = True
        value.owner, value.service, value.last = (
            self.value["owner"],
            self.value["service"],
            self.value,
        )
        return value


class StandbyStatusRetryTests(unittest.TestCase):
    def test_stock_f6_status_failure_is_not_quarantined_but_gets_independent_result_before_retry(
        self,
    ):
        native = NativeD1()
        native.failure_result = 0xF6
        control = native.control()
        control._submit_sleep(proof())
        original = control.bootstrap.session
        result = control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(result["state"], "SLEEP")
        self.assertFalse(original.quarantined)
        self.assertTrue(original.operations[-1]["reusable"])
        detail = control.last_status_error
        self.assertEqual(detail["result"], 0xF6)
        self.assertTrue(detail["completed"])
        self.assertTrue(detail["start_accepted"])
        self.assertFalse(detail["quarantined"])
        self.assertTrue(detail["reusable"])
        self.assertEqual(detail["exchanges_tail"][-1]["rx_payload_hex"], "050000f6")
        self.assertEqual(control.status_retry_count, 1)
        failed = next(iter(native.failed_ids))
        at = max(
            i
            for i, row in enumerate(native.events)
            if row[0] == "START" and row[1] == failed and row[2].endswith(" status")
        )
        results = [
            i
            for i, row in enumerate(native.events)
            if i > at and row == ("RESULT", failed)
        ]
        self.assertEqual(len(results), 2)  # First terminal, then independent RO proof.
        next_start = next(
            i for i, row in enumerate(native.events) if i > at and row[0] == "START"
        )
        self.assertGreater(next_start, results[-1])
        self.assertNotEqual(native.events[next_start][1], failed)
        self.assertEqual(
            sum(row[0] == "START" and " sleep " in row[2] for row in native.events), 1
        )
        self.assertFalse(any(row[0] == "STOP" for row in native.events))

    def test_stock_f6_hash_failure_is_not_a_status_retry_and_keeps_actual_operation_detail(
        self,
    ):
        native = NativeD1()
        native.failure_result = 0xF6
        native.fail_hash = True
        control = native.control()
        control._verify_on_snapshot = True
        with self.assertRaises(BootstrapError) as got:
            control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(control.status_retry_count, 0)
        self.assertEqual(sum(row[0] == "START" for row in native.events), 1)
        detail = got.exception.standby_status_detail
        self.assertIn("sha256sum", detail["command"])
        self.assertTrue(detail["expected_command"].endswith(" status"))
        self.assertFalse(detail["operation_matches_status"])
        self.assertFalse(detail["readonly_retry_allowed"])
        self.assertEqual(detail["result"], 0xF6)
        self.assertFalse(detail["quarantined"])

    def test_stock_f6_followed_by_running_or_late_result_never_starts_another_task(
        self,
    ):
        for field in ("resolve_running", "resolve_late"):
            native = NativeD1()
            native.failure_result = 0xF6
            control = native.control()
            control._submit_sleep(proof())
            setattr(native, field, True)
            before = sum(row[0] == "START" for row in native.events)
            with (
                self.subTest(field=field),
                self.assertRaises(m.StandbyUncertain) as got,
            ):
                control.wait("0000000000000001", "SLEEP", timeout=12)
            self.assertEqual(
                sum(row[0] == "START" for row in native.events), before + 1
            )
            self.assertEqual(control.pending_command, "0000000000000001")
            self.assertIsNotNone(control.pending_task)
            self.assertEqual(got.exception.standby_status_detail["result"], 0xF6)
            self.assertIn("resolution_error", control.last_status_error)

    def test_actual_operation_diagnostic_is_bounded_and_never_modifies_wire_record(
        self,
    ):
        native = NativeD1()
        control = native.control()
        session = control.bootstrap.session
        command = "status-" + "\U0001f680" * 10000
        operation = dict(
            command=command,
            task_id=123,
            completed=True,
            start_accepted=True,
            result=0xF6,
            reusable=True,
            error="x" * 10000,
            exchanges=[
                dict(
                    tx_payload_hex="00" * 10000,
                    rx_payload_hex="ff" * 10000,
                    status=0,
                    result=0xF6,
                )
            ]
            * 100,
        )
        session.operations.append(operation)
        session.owned_task_id = 123
        error = BootstrapError("problem-" + "\U0001f680" * 10000)
        detail = control._status_error_detail(error, "python3 fixed status")
        self.assertLess(len(json.dumps(detail, allow_nan=False).encode()), 8192)
        self.assertLessEqual(len(detail["command"]), 500)
        self.assertLessEqual(len(detail["error"]), 384)
        self.assertEqual(len(detail["exchanges_tail"]), 4)
        self.assertTrue(
            all(len(row["rx_payload_hex"]) <= 128 for row in detail["exchanges_tail"])
        )
        self.assertEqual(operation["command"], command)
        self.assertIsNot(error.standby_status_detail, detail)
        self.assertEqual(native.events, [])

    def test_real_nonzero_quarantine_resolved_before_fresh_status_without_repeating_sleep(
        self,
    ):
        native = NativeD1()
        control = native.control()
        control._submit_sleep(proof())
        original = control.bootstrap.session
        self.assertEqual(control.pending_command, "0000000000000001")
        value = control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(value["state"], "SLEEP")
        self.assertIsNone(control.pending_command)
        self.assertTrue(original.quarantined)
        self.assertEqual(
            (original.operations[-1]["completed"], original.operations[-1]["result"]),
            (True, 1),
        )
        self.assertTrue(original.operations[-1]["start_accepted"])
        self.assertEqual(control.status_retry_count, 1)
        self.assertIsNotNone(control.last_status_error)
        starts = [row for row in native.events if row[0] == "START"]
        self.assertEqual(sum(" sleep " in row[2] for row in starts), 1)
        self.assertEqual(
            sum(row[2].endswith(" status") for row in starts), 3
        )  # Before input + failed + recovered.
        failed = next(iter(native.failed_ids))
        failed_start = max(
            i
            for i, row in enumerate(native.events)
            if row[0] == "START" and row[1] == failed and row[2].endswith(" status")
        )
        first = native.events.index(("RESULT", failed), failed_start + 1)
        after = native.events.index(("RESULT", failed), first + 1)
        self.assertEqual(
            native.events[after + 1][0], "RESULT"
        )  # New task's missing-ID preflight.
        self.assertFalse(any(row[0] == "STOP" for row in native.events))

    def test_conclusive_failure_followed_by_running_result_never_starts_another_task(
        self,
    ):
        native = NativeD1()
        control = native.control()
        control._submit_sleep(proof())
        native.resolve_running = True
        before = sum(row[0] == "START" for row in native.events)
        with self.assertRaises(m.StandbyUncertain):
            control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(sum(row[0] == "START" for row in native.events), before + 1)
        self.assertEqual(control.pending_command, "0000000000000001")
        self.assertIn(control.pending_task, native.failed_ids)
        self.assertTrue(control.bootstrap.session.quarantined)

    def test_unknown_lost_status_reply_is_not_misclassified_as_completed_frontend_exit1(
        self,
    ):
        native = NativeD1()
        control = native.control()
        control._submit_sleep(proof())
        native.lost_status = True
        before = sum(row[0] == "START" for row in native.events)
        with self.assertRaises(OSError):
            control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(sum(row[0] == "START" for row in native.events), before + 1)
        self.assertEqual(control.status_retry_count, 0)
        self.assertEqual(control.pending_command, "0000000000000001")

    def test_source_hash_exit1_and_late_resolved_terminal_do_not_allow_status_replay(
        self,
    ):
        native = NativeD1()
        control = native.control()
        control._verify_on_snapshot = True
        native.fail_hash = True
        with self.assertRaises(BootstrapError):
            control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(sum(row[0] == "START" for row in native.events), 1)
        self.assertEqual(control.status_retry_count, 0)
        native = NativeD1()
        control = native.control()
        control._submit_sleep(proof())
        native.resolve_late = True
        before = sum(row[0] == "START" for row in native.events)
        with self.assertRaises(m.StandbyUncertain):
            control.wait("0000000000000001", "SLEEP", timeout=12)
        self.assertEqual(sum(row[0] == "START" for row in native.events), before + 1)

    def test_actual_native_guard_nonzero_quarantine_is_resolved_before_readonly_recheck(
        self,
    ):
        native = NativeD1()
        control = native.control()
        native.fail_guard = 2
        _wait_native(control, clock=native.clock, sleep=native.clock.sleep)
        starts = [row for row in native.events if row[0] == "START"]
        self.assertEqual(len(starts), 3)
        self.assertTrue(all("max_speed" in row[2] for row in starts))
        self.assertEqual(native.clock.now, 1)  # Two bounded .5s polls.
        for tid in native.failed_ids:
            at = next(
                i
                for i, row in enumerate(native.events)
                if row[0] == "START" and row[1] == tid
            )
            self.assertEqual(
                sum(row == ("RESULT", tid) for row in native.events[at + 1 :]), 2
            )
        self.assertEqual(len(native.adapters), 3)
        self.assertFalse(any(row[0] == "STOP" for row in native.events))


if __name__ == "__main__":
    unittest.main()
