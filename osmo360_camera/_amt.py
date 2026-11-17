"""Internal AMT task transport for camera helper lifecycle operations.

Not a public Camera API or a command-line shell. Callers supply reviewed,
bounded diagnostic/bridge commands. Never authenticates, changes firmware,
retries a delivered START, or resets another task. The injected transport must
correlate DUML sequence, command and addresses before returning a reply.

``reuse_owned_id=True`` avoids consuming a retained native task slot for every
command. Only natural completion (00/F6) permits reuse; timeout, STOP or an
uncertain exchange quarantines this session. A new session is not a recovery
mechanism for a task that might still be running.
The owned ID is valid only for this unchanged transport/AMT process lifetime;
discard this session after any reconnect, reboot or service restart.
"""

import secrets
import struct
import threading
import time
from ._amt_codec import decode_reply, result_payload, stop_payload


def wait_ready(transport, *, timeout=20):
    """Wait for AMT after boot using RESULT for one missing ID, never START.

    The normal control service can become ready before the factory diagnostic
    client. Only a transport RequestTimeout is retried; malformed/unsupported
    replies fail immediately. This probe allocates no native task slot.
    """
    from .transport import RequestTimeout

    if type(timeout) not in (int, float) or not 0 < timeout <= 60:
        raise ValueError("AMT readiness timeout must be0..60seconds")
    started = time.monotonic()
    deadline = started + timeout
    payload = result_payload(secrets.randbelow(0xFFFFFFFE) + 1)
    attempts = 0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RequestTimeout("AMT did not become ready after boot")
        attempts += 1
        try:
            reply = transport.request(
                0, 0xD1, payload, receiver=0x68, timeout=min(1, remaining)
            ).payload
        except RequestTimeout:
            if not transport.connected:
                raise
            time.sleep(min(0.1, max(0, deadline - time.monotonic())))
            continue
        decoded = decode_reply(reply, expected_subcommand=5)
        if time.monotonic() >= deadline:
            raise RequestTimeout("AMT readiness reply arrived after deadline")
        if (decoded.status, decoded.result) != (0xE0, 0xF3):
            raise RuntimeError("Unexpected AMT readiness response:" + reply.hex())
        return dict(
            ready=True,
            attempts=attempts,
            elapsed=time.monotonic() - started,
            probe="read-only RESULT for an unallocated task ID",
        )


class D1Session:
    def __init__(self, transport, *, reuse_owned_id=False):
        if type(reuse_owned_id) is not bool:
            raise ValueError("reuse_owned_id must be a bool")
        self.transport = transport
        self.operations = []
        self.reuse_owned_id = reuse_owned_id
        self.owned_task_id = None
        self.quarantined = False
        self.quarantine_reason = None
        self._owned_preflight_complete = False
        self._owned_reusable = False
        self._lock = threading.Lock()

    def _quarantine(self, op, reason):
        if self.reuse_owned_id:
            self.quarantined = True
            self.quarantine_reason = reason
            self._owned_reusable = False
            op.update(quarantined=True, quarantine_reason=reason, reusable=False)

    def run(self, command, *, timeout_ms=3000, pre_send=None):
        """Run one reviewed command; retain the existing operation report API.

        With reuse enabled, all runs are serialized around one initially missing
        ID. Explicit E4 rejections after an owned natural completion may be
        retried for at most one second while the previous worker finishes its
        timer cleanup. No accepted/uncertain START is ever resent.
        """
        if not isinstance(command, str) or "\0" in command:
            raise ValueError("command must be a NUL-free string")
        if pre_send is not None and not callable(pre_send):
            raise TypeError("Optional delivery guard must be callable")
        raw = command.encode("ascii") + b"\0"
        if (
            not 1 <= len(raw) <= 501
            or type(timeout_ms) is not int
            or not 100 <= timeout_ms <= 10000
        ):
            raise ValueError("bounded ASCII command and100..10000ms timer required")
        with self._lock:
            if self.quarantined:
                raise RuntimeError(
                    "D1 reuse session quarantined: " + self.quarantine_reason
                )
            return self._run(command, raw, timeout_ms, pre_send)

    def _run(self, command, raw, timeout_ms, pre_send=None):
        if self.reuse_owned_id:
            if self.owned_task_id is None:
                self.owned_task_id = secrets.randbelow(0xFFFFFFFE) + 1
            tid = self.owned_task_id
        else:
            tid = secrets.randbelow(0xFFFFFFFE) + 1
        reused = self.reuse_owned_id and self._owned_preflight_complete
        allow_busy_retry = reused and self._owned_reusable
        self._owned_reusable = False
        limit = time.monotonic() + timeout_ms / 1000 + 4
        op = {
            "command": command,
            "task_id": tid,
            "timeout_ms": timeout_ms,
            "exchanges": [],
            "start_attempts": 0,
            "start_accepted": False,
            "reuse_owned_id": self.reuse_owned_id,
            "reused_task_id": reused,
            "preflight_performed": False,
            "quarantined": False,
            "reusable": False,
        }
        self.operations.append(op)
        attempted = False

        def query(payload, expected, deadline):
            nonlocal attempted
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("D1 host deadline")
            row = {"tx_payload_hex": payload.hex()}
            op["exchanges"].append(row)
            reply = self.transport.request(
                0, 0xD1, payload, receiver=0x68, timeout=min(2, remaining)
            ).payload
            row["rx_payload_hex"] = reply.hex()
            r = decode_reply(reply, expected_subcommand=expected)
            row.update(status=r.status, result=r.result)
            # A late but definitive rejection/terminal result still quarantines
            # reuse, but must not STOP a rejected or already-finishing worker.
            if (expected == 1 and r.status) or (
                expected == 5 and not r.status and r.result != 0xF0
            ):
                attempted = False
            if time.monotonic() >= deadline:
                raise TimeoutError("D1 response arrived after host deadline")
            return r

        try:
            if not reused:
                op["preflight_performed"] = True
                r = query(result_payload(tid), 5, limit)
                if (r.status, r.result) != (0xE0, 0xF3):
                    raise RuntimeError("Task ID unavailable")
                if self.reuse_owned_id:
                    self._owned_preflight_complete = True
            body = (
                b"\0\0"
                + raw.ljust(512, b"\0")
                + struct.pack("<IIIH", len(raw), tid, timeout_ms, 0)
            )
            busy_deadline = None
            while True:
                start_deadline = (
                    min(limit, busy_deadline) if busy_deadline is not None else limit
                )
                if time.monotonic() >= start_deadline:
                    raise TimeoutError("D1 prior worker cleanup deadline")
                if pre_send is not None:
                    try:
                        pre_send()
                    except BaseException:
                        op["delivery_guard_rejected"] = True
                        raise
                attempted = True
                op["start_attempts"] += 1
                r = query(body, 1, start_deadline)
                if not r.status:
                    op["start_accepted"] = True
                    break
                attempted = False
                if r.status == 0xE4 and allow_busy_retry:
                    if busy_deadline is None:
                        busy_deadline = min(limit, time.monotonic() + 1)
                    remaining = busy_deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("D1 prior worker cleanup deadline")
                    time.sleep(min(0.05, remaining))
                    continue
                raise RuntimeError(f"D1 START rejected:{r.raw.hex()}")
            while True:
                r = query(result_payload(tid), 5, limit)
                if r.status:
                    raise RuntimeError(f"D1 result rejected:{r.raw.hex()}")
                if r.result != 0xF0:
                    op.update(completed=True, result=r.result, success=r.result == 0)
                    if self.reuse_owned_id and r.result in (0, 0xF6):
                        self._owned_reusable = True
                        op["reusable"] = True
                    elif self.reuse_owned_id:
                        self._quarantine(
                            op, f"non-reusable native result {r.result:02x}"
                        )
                    return op
                time.sleep(min(0.05, max(0, limit - time.monotonic())))
        except BaseException as error:
            op["error"] = f"{type(error).__name__}: {error}"
            if op.get("delivery_guard_rejected") and not attempted:
                self._owned_reusable = allow_busy_retry
            else:
                self._quarantine(op, op["error"])
            if attempted:
                try:
                    r = query(stop_payload(tid), 3, time.monotonic() + 1)
                    op["cleanup_reply"] = r.raw.hex()
                except Exception as cleanup:
                    op["cleanup_error"] = f"{type(cleanup).__name__}: {cleanup}"
            raise
