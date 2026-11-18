"""Private controller for the camera's pinned RAM standby coordinator.

The caller owns/reconnects/serializes the USB transport. This module never opens
USB, discovers a camera, or configures host networking. It sends fixed commands
to the camera coordinator. Source files are verified against their pinned
hashes before use, whether copied from installed helpers or uploaded to RAM.
"""

import base64
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import re
import secrets
import shlex
import time

from . import _app
from ._amt import D1Session
from ._amt_codec import decode_reply, result_payload
from ._usb_bootstrap import USBBootstrap, BootstrapError

SOURCES = {
    "usb_standby_owner.py": "5a2199c2120a35f1b1b20bb22315dbd5b54b5e4ba89a2a9cd9f35afb77ef088e",
    "gui_qs_exit_forward_probe.py": "9db827623c2e7f9767fc8a52ba5dfc601a768fafdbc1687666b1cbe7cf26e959",
}
SYS_SHA = "f2fc2a40b8c10b0d1a8289089bb0ab6700372fa062404f44c3dcac00ad513029"
WORK = r"/dev/shm/osmo-usb-standby-([0-9a-f]{12})"
STATES = frozenset(
    ("AWAKE", "SLEEP", "SLEEPING", "WAKING", "INITIALIZING", "RECOVERY_PENDING")
)
Q = shlex.quote


class StandbyControlError(RuntimeError):
    pass


class StandbyUncertain(StandbyControlError):
    """A delivered/uncertain input intent is retained; never resubmit its key."""

    def __init__(self, message, *, command_id=None):
        super().__init__(message)
        self.command_id = command_id


def _token(value):
    return (
        type(value) is dict
        and set(value) == {"pid", "start"}
        and type(value["pid"]) is int
        and value["pid"] > 1
        and isinstance(value["start"], str)
        and re.fullmatch(r"[0-9]{1,24}", value["start"])
    )


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identifier(value):
    return isinstance(value, str) and bool(re.fullmatch("[0-9a-f]{16}", value))


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate snapshot JSON key")
        result[key] = value
    return result


def _diagnostic_text(value, maximum):
    if value is None:
        return None
    try:
        text = value if isinstance(value, str) else repr(value)
    except Exception:
        text = "<unprintable " + type(value).__name__ + ">"
    return text[:maximum].encode("ascii", "backslashreplace")[:maximum].decode("ascii")


def _diagnostic_scalar(value):
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        return (
            value
            if value.bit_length() <= 64
            else dict(type="int", bits=value.bit_length())
        )
    if type(value) is float and math.isfinite(value):
        return value
    return _diagnostic_text(value, 80)


def parse_snapshot(data, *, nonce, owner=None, service=None):
    """Validate actual9529 status export; stable state is its source-backed barrier.

    The owner validates its boot_id internally. A successful fresh status CLI
    cannot use an old boot's vanished RAM work. The export does not independently
    echo boot_id or both timerfd receipts, so those fields are not invented here.
    """
    if (
        not isinstance(data, (bytes, bytearray, memoryview))
        or not 0 < len(data) <= 16384
    ):
        raise ValueError("Bounded snapshot bytes required")
    value = json.loads(
        bytes(data),
        object_pairs_hook=_object,
        parse_constant=lambda text: (_ for _ in ()).throw(ValueError("Nonfinite JSON")),
    )
    if (
        type(value) is not dict
        or value.get("nonce") != nonce
        or not _token(value.get("owner"))
        or not _token(value.get("service"))
        or value.get("state") not in STATES
        or value.get("retained_shadow") is not True
        or value.get("native_restored") is not False
        or value.get("periodic_reboot") is not False
        or value.get("owner_alive") is not True
        or type(value.get("last_sequence")) is not int
        or not 0 <= value["last_sequence"] <= 0xFFFFFFFFFFFFFFFF
        or not all(
            _finite(value.get("at", {}).get(key))
            for key in ("boottime", "monotonic", "realtime")
        )
    ):
        raise ValueError("Owned live standby snapshot schema")
    if (
        owner is not None
        and value["owner"] != owner
        or service is not None
        and value["service"] != service
    ):
        raise ValueError("Original owner/system service changed")
    ready = value.get("requested_readiness")
    if (
        type(ready) is not dict
        or ready.get("service") != value["service"]
        or ready.get("verified_executable") != SYS_SHA
        or ready.get("clients_ever_ready") != 1
        or ready.get("reboot_disabled") != 0
        or any(
            type(ready.get(key)) is not int
            for key in ("clients_ever_ready", "reboot_disabled", "system_power_level")
        )
    ):
        raise ValueError("Fresh native readiness snapshot required")
    if value["state"] == "RECOVERY_PENDING" or value.get("error"):
        raise StandbyControlError("Owner requested recovery; no new input")
    sequence = value["last_sequence"]
    if sequence:
        if value.get("command_id") != f"{sequence:016x}" or value.get("action") not in (
            "sleep",
            "wake",
        ):
            raise ValueError("Current command sequence/action mismatch")
    elif value["state"] != "AWAKE":
        raise ValueError("Uninitialized owner cannot claim a stable state")
    if value["state"] in ("AWAKE", "SLEEP"):
        level = ready.get("system_power_level")
        if level not in ((0,) if value["state"] == "AWAKE" else (2, 3)):
            raise ValueError("Cached owner state disagrees with current native manager")
        if sequence:
            result = value.get("last_result", {})
            if (
                (
                    result.get("owner"),
                    result.get("nonce"),
                    result.get("command_id"),
                    result.get("action"),
                )
                != (value["owner"], nonce, value["command_id"], value["action"])
                or result.get("complete") is not True
                or type(result.get("lpc_set_calls")) is not int
                or result.get("lpc_set_calls") != 0
                or type(result.get("get_level")) is not int
                or result.get("service") != value["service"]
            ):
                raise ValueError("Stable command terminal receipt mismatch")
            pulse = result.get("pulse", {})
            action = result["action"]
            if (
                pulse.get("key"),
                pulse.get("hold_seconds"),
                pulse.get("press_release_complete"),
            ) != (357, 1.1 if action == "sleep" else 0.1, True):
                raise ValueError("Stock QS release proof missing")
            if action == "sleep":
                gui = result.get("gui", {})
                if (
                    value["state"] != "SLEEP"
                    or result.get("get_level") not in (2, 3)
                    or result.get("shadow_value") != "none"
                    or gui.get("processing3_after_intent") is not True
                    or gui.get("done3_after_intent") is not True
                ):
                    raise ValueError("Sleep GUI/native terminal proof missing")
            else:
                exited = result.get("real_exit", {})
                if (
                    value["state"] != "AWAKE"
                    or result.get("get_level") != 0
                    or result.get("shadow_value") != "exit"
                    or (
                        exited.get("completed"),
                        exited.get("bytes_written"),
                        exited.get("literal"),
                        exited.get("real_role_after"),
                    )
                    != (True, 5, "exit", "peripheral")
                ):
                    raise ValueError("Wake real EXIT/native terminal proof missing")
    return deepcopy(value)


@dataclass(frozen=True)
class _EntryProof:
    """Private orchestration evidence; this class itself performs no hardware I/O."""

    verified_at: float
    recording_state: int
    battery_present: bool
    activation_state: int
    imu_restored: bool
    preview_restored: bool
    query: bytes
    app_before: _app.QueryReply
    app_after: _app.QueryReply

    def validate(self, now):
        if (
            not _finite(self.verified_at)
            or not 0 <= now - self.verified_at <= 5
            or type(self.recording_state) is not int
            or self.recording_state != 0
            or type(self.activation_state) is not int
            or self.activation_state != 1
            or self.battery_present is not True
            or self.imu_restored is not True
            or self.preview_restored is not True
            or self.query != _app.query_payload()
            or not isinstance(self.app_before, _app.QueryReply)
            or not isinstance(self.app_after, _app.QueryReply)
            or len(self.app_before.own_records) != 1
            or self.app_after.own_records
        ):
            raise ValueError(
                "Fresh correlated idle/teardown/QUERY32 present-to-absent evidence required"
            )


class StandbyControl:
    """Private per-boot controller using ONLY a caller-owned diagnostic adapter.

    Caller holds the supplied shared control_lock and handles USB reconnection.
    This object never replaces/opens the transport. bind_bootstrap accepts only
    another caller-owned adapter and retains any old unknown task quarantine.
    detach closes no device resource and intentionally leaves stable Sleep.
    """

    def __init__(
        self,
        bootstrap,
        *,
        source_provider,
        control_lock,
        clock=time.monotonic,
        sleep=time.sleep,
        nonce=None,
        existing=None,
        adapter_factory=None,
        source_stager=None,
    ):
        if source_stager is not None and not callable(source_stager):
            raise TypeError("Optional source stager must be callable")
        self.bootstrap = bootstrap
        self._check_bootstrap(bootstrap)
        self.sources, self.lock, self.clock, self.sleep = (
            source_provider,
            control_lock,
            clock,
            sleep,
        )
        self.source_stager = source_stager
        self._adapter_factory = adapter_factory or (
            lambda transport: USBBootstrap(
                transport, session=D1Session(transport, reuse_owned_id=True)
            )
        )
        self.nonce = (
            (existing.get("nonce") if existing is not None else secrets.token_hex(6))
            if nonce is None
            else nonce
        )
        if not isinstance(self.nonce, str) or not re.fullmatch(
            "[0-9a-f]{12}", self.nonce
        ):
            raise ValueError("Exact12-hex owned nonce required")
        self.work = "/dev/shm/osmo-usb-standby-" + self.nonce
        self.owner = self.service = self.last = None
        self.pending_command = self.pending_task = None
        self.launch_attempted = self.finalize_attempted = self.detached = False
        self._owner_launch_intent = self._verify_on_snapshot = existing is not None
        self._unknown_task = False
        self.last_status_error = None
        self.status_retry_count = 0
        if existing is not None:
            if (
                type(existing) is not dict
                or existing.get("work") != self.work
                or existing.get("nonce") != self.nonce
                or not _token(existing.get("owner"))
                or not _token(existing.get("service"))
            ):
                raise ValueError(
                    "Explicit same-work/nonce/owner/service attachment receipt required"
                )
            self.owner, self.service = (
                deepcopy(existing["owner"]),
                deepcopy(existing["service"]),
            )
            self.pending_command = existing.get("pending_command")
            self.pending_task = existing.get("pending_task")
            self._unknown_task = existing.get("unknown_task", False)
            self.launch_attempted = True

    @staticmethod
    def _check_bootstrap(bootstrap):
        if (
            getattr(bootstrap.session, "transport", None) is not bootstrap.transport
            or getattr(bootstrap.session, "reuse_owned_id", False) is not True
        ):
            raise ValueError(
                "Already-owned reusable D1 session on the same transport required"
            )

    def bind_bootstrap(self, bootstrap):
        """Caller reconnected the camera; preserve prior task uncertainty first."""
        with self.lock:
            self._remember_quarantine()
            self._check_bootstrap(bootstrap)
            self.bootstrap = bootstrap

    def _remember_quarantine(self):
        session = self.bootstrap.session
        if getattr(session, "quarantined", False):
            task = getattr(session, "owned_task_id", None)
            if type(task) is not int or not 1 <= task <= 0xFFFFFFFF:
                self._unknown_task = True
            elif self.pending_task not in (None, task):
                self._unknown_task = True
            else:
                self.pending_task = task

    def resolve_pending(self, *, timeout=5):
        """Only RO RESULT; no START/STOP, key, reset or transport reconnect."""
        with self.lock:
            return self._resolve_pending(timeout=timeout)

    def _resolve_pending(self, *, timeout=5):
        if not _finite(timeout) or not 0 < timeout <= 10:
            raise ValueError("Finite0<RESULT timeout<=10 required")
        self._remember_quarantine()
        if self._unknown_task:
            raise StandbyUncertain(
                "Unknown prior task identity; no new diagnostic START"
            )
        if self.pending_task is None:
            return
        deadline = self.clock() + timeout
        while True:
            if self.clock() >= deadline:
                raise StandbyUncertain("Prior RESULT deadline; no new diagnostic START")
            raw = self.bootstrap.transport.request(
                0,
                0xD1,
                result_payload(self.pending_task),
                receiver=0x68,
                timeout=min(0.5, max(0.01, deadline - self.clock())),
            ).payload
            result = decode_reply(raw, expected_subcommand=5)
            if self.clock() >= deadline:
                raise StandbyUncertain("Prior RESULT arrived after deadline; no START")
            if (
                (result.status, result.result) == (0xE0, 0xF3)
                or result.status == 0
                and result.result != 0xF0
            ):
                # A fresh adapter is permitted only after the old task is
                # terminal/missing. It still uses the caller's existing handle.
                self.bootstrap = self._adapter_factory(self.bootstrap.transport)
                self.pending_task = None
                return
            if result.status or self.clock() >= deadline:
                raise StandbyUncertain(
                    "Prior diagnostic task remains unknown/running; no START"
                )
            self.sleep(min(0.05, max(0, deadline - self.clock())))

    def _run(self, command, timeout_ms=5000, pre_send=None):
        if self.detached:
            raise StandbyControlError(
                "Controller detached; explicitly attach the same owner"
            )
        self._resolve_pending()
        if not command.isascii() or not 0 < len(command) <= 500 or "\0" in command:
            raise ValueError("Fixed bounded owner command required")
        try:
            return (
                self.bootstrap.run_checked(
                    command, timeout_ms=timeout_ms, pre_send=pre_send
                )
                if pre_send is not None
                else self.bootstrap.run_checked(command, timeout_ms=timeout_ms)
            )
        except BaseException:
            self._remember_quarantine()
            raise

    def _status_error_detail(self, error, expected_command):
        """Bounded actual wire-operation facts, captured BEFORE adapter rotation."""
        session = self.bootstrap.session
        operations = getattr(session, "operations", None)
        op = (
            operations[-1]
            if isinstance(operations, list)
            and operations
            and type(operations[-1]) is dict
            else {}
        )
        detail = dict(
            error=_diagnostic_text(error, 384),
            error_type=type(error).__name__,
            expected_command=_diagnostic_text(expected_command, 500),
            command=_diagnostic_text(op.get("command"), 500),
            operation_matches_status=op.get("command") == expected_command,
            operation_count=len(operations) if isinstance(operations, list) else None,
            task_id=_diagnostic_scalar(op.get("task_id")),
            owned_task_id=_diagnostic_scalar(getattr(session, "owned_task_id", None)),
            result=_diagnostic_scalar(op.get("result")),
            completed=_diagnostic_scalar(op.get("completed")),
            start_accepted=_diagnostic_scalar(op.get("start_accepted")),
            success=_diagnostic_scalar(op.get("success")),
            operation_error=_diagnostic_text(op.get("error"), 192),
            quarantined=_diagnostic_scalar(getattr(session, "quarantined", None)),
            quarantine_reason=_diagnostic_text(
                getattr(session, "quarantine_reason", None), 192
            ),
            operation_quarantined=_diagnostic_scalar(op.get("quarantined")),
            operation_quarantine_reason=_diagnostic_text(
                op.get("quarantine_reason"), 192
            ),
            reusable=_diagnostic_scalar(op.get("reusable")),
            start_attempts=_diagnostic_scalar(op.get("start_attempts")),
            timeout_ms=_diagnostic_scalar(op.get("timeout_ms")),
            pending_task=_diagnostic_scalar(self.pending_task),
            at=_diagnostic_scalar(self.clock()),
            exchanges_tail=[],
        )
        exchanges = op.get("exchanges")
        if isinstance(exchanges, list):
            for row in exchanges[-4:]:
                if type(row) is not dict:
                    continue
                tx, rx = row.get("tx_payload_hex"), row.get("rx_payload_hex")
                detail["exchanges_tail"].append(
                    dict(
                        tx_subcommand_hex=tx[:4] if isinstance(tx, str) else None,
                        tx_bytes=len(tx) // 2 if isinstance(tx, str) else None,
                        rx_payload_hex=_diagnostic_text(rx, 128),
                        rx_truncated=len(rx) > 128 if isinstance(rx, str) else False,
                        status=_diagnostic_scalar(row.get("status")),
                        result=_diagnostic_scalar(row.get("result")),
                    )
                )
        self.last_status_error = detail
        self._attach_status_detail(error, detail)
        return detail

    @staticmethod
    def _attach_status_detail(error, detail):
        try:
            error.standby_status_detail = deepcopy(detail)
        except Exception:
            pass  # Diagnostics must never replace the original exception.

    def _resolve_readonly_exit1(self, command, *, deadline):
        """Permit another expected RO command after a conclusive terminal failure.

        A frontend may naturally exit before its owner acknowledges status.
        The stock worker collapses normal nonzero child exits to F6; D1 marks
        F6 reusable, while Bootstrap.run_checked correctly rejects its failure.
        Legacy explicit1 is supported too. Do not replace either adapter until
        correlated RESULT again proves the exact owned task terminal/missing.
        Unknown transport failures, a running task, and other commands are not
        permission to start another task. This method never retries an input.
        """
        with self.lock:
            session = self.bootstrap.session
            operations = getattr(session, "operations", None)
            if not isinstance(operations, list) or not operations:
                return False
            op = operations[-1]
            if (
                type(op) is not dict
                or op.get("command") != command
                or op.get("completed") is not True
                or op.get("start_accepted") is not True
                or type(op.get("result")) is not int
                or op["result"] not in (1, 0xF6)
                or op.get("error") is not None
                or op.get("task_id") != session.owned_task_id
            ):
                return False
            if op["result"] == 1 and session.quarantined is not True:
                return False
            if (
                op["result"] == 0xF6
                and session.quarantined is not True
                and op.get("reusable") is not True
            ):
                return False
            remaining = deadline - self.clock()
            if remaining <= 0:
                raise StandbyUncertain("Read-only retry deadline; no new START")
            # Natural F6 is NOT quarantined. Explicitly pin its already-owned
            # task so _resolve_pending cannot silently skip independent RESULT.
            task = session.owned_task_id
            if (
                type(task) is not int
                or not 1 <= task <= 0xFFFFFFFF
                or self.pending_task not in (None, task)
            ):
                return False
            self.pending_task = task
            self._resolve_pending(timeout=min(1, remaining))
            if self.clock() >= deadline:
                raise StandbyUncertain(
                    "Read-only terminal RESULT arrived after retry deadline"
                )
            return True

    def _script(self):
        return Q(self.work + "/usb_standby_owner.py")

    def _snapshot(self):
        if self._verify_on_snapshot:
            for name, digest in SOURCES.items():
                self._run(
                    '[ "$(sha256sum '
                    + Q(self.work + "/" + name)
                    + ' | cut -d" " -f1)" = '
                    + digest
                    + " ]"
                )
            self._verify_on_snapshot = False
        self._run("python3 " + self._script() + " status")
        raw = self.bootstrap.pull_bytes(
            "/blackbox/export_tmp/osmo_probe_" + self.nonce + ".json", max_bytes=16384
        )
        value = parse_snapshot(
            raw, nonce=self.nonce, owner=self.owner, service=self.service
        )
        if (
            self.last is not None
            and value["at"]["boottime"] < self.last["at"]["boottime"]
        ):
            raise ValueError("Owner boot clock went backwards")
        self.owner, self.service, self.last = value["owner"], value["service"], value
        if self.pending_command == value.get("command_id") and value["state"] in (
            "AWAKE",
            "SLEEP",
        ):
            self.pending_command = None
        return deepcopy(value)

    def snapshot(self):
        with self.lock:
            if not self.launch_attempted:
                raise StandbyControlError("No owned launch/attachment")
            return self._snapshot()

    def stage_sources(self, files, work, *, guard=None):
        """Copy a verified installed source group, or explicitly report absence.

        The stager uses this controller's serialized task path. Exceptions and
        an uncertain/foreign installation never fall back to another upload.
        """
        with self.lock:
            if self.source_stager is None:
                return False
            if guard is not None:
                guard()
            value = self.source_stager(self, files, work, guard=guard)
            if type(value) is not bool:
                raise TypeError(
                    "Source stager must return an explicit handled/absent bool"
                )
            if guard is not None:
                guard()
            return value

    def launch(self, *, recording_state, battery_present, activation_state):
        with self.lock:
            if self.launch_attempted or self.detached:
                raise StandbyControlError(
                    "One fresh launch only; attach an existing owner explicitly"
                )
            if (
                type(recording_state) is not int
                or recording_state != 0
                or battery_present is not True
                or type(activation_state) is not int
                or activation_state != 1
            ):
                raise ValueError(
                    "Fresh Record0/battery1/activation1 launch guards required"
                )
            files = self.sources()
            if type(files) is not dict or set(files) != set(SOURCES):
                raise ValueError("Exact two-source allowlist required")
            for name, data in files.items():
                if (
                    type(data) is not bytes
                    or not 0 < len(data) <= 98304
                    or b"\0" in data
                    or not data.isascii()
                    or hashlib.sha256(data).hexdigest() != SOURCES[name]
                ):
                    raise ValueError("Pinned camera source bytes required: " + name)
            # Retire this work before its first mutation. Upload/launch errors
            # cannot silently reuse a half-published directory or replay INIT.
            self.launch_attempted = True
            self._run(
                '[ "$(readlink -f /dev/shm)" = /dev/shm ] && '
                '[ "$(awk \'$2=="/dev/shm" {print $3}\' /proc/mounts)" = tmpfs ] && '
                "umask 077 && mkdir " + Q(self.work)
            )
            if not self.stage_sources(files, self.work):
                for name, data in files.items():
                    target = Q(self.work + "/" + name)
                    self._run("set -C && : > " + target)
                    for at in range(0, len(data), 240):
                        piece = base64.b64encode(data[at : at + 240]).decode("ascii")
                        self._run("printf %s " + piece + " | base64 -d >> " + target)
                    self._run(
                        '[ "$(sha256sum '
                        + target
                        + ' | cut -d" " -f1)" = '
                        + SOURCES[name]
                        + " ]"
                    )
            self._owner_launch_intent = True
            self._run(
                "python3 "
                + self._script()
                + " launch --record0 --battery-present1 --activation-state1",
                10000,
            )
            return self._snapshot()

    def _submit(self, action, proof=None):
        value = self._snapshot()
        if self.pending_command is not None or value["state"] != (
            "AWAKE" if action == "sleep" else "SLEEP"
        ):
            raise StandbyControlError(
                "Matching stable state and no pending input required"
            )
        number = value["last_sequence"] + 1
        if number > 0xFFFFFFFFFFFFFFFF:
            raise StandbyControlError(
                "Command sequence exhausted; explicit recovery required"
            )
        if proof is not None:
            proof.validate(
                self.clock()
            )  # Freshness follows the potentially slow status transaction.
        identifier = f"{number:016x}"
        suffix = (
            " --record0 --battery-present1 --activation-state1 --streams-closed --own-app-absent"
            if action == "sleep"
            else ""
        )
        attempted = False

        def before_send():
            nonlocal attempted
            if proof is not None:
                proof.validate(self.clock())
            self.pending_command = identifier  # BEFORE the one possibly delivered call.
            attempted = True

        try:
            self._run(
                "python3 "
                + self._script()
                + " "
                + action
                + " --command-id "
                + identifier
                + suffix,
                pre_send=before_send,
            )
        except BaseException as error:
            if not attempted:
                raise  # Proof/readiness failed before any input delivery.
            raise StandbyUncertain(
                "Owner command receipt lost/rejected; query, never repeat its key",
                command_id=identifier,
            ) from error
        return dict(
            command_id=identifier, submitted=True, input_completion_proven=False
        )

    def _submit_sleep(self, proof):
        """Only the future internal orchestrator supplies correlated teardown proof."""
        with self.lock:
            if not isinstance(proof, _EntryProof):
                raise TypeError("Internal typed teardown proof required")
            proof.validate(self.clock())
            return self._submit("sleep", proof)

    def submit_wake(self):
        with self.lock:
            return self._submit("wake")

    def wait(self, identifier, target, *, timeout=15):
        if (
            not _identifier(identifier)
            or target not in ("AWAKE", "SLEEP")
            or not _finite(timeout)
            or timeout <= 0
        ):
            raise ValueError(
                "Exact command/target and bounded positive timeout required"
            )
        with self.lock:
            deadline = self.clock() + timeout
            while True:
                if self.clock() >= deadline:
                    raise StandbyUncertain(
                        "Command proof deadline; no input retry", command_id=identifier
                    )
                try:
                    value = self._snapshot()
                except BaseException as error:
                    command = "python3 " + self._script() + " status"
                    detail = self._status_error_detail(error, command)
                    if not isinstance(error, BootstrapError):
                        raise
                    try:
                        retry = self._resolve_readonly_exit1(command, deadline=deadline)
                    except BaseException as resolution_error:
                        detail["resolution_error"] = _diagnostic_text(
                            resolution_error, 256
                        )
                        self._attach_status_detail(resolution_error, detail)
                        raise
                    detail["readonly_retry_allowed"] = retry
                    self._attach_status_detail(error, detail)
                    if not retry:
                        raise
                    self.status_retry_count += 1
                    self.sleep(min(0.1, max(0, deadline - self.clock())))
                    continue
                if self.clock() >= deadline:
                    raise StandbyUncertain(
                        "Snapshot arrived after command proof deadline; no retry",
                        command_id=identifier,
                    )
                if value.get("command_id") == identifier and value["state"] == target:
                    return value
                if self.clock() >= deadline:
                    raise StandbyUncertain(
                        "Terminal command proof deadline; no input retry",
                        command_id=identifier,
                    )
                self.sleep(min(0.1, max(0, deadline - self.clock())))

    def detach(self):
        with self.lock:
            self._remember_quarantine()
            self.detached = True
            return dict(
                work=self.work,
                nonce=self.nonce,
                owner=deepcopy(self.owner),
                service=deepcopy(self.service),
                pending_command=self.pending_command,
                pending_task=self.pending_task,
                unknown_task=self._unknown_task,
                retained_shadow=True
                if self.owner is not None
                else None
                if self._owner_launch_intent
                else False,
                retained_shadow_possible=self._owner_launch_intent,
                native_restored=False,
                reboot_requested=False,
            )

    def finalize(self, *, guard=None):
        """Explicit reboot request only. Receipt is acceptance, never restored proof."""
        with self.lock:
            if guard is not None and not callable(guard):
                raise TypeError("Optional finalize delivery guard must be callable")
            if (
                self.finalize_attempted
                or not self.launch_attempted
                or self.owner is None
            ):
                raise StandbyControlError("Known owner and one explicit finalize only")

            # A still-running old task may block before delivery. Retire only
            # immediately before this request can actually reach native START.
            def before_send():
                if guard is not None:
                    guard()
                self.finalize_attempted = True

            self._run("python3 " + self._script() + " finalize", pre_send=before_send)
            return dict(reboot_requested=True, restored=False, retained_shadow=True)
