"""Private orchestration behind Camera.usb_stream and stock standby.

The assembler injects the source-pinned RAM owner, prepared-plan backend
and native idle observer.
No research-directory imports or host-specific network commands occur here.
"""

from collections import deque
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
import json
import math
import threading
import time

from . import _app
from ._standby_control import _EntryProof, StandbyUncertain, parse_snapshot
from ._local_sleep_entry import (
    LocalSleepEntryProof,
    submit_local_sleep,
    _collector_closed,
)
from .vendor.duml import Message


class StreamSessionError(RuntimeError):
    def __init__(self, message, report):
        super().__init__(message)
        self.report = report


@dataclass(frozen=True)
class _IdleEvidence:
    verified_at: float
    recording_state: int
    activation_state: int
    battery_present: bool

    def validate(self, now):
        if (
            type(self.verified_at) not in (int, float)
            or not math.isfinite(self.verified_at)
            or not 0 <= now - self.verified_at <= 5
            or type(self.recording_state) is not int
            or self.recording_state != 0
            or type(self.activation_state) is not int
            or self.activation_state != 1
            or self.battery_present is not True
        ):
            raise ValueError(
                "Fresh native Record0/activation1/battery1 evidence required"
            )


class USBStreamSession:
    """One stable wrapper, fresh capture objects after each native wake.

    Lifecycle calls are serialized; data reads borrow active objects briefly
    and never issue a camera control command. Wake returns on fresh joint data;
    explicit verify_control(), or the next sleep, performs the deferred control
    check. The prepared watcher has already source-verified the owner wake/EXIT
    before it can expose the lease. Default close never reboots stable Sleep.
    """

    def __init__(
        self,
        camera,
        *,
        control,
        prepared_factory,
        idle_probe,
        reconnect_control=None,
        preview_options=None,
        imu_options=None,
        clock=time.monotonic,
        sleep=time.sleep,
        fast_sleep=False,
        retain_ecm=False,
        retained_factory=None,
    ):
        if type(fast_sleep) is not bool:
            raise ValueError("fast_sleep must be a bool")
        if (
            type(retain_ecm) is not bool
            or retain_ecm
            and (not fast_sleep or not callable(retained_factory))
        ):
            raise ValueError(
                "retain_ecm requires fast_sleep and a Linux retained backend"
            )
        self.camera, self.control = camera, control
        self.prepared_factory, self.idle_probe = prepared_factory, idle_probe
        self.reconnect_control = reconnect_control
        self.preview_options, self.imu_options = (
            dict(preview_options or {}),
            dict(imu_options or {}),
        )
        self.clock, self._sleep = clock, sleep
        self.fast_sleep = fast_sleep
        self.retain_ecm, self.retained_factory = retain_ecm, retained_factory
        self._retained_backend = None
        self._prepare_thread = self._prepare_generation = self._prepare_error = None
        self._prepare_done = threading.Event()
        self._prepare_done.set()
        self._prepare_abandoned = threading.Event()
        self._prepare_ready_callback = None
        self._lock = threading.RLock()
        self._state = "NEW"
        self._preview = self._imu = self._prepared = None
        self._wake_thread = None
        self._wake_outcome = {}
        self._wake_id = None
        self._late_verified = True
        self._native_floor_ns = 0
        self._stable_sleep_at = None
        self._primed_video, self._primed_imu = deque(), deque()
        self.report = dict(
            state="NEW",
            errors=[],
            cycles=[],
            retained_shadow=False,
            native_restored=False,
            recording_started=False,
            wifi_changed=False,
        )

    @property
    def state(self):
        return self._state

    @property
    def owns_camera_resources(self):
        """Camera.close must not bypass this owner after an uncertain cleanup."""
        return self._state not in ("NEW", "DETACHED")

    @property
    def cleanup_safe(self):
        return self._state == "DETACHED" and not self.report.get(
            "cleanup_deferred", False
        )

    @property
    def stats(self):
        with self._lock:
            return dict(
                state=self._state,
                preview=deepcopy(self._preview.stats) if self._preview else None,
                imu=deepcopy(self._imu.stats) if self._imu else None,
                control_verified=self._late_verified,
                report=deepcopy(self.report),
            )

    def _set_state(self, value):
        self._state = self.report["state"] = value

    def _idle(self):
        value = self.idle_probe(self.camera)
        if not isinstance(value, _IdleEvidence):
            raise TypeError("Injected native observer must return _IdleEvidence")
        value.validate(self.clock())
        return value

    def _owner_receipt(self):
        return dict(
            work=self.control.work,
            nonce=self.control.nonce,
            owner=deepcopy(self.control.owner),
            service=deepcopy(self.control.service),
        )

    def start(self):
        with self._lock:
            if self._state != "NEW":
                raise StreamSessionError("Session start is single-use", self.report)
            self._set_state("STARTING")
            try:
                self.camera.connect()
                with self.camera._control_lock:
                    if (
                        self.camera._imu is not None
                        or self.camera._usb_preview is not None
                    ):
                        raise StreamSessionError(
                            "Camera already owns capture resources; no standby owner was launched",
                            self.report,
                        )
                    idle = self._idle()
                    if not self.control.launch_attempted:
                        self.control.launch(
                            recording_state=idle.recording_state,
                            battery_present=idle.battery_present,
                            activation_state=idle.activation_state,
                        )
                    else:
                        if self.control.snapshot()["state"] != "AWAKE":
                            raise ValueError(
                                "Initial capture requires the explicitly attached owner AWAKE"
                            )
                    self.report["owner"] = self._owner_receipt()
                    self.report["retained_shadow"] = True
                    # Optional factory capability: cache immutable reviewed
                    # watcher sources once while native five-function USB is
                    # still active, before the initial preview rebind.
                    preload = getattr(self.prepared_factory, "preload_assets", None)
                    if preload is not None:
                        if not callable(preload):
                            raise TypeError(
                                "Declared prepared preload capability must be callable"
                            )
                        self.report["prepared_assets"] = preload(
                            self.camera,
                            self.control,
                            owner_receipt=self._owner_receipt(),
                        )
                    if self.retain_ecm:
                        self.report["retained_assets"] = (
                            self.retained_factory.preload_assets(
                                self.camera,
                                self.control,
                                owner_receipt=self._owner_receipt(),
                            )
                        )
                self._preview = self.camera.usb_preview(**self.preview_options)
                self._preview.start()
                self._imu = self.camera.imu(**self.imu_options)
                self._imu.start()
                # Do not substitute host time for native timestamps. Preserve
                # these actual samples for the first user reads.
                sample = self._imu.read(timeout=10)
                frame = self._preview.read(timeout=10)
                if (
                    type(sample.timestamp_ns) is not int
                    or type(frame.timestamp_ns) is not int
                ):
                    raise ValueError("Initial joint data lacks native timestamps")
                self._primed_imu.append(sample)
                self._primed_video.append(frame)
                self._native_floor_ns = max(sample.timestamp_ns, frame.timestamp_ns)
                self._set_state("AWAKE")
                return self
            except BaseException as error:
                self.report["errors"].append("Start: " + repr(error))
                self._set_state("RECOVERY_REQUIRED")
                raise

    def attach_sleep(self, prepared, *, native_floor_ns, snapshot):
        """Adopt a RO-verified existing Sleep using memory validation only.

        The resume assembler already proved camera RAM/boot/plan ownership and
        supplies the exact current control.last snapshot. No device command,
        host preflight, source upload, data start, or input happens here.
        """
        with self._lock:
            if self._state != "NEW":
                raise StreamSessionError(
                    "Only a NEW core can attach existing Sleep", self.report
                )
            if type(native_floor_ns) is not int or native_floor_ns < 0:
                raise ValueError(
                    "Native timestamp floor must be nonnegative integer nanoseconds"
                )
            if (
                prepared.camera is not self.camera
                or prepared.control is not self.control
                or prepared.owner_receipt != self._owner_receipt()
                or not prepared.prepared
                or prepared.closed
                or prepared.abandoned
                or prepared.arm_attempted
                or prepared.possible_future_apply
                or prepared.lease_proven
                or prepared.attach_attempted
                or prepared.imu_started
                or prepared.imu.pid is not None
                or prepared.preview._scheduled
                or prepared.preview._stream is not None
                or prepared.preview._lease is not None
                or self.camera._imu is not None
                or self.camera._usb_preview is not None
                or getattr(prepared, "native_floor_ns", native_floor_ns)
                != native_floor_ns
            ):
                raise StreamSessionError(
                    "Existing unarmed backend/camera/owner/floor mismatch", self.report
                )
            if (
                getattr(self.control, "launch_attempted", False) is not True
                or getattr(self.control, "detached", False)
                or getattr(self.control, "pending_command", None) is not None
                or getattr(self.control, "pending_task", None) is not None
                or getattr(self.control, "_unknown_task", False)
                or getattr(getattr(self.control, "bootstrap", None), "session", None)
                is not None
                and self.control.bootstrap.session.quarantined
            ):
                raise StreamSessionError(
                    "Known attached controller without pending input/task required",
                    self.report,
                )
            value = parse_snapshot(
                json.dumps(snapshot, allow_nan=False).encode("ascii"),
                nonce=self.control.nonce,
                owner=self.control.owner,
                service=self.control.service,
            )
            if value != self.control.last:
                raise StreamSessionError(
                    "Exact current controller snapshot required", self.report
                )
            sleep_id, wake_id = prepared.sleep_id, prepared.wake_id
            if (
                not isinstance(sleep_id, str)
                or not isinstance(wake_id, str)
                or len(sleep_id) != 16
                or len(wake_id) != 16
                or any(
                    character not in "0123456789abcdef"
                    for character in sleep_id + wake_id
                )
                or int(sleep_id, 16) < 1
                or int(wake_id, 16) != int(sleep_id, 16) + 1
                or value["state"] != "SLEEP"
                or value.get("command_id") != sleep_id
                or value["last_sequence"] != int(sleep_id, 16)
            ):
                raise StreamSessionError(
                    "Exact stable Sleep and next prepared Wake required", self.report
                )
            self._prepared, self._wake_id = prepared, wake_id
            self._native_floor_ns = native_floor_ns
            # New host process/OS epoch. Never adopt a persisted host clock.
            self._stable_sleep_at = self.clock()
            self._late_verified = True
            self.report.update(
                attached_existing=True,
                owner=self._owner_receipt(),
                retained_shadow=True,
                native_restored=False,
            )
            self.report["cycles"].append(
                dict(
                    sleep_id=sleep_id,
                    wake_id=wake_id,
                    sleeping=deepcopy(value),
                    prepared=deepcopy(prepared.report),
                    attached_existing=True,
                )
            )
            self._set_state("SLEEP")
            return self

    def _active(self, kind):
        with self._lock:
            if self._state != "AWAKE":
                raise StreamSessionError(
                    "Data is available only while AWAKE", self.report
                )
            queue = self._primed_video if kind == "video" else self._primed_imu
            source = self._preview if kind == "video" else self._imu
            if queue:
                return source, queue.popleft()
            return source, None

    def _observed(self, kind, source, value):
        with self._lock:
            current = self._preview if kind == "video" else self._imu
            if self._state != "AWAKE" or source is not current:
                raise StreamSessionError(
                    "Read crossed a capture lifecycle transition", self.report
                )
            if type(value.timestamp_ns) is int:
                self._native_floor_ns = max(self._native_floor_ns, value.timestamp_ns)
        return value

    def read_video(self, timeout=None):
        source, value = self._active("video")
        value = source.read(timeout=timeout) if value is None else value
        return self._observed("video", source, value)

    def read_imu(self, timeout=None):
        source, value = self._active("imu")
        value = source.read(timeout=timeout) if value is None else value
        return self._observed("imu", source, value)

    def _app_query(self):
        reply = self.camera._transport.request(
            0, 0x88, _app.query_payload(), receiver=0x28, timeout=0.5
        )
        return _app.decode_query_reply(reply.payload)

    def _unregister_app(self):
        stream = self._preview._stream
        if not stream.pause(timeout=3, keep_answer_requests=True):
            stream.resume(trigger_encoder=False)
            raise ValueError("Preview activity did not quiesce; no unregister/Sleep")
        before = self._app_query()
        if len(before.own_records) != 1:
            raise ValueError("Exactly one present own APP metadata record required")
        transport = self.camera._transport
        with transport._state_lock:
            transport._sequence = (transport._sequence + 1) & 65535
            sequence = transport._sequence
        row = dict(
            sequence=sequence,
            source=2,
            target=0x28,
            attempts=1,
            before=before.raw.hex(),
            acknowledgement_is_not_completion=True,
        )
        self.report.setdefault("app_teardown", []).append(row)
        try:
            transport._write(
                Message(
                    2,
                    0x28,
                    sequence,
                    0x40,
                    0,
                    0x88,
                    _app.registration_payload(unregister=True),
                )
            )
        except Exception as error:
            row["delivery_error"] = repr(
                error
            )  # Never retry input; only QUERY32 follows.
        deadline = self.clock() + 3
        while True:
            after = self._app_query()
            if self.clock() >= deadline:
                raise ValueError(
                    "APP absence proof deadline; unregister is never repeated"
                )
            row["after"] = after.raw.hex()
            if not after.own_records:
                return before, after
            self._sleep(min(0.05, max(0, deadline - self.clock())))

    @staticmethod
    def _mask_proven(imu, stopped):
        return (
            type(stopped) is dict
            and stopped.get("restored") is True
            and not stopped.get("errors")
            and type(imu.stats.get("collector_returncode")) is int
            and imu.stats["collector_returncode"] == 0
        )

    def _stop_active(self):
        # Keep the lease/IP alive until DIAG explicitly restores its log mask.
        if self._imu is not None:
            stopped = self._imu.stop(timeout=25)
            self.report["last_imu_stop"] = deepcopy(stopped)
            if not self._mask_proven(self._imu, stopped):
                raise StreamSessionError(
                    "IMU mask/process restoration not proven; lease retained",
                    self.report,
                )
            ticks = self._imu.stats.get("last_timestamp_ticks")
            if type(ticks) is int and ticks >= 0:
                self._native_floor_ns = max(
                    self._native_floor_ns, ticks * 1_000_000_000 // 19_200_000
                )
        if self._prepared is not None and self._prepared.lease_proven:
            stopped = self._prepared.stop()
        elif self._preview is not None:
            stopped = self._preview.stop()
        else:
            stopped = None
        if (
            self._preview is not None
            and self._preview.report.get("restored") is not True
        ):
            raise StreamSessionError(
                "Native five-function USB restoration not proven", self.report
            )
        self.report["last_preview_stop"] = deepcopy(stopped)
        self._preview = self._imu = None
        self._primed_video.clear()
        self._primed_imu.clear()

    def verify_control(self, *, timeout=20):
        self._wait_preparation(timeout=timeout)
        with self._lock:
            if self._late_verified:
                return self.control.last
            if self._wake_thread is not None:
                self._wake_thread.join(timeout)
                if self._wake_thread.is_alive():
                    raise StandbyUncertain(
                        "Wake submission still active; no new diagnostic START",
                        command_id=self._wake_id,
                    )
            # The reconnect callback is READ-ONLY and caller-owned. It must not
            # start a diagnostic task to escape the old session's quarantine.
            if self.reconnect_control is not None:
                self.control.bind_bootstrap(self.reconnect_control(self.camera))
            value = self.control.wait(self._wake_id, "AWAKE", timeout=timeout)
            self._late_verified = True
            self.report["late_control_proof"] = deepcopy(value)
            return value

    def sleep(self):
        self._wait_preparation(timeout=90)
        with self._lock:
            if self._state != "AWAKE":
                raise StreamSessionError("Sleep requires AWAKE", self.report)
            self.verify_control()
            if self.fast_sleep:
                return self._sleep_local_first()
            self._set_state("PREPARING_SLEEP")
            try:
                with self.camera._control_lock:
                    before, after = self._unregister_app()
                    self._stop_active()
                    idle = self._idle()
                    current = self.control.snapshot()
                    if current["state"] != "AWAKE":
                        raise ValueError("Native owner is not AWAKE before preparation")
                    number = current["last_sequence"] + 1
                    if number + 1 > 0xFFFFFFFFFFFFFFFF:
                        raise ValueError("Standby command sequence exhausted")
                    sleep_id, self._wake_id = f"{number:016x}", f"{number + 1:016x}"
                    self._prepared = self.prepared_factory(
                        self.camera,
                        self.control,
                        owner_receipt=self._owner_receipt(),
                        sleep_id=sleep_id,
                        wake_id=self._wake_id,
                    )
                    self._prepared.prepare()
                    # Uploads take time: refresh all native idle evidence AFTER
                    # staging, without resending APP unregister.
                    idle = self._idle()
                    after = self._app_query()
                    proof = _EntryProof(
                        idle.verified_at,
                        idle.recording_state,
                        idle.battery_present,
                        idle.activation_state,
                        True,
                        True,
                        _app.query_payload(),
                        before,
                        after,
                    )
                    submitted = self.control._submit_sleep(proof)
                    if submitted["command_id"] != sleep_id:
                        raise ValueError(
                            "Prepared plan and owner command sequence diverged"
                        )
                    terminal = self.control.wait(sleep_id, "SLEEP", timeout=12)
                self._stable_sleep_at = self.clock()
                self.report["cycles"].append(
                    dict(
                        sleep_id=sleep_id,
                        wake_id=self._wake_id,
                        sleeping=deepcopy(terminal),
                        prepared=deepcopy(self._prepared.report),
                    )
                )
                self._set_state("SLEEP")
                return deepcopy(terminal)
            except BaseException as error:
                self.report["errors"].append("Sleep preparation/entry: " + repr(error))
                # Preserve the exact failed read-only status transaction before
                # close/recovery creates another diagnostic adapter. This is
                # evidence only; it never authorizes another native key input.
                self.report["standby_status_error"] = deepcopy(
                    getattr(self.control, "last_status_error", None)
                )
                self.report["standby_status_retry_count"] = getattr(
                    self.control, "status_retry_count", 0
                )
                self._set_state("RECOVERY_REQUIRED")
                raise

    def _stop_local_capture(self):
        """Close our producers; retain the applied preview/lease for rollback."""
        preview, imu = self._preview, self._imu
        if (
            preview is None
            or imu is None
            or self.camera._usb_preview is not preview
            or self.camera._imu is not imu
            or not preview._scheduled
        ):
            raise StreamSessionError(
                "Exact owned active capture required for fast Sleep", self.report
            )
        stopped = imu.stop(timeout=25)
        self.report["last_imu_stop"] = deepcopy(stopped)
        if not self._mask_proven(imu, stopped) or not _collector_closed(imu):
            raise StreamSessionError(
                "Original IMU mask/collector exit not proven", self.report
            )
        tick = imu.stats.get("last_timestamp_ticks")
        if type(tick) is int and tick >= 0:
            self._native_floor_ns = max(
                self._native_floor_ns, tick * 1_000_000_000 // 19_200_000
            )
        stream = preview._stream
        stream.stop()
        if (
            stream._thread.is_alive()
            or not stream._link.stopped.is_set()
            or stream._link._sock.fileno() != -1
        ):
            raise StreamSessionError("Video worker/socket did not close", self.report)
        if self.camera._imu is not imu:
            raise StreamSessionError(
                "IMU ownership changed during local stop", self.report
            )
        self.camera._imu = None
        self._primed_video.clear()
        self._primed_imu.clear()
        return preview, imu, stream

    def _sleep_local_first(self):
        self._set_state("PREPARING_SLEEP")
        retained = None
        try:
            with self.camera._control_lock:
                if self.retain_ecm:
                    # A new process can first wake a schema1 native5 receipt.
                    # Its retained cache is staged once, while AWAKE, before
                    # that process's first retained Sleep request.
                    self.retained_factory.preload_assets(
                        self.camera, self.control, owner_receipt=self._owner_receipt()
                    )
                    current = self.control.snapshot()
                    if current["state"] != "AWAKE":
                        raise ValueError("Retained preparation requires AWAKE")
                    number = current["last_sequence"] + 1
                    if number + 1 > 0xFFFFFFFFFFFFFFFF:
                        raise ValueError("Standby command sequence exhausted")
                    retained = self.retained_factory(
                        self.camera,
                        self.control,
                        owner_receipt=self._owner_receipt(),
                        sleep_id=f"{number:016x}",
                        wake_id=f"{number + 1:016x}",
                        preview=self._preview,
                    )
                    retained.prepare()
                before, after = self._unregister_app()
                preview, imu, stream = self._stop_local_capture()
                idle = self._idle()
                after = self._app_query()
                lease, plan = preview._lease, preview._plan
                proof = LocalSleepEntryProof(
                    self.camera,
                    preview,
                    stream,
                    lease,
                    imu,
                    idle.verified_at,
                    idle.recording_state,
                    idle.activation_state,
                    idle.battery_present,
                    True,
                    imu.stats["collector_returncode"],
                    True,
                    True,
                    True,
                    lease.healthy,
                    lease.last_ack_monotonic,
                    plan["lease"]["nonce"],
                    plan["work_dir"],
                    _app.query_payload(),
                    before,
                    after,
                )
                submitted = submit_local_sleep(self.control, proof)
                sleep_id = submitted["command_id"]
                self._wake_id = f"{int(sleep_id, 16) + 1:016x}"
                if retained is not None and (retained.sleep_id, retained.wake_id) != (
                    sleep_id,
                    self._wake_id,
                ):
                    raise ValueError(
                        "Retained manifest and native command generation diverged"
                    )
                if retained is not None:
                    # Retire the previous cycle's cleanup proof immediately
                    # after the one new Sleep was accepted, before status wait.
                    self._prepared = self._retained_backend = retained
                terminal = self.control.wait(sleep_id, "SLEEP", timeout=12)
            self._stable_sleep_at = self.clock()
            self.report["cycles"].append(
                dict(
                    sleep_id=sleep_id,
                    wake_id=self._wake_id,
                    sleeping=deepcopy(terminal),
                    fast_sleep=True,
                    preparing_wake=retained is None,
                    retained_ecm=retained is not None,
                )
            )
            self._set_state("SLEEP")
            if retained is not None:
                self._prepared = self._retained_backend = retained
                self.report["preparing_wake"] = False
                self.report["cycles"][-1]["prepared"] = deepcopy(retained.report)
            else:
                self._start_preparation(sleep_id, self._wake_id, preview)
            return deepcopy(terminal)
        except BaseException as error:
            if (
                retained is not None
                and getattr(self.control, "pending_command", None) == retained.sleep_id
            ):
                # A lost input receipt is still a possibly delivered Sleep.
                self._prepared = self._retained_backend = retained
            self.report["errors"].append("Fast Sleep entry: " + repr(error))
            self._set_state("RECOVERY_REQUIRED")
            raise

    def _start_preparation(self, sleep_id, wake_id, preview):
        """Caller holds the core lock; joining always happens outside it."""
        self._prepare_generation, self._prepare_error = sleep_id, None
        self._prepare_done.clear()
        self._prepare_abandoned.clear()
        self.report["preparing_wake"] = True
        self._prepare_thread = threading.Thread(
            target=self._prepare_while_sleeping,
            args=(sleep_id, wake_id, preview),
            name="osmo-sleep-prepare",
            daemon=False,
        )
        try:
            self._prepare_thread.start()
        except BaseException as error:
            if (
                self._prepare_thread.ident is None
                and not self._prepare_thread.is_alive()
            ):
                self._prepare_thread = None
                self._prepare_error = error
                self._prepare_done.set()
            raise

    def _prepare_while_sleeping(self, sleep_id, wake_id, old_preview):
        """One finite preparation; never acquires the wrapper lifecycle lock."""
        candidate = publication = error = None
        started = self.clock()
        deadline = started + 90
        phases = {}

        @contextmanager
        def phase(name):
            began = self.clock()
            try:
                yield
            finally:
                phases[name] = self.clock() - began

        def timely():
            if self.clock() >= deadline or self._prepare_abandoned.is_set():
                raise TimeoutError("Post-Sleep preparation deadline")

        try:
            with self.camera._control_lock:
                timely()
                if (
                    self.camera._imu is not None
                    or self.camera._usb_preview is not old_preview
                ):
                    raise StreamSessionError(
                        "Foreign producer prevents background USB rollback", self.report
                    )
                self.control.resolve_pending()
                with phase("old_preview_stop"):
                    old_preview.stop(budget=timely)
                if old_preview.report.get("restored") is not True:
                    raise StreamSessionError(
                        "Native USB restoration while Sleep not proven", self.report
                    )
                with phase("reconnect"):
                    if self.reconnect_control is not None:
                        self.control.bind_bootstrap(self.reconnect_control(self.camera))
                timely()
                with phase("status_before"):
                    current = self.control.snapshot()
                if current["state"] != "SLEEP" or current.get("command_id") != sleep_id:
                    raise ValueError("Camera changed native Sleep during USB rollback")
                with phase("prepare_after_sleep"):
                    candidate = self.prepared_factory(
                        self.camera,
                        self.control,
                        owner_receipt=self._owner_receipt(),
                        sleep_id=sleep_id,
                        wake_id=wake_id,
                    )
                    candidate.prepare_after_sleep(guard=timely)
                timely()
                with phase("status_after"):
                    current = self.control.snapshot()
                if current["state"] != "SLEEP" or current.get("command_id") != sleep_id:
                    raise ValueError("Exact native Sleep changed during preparation")
                with phase("publish"):
                    if self._prepare_ready_callback is not None:
                        publication = self._prepare_ready_callback(
                            candidate, self._native_floor_ns, guard=timely
                        )
                timely()
        except BaseException as failure:
            error = failure
        phases["total"] = self.clock() - started
        # Foreground control uses C→U. Release U BEFORE committing under C.
        with self._lock:
            self.report["preparation_phase_seconds"] = phases
            self.report["cycles"][-1]["preparation_phase_seconds"] = dict(phases)
            if self._prepare_generation != sleep_id or self._state != "SLEEP":
                error = error or RuntimeError(
                    "Post-Sleep preparation generation retired"
                )
            if error is None:
                self._prepared, self._preview, self._imu = candidate, None, None
                self._retained_backend = None
                self.report["preparing_wake"] = False
                self.report["cycles"][-1].update(
                    preparing_wake=False, prepared=deepcopy(candidate.report)
                )
                if publication is not None:
                    self.report["prepared_receipt"] = deepcopy(publication)
            else:
                self._prepare_error = error
                self.report["preparation_error"] = repr(error)
                self.report["cleanup_deferred"] = True
                self._set_state("RECOVERY_REQUIRED")
            self._prepare_done.set()

    def _wait_preparation(self, *, timeout=90, allow_error=False):
        """Join before taking core/control locks; worker needs both to finish."""
        with self._lock:
            thread, generation = self._prepare_thread, self._prepare_generation
            error = self._prepare_error
        if thread is None:
            if error is not None and not allow_error:
                raise StreamSessionError(
                    "Preparation did not start; owner retained", self.report
                ) from error
            return
        if thread is threading.current_thread():
            raise RuntimeError("Preparation cannot join itself")
        thread.join(timeout)
        with self._lock:
            if thread.is_alive() or not self._prepare_done.is_set():
                self._prepare_abandoned.set()
                self.report["cleanup_deferred"] = True
                self._set_state("RECOVERY_REQUIRED")
                raise StreamSessionError(
                    "Preparation still owns resources; no new action", self.report
                )
            if generation != self._prepare_generation:
                raise StreamSessionError(
                    "Preparation generation changed while waiting", self.report
                )
            if self._prepare_error is not None and not allow_error:
                raise StreamSessionError(
                    "Post-Sleep preparation failed; owner retained", self.report
                ) from self._prepare_error

    def _wake_once(self):
        try:
            value = self.control.submit_wake()
            self._wake_outcome["submission"] = value
            if value["command_id"] != self._wake_id:
                self._wake_outcome["error"] = ValueError(
                    "Prepared wake command ID mismatch"
                )
        except BaseException as error:
            self._wake_outcome["error"] = (
                error  # No second input, even on a lost USB reply.
            )

    def wake(self, *, timeout=45):
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("Finite positive wake timeout required")
        waiting = self.clock()
        self._wait_preparation(timeout=timeout)
        timeout -= self.clock() - waiting
        if timeout <= 0:
            raise TimeoutError("Wake deadline while waiting for Sleep preparation")
        with self._lock:
            if (
                type(timeout) not in (int, float)
                or not math.isfinite(timeout)
                or timeout <= 0
            ):
                raise ValueError("Finite positive wake timeout required")
            if self._state != "SLEEP" or self._prepared is None:
                raise StreamSessionError(
                    "Wake requires a prepared stable Sleep", self.report
                )
            self._set_state("WAKING")
            began = self.clock()
            try:
                self._prepared.arm_watcher()
                self._wake_outcome = {}
                self._late_verified = False
                self._wake_thread = threading.Thread(
                    target=self._wake_once, name="osmo-stock-wake", daemon=True
                )
                wake_host_ns = time.monotonic_ns()
                self._wake_thread.start()
                elapsed = max(0, began - self._stable_sleep_at)
                floor = self._native_floor_ns + int(elapsed * 1_000_000_000)
                self._preview, attached_imu = self._prepared.attach_data_first(
                    timeout=timeout, native_floor_ns=floor, wake_host_ns=wake_host_ns
                )
                imu_starter = getattr(self._prepared, "start_imu_after_video", None)
                imu_waiting = callable(imu_starter)
                self._imu = None if imu_waiting else attached_imu
                # The backend starts capture without a late control wait. Its
                # witness validates actual native and host freshness; preserve
                # the first accepted records instead of hiding consumed data.
                witness = self._prepared.witness
                video_ready = imu_ready = False
                deadline = began + timeout
                while not (video_ready and imu_ready):
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise TimeoutError(
                            "Fresh joint video/IMU deadline; no input retry"
                        )
                    if not imu_ready and not imu_waiting:
                        try:
                            sample = self._imu.read(timeout=min(0.1, remaining))
                        except TimeoutError:
                            pass
                        else:
                            if witness.observe_imu(sample):
                                self._primed_imu.append(sample)
                                imu_ready = True
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise TimeoutError("Fresh joint data arrived after deadline")
                    if not video_ready:
                        try:
                            frame = self._preview.read(timeout=min(0.1, remaining))
                        except TimeoutError:
                            pass
                        else:
                            if witness.observe_video(frame):
                                self._primed_video.append(frame)
                                video_ready = True
                                if imu_waiting:
                                    if self.clock() >= deadline:
                                        raise TimeoutError(
                                            "Fresh video arrived after DIAG start deadline"
                                        )
                                    self._imu = attached_imu
                                    imu_starter(frame)
                                    imu_waiting = False
                if self.clock() >= deadline or not witness.joint_ready:
                    raise TimeoutError(
                        "Joint freshness proof not complete within deadline"
                    )
                self._prepared.report.update(
                    media_ready_proven=True, freshness=deepcopy(witness.report)
                )
                self.report["cycles"][-1].update(
                    data_ready_seconds=self.clock() - began,
                    native_floor_ns=floor,
                    prepared=deepcopy(self._prepared.report),
                    control_verified_later=False,
                )
                self._set_state("AWAKE")
                return self
            except BaseException as error:
                self.report["errors"].append("Wake: " + repr(error))
                self._set_state("RECOVERY_REQUIRED")
                raise

    def close(self):
        with self._lock:
            if (
                self.retain_ecm
                and self._state == "RECOVERY_REQUIRED"
                and (
                    getattr(self.control, "pending_command", None) is not None
                    or getattr(self.control, "pending_task", None) is not None
                    or getattr(self.control, "_unknown_task", False)
                    or getattr(
                        getattr(self.control.bootstrap, "session", None),
                        "quarantined",
                        False,
                    )
                )
            ):
                self.report["cleanup_deferred"] = True
                raise StreamSessionError(
                    "Unknown retained transition; explicit recovery required",
                    self.report,
                )
            retained = self._retained_backend
            if (
                retained is not None
                and self._state == "SLEEP"
                and not retained.arm_attempted
            ):
                # Stable close keeps the existing schema1 process-resume path:
                # restore native5 and stage its unused plan before detaching.
                if self._prepare_generation != retained.sleep_id:
                    self._start_preparation(
                        retained.sleep_id, retained.wake_id, self._preview
                    )
            elif retained is not None and not retained.lease_proven:
                self.report["cleanup_deferred"] = True
                raise StreamSessionError(
                    "Retained reset is uncertain; explicit recovery required",
                    self.report,
                )
        self._wait_preparation(timeout=90)
        with self._lock:
            if self._state == "DETACHED":
                return deepcopy(self.report)
            primary = None
            try:
                if self._preview is not None or self._imu is not None:
                    try:
                        self.verify_control()
                    except BaseException:
                        if self._prepared is None:
                            raise
                        # No new D1/early rollback while a wake task is unknown.
                        self.report["deferred_local_cleanup"] = (
                            self._prepared.abandon_local()
                        )
                        self._preview = self._imu = None
                        raise
                    self._stop_active()
                elif self._prepared is not None:
                    if self._prepared.lease_proven:
                        try:
                            self.verify_control()
                        except BaseException:
                            self.report["deferred_local_cleanup"] = (
                                self._prepared.abandon_local()
                            )
                            raise
                        self.report["prepared_stop"] = self._prepared.stop()
                    else:
                        # stop() first proves cancellation/no future apply, then
                        # closes its unstarted host placeholders. It cannot
                        # issue normal USB rollback before that proof.
                        self.report["prepared_cancel"] = self._prepared.stop()
            except BaseException as error:
                primary = error
                self.report["errors"].append("Close: " + repr(error))
                self.report["cleanup_deferred"] = True
            if primary is not None:
                # Retain the controller and Camera ownership so outer close()
                # cannot issue an armed-absent rollback or retry owned IMU.
                self.report["owner_retained_on_error"] = True
                self._set_state("RECOVERY_REQUIRED")
                raise StreamSessionError(
                    "Session retains ownership after deferred cleanup; no implicit reboot",
                    self.report,
                ) from primary
            self.report["detach"] = self.control.detach()
            self.report["retained_shadow"] = self.report["detach"]["retained_shadow"]
            self._set_state("DETACHED")
            return deepcopy(self.report)

    def recover(self):
        """Explicit recovery request; does not claim reboot/restoration completed."""
        self._wait_preparation(timeout=90, allow_error=True)
        with self._lock:
            if self._wake_thread is not None:
                self._wake_thread.join(15)
                if self._wake_thread.is_alive():
                    raise StandbyUncertain(
                        "Wake task still active; no new recovery START",
                        command_id=self._wake_id,
                    )
            if self.reconnect_control is not None:
                self.control.bind_bootstrap(self.reconnect_control(self.camera))
            value = self.control.finalize()
            self.report["explicit_recovery"] = value
            self._set_state("RECOVERY_PENDING")
            return value

    def __enter__(self):
        return self.start()

    def __exit__(self, kind, error, traceback):
        try:
            self.close()
        except Exception as cleanup:
            if error is None:
                raise
            if hasattr(error, "add_note"):
                error.add_note("USB stream cleanup: " + str(cleanup))
        return False
