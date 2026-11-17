"""Private prepared USB preview backend, with caller-owned standby/control.

No Camera API/export or device source copies are installed here. The caller
injects the two exact reviewed watcher sources and the explicit046 plan builder.
Prepare uploads immutable assets once, then fresh single-use plans/manifests.
Attach uses only host networking, lease and data interfaces: no state/connect/
diagnostic readiness wait. The coordinator owns late control/fresh-data proof.
"""

import base64
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
import re
import secrets
import shlex
import time

from . import _host_network
from ._preview_lease import PreviewLease
from ._standby_control import _token
from ._usb_bundle import build_bundle, NAMES
from ._usb_plan import _validate_runtime
from .imu import IMUStream, TICK_HZ
from .usb_preview import USBPreview
from .video import VideoStream, _video_evidence, _motion_evidence

WATCHER = "prepared_preview_wake_047.py"
SOURCES = {
    WATCHER: "246c4e253c07c675f4992b6b4ec494fda3c2d973ad7ad91801134203e569d3a5",
    "prepared_preview_wake_045.py": "33c8a2a1af90fe94dd7c93b58d4f15234b5c5dc0f35e088420c16c1118a654fc",
}
OWNER_SHA = "9529f3ee74274b5458a4dcda670a1efc94e6c607b7e65971629d03de7058aeb8"
Q = shlex.quote


class PreparedStreamError(RuntimeError):
    pass


class UnknownFutureApply(PreparedStreamError):
    """No early rollback: retain owner until an explicit verified recovery."""

    recovery_required = True


def bounded(command):
    if (
        not command.isascii()
        or not 0 < len(command.encode("ascii")) <= 500
        or "\0" in command
    ):
        raise ValueError("Prepared command exceeds reviewed500-byte ASCII bound")
    return command


def receipt(value):
    if (
        type(value) is not dict
        or set(value) != {"work", "nonce", "owner", "service"}
        or not isinstance(value["nonce"], str)
        or not re.fullmatch("[0-9a-f]{12}", value["nonce"])
        or value["work"] != "/dev/shm/osmo-usb-standby-" + value["nonce"]
        or not _token(value["owner"])
        or not _token(value["service"])
    ):
        raise ValueError("Explicit same-boot standby work/nonce/owner/service required")
    return deepcopy(value)


def same_owner(control, expected):
    if (control.work, control.nonce, control.owner, control.service) != (
        expected["work"],
        expected["nonce"],
        expected["owner"],
        expected["service"],
    ):
        raise PreparedStreamError("Original standby owner/system service changed")


def preparation_run(control, command, guard=None):
    if guard is not None:
        guard()
    result = (
        control._run(command, pre_send=guard)
        if guard is not None
        else control._run(command)
    )
    if guard is not None:
        guard()
    return result


def upload(control, path, data, *, guard=None):
    """Fixed fresh output only. Any uncertainty is left owned, never retried."""
    preparation_run(control, bounded("set -C && : > " + Q(path)), guard)
    for offset in range(0, len(data), 210):
        piece = base64.b64encode(data[offset : offset + 210]).decode("ascii")
        preparation_run(
            control, bounded("printf %s " + piece + " | base64 -d >> " + Q(path)), guard
        )
    digest = hashlib.sha256(data).hexdigest()
    preparation_run(
        control,
        bounded('[ "$(sha256sum ' + Q(path) + ' | cut -d" " -f1)" = ' + digest + " ]"),
        guard,
    )


class PreparedAssets:
    """One immutable RAM asset pair per exact owner/boot; plans are never shared."""

    def __init__(self, control, owner_receipt, source_provider, *, nonce=None):
        self.control, self.owner_receipt, self.source_provider = (
            control,
            receipt(owner_receipt),
            source_provider,
        )
        self.nonce = secrets.token_hex(6) if nonce is None else nonce
        if not isinstance(self.nonce, str) or not re.fullmatch(
            "[0-9a-f]{12}", self.nonce
        ):
            raise ValueError("Exact asset nonce required")
        self.work = "/dev/shm/osmo-prepared-assets-" + self.nonce
        self.attempted = self.staged = False

    def preload_assets(self):
        with self.control.lock:
            same_owner(self.control, self.owner_receipt)
            if self.staged:
                return self
            if self.attempted:
                raise PreparedStreamError(
                    "Asset staging was uncertain; no automatic retry"
                )
            current = self.control.snapshot()
            if current["state"] != "AWAKE":
                raise PreparedStreamError("Preload assets while the owner is awake")
            files = self.source_provider()
            if type(files) is not dict or set(files) != set(SOURCES):
                raise ValueError("Exact047/045 asset allowlist required")
            for name, data in files.items():
                if (
                    type(data) is not bytes
                    or not 0 < len(data) <= 100000
                    or not data.isascii()
                    or b"\0" in data
                    or hashlib.sha256(data).hexdigest() != SOURCES[name]
                ):
                    raise ValueError("Reviewed watcher bytes required: " + name)
            self.attempted = True
            self.control._run(
                bounded(
                    '[ "$(readlink -f /dev/shm)" = /dev/shm ] && '
                    '[ "$(awk \'$2=="/dev/shm" {print $3}\' /proc/mounts)" = tmpfs ] && '
                    "umask 077 && mkdir " + Q(self.work)
                )
            )
            stage = getattr(self.control, "stage_sources", None)
            if stage is not None and stage(files, self.work):
                for name in files:
                    self.control._run(bounded("chmod 400 " + Q(self.work + "/" + name)))
            else:
                for name, data in files.items():
                    path = self.work + "/" + name
                    upload(self.control, path, data)
                    self.control._run(bounded("chmod 400 " + Q(path)))
            program = (
                'import json,pathlib,sys;w=pathlib.Path(sys.argv[1]);c=json.loads((w/"config.json").read_text());'
                'assert c["boot_id"]==pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip();'
                'f=open("cache.json","x");json.dump(c,f);f.close()'
            )
            self.control._run(
                bounded(
                    "cd "
                    + Q(self.work)
                    + " && python3 -c "
                    + Q(program)
                    + " "
                    + Q(self.owner_receipt["work"])
                )
            )
            self.control._run(bounded("chmod 400 " + Q(self.work + "/cache.json")))
            self.staged = True
            return self

    def _guard_command(self):
        program = (
            'import prepared_preview_wake_047 as m,sys;b=m.base;a=m.Path.cwd();c=b.read_json(a/"cache.json");'
            'o=b.read_json(m.Path(sys.argv[1])/"config.json");'
            'assert all(c[k]==o[k] for k in ("owner","sys_service","nonce","boot_id"));'
            'assert c["boot_id"]==m.Path("/proc/sys/kernel/random/boot_id").read_text().strip();b.private(a,directory=True)'
        )
        return bounded(
            "cd "
            + Q(self.work)
            + " && python3 -c "
            + Q(program)
            + " "
            + Q(self.owner_receipt["work"])
        )

    def link_into(self, work, *, guard=None):
        """Fresh exclusive hardlinks only, revalidated SHA/inode, never symlinks."""
        with self.control.lock:
            same_owner(self.control, self.owner_receipt)
            if not self.staged or not re.fullmatch(
                "/dev/shm/osmo-prepared-preview047-[0-9a-f]{12}", work
            ):
                raise PreparedStreamError(
                    "Staged exact assets and fresh watcher work required"
                )
            # Check both immutable Python bytes BEFORE importing either cached
            # module in the camera-side guard/link commands.
            for name, digest in SOURCES.items():
                preparation_run(
                    self.control,
                    bounded(
                        '[ "$(sha256sum '
                        + Q(self.work + "/" + name)
                        + ' | cut -d" " -f1)" = '
                        + digest
                        + " ]"
                    ),
                    guard,
                )
            preparation_run(self.control, self._guard_command(), guard)
            preparation_run(
                self.control, bounded("umask 077 && mkdir " + Q(work)), guard
            )
            for name, digest in SOURCES.items():
                program = (
                    "import prepared_preview_wake_047 as m,os,sys;p=m.Path.cwd()/sys.argv[1];b=m.base;a=b.private(p);"
                    "assert not a.st_mode&146 and m.hashlib.sha256(b.read_bytes(p,65536)).hexdigest()==sys.argv[3];"
                    "os.link(p,sys.argv[2],follow_symlinks=False)"
                )
                target = work + "/" + name
                preparation_run(
                    self.control,
                    bounded(
                        "cd "
                        + Q(self.work)
                        + " && python3 -c "
                        + Q(program)
                        + " "
                        + name
                        + " "
                        + Q(target)
                        + " "
                        + digest
                    ),
                    guard,
                )
                proof = (
                    "import os,sys,hashlib;a,b=map(os.lstat,sys.argv[1:3]);"
                    "assert(a.st_dev,a.st_ino)==(b.st_dev,b.st_ino);"
                    'assert hashlib.sha256(open(sys.argv[2],"rb").read()).hexdigest()==sys.argv[3]'
                )
                preparation_run(
                    self.control,
                    bounded(
                        "python3 -c "
                        + Q(proof)
                        + " "
                        + Q(self.work + "/" + name)
                        + " "
                        + Q(target)
                        + " "
                        + digest
                    ),
                    guard,
                )
            preparation_run(self.control, self._guard_command(), guard)


class FreshnessWitness:
    """Caller-fed native/host-epoch evidence; never fabricates or converts time."""

    def __init__(self, native_floor_ns, wake_host_ns):
        if any(
            type(value) is not int or value < 0
            for value in (native_floor_ns, wake_host_ns)
        ):
            raise ValueError(
                "Native floor and host wake epoch must be nonnegative integer nanoseconds"
            )
        self.floor, self.host_epoch = native_floor_ns, wake_host_ns
        self.first_video = self.first_imu = None
        self.latest_video = self.latest_imu = self.first_video_motion = None
        self.rejected_video = self.rejected_imu = 0

    def _observe(self, sample, kind):
        native, host = sample.timestamp_ns, sample.host_receive_monotonic_ns
        evidence = (
            _video_evidence(sample) if kind == "video"
            else dict(timestamp_ns=native, host_receive_monotonic_ns=host)
        )
        setattr(self, "latest_" + kind, evidence)
        valid = (
            type(native) is int
            and native > self.floor
            and type(host) is int
            and host >= self.host_epoch
        )
        if not valid:
            setattr(self, "rejected_" + kind, getattr(self, "rejected_" + kind) + 1)
            return False
        name = "first_" + kind
        if getattr(self, name) is None:
            setattr(
                self, name, dict(timestamp_ns=native, host_receive_monotonic_ns=host)
            )
            if kind == "video":
                self.first_video_motion = deepcopy(evidence["motion"])
        return True

    def snapshot_latest(self, *, video=None, imu=None):
        """Refresh diagnostics from producer snapshots, never freshness gates."""
        if isinstance(video, dict):
            self.latest_video = dict(
                timestamp_ns=video.get("timestamp_ns"),
                host_receive_monotonic_ns=video.get("host_receive_monotonic_ns"),
                motion=_motion_evidence(video.get("motion")),
            )
        if isinstance(imu, dict):
            ticks = imu.get("last_timestamp_ticks")
            if type(ticks) is int and ticks >= 0:
                self.latest_imu = dict(
                    timestamp_ns=ticks * 1_000_000_000 // TICK_HZ,
                    host_receive_monotonic_ns=imu.get("last_host_receive_monotonic_ns"),
                )

    def observe_video(self, frame):
        return self._observe(frame, "video")

    def observe_imu(self, sample):
        return self._observe(sample, "imu")

    @property
    def joint_ready(self):
        return self.first_video is not None and self.first_imu is not None

    @property
    def report(self):
        return dict(
            native_floor_ns=self.floor,
            wake_host_ns=self.host_epoch,
            joint_ready=self.joint_ready,
            first_video=deepcopy(self.first_video),
            first_imu=deepcopy(self.first_imu),
            first_video_motion=deepcopy(self.first_video_motion),
            latest_video=deepcopy(self.latest_video),
            latest_imu=deepcopy(self.latest_imu),
            rejected_video=self.rejected_video,
            rejected_imu=self.rejected_imu,
        )


class PreparedStream:
    def __init__(
        self,
        camera,
        control,
        *,
        owner_receipt,
        sleep_id,
        wake_id,
        source_provider,
        plan_builder,
        assets=None,
        network_guard=None,
        runtime=None,
        decode=True,
        queue_size=1,
        imu_options=None,
        preview_factory=USBPreview,
        imu_factory=IMUStream,
        lease_factory=PreviewLease,
        video_factory=VideoStream,
        clock=time.monotonic,
    ):
        self.camera, self.control, self.owner_receipt = (
            camera,
            control,
            receipt(owner_receipt),
        )
        if (
            control.lock is not camera._control_lock
            or control.bootstrap.transport is not camera._transport
        ):
            raise ValueError(
                "Caller-owned camera transport and shared control lock required"
            )
        for value in (sleep_id, wake_id):
            if not isinstance(value, str) or not re.fullmatch("[0-9a-f]{16}", value):
                raise ValueError("Exact Sleep/Wake command IDs required")
        if int(sleep_id, 16) < 1 or int(wake_id, 16) != int(sleep_id, 16) + 1:
            raise ValueError("Consecutive Sleep then Wake required")
        self.sleep_id, self.wake_id = sleep_id, wake_id
        self.assets = assets or PreparedAssets(
            control, self.owner_receipt, source_provider
        )
        if (
            self.assets.control is not control
            or self.assets.owner_receipt != self.owner_receipt
        ):
            raise ValueError("Asset cache belongs to another standby owner")
        self.network = (
            network_guard
            if network_guard is not None
            else _host_network.preview_guard()
        )
        self.runtime, self.plan_builder, self.clock = runtime, plan_builder, clock
        self.decode, self.queue_size = decode, queue_size
        self.imu_options = (
            dict(read_buffer_bytes=65536) if imu_options is None else dict(imu_options)
        )
        self.preview_factory, self.imu_factory = preview_factory, imu_factory
        self.lease_factory, self.video_factory = lease_factory, video_factory
        self.nonce = secrets.token_hex(6)
        self.work = "/dev/shm/osmo-prepared-preview047-" + self.nonce
        self.script = self.work + "/" + WATCHER
        self.preview = self.imu = self.plan = self.baseline = self.witness = None
        self.prepared = self.prepare_attempted = self.arm_attempted = self.closed = (
            self.abandoned
        ) = False
        self.possible_future_apply = self.lease_proven = self.imu_started = (
            self.attach_attempted
        ) = False
        self.report = dict(errors=[], native_restored=False, retained_shadow=True)

    def preload_assets(self):
        return self.assets.preload_assets()

    def prepare(self):
        return self._prepare(sleeping=False)

    def prepare_after_sleep(self, *, guard=None):
        """Stage an unused plan after proven Sleep and full native USB restore.

        The immutable sources must already have been preloaded while awake.
        The caller has stopped all producers and restored native five-function
        USB; the normal strict runtime collector proves that baseline again.
        """
        return self._prepare(sleeping=True, guard=guard)

    def _prepare(self, *, sleeping, guard=None):
        with self.control.lock:
            if self.prepare_attempted or self.closed:
                raise PreparedStreamError("Fresh prepared stream is single-use")
            same_owner(self.control, self.owner_receipt)
            if guard is not None:
                guard()
            current = self.control.snapshot()
            if guard is not None:
                guard()
            if sleeping:
                if (
                    current["state"] != "SLEEP"
                    or current.get("command_id") != self.sleep_id
                    or current["last_sequence"] != int(self.sleep_id, 16)
                    or not self.assets.staged
                ):
                    raise PreparedStreamError(
                        "Prepare only after this exact Sleep with preloaded assets"
                    )
            elif current["state"] != "AWAKE" or current["last_sequence"] + 1 != int(
                self.sleep_id, 16
            ):
                raise PreparedStreamError("Prepare only before the exact next Sleep")
            if self.camera._imu is not None or self.camera._usb_preview is not None:
                raise PreparedStreamError(
                    "Coordinator must fully stop owned producers before prepare"
                )
            self.prepare_attempted = True
            self.preload_assets()
            self.baseline = self.network.preflight()
            if guard is not None:
                guard()
            try:
                self.control.resolve_pending()
                self.runtime = self.runtime or (
                    self.control.bootstrap.query_runtime(budget=guard)
                    if guard is not None
                    else self.control.bootstrap.query_runtime()
                )
                if guard is not None:
                    guard()
                _validate_runtime(self.runtime)
                self.plan = self.plan_builder(self.runtime)
                expected = build_bundle(self.plan["scripts"], self.plan["work_dir"])
                if (
                    set(self.plan["scripts"]) != NAMES
                    or self.plan["commands"] != expected["commands"]
                    or self.plan["lease"]["setup_seconds"] != 90
                    or self.plan["lease"]["lease_seconds"] != 10
                ):
                    raise ValueError(
                        "Exact coherent fresh046 plan/90setup/10lease required"
                    )
                for name, source in self.plan["scripts"].items():
                    if self.plan["sha256"][name] != hashlib.sha256(
                        source.encode("ascii")
                    ).hexdigest() or self.plan["byte_lengths"][name] != len(
                        source.encode("ascii")
                    ):
                        raise ValueError("Fresh plan script hash/length mismatch")
                if self.plan["bundle"]["plain_sha256"] != expected["plain_sha256"]:
                    raise ValueError("Fresh plan ready marker mismatch")
                for command in self.plan["commands"]:
                    preparation_run(self.control, bounded(command), guard)
                self.assets.link_into(self.work, guard=guard)
                manifest = dict(
                    schema=1,
                    nonce=self.nonce,
                    standby_work=self.owner_receipt["work"],
                    standby_nonce=self.owner_receipt["nonce"],
                    standby_owner=self.owner_receipt["owner"],
                    owner_sha256=OWNER_SHA,
                    sys_service=self.owner_receipt["service"],
                    sleep_command_id=self.sleep_id,
                    wake_command_id=self.wake_id,
                    plan_work=self.plan["work_dir"],
                    scripts={
                        name: dict(
                            sha256=self.plan["sha256"][name],
                            bytes=self.plan["byte_lengths"][name],
                        )
                        for name in self.plan["scripts"]
                    },
                    bundle_plain_sha256=expected["plain_sha256"],
                    native_baseline=asdict(self.runtime),
                )
                data = (
                    json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n"
                ).encode("ascii")
                if len(data) > 16384:
                    raise ValueError("Prepared manifest exceeds16KiB")
                upload(self.control, self.work + "/manifest.json", data, guard=guard)
                self.preview = self.preview_factory(
                    self.camera,
                    decode=self.decode,
                    queue_size=self.queue_size,
                    network_guard=self.network,
                )
                self.preview._started = True
                self.preview._runtime, self.preview._plan, self.preview._baseline = (
                    self.runtime,
                    self.plan,
                    self.baseline,
                )
                self.preview.report.update(
                    prepared=True,
                    runtime=asdict(self.runtime),
                    work_dir=self.plan["work_dir"],
                )
                self.imu = self.imu_factory(**self.imu_options)  # No start/USB claim.
                self.prepared = True
                self.report.update(
                    prepared=True,
                    work=self.work,
                    plan_work=self.plan["work_dir"],
                    manifest=manifest,
                    assets_work=self.assets.work,
                    immutable_assets_reused=True,
                )
                return self
            except BaseException:
                self.control._remember_quarantine()
                raise

    def arm_watcher(self):
        with self.control.lock:
            if not self.prepared or self.arm_attempted or self.closed:
                raise PreparedStreamError("One watcher arm per prepared stream")
            same_owner(self.control, self.owner_receipt)
            current = self.control.snapshot()
            if (
                current["state"] != "SLEEP"
                or current.get("command_id") != self.sleep_id
            ):
                raise PreparedStreamError("Arm near Wake only after exact stable Sleep")

            def intent():
                self.arm_attempted = self.possible_future_apply = True
                self.preview._scheduled = True

            try:
                self.control._run(
                    "python3 " + Q(self.script) + " launch", pre_send=intent
                )
            except BaseException as error:
                if self.possible_future_apply:
                    self.report["recovery_required"] = True
                    raise UnknownFutureApply(
                        "Watcher launch uncertain; strict cancel or explicit recovery required"
                    ) from error
                raise
            self.report["watcher_armed"] = True
            return self

    def attach_data_first(self, *, timeout=45, native_floor_ns, wake_host_ns):
        if (
            not self.possible_future_apply
            or self.lease_proven
            or self.closed
            or self.attach_attempted
        ):
            raise PreparedStreamError("One data attachment after watcher arm required")
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("Finite positive data attachment timeout required")
        self.witness = FreshnessWitness(native_floor_ns, wake_host_ns)
        self.attach_attempted = True
        deadline = self.clock() + timeout

        def remaining():
            value = deadline - self.clock()
            if value <= 0:
                raise TimeoutError("Prepared data attachment deadline")
            return value

        try:
            guard = self.network.wait_ready(
                self.baseline, self.runtime.host_addr, timeout=remaining()
            )
            remaining()
            self.preview.report["network"] = guard
            lease = self.preview._lease = self.lease_factory(
                dict(self.plan["lease"], interface=guard["ncm_interface"])
            )
            lease.start(timeout=min(12, remaining()))
            if lease.last_ack is None or not lease.healthy:
                raise PreparedStreamError(
                    "Exact healthy lease ACK required before USB cleanup adoption"
                )
            self.lease_proven, self.possible_future_apply = True, False
            self.report["lease_ack"] = deepcopy(lease.last_ack)
            remaining()  # A late ACK is still executed-plan cleanup evidence.
            if self.camera._imu is not None or self.camera._usb_preview is not None:
                raise PreparedStreamError(
                    "Another producer acquired camera ownership during Wake"
                )
            # Sole coordinator owns object creation. Do not wait on the USB
            # control lock here: the lost wake reply may still be timing out.
            self.camera._usb_preview, self.camera._imu = self.preview, self.imu
            stream = self.preview._stream = self.video_factory(
                decode=self.decode,
                queue_size=self.queue_size,
                host=self.plan["lease"]["camera_ip"],
                local_address=self.plan["lease"]["host_ip"],
                interface=guard["ncm_interface"],
            )
            stream.start()
            self.imu_started = True  # Partial startup must still restore mask.
            self.imu.start()
            remaining()
            self.report.update(data_started=True, media_ready_proven=False)
            return self.preview, self.imu
        except BaseException as error:
            self.report["errors"].append(type(error).__name__ + ": " + str(error))
            if not self.lease_proven:
                self.report["recovery_required"] = True
                raise UnknownFutureApply(
                    "No conclusive lease ACK; never race a future apply with rollback"
                ) from error
            raise

    def cancel_before_apply(self):
        with self.control.lock:
            if self.lease_proven:
                raise PreparedStreamError("Lease-proven plan uses normal owned cleanup")
            if not self.possible_future_apply:
                return dict(no_future_apply=True, watcher_never_armed=True)
            try:
                self.control._run("python3 " + Q(self.script) + " cancel")
            except BaseException as error:
                self.report["recovery_required"] = True
                raise UnknownFutureApply(
                    "Prepared cancel cannot prove no intent; explicit recovery required"
                ) from error
            self.possible_future_apply = False
            self.preview._scheduled = False
            self.report["cancel_proved_no_future_apply"] = True
            return dict(no_future_apply=True, watcher_never_armed=False)

    def stop(self):
        if self.closed:
            return self.report
        if self.abandoned:
            raise PreparedStreamError(
                "Local data abandoned; explicit recovery must prove native restoration"
            )
        if self.lease_proven:
            if (
                self.control.pending_task is not None
                or self.control._unknown_task
                or self.control.bootstrap.session.quarantined
            ):
                raise PreparedStreamError(
                    "Old diagnostic task unknown; retain heartbeat or abandon_local explicitly"
                )
            if self.camera._imu not in (
                None,
                self.imu,
            ) or self.camera._usb_preview not in (None, self.preview):
                raise PreparedStreamError(
                    "Foreign camera producer prevents owned USB rollback"
                )
            if not self.imu_started:
                self.imu.stop()
            self.report["preview_stop"] = self.preview.stop()
            self.report["native_restored"] = self.preview.report.get("restored") is True
        else:
            self.cancel_before_apply()
            if self.imu is not None:
                self.imu.stop()  # Unstarted: no collector, mask or USB action.
            if self.preview is not None:
                self.preview._closed = True
            self.report["overlay_never_applied"] = True
        self.closed = True
        return self.report

    def abandon_local(self):
        """No D1/rollback/IP deletion; lease watchdog and role owner remain.

        Mask restoration is a barrier before stopping local heartbeat. An
        unknown mask keeps heartbeat and IP available for the collector.
        """
        error = None
        if self.imu is not None:
            try:
                proof = self.imu.stop()
            except BaseException as failure:
                proof, error = self.imu.report, failure
            code = self.imu.stats.get("collector_returncode")
            if not (
                (proof is None and self.imu.pid is None and not self.imu_started)
                or isinstance(proof, dict)
                and proof.get("restored") is True
                and type(code) is int
            ):
                raise PreparedStreamError(
                    "IMU mask unknown; retain lease heartbeat and IP"
                ) from error
            self.report["local_imu_stop"] = proof
        if self.preview is not None and self.preview._stream is not None:
            try:
                self.preview._stream.stop()
            except BaseException as failure:
                # Metadata/decoder failures still leave a stopped data thread.
                # Preserve that fault without blocking local lease shutdown.
                if self.preview._stream.stats.get("stopped") is not True:
                    raise PreparedStreamError(
                        "Video reader not stopped; retain heartbeat and IP"
                    ) from failure
                self.report["errors"].append("Video stop: " + str(failure))
                if error is None:
                    error = failure
        if self.preview is not None and self.preview._lease is not None:
            self.preview._lease.stop(close_camera=False)
        if self.camera._imu is self.imu:
            self.camera._imu = None
        if self.camera._usb_preview is self.preview:
            self.camera._usb_preview = None
        self.abandoned = True
        self.report.update(
            local_abandoned=True,
            native_restored=False,
            retained_shadow=True,
            lease_watchdog_restoration_deferred=True,
            recovery_required=True,
        )
        if error is not None:
            self.report["errors"].append("IMU stop: " + str(error))
            raise error
        return self.report
