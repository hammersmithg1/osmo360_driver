"""Opt-in Linux warm standby retaining one already-applied ECM plan.

Only the reviewed near-Wake helper can rebind UDC. The host may provision a
replacement USB NIC to query its receipt, but starts no DIAG/video until that
receipt and a fresh original-lease ACK both pass. No apply script is rerun.
"""

from copy import deepcopy
import hashlib
import json
import math
import re
import secrets
import shlex
import socket
import struct
import time

from ._prepared_stream import (
    FreshnessWitness,
    OWNER_SHA,
    PreparedStreamError,
    UnknownFutureApply,
    bounded,
    receipt,
    same_owner,
    upload,
    SOURCES as BASE_SOURCES,
)
from ._preview_lease import PreviewLease, PreviewLeaseError
from ._usb_bundle import NAMES
from ._usb_bootstrap import BootstrapError
from . import _linux_network
from ._linux_usb_network import USBGenerationVanished
from ._local_sleep_entry import _collector_closed
from .imu import IMUStream
from .usb_preview import USBPreview
from .video import VideoStream
from .vendor.util import bind_socket

HELPER = "retained_sleep_060.py"
# The camera source is installed only after its independent source/HW review.
HELPER_SHA = "323527c16a9094ad90152e30d1e284a3297f450a7cbe8abdf91dab7ca083dde9"
SOURCES = dict(BASE_SOURCES, **{HELPER: HELPER_SHA})
Q = shlex.quote
QUERY = struct.Struct("!8s16sQ")
MAGIC = b"OSMO060Q"


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def completion_ticket(lease_nonce, helper_nonce, wake_id):
    return hashlib.sha256(
        bytes.fromhex(lease_nonce)
        + bytes.fromhex(helper_nonce)
        + bytes.fromhex(wake_id)
    ).digest()[:16]


def decode_completion(raw, *, manifest, ticket, challenge):
    """A correlated receipt, never a metadata-only or UDP-arrival claim."""
    if type(raw) is not bytes or not 0 < len(raw) <= 1024:
        raise ValueError("Bounded completion datagram required")

    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError("Duplicate completion field")
            value[key] = item
        return value

    value = json.loads(raw, object_pairs_hook=unique)
    expected = dict(
        schema=1,
        nonce=manifest["nonce"],
        standby_owner=manifest["standby_owner"],
        sys_service=manifest["sys_service"],
        wake_command_id=manifest["wake_command_id"],
        ticket=ticket.hex(),
        challenge=challenge,
        terminal=True,
        reset_attempted=1,
        udc_rebind_completed=True,
        original_plan_work=manifest["lease_work"],
    )
    if (
        type(value) is not dict
        or set(value) != set(expected) | {"lease_expires", "now"}
        or any(value.get(key) != item for key, item in expected.items())
        or type(value["schema"]) is not int
        or type(value["reset_attempted"]) is not int
        or type(value["challenge"]) is not int
        or value["terminal"] is not True
        or value["udc_rebind_completed"] is not True
        or not _finite(value["now"])
        or not _finite(value["lease_expires"])
        or not 0 < value["lease_expires"] - value["now"] <= 10.000001
    ):
        raise ValueError("Exact fresh original-plan completion required")
    return value


class RetainedAssets:
    """Separate immutable060/047/045 cache; PreparedAssets is unchanged."""

    def __init__(self, control, owner_receipt, source_provider):
        self.control, self.owner_receipt = control, receipt(owner_receipt)
        self.source_provider = source_provider
        self.work = self.owner_receipt["work"] + "/retained-assets-" + HELPER_SHA[:12]
        self.attempted = self.staged = False

    def preload_assets(self):
        with self.control.lock:
            same_owner(self.control, self.owner_receipt)
            if self.staged:
                return self
            if self.attempted:
                raise PreparedStreamError("Retained asset staging is single-use")
            if self.control.snapshot()["state"] != "AWAKE":
                raise PreparedStreamError(
                    "Stage immutable retained sources while AWAKE"
                )
            files = self.source_provider()
            if type(files) is not dict or set(files) != set(SOURCES):
                raise ValueError("Exact retained source allowlist required")
            for name, data in files.items():
                if (
                    type(data) is not bytes
                    or not 0 < len(data) <= 65536
                    or not data.isascii()
                    or b"\0" in data
                    or hashlib.sha256(data).hexdigest() != SOURCES[name]
                ):
                    raise ValueError("Reviewed retained source hash required: " + name)
            self.attempted = True
            absent = "[ ! -e " + Q(self.work) + " ] && [ ! -L " + Q(self.work) + " ]"
            try:
                self.control._run(bounded(absent))
            except BootstrapError:
                # A natural test exit is F6 in this firmware. Resolve that
                # exact RO task before any new START; unknown tasks stay fatal.
                if not self.control._resolve_readonly_exit1(
                    absent, deadline=self.control.clock() + 5
                ):
                    raise
                self._validate_cache()
                self.staged = True
                return self
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
                    upload(self.control, self.work + "/" + name, data)
                    self.control._run(bounded("chmod 400 " + Q(self.work + "/" + name)))
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
            self._validate_cache()
            self.staged = True
            return self

    def _validate_cache(self):
        names = tuple(SOURCES) + ("cache.json",)
        program = (
            "import pathlib,stat,sys;w=pathlib.Path(sys.argv[1]);s=w.lstat();"
            "assert w.resolve()==w and stat.S_ISDIR(s.st_mode) and s.st_uid==0 and stat.S_IMODE(s.st_mode)==448;"
            "assert set(p.name for p in w.iterdir())==set(sys.argv[2:])"
        )
        self.control._run(
            bounded(
                "python3 -c " + Q(program) + " " + Q(self.work) + " " + " ".join(names)
            )
        )
        private = (
            "import os,stat,sys;s=os.lstat(sys.argv[1]);"
            "assert stat.S_ISREG(s.st_mode) and s.st_uid==0 and stat.S_IMODE(s.st_mode)==256"
        )
        for name in names:
            self.control._run(
                bounded("python3 -c " + Q(private) + " " + Q(self.work + "/" + name))
            )
        for name, digest in SOURCES.items():
            self.control._run(
                bounded(
                    '[ "$(sha256sum '
                    + Q(self.work + "/" + name)
                    + ' | cut -d" " -f1)" = '
                    + digest
                    + " ]"
                )
            )
        guard = (
            "import prepared_preview_wake_047 as m,sys;b=m.base;a=m.Path.cwd();"
            'c=b.read_json(a/"cache.json");o=b.read_json(m.Path(sys.argv[1])/"config.json");'
            'assert all(c[k]==o[k] for k in ("owner","sys_service","nonce","boot_id"));'
            'assert c["boot_id"]==m.Path("/proc/sys/kernel/random/boot_id").read_text().strip();b.private(a,directory=True)'
        )
        self.control._run(
            bounded(
                "cd "
                + Q(self.work)
                + " && python3 -B -c "
                + Q(guard)
                + " "
                + Q(self.owner_receipt["work"])
            )
        )

    def link_into(self, work):
        with self.control.lock:
            same_owner(self.control, self.owner_receipt)
            if not self.staged:
                raise PreparedStreamError("Retained sources must be preloaded")
            self._validate_cache()
            self.control._run(bounded("umask 077 && mkdir " + Q(work)))
            for name, digest in SOURCES.items():
                program = (
                    "import prepared_preview_wake_047 as m,os,sys;p=m.Path.cwd()/sys.argv[1];b=m.base;a=b.private(p);"
                    "assert not a.st_mode&146 and m.hashlib.sha256(b.read_bytes(p,65536)).hexdigest()==sys.argv[3];"
                    "os.link(p,sys.argv[2],follow_symlinks=False)"
                )
                target = work + "/" + name
                self.control._run(
                    bounded(
                        "cd "
                        + Q(self.work)
                        + " && python3 -B -c "
                        + Q(program)
                        + " "
                        + name
                        + " "
                        + Q(target)
                        + " "
                        + digest
                    )
                )
                proof = (
                    "import os,sys,hashlib;a,b=map(os.lstat,sys.argv[1:3]);"
                    "assert(a.st_dev,a.st_ino)==(b.st_dev,b.st_ino);"
                    'assert hashlib.sha256(open(sys.argv[2],"rb").read()).hexdigest()==sys.argv[3]'
                )
                self.control._run(
                    bounded(
                        "python3 -c "
                        + Q(proof)
                        + " "
                        + Q(self.work + "/" + name)
                        + " "
                        + Q(target)
                        + " "
                        + digest
                    )
                )


class RetainedStream:
    """One retained original plan and one near-Wake helper; never a reapply."""

    def __init__(
        self,
        camera,
        control,
        *,
        owner_receipt,
        sleep_id,
        wake_id,
        preview,
        assets,
        decode=True,
        queue_size=1,
        imu_options=None,
        preview_factory=USBPreview,
        lease_factory=PreviewLease,
        video_factory=VideoStream,
        imu_factory=IMUStream,
        socket_factory=socket.socket,
        clock=time.monotonic,
        sleep=time.sleep,
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
            raise ValueError("Same camera control handle/lock required")
        if (
            not isinstance(sleep_id, str)
            or not re.fullmatch("[0-9a-f]{16}", sleep_id)
            or not isinstance(wake_id, str)
            or not re.fullmatch("[0-9a-f]{16}", wake_id)
            or int(sleep_id, 16) < 1
            or int(wake_id, 16) != int(sleep_id, 16) + 1
        ):
            raise ValueError("Exact consecutive retained command IDs required")
        self.sleep_id, self.wake_id, self.old_preview = sleep_id, wake_id, preview
        self.assets, self.decode, self.queue_size = assets, decode, queue_size
        self.imu_options = dict(queue_size=16384, read_buffer_bytes=65536)
        self.imu_options.update(imu_options or {})
        self.preview_factory, self.lease_factory = preview_factory, lease_factory
        self.video_factory, self.imu_factory = video_factory, imu_factory
        self.socket_factory, self.clock, self.sleep = socket_factory, clock, sleep
        self.nonce = secrets.token_hex(6)
        self.work = "/dev/shm/osmo-retained-sleep060-" + self.nonce
        self.script = self.work + "/" + HELPER
        self.plan, self.runtime, self.network = (
            preview._plan,
            preview._runtime,
            preview.network,
        )
        self.baseline = preview._baseline
        self.preview = self.imu = self.witness = self.manifest = None
        self.prepared = self.prepare_attempted = self.arm_attempted = self.closed = (
            False
        )
        self.attach_attempted = self.imu_started = self.abandoned = False
        self.possible_future_apply = self.lease_proven = False
        self.sequence = None
        self._provisional_lease = None
        self._provisional_row = None
        self.report = dict(retained_ecm=True, errors=[], native_restored=False)

    def prepare(self):
        with self.control.lock:
            if self.prepare_attempted:
                raise PreparedStreamError("Retained helper is single-use")
            same_owner(self.control, self.owner_receipt)
            state = self.control.snapshot()
            old = self.old_preview
            if (
                state["state"] != "AWAKE"
                or state["last_sequence"] + 1 != int(self.sleep_id, 16)
                or self.camera._usb_preview is not old
                or not old._scheduled
                or old._closed
                or not old._lease.healthy
                or not old._lease.last_ack
                or old._lease.last_ack["state"] != 1
                or not self.assets.staged
                or self.assets.control is not self.control
                or self.assets.owner_receipt != self.owner_receipt
                or set(self.plan["sha256"]) != NAMES
                or self.network.identity is None
                or self.network.report.get("ready", {}).get("ok") is not True
            ):
                raise PreparedStreamError(
                    "Exact active Linux ECM plan/lease and AWAKE owner required"
                )
            self.prepare_attempted = True
            self.assets.link_into(self.work)
            port = 49152 + secrets.randbelow(16384)
            if port == self.plan["lease"]["port"]:
                port = 49152 + (port - 49151) % 16384
            lease = self.plan["lease"]
            value = dict(
                schema=1,
                nonce=self.nonce,
                standby_work=self.control.work,
                standby_nonce=self.control.nonce,
                standby_owner=self.control.owner,
                owner_sha256=OWNER_SHA,
                sys_service=self.control.service,
                sleep_command_id=self.sleep_id,
                wake_command_id=self.wake_id,
                lease_work=self.plan["work_dir"],
                lease_nonce=lease["nonce"],
                lease_port=lease["port"],
                plan_sha256=self.plan["sha256"],
                udc=self.runtime.udc,
                dev_mac=self.runtime.dev_addr,
                host_mac=self.runtime.host_addr,
                camera_ip=lease["camera_ip"],
                host_ip=lease["host_ip"],
                host_sequence=old._lease.last_ack["sequence"],
                completion_port=port,
            )
            data = json.dumps(
                value, sort_keys=True, separators=(",", ":"), allow_nan=False
            ).encode("ascii")
            if len(data) > 16384:
                raise ValueError("Retained manifest exceeds16KiB")
            upload(self.control, self.work + "/manifest.json", data)
            self.manifest, self.prepared = value, True
            self.report.update(
                staged=True, work=self.work, original_plan_work=self.plan["work_dir"]
            )
            return self

    def arm_watcher(self):
        with self.control.lock:
            if not self.prepared or self.arm_attempted or self.closed:
                raise PreparedStreamError("One near-Wake retained helper arm required")
            same_owner(self.control, self.owner_receipt)
            state = self.control.snapshot()
            if state["state"] != "SLEEP" or state.get("command_id") != self.sleep_id:
                raise PreparedStreamError("Arm only during the exact stable Sleep")
            if not self.old_preview._lease.healthy:
                raise PreparedStreamError(
                    "Original lease must remain healthy throughout Sleep"
                )

            def intent():
                self.arm_attempted = self.possible_future_apply = True

            try:
                self.control._run(
                    "python3 -B " + Q(self.script) + " launch", pre_send=intent
                )
                # Local closure sends no CLOSE opcode and cannot roll back ECM.
                self.old_preview._lease.stop(close_camera=False)
                self.sequence = self.old_preview._lease._sequence
            except BaseException as error:
                self.report["recovery_required"] = True
                raise UnknownFutureApply(
                    "Retained arm/lease closure uncertain; no replay or early rollback"
                ) from error
            self.report.update(
                watcher_armed=True,
                old_lease_sequence=self.sequence,
                old_lease_local_only=True,
            )
            return self

    def _stop_provisional(self):
        """Retire liveness only; unknown native work never permits CLOSE."""
        lease = self._provisional_lease
        if lease is None:
            return
        try:
            lease.stop(close_camera=False)
        except Exception as error:
            if self._provisional_row is not None:
                self._provisional_row["stop_error"] = repr(error)
            raise
        else:
            if self._provisional_row is not None:
                self._provisional_row["stopped_locally"] = True
                self._provisional_row["stopped_host_monotonic_ns"] = time.monotonic_ns()
        finally:
            self.sequence = max(self.sequence, lease._sequence)
            if self._provisional_row is not None:
                self._provisional_row.update(
                    last_sent_sequence=lease._sequence,
                    last_ack=deepcopy(lease.last_ack),
                )
        self._provisional_lease = self._provisional_row = None

    def _start_provisional(self, ready, remaining):
        """Preserve our existing RAM plan while the final receipt is pending.

        A trusted host NIC authorizes only original-nonce lease DATA here.
        It does not prove the final reset, permit media/DIAG, or release any
        controller quarantine or native transition guard.
        """
        config = dict(self.plan["lease"], interface=ready["ncm_interface"])
        lease = self._provisional_lease = self.lease_factory(config)
        lease._sequence = self.sequence
        row = self._provisional_row = dict(
            interface=ready["ncm_interface"],
            ifindex=self.network.identity.ifindex,
            started_host_monotonic_ns=time.monotonic_ns(),
            first_sent_after=self.sequence,
            completion_proven=False,
        )
        self.report.setdefault("lease_handoff", []).append(row)
        lease.start(timeout=min(3, remaining()))
        if (not lease.healthy or not lease.last_ack
                or lease.last_ack["sequence"] <= self.sequence):
            raise PreparedStreamError("Provisional original-nonce lease ACK required")
        row["first_ack"] = deepcopy(lease.last_ack)
        row["first_ack_host_monotonic_ns"] = time.monotonic_ns()
        remaining()

    def _completion(self, remaining):
        ticket = completion_ticket(
            self.manifest["lease_nonce"], self.nonce, self.wake_id
        )
        sock = None
        generations = 0
        completed = False
        try:
            while True:
                remaining()
                absent, _ = self.network._old_index_absent(self.network.identity)
                if absent:
                    self._stop_provisional()
                    if sock is not None:
                        sock.close()
                        sock = None
                    try:
                        self.network, ready = self.network.rebind_after_usb_reset(
                            timeout=remaining()
                        )
                    except USBGenerationVanished as error:
                        self.report.setdefault("vanished_generations", []).append(
                            deepcopy(error.proof)
                        )
                        self.sleep(min(0.01, remaining()))
                        continue
                    generations += 1
                    self.report.setdefault("network_generations", []).append(
                        deepcopy(ready)
                    )
                    try:
                        self._start_provisional(ready, remaining)
                    except (OSError, PreviewLeaseError):
                        if self.network._old_index_absent(self.network.identity)[0]:
                            self._stop_provisional()
                            continue
                        raise
                    sock = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
                    bind_socket(
                        sock, self.plan["lease"]["host_ip"], ready["ncm_interface"]
                    )
                if sock is None:
                    self.sleep(min(0.02, remaining()))
                    continue
                challenge = secrets.randbits(64)
                sock.settimeout(min(0.05, remaining()))
                try:
                    sock.sendto(
                        QUERY.pack(MAGIC, ticket, challenge),
                        (
                            self.plan["lease"]["camera_ip"],
                            self.manifest["completion_port"],
                        ),
                    )
                    raw, peer = sock.recvfrom(1025)
                except (OSError, socket.timeout):
                    self.sleep(min(0.01, remaining()))
                    continue  # Only data-only receipt polls; no native input.
                if peer != (
                    self.plan["lease"]["camera_ip"],
                    self.manifest["completion_port"],
                ):
                    continue
                remaining()
                try:
                    value = decode_completion(
                        raw, manifest=self.manifest, ticket=ticket, challenge=challenge
                    )
                except (ValueError, TypeError, KeyError):
                    continue
                if self.network._old_index_absent(self.network.identity)[0]:
                    continue
                # Recheck the pinned generation and all original host state;
                # the receipt can never authorize alias/route drift.
                snapshot, state = self.network._snapshot()
                current = self.network._current((snapshot, state))
                self.network._unchanged_other_state(state)
                ready = _linux_network.evaluate(
                    self.baseline, after=snapshot, host_mac=self.runtime.host_addr
                )
                if current is None or current["pending_linkdown"] or not ready["ok"]:
                    raise PreparedStreamError("Completion generation lost carrier")
                self.report.update(
                    completion=value, completion_verified=True, generations=generations
                )
                ready["owned_session_static"] = True
                completed = True
                return ready
        finally:
            if sock is not None:
                sock.close()
            if not completed:
                try:
                    self._stop_provisional()
                except Exception as error:
                    self.report["provisional_stop_error"] = repr(error)

    def attach_data_first(self, *, timeout=45, native_floor_ns, wake_host_ns):
        if not self.possible_future_apply or self.attach_attempted or self.closed:
            raise PreparedStreamError(
                "One attachment after retained helper arm required"
            )
        if not _finite(timeout) or timeout <= 0:
            raise ValueError("Finite positive retained attachment timeout required")
        self.attach_attempted = True
        self.witness = FreshnessWitness(native_floor_ns, wake_host_ns)
        deadline = self.clock() + timeout

        def remaining():
            value = deadline - self.clock()
            if value <= 0:
                raise TimeoutError("Retained completion/data deadline")
            return value

        try:
            ready = self._completion(remaining)
            remaining()
            fresh = self.preview = self.preview_factory(
                self.camera,
                decode=self.decode,
                queue_size=self.queue_size,
                network_guard=self.network,
            )
            fresh._started = fresh._scheduled = True
            fresh._runtime, fresh._plan, fresh._baseline = (
                self.runtime,
                self.plan,
                self.baseline,
            )
            lease = self._provisional_lease
            if lease is None:
                raise PreparedStreamError("Final generation has no owned provisional lease")
            fresh._lease = lease
            before = lease._sequence
            ack = lease.renew()
            if (not lease.healthy or not ack or ack["sequence"] <= before
                    or self.network._old_index_absent(self.network.identity)[0]):
                raise PreparedStreamError("Fresh final-generation lease ACK required")
            if self._provisional_row is not None:
                self._provisional_row.update(
                    completion_proven=True, adopted_ack=deepcopy(ack),
                    adopted_host_monotonic_ns=time.monotonic_ns(),
                )
            # This exact completed plan plus a new ACK permits normal cleanup,
            # even if the host budget expires immediately after the proof.
            self.lease_proven, self.possible_future_apply = True, False
            config = dict(self.plan["lease"], interface=ready["ncm_interface"])
            self._provisional_lease = self._provisional_row = None
            self.report["lease_ack"] = deepcopy(lease.last_ack)
            # A late ACK still proves the original plan for normal cleanup.
            remaining()
            if (
                self.camera._imu is not None
                or self.camera._usb_preview is not self.old_preview
            ):
                raise PreparedStreamError(
                    "Foreign producer prevents retained plan adoption"
                )
            self.old_preview._scheduled = False
            self.old_preview._closed = True
            self.camera._usb_preview = fresh
            fresh.report.update(
                retained_ecm=True,
                network=deepcopy(ready),
                work_dir=self.plan["work_dir"],
            )
            self.imu = self.imu_factory(**self.imu_options)
            self.camera._imu = self.imu
            stream = fresh._stream = self.video_factory(
                decode=self.decode,
                queue_size=self.queue_size,
                host=config["camera_ip"],
                local_address=config["host_ip"],
                interface=ready["ncm_interface"],
            )
            stream.start()
            remaining()
            self.report.update(
                data_started=False, imu_start_pending=True, media_ready_proven=False
            )
            return fresh, self.imu
        except BaseException as error:
            try:
                self._stop_provisional()
            except Exception as cleanup:
                self.report["provisional_stop_error"] = repr(cleanup)
                if hasattr(error, "add_note"):
                    error.add_note("Provisional local stop: " + repr(cleanup))
            self.report["errors"].append(repr(error))
            if not self.lease_proven:
                self.report["recovery_required"] = True
                raise UnknownFutureApply(
                    "No final reset/lease proof; explicit recovery required"
                ) from error
            raise

    def start_imu_after_video(self, frame):
        accepted = dict(
            timestamp_ns=frame.timestamp_ns,
            host_receive_monotonic_ns=frame.host_receive_monotonic_ns,
        )
        if (
            self.closed
            or not self.attach_attempted
            or not self.lease_proven
            or self.imu_started
            or self.imu is None
            or self.camera._imu is not self.imu
            or self.camera._usb_preview is not self.preview
            or self.witness is None
            or self.witness.first_video != accepted
            or not self.preview._lease.healthy
        ):
            raise PreparedStreamError(
                "One owned DIAG start after accepted fresh video required"
            )
        self.imu_started = True  # Any partial start retains the strict mask barrier.
        self.report["imu_start_pending"] = False
        self.imu.start()
        self.report["data_started"] = True
        return self.imu

    def cancel_before_apply(self):
        # An armed reset may already be in progress even if status is unreadable.
        if self.possible_future_apply:
            raise UnknownFutureApply("Retained reset may be live; no normal rollback")
        return dict(no_future_apply=True, watcher_never_armed=not self.arm_attempted)

    def stop(self):
        if self.closed:
            return self.report
        if not self.lease_proven:
            raise UnknownFutureApply(
                "Retained Sleep closes through full preparation or explicit recovery"
            )
        if self.camera._imu not in (None, self.imu) or self.camera._usb_preview not in (
            None,
            self.preview,
            self.old_preview,
        ):
            raise PreparedStreamError("Foreign producer prevents retained rollback")
        if self.camera._usb_preview is self.old_preview:
            self.old_preview._scheduled = False
            self.old_preview._closed = True
            self.camera._usb_preview = self.preview
        self.report["preview_stop"] = self.preview.stop()
        self.report["native_restored"] = self.preview.report.get("restored") is True
        self.old_preview.report["restored"] = self.report["native_restored"]
        self.closed = True
        return self.report

    def abandon_local(self):
        # Reuse the proven partial-capture mask barrier, without any D1/IP
        # cleanup. An unknown reset is recovered only by explicit owner reboot.
        if self.imu is not None:
            proof = self.imu.stop()
            if self.imu_started and (
                type(proof) is not dict
                or proof.get("restored") is not True
                or proof.get("errors")
                or not _collector_closed(self.imu)
            ):
                raise PreparedStreamError("Unknown IMU mask; retain heartbeat and IP")
        if self.preview is not None and self.preview._stream is not None:
            self.preview._stream.stop()
        lease = self.preview._lease if self.preview is not None else None
        if lease is None:
            lease = self.old_preview._lease
        lease.stop(close_camera=False)
        self.abandoned = True
        return dict(local_only=True, recovery_required=True)
