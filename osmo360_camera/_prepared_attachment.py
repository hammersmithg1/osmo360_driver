"""Private export/reattach for an UNARMED prepared plan in stable camera Sleep.

Metadata is JSON data, never a command or a host-clock anchor. Restore rebuilds
the same reviewed plan and proves the existing camera RAM work is still unused.
It never uploads, overwrites, arms a watcher, starts data, sends QS, or requests
recovery. The caller discovers/attaches the known standby owner first.
"""

from copy import deepcopy
from dataclasses import asdict, fields
import hashlib
import json
import re
import shlex
import time

from . import _host_network, _prepared_stream as prepared
from ._preview_lease import PreviewLease
from ._usb_bootstrap import USBRuntime
from ._usb_bundle import NAMES, build_bundle
from ._usb_plan import _validate_runtime
from .imu import IMUStream
from .usb_preview import USBPreview
from .video import VideoStream

MAX_METADATA = 16384
Q = shlex.quote


class PreparedAttachmentError(prepared.PreparedStreamError):
    pass


def _profiles():
    return dict(prepared.SOURCES, **{"usb_standby_owner.py": prepared.OWNER_SHA})


def _json(value):
    data = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("ascii")
    if not 0 < len(data) <= MAX_METADATA:
        raise ValueError("Prepared attachment metadata exceeds16KiB")
    return data


def _unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("Duplicate prepared metadata key")
        value[key] = item
    return value


def _runtime(value):
    if type(value) is not dict or set(value) != {
        item.name for item in fields(USBRuntime)
    }:
        raise ValueError("Exact USBRuntime fields required")
    value = deepcopy(value)
    if (
        type(value["original_links"]) is not list
        or len(value["original_links"]) != 5
        or type(value["camera_ipv4"]) is not list
        or any(type(row) is not list or len(row) != 2 for row in value["camera_ipv4"])
    ):
        raise ValueError("JSON native links/IPv4 schema")
    value["original_links"] = tuple(value["original_links"])
    value["camera_ipv4"] = tuple(tuple(row) for row in value["camera_ipv4"])
    result = USBRuntime(**value)
    _validate_runtime(result)
    return result


def _manifest(value, runtime):
    plan = value["plan"]
    return dict(
        schema=1,
        nonce=value["prepared_nonce"],
        standby_work=value["owner_receipt"]["work"],
        standby_nonce=value["owner_receipt"]["nonce"],
        standby_owner=value["owner_receipt"]["owner"],
        owner_sha256=prepared.OWNER_SHA,
        sys_service=value["owner_receipt"]["service"],
        sleep_command_id=value["sleep_id"],
        wake_command_id=value["wake_id"],
        plan_work=plan["work_dir"],
        scripts={
            name: dict(sha256=plan["sha256"][name], bytes=plan["byte_lengths"][name])
            for name in plan["sha256"]
        },
        bundle_plain_sha256=plan["plain_sha256"],
        native_baseline=asdict(runtime),
    )


def decode_metadata(value):
    if isinstance(value, (bytes, bytearray, memoryview)):
        if not 0 < len(value) <= MAX_METADATA:
            raise ValueError("Bounded prepared metadata bytes required")
        value = json.loads(
            bytes(value),
            object_pairs_hook=_unique,
            parse_constant=lambda name: (_ for _ in ()).throw(
                ValueError("Nonfinite metadata")
            ),
        )
    else:
        value = json.loads(_json(value))  # Return a detached, JSON-safe object.
    required = {
        "schema",
        "owner_receipt",
        "profile_hashes",
        "sleep_id",
        "wake_id",
        "prepared_work",
        "prepared_nonce",
        "assets_work",
        "assets_nonce",
        "runtime",
        "plan",
        "manifest_sha256",
        "native_floor_ns",
    }
    if (
        type(value) is not dict
        or set(value) != required
        or type(value["schema"]) is not int
        or value["schema"] != 1
    ):
        raise ValueError("Unknown prepared attachment metadata schema")
    prepared.receipt(value["owner_receipt"])
    if value["profile_hashes"] != _profiles():
        raise ValueError("Retained owner/watcher profile differs from this SDK")
    for key in ("prepared_nonce", "assets_nonce"):
        if not isinstance(value[key], str) or not re.fullmatch(
            "[0-9a-f]{12}", value[key]
        ):
            raise ValueError("Exact owned prepared/asset nonces required")
    if (
        value["prepared_work"]
        != "/dev/shm/osmo-prepared-preview047-" + value["prepared_nonce"]
        or value["assets_work"]
        != "/dev/shm/osmo-prepared-assets-" + value["assets_nonce"]
    ):
        raise ValueError("Exact canonical prepared/asset RAM roots required")
    for key in ("sleep_id", "wake_id"):
        if not isinstance(value[key], str) or not re.fullmatch(
            "[0-9a-f]{16}", value[key]
        ):
            raise ValueError("Exact prepared Sleep/Wake IDs required")
    if (
        int(value["sleep_id"], 16) < 1
        or int(value["wake_id"], 16) != int(value["sleep_id"], 16) + 1
    ):
        raise ValueError("Exact previous Sleep and next Wake sequence required")
    if type(value["native_floor_ns"]) is not int or value["native_floor_ns"] < 0:
        raise ValueError(
            "Native camera-clock floor must be nonnegative integer nanoseconds"
        )
    runtime = _runtime(value["runtime"])
    plan = value["plan"]
    if (
        type(plan) is not dict
        or set(plan)
        != {
            "tag",
            "nonce",
            "port",
            "work_dir",
            "sha256",
            "byte_lengths",
            "plain_sha256",
        }
        or not isinstance(plan["tag"], str)
        or not re.fullmatch("[A-Za-z0-9_-]{1,32}", plan["tag"])
        or plan["work_dir"] != "/dev/shm/osmo-ncm-" + plan["tag"]
        or not isinstance(plan["nonce"], str)
        or not re.fullmatch("[0-9a-f]{32}", plan["nonce"])
        or type(plan["port"]) is not int
        or not 49152 <= plan["port"] <= 65535
        or type(plan["sha256"]) is not dict
        or set(plan["sha256"]) != NAMES
        or type(plan["byte_lengths"]) is not dict
        or set(plan["byte_lengths"]) != NAMES
    ):
        raise ValueError("Exact deterministic fresh-plan parameters required")
    for name in NAMES:
        if (
            not isinstance(plan["sha256"][name], str)
            or not re.fullmatch("[0-9a-f]{64}", plan["sha256"][name])
            or type(plan["byte_lengths"][name]) is not int
            or not 0 < plan["byte_lengths"][name] <= 65536
        ):
            raise ValueError("Exact seven script hash/length records required")
    for digest in (plan["plain_sha256"], value["manifest_sha256"]):
        if not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
            raise ValueError("Exact plan/manifest SHA256 required")
    if (
        hashlib.sha256(_json(_manifest(value, runtime)) + b"\n").hexdigest()
        != value["manifest_sha256"]
    ):
        raise ValueError("Metadata does not reconstruct its exact uploaded manifest")
    _json(value)
    return value


def _stable_sleep(control, owner_receipt, sleep_id, *, deadline=None):
    prepared.same_owner(control, owner_receipt)
    value = (control.snapshot() if deadline is None else
             control.wait_sleep_snapshot(sleep_id, deadline=deadline))
    prepared.same_owner(control, owner_receipt)
    if (
        value["state"] != "SLEEP"
        or value.get("command_id") != sleep_id
        or type(value.get("last_sequence")) is not int
        or value["last_sequence"] != int(sleep_id, 16)
        or control.pending_command is not None
    ):
        raise PreparedAttachmentError(
            "Exact stable Sleep with no pending input required"
        )
    return value


def export_prepared(backend, *, native_floor_ns, deadline=None):
    """Export only a staged, never-armed backend after its completed Sleep."""
    if not isinstance(backend, prepared.PreparedStream):
        raise TypeError("Owned PreparedStream required")
    with backend.control.lock:
        if (
            not backend.prepared
            or backend.closed
            or backend.abandoned
            or backend.arm_attempted
            or backend.possible_future_apply
            or backend.lease_proven
            or backend.attach_attempted
            or backend.imu_started
            or backend.imu.pid is not None
            or backend.preview._scheduled
            or backend.preview._stream is not None
            or backend.preview._lease is not None
            or backend.camera._imu is not None
            or backend.camera._usb_preview is not None
            or not backend.assets.staged
        ):
            raise PreparedAttachmentError(
                "Only an unarmed, unstarted prepared Sleep can be exported"
            )
        _stable_sleep(backend.control, backend.owner_receipt, backend.sleep_id, deadline=deadline)
        plan = backend.plan
        prefix = "/dev/shm/osmo-ncm-"
        if not plan["work_dir"].startswith(prefix):
            raise ValueError("Exact single-use plan RAM root required")
        value = dict(
            schema=1,
            owner_receipt=deepcopy(backend.owner_receipt),
            profile_hashes=_profiles(),
            sleep_id=backend.sleep_id,
            wake_id=backend.wake_id,
            prepared_work=backend.work,
            prepared_nonce=backend.nonce,
            assets_work=backend.assets.work,
            assets_nonce=backend.assets.nonce,
            runtime=asdict(backend.runtime),
            native_floor_ns=native_floor_ns,
            plan=dict(
                tag=plan["work_dir"][len(prefix) :],
                nonce=plan["lease"]["nonce"],
                port=plan["lease"]["port"],
                work_dir=plan["work_dir"],
                sha256=deepcopy(plan["sha256"]),
                byte_lengths=deepcopy(plan["byte_lengths"]),
                plain_sha256=plan["bundle"]["plain_sha256"],
            ),
            manifest_sha256="0" * 64,
        )
        canonical = json.loads(_json(value))
        manifest = _manifest(canonical, backend.runtime)
        if manifest != backend.report.get("manifest"):
            # Tuples inside runtime are JSON arrays on both actual upload and
            # receipt. Compare canonical forms, not Python container classes.
            if _json(manifest) != _json(backend.report.get("manifest")):
                raise ValueError("Backend manifest changed after its upload")
        canonical["manifest_sha256"] = hashlib.sha256(
            _json(manifest) + b"\n"
        ).hexdigest()
        return decode_metadata(canonical)


def _ro(control, work, program, *args):
    compile(program, "prepared-attachment-readonly", "exec")
    command = "cd " + Q(work) + " && python3 -B -c " + Q(program)
    if args:
        command += " " + " ".join(Q(str(item)) for item in args)
    control._run(prepared.bounded(command))


def _prove_camera(control, value, assets):
    work = value["prepared_work"]
    # Never import an unverified camera-side module. -B prevents bytecode writes.
    for root in (work, value["assets_work"]):
        for name, digest in prepared.SOURCES.items():
            control._run(
                prepared.bounded(
                    '[ "$(sha256sum '
                    + Q(root + "/" + name)
                    + ' | cut -d" " -f1)" = '
                    + digest
                    + " ]"
                )
            )
    control._run(assets._guard_command().replace("python3 -c ", "python3 -B -c ", 1))
    control._run(
        prepared.bounded(
            '[ "$(sha256sum '
            + Q(work + "/manifest.json")
            + ' | cut -d" " -f1)" = '
            + value["manifest_sha256"]
            + " ]"
        )
    )
    program = (
        'import prepared_preview_wake_047 as m;w=m.environment();c=m.validate_manifest(m.base.read_json(w/"manifest.json"),w);'
        's=m.source_guard(c);r=m.command_proof(c,s,c["sleep_command_id"],"sleep");v=m.base.read_json(s/"status.json");'
        'assert v["state"]=="SLEEP" and v["last_result"]==r["result"] and not(s/"request.json").exists();'
        "m.base.plan_proof(c);m.base.native_proof(c)"
    )
    _ro(control, work, program)
    exact_files = (
        'import prepared_preview_wake_047 as m;w=m.environment();c=m.base.read_json(w/"manifest.json");'
        'assert {p.name for p in w.iterdir()}=={"prepared_preview_wake_047.py","prepared_preview_wake_045.py","manifest.json"};'
        'p=m.Path(c["plan_work"]);m.base.private(p,directory=True);'
        'assert {x.name for x in p.iterdir()}==set(c["scripts"])|{"unpack.py","scripts.z","bundle.ready"}'
    )
    _ro(control, work, exact_files)
    for name in prepared.SOURCES:
        proof = (
            "import os,sys,stat; a,b=map(os.lstat,sys.argv[1:3]);"
            "assert(a.st_dev,a.st_ino)==(b.st_dev,b.st_ino);"
            "assert all(stat.S_ISREG(x.st_mode) and x.st_uid==0 and not x.st_mode&146 for x in (a,b))"
        )
        _ro(control, work, proof, value["assets_work"] + "/" + name, work + "/" + name)


def restore_prepared(
    camera,
    control,
    metadata,
    *,
    source_provider,
    plan_builder,
    network_guard=None,
    decode=True,
    queue_size=1,
    imu_options=None,
    preview_factory=USBPreview,
    imu_factory=IMUStream,
    lease_factory=PreviewLease,
    video_factory=VideoStream,
    clock=time.monotonic,
    deadline=None,
):
    """Return local unstarted objects for the same existing, never-armed plan.

    Any mismatch fails without upload, input, arm, rollback, or auto-recovery.
    No host monotonic timestamp survives the old process; only native floor does.
    """
    value = decode_metadata(metadata)
    with control.lock:
        if camera._imu is not None or camera._usb_preview is not None:
            raise PreparedAttachmentError("Do not adopt over existing camera producers")
        _stable_sleep(control, value["owner_receipt"], value["sleep_id"], deadline=deadline)
        runtime = _runtime(value["runtime"])
        expected = value["plan"]
        plan = plan_builder(
            runtime, tag=expected["tag"], nonce=expected["nonce"], port=expected["port"]
        )
        bundle = build_bundle(plan["scripts"], plan["work_dir"])
        if (
            plan["work_dir"] != expected["work_dir"]
            or plan["sha256"] != expected["sha256"]
            or plan["byte_lengths"] != expected["byte_lengths"]
            or plan["bundle"]["plain_sha256"] != expected["plain_sha256"]
            or bundle["plain_sha256"] != expected["plain_sha256"]
            or plan["commands"] != bundle["commands"]
            or plan["lease"]["setup_seconds"] != 90
            or plan["lease"]["lease_seconds"] != 10
        ):
            raise PreparedAttachmentError(
                "Local plan builder differs from retained exact046 scripts"
            )
        assets = prepared.PreparedAssets(
            control,
            value["owner_receipt"],
            source_provider,
            nonce=value["assets_nonce"],
        )
        _prove_camera(control, value, assets)
        _stable_sleep(control, value["owner_receipt"], value["sleep_id"], deadline=deadline)
        network = (
            network_guard
            if network_guard is not None
            else _host_network.preview_guard()
        )
        baseline = (
            network.preflight()
        )  # Fresh host baseline; never restore old host clocks/routes.
        assets.attempted = assets.staged = True  # Only after all read-only proof.
        backend = prepared.PreparedStream(
            camera,
            control,
            owner_receipt=value["owner_receipt"],
            sleep_id=value["sleep_id"],
            wake_id=value["wake_id"],
            source_provider=source_provider,
            plan_builder=plan_builder,
            assets=assets,
            network_guard=network,
            runtime=runtime,
            decode=decode,
            queue_size=queue_size,
            imu_options=imu_options,
            preview_factory=preview_factory,
            imu_factory=imu_factory,
            lease_factory=lease_factory,
            video_factory=video_factory,
            clock=clock,
        )
        backend.nonce, backend.work = value["prepared_nonce"], value["prepared_work"]
        backend.script = backend.work + "/" + prepared.WATCHER
        backend.runtime, backend.plan, backend.baseline = runtime, plan, baseline
        backend.preview = preview_factory(
            camera, decode=decode, queue_size=queue_size, network_guard=network
        )
        backend.preview._started = True
        backend.preview._runtime, backend.preview._plan, backend.preview._baseline = (
            runtime,
            plan,
            baseline,
        )
        backend.preview.report.update(
            prepared=True, runtime=asdict(runtime), work_dir=plan["work_dir"]
        )
        backend.imu = imu_factory(**backend.imu_options)
        backend.prepared = backend.prepare_attempted = True
        backend.native_floor_ns = value["native_floor_ns"]
        backend.report.update(
            prepared=True,
            attached_existing=True,
            work=backend.work,
            plan_work=plan["work_dir"],
            assets_work=assets.work,
            immutable_assets_reused=True,
            manifest=_manifest(value, runtime),
            attachment_metadata=deepcopy(value),
            native_floor_ns=value["native_floor_ns"],
        )
        return backend
