"""Lazy discovery/reattachment of this SDK's per-boot RAM standby owner.

Discovery reads the role bind and never guesses a key, unmounts, or reboots.
Only the small diagnostic export uses the stock diagnostic directory. Saved
prepared-plan metadata stays in the existing owner's RAM directory.
"""

import base64
from copy import deepcopy
from functools import partial
import hashlib
import json
import re
import secrets
import shlex
import threading
import time

from ._prepared_attachment import decode_metadata, export_prepared, restore_prepared
from ._prepared_plan import build_prepared_fast_plan
from ._prepared_stream import preparation_run
from ._standby_control import SOURCES, parse_snapshot
from ._stream_defaults import build_stream_session
from ._stream_session import StreamSessionError
from ._usb_bootstrap import BootstrapError

TARGET = "/sys/devices/platform/soc/a600000.ssusb/mode"
WORK = r"/dev/shm/osmo-usb-standby-([0-9a-f]{12})"
Q = shlex.quote


def _command(program, *, work=None):
    compile(program, "standby-attachment", "exec")
    text = (
        ("cd " + Q(work) + " && " if work else "umask 077; ")
        + "python3 -B -c "
        + Q(program)
    )
    if not text.isascii() or len(text) > 500:
        raise ValueError("Fixed attachment command exceeds500 bytes")
    return text


def _discovery_program(
    output, *, target=TARGET, pattern="/dev/shm/osmo-usb-standby-*/role.txt"
):
    # os.stat tuple slots1:3 are inode/device; use the same pair for the export
    # cleanup. O_EXCL prevents adopting an older diagnostic file.
    return (
        "import os,json,glob;t=" + repr(target) + ";s=lambda p:list(os.stat(p)[1:3]);"
        "v={'m':[x.split() for x in open('/proc/self/mountinfo') if x.split()[4]==t],"
        "'w':[p[:-9] for p in glob.glob(" + repr(pattern) + ") if s(p)==s(t)]};"
        "f=open(" + repr(output) + ",'x');v['i']=s(f.fileno());d=json.dumps(v);"
        "assert len(d)<16384;f.write(d);f.close()"
    )


def _json(raw):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate attachment JSON key")
            result[key] = value
        return result

    if not isinstance(raw, bytes) or not 0 < len(raw) <= 16384:
        raise ValueError("Bounded attachment JSON required")
    return json.loads(
        raw,
        object_pairs_hook=unique,
        parse_constant=lambda text: (_ for _ in ()).throw(ValueError("Nonfinite JSON")),
    )


def _discover(control):
    control._run(
        _command(
            "from pathlib import Path;p=Path('/blackbox');assert p.is_dir() and p.resolve()==p;"
            "p=p/'export_tmp';p.mkdir(mode=448,exist_ok=True);assert p.is_dir() and not p.is_symlink() and p.resolve()==p"
        )
    )
    output = "/blackbox/export_tmp/osmo_resume_" + secrets.token_hex(6) + ".json"
    control._run(_command(_discovery_program(output)))
    value = _json(control.bootstrap.pull_bytes(output, max_bytes=16384))
    if (
        type(value) is not dict
        or set(value) != {"m", "w", "i"}
        or type(value["m"]) is not list
        or type(value["w"]) is not list
        or type(value["i"]) is not list
        or len(value["i"]) != 2
        or any(type(n) is not int or n < 1 for n in value["i"])
    ):
        raise ValueError("Malformed role discovery export")
    # Delete only the exclusive diagnostic file whose inode was exported.
    control._run(
        _command(
            "import os;p="
            + repr(output)
            + ";assert list(os.stat(p)[1:3])=="
            + repr(value["i"])
            + ";os.unlink(p)"
        )
    )
    rows, paths = value["m"], value["w"]
    if not rows and not paths:
        return None
    if (
        len(rows) != 1
        or len(paths) != 1
        or not isinstance(paths[0], str)
        or not re.fullmatch(WORK, paths[0])
    ):
        raise ValueError(
            "Unknown or ambiguous retained role shadow; no automatic recovery"
        )
    row, work = rows[0], paths[0]
    if (
        type(row) is not list
        or len(row) < 10
        or row[4] != TARGET
        or row[3] != "/" + work.rsplit("/", 1)[1] + "/role.txt"
        or "-" not in row
        or row[row.index("-") + 1] != "tmpfs"
    ):
        raise ValueError("Role mount does not belong to this exact RAM owner")
    return work


def _status_at(control, work):
    for name, digest in SOURCES.items():
        control._run(
            '[ "$(sha256sum '
            + Q(work + "/" + name)
            + ' | cut -d" " -f1)" = '
            + digest
            + " ]"
        )
    # Execute only after both packaged SHA256s matched the retained files.
    control._run("python3 -B " + Q(work + "/usb_standby_owner.py") + " status")
    nonce = re.fullmatch(WORK, work)[1]
    return parse_snapshot(
        control.bootstrap.pull_bytes(
            "/blackbox/export_tmp/osmo_probe_" + nonce + ".json", max_bytes=16384
        ),
        nonce=nonce,
    )


def _read_metadata(control):
    program = (
        "import usb_standby_owner as m; b=m.base_module();w=m.environment(b);c=m.configuration(w);"
        "p=w/'status.lock';p.mkdir();\ntry:m.export_status(b,w,c,m.read(w/'session.json'))\nfinally:p.rmdir()"
    )
    control._run(_command(program, work=control.work))
    raw = control.bootstrap.pull_bytes(
        "/blackbox/export_tmp/osmo_probe_" + control.nonce + ".json", max_bytes=16384
    )
    value = decode_metadata(raw)
    control.snapshot()  # Restore the regular diagnostic export and fresh Sleep proof.
    return value


def _wait_native(control, *, clock=time.monotonic, sleep=time.sleep):
    """An old owned preview lease may need10s to remove its ECM link.

    Observe only; never remove a link or adopt a foreign overlay. The ordinary
    preview bootstrap subsequently verifies the full native runtime again.
    """
    command = _command(
        "from pathlib import Path;p=Path('/sys/kernel/config/usb_gadget/g1').resolve();"
        "q=p/'configs/c.1';assert not(q/'f6').exists() and (p/'max_speed').read_text().strip()=='super-speed-plus';"
        "assert [(q/('f'+str(i))).resolve() for i in range(1,6)]=="
        "[p/'functions'/x for x in ('ffs.diag','ffs.bulk','ffs.bulk1','ffs.bulk5','ffs.bulk4')]"
    )
    deadline = clock() + 12
    while True:
        if clock() >= deadline:
            raise TimeoutError(
                "Retained awake owner has a non-native USB overlay; no automatic rollback"
            )
        try:
            control._run(command)
        except BootstrapError:
            if not control._resolve_readonly_exit1(command, deadline=deadline):
                raise
            sleep(min(0.5, max(0, deadline - clock())))
            continue
        if clock() >= deadline:
            raise TimeoutError("Native USB observation arrived after deadline")
        return


def _publish_metadata(control, value, *, guard=None):
    value = decode_metadata(value)
    expected = dict(
        work=control.work,
        nonce=control.nonce,
        owner=control.owner,
        service=control.service,
    )
    if value["owner_receipt"] != expected:
        raise ValueError("Receipt belongs to another RAM owner")
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("ascii")
    if len(raw) > 16384:
        raise ValueError("Session receipt exceeds16KiB")
    temporary = control.work + "/session.next-" + secrets.token_hex(6)
    preparation_run(control, "set -C; umask 077; : > " + Q(temporary), guard)
    # Every START is a different fixed append. An uncertain command aborts this
    # publication; it is never replayed and cannot publish a partial receipt.
    for at in range(0, len(raw), 210):
        preparation_run(
            control,
            "printf %s "
            + base64.b64encode(raw[at : at + 210]).decode("ascii")
            + " | base64 -d >> "
            + Q(temporary),
            guard,
        )
    preparation_run(
        control,
        '[ "$(sha256sum '
        + Q(temporary)
        + ' | cut -d" " -f1)" = '
        + hashlib.sha256(raw).hexdigest()
        + " ]",
        guard,
    )
    if guard is not None:
        guard()
    snapshot = control.snapshot()
    if guard is not None:
        guard()
    if snapshot["state"] != "SLEEP" or snapshot["command_id"] != value["sleep_id"]:
        raise ValueError("Receipt publication requires the same fresh stable Sleep")
    program = (
        "import usb_standby_owner as m,os;w=m.environment(m.base_module());c=m.configuration(w);"
        "p=w/"
        + repr(temporary.rsplit("/", 1)[1])
        + ";v=m.read(p);s=m.read(w/'status.json');"
        "assert s['state']=='SLEEP' and s['command_id']==v['sleep_id'] and "
        "s['owner']==v['owner_receipt']['owner']==c['owner'];m.replace(w/'session.json',v);p.unlink()"
    )
    preparation_run(control, _command(program, work=control.work), guard)
    return dict(published=True, bytes=len(raw), sleep_id=value["sleep_id"])


class ResumableUSBStream:
    """Public-facing lazy wrapper; stable close deliberately retains RAM Sleep."""

    def __init__(self, camera, **options):
        self.camera, self.options = camera, dict(options)
        self._core = build_stream_session(camera, **options)  # Pure validation.
        self._core._prepare_ready_callback = self._publish_ready
        self._lock = threading.RLock()
        self._started = self._unknown_shadow = False
        self.attachment = dict(discovered=False, existing=False)

    @property
    def report(self):
        value = self._core.report
        value["attachment"] = deepcopy(self.attachment)
        value["attachment"]["retained_shadow_unverified"] = self._unknown_shadow
        return value

    @property
    def state(self):
        return self._core.state

    @property
    def owns_camera_resources(self):
        return self._unknown_shadow or self._core.owns_camera_resources

    @property
    def cleanup_safe(self):
        return not self._unknown_shadow and self._core.cleanup_safe

    @property
    def stats(self):
        value = self._core.stats
        value["report"]["attachment"] = deepcopy(self.attachment)
        return value

    def start(self):
        with self._lock:
            if self._started:
                raise StreamSessionError(
                    "Resumable stream start is single-use", self.report
                )
            self._started = True
            self.camera.connect()
            with self.camera._control_lock:
                if self.camera._imu is not None or self.camera._usb_preview is not None:
                    raise StreamSessionError(
                        "Existing camera producers cannot be borrowed", self.report
                    )
                manager = self._core.idle_probe.__self__
                manager.ensure(deadline=manager.clock() + manager.timeout, resolve=True)
                control = self._core.control
                self._unknown_shadow = (
                    True  # Unknown discovery never authorizes a new owner.
                )
                work = _discover(control)
                self.attachment["discovered"] = True
                if work is not None:
                    snapshot = _status_at(control, work)
                    receipt = dict(
                        work=work,
                        nonce=snapshot["nonce"],
                        owner=snapshot["owner"],
                        service=snapshot["service"],
                    )
                    self._core = build_stream_session(
                        self.camera,
                        **self.options,
                        existing=receipt,
                        bootstrap=control.bootstrap,
                    )
                    self._core._prepare_ready_callback = self._publish_ready
                    control = self._core.control
                    control.last = deepcopy(snapshot)
                    self.attachment.update(
                        existing=True, work=work, state=snapshot["state"]
                    )
                    if snapshot["state"] == "SLEEP":
                        metadata = _read_metadata(control)
                        backend = restore_prepared(
                            self.camera,
                            control,
                            metadata,
                            source_provider=self._core.prepared_factory.source_provider,
                            plan_builder=partial(build_prepared_fast_plan, fast=True),
                            decode=self.options.get("decode", True),
                            queue_size=self.options.get("queue_size", 1),
                            imu_options=self._core.imu_options,
                        )
                        self._core.prepared_factory.assets = backend.assets
                        self._core.attach_sleep(
                            backend,
                            native_floor_ns=metadata["native_floor_ns"],
                            snapshot=control.last,
                        )
                    elif snapshot["state"] != "AWAKE":
                        raise ValueError(
                            "Owner transition is pending; no automatic key or recovery"
                        )
                    else:
                        _wait_native(control)
                self._unknown_shadow = False
            if self._core.state == "SLEEP":
                self._core.wake(timeout=self.options.get("timeout", 60))
            else:
                self._core.start()
            return self

    def sleep(self):
        with self._lock:
            result = self._core.sleep()
            if self._core.fast_sleep:
                # The worker publishes only after complete native5/plan proof.
                # It never takes this wrapper lock: wake/close may hold it while
                # joining the worker.
                return result
            try:
                with self.camera._control_lock:
                    metadata = export_prepared(
                        self._core._prepared,
                        native_floor_ns=self._core._native_floor_ns,
                    )
                    self.attachment["receipt"] = _publish_metadata(
                        self._core.control, metadata
                    )
            except BaseException as error:
                self.attachment["publication_error"] = repr(error)
                raise  # Actual stable Sleep remains owned; no automatic recovery.
            return result

    def _publish_ready(self, backend, native_floor_ns, *, guard=None):
        """Worker callback: caller holds control lock, NEVER wrapper lock."""
        metadata = export_prepared(backend, native_floor_ns=native_floor_ns)
        if guard is not None:
            guard()
        return _publish_metadata(backend.control, metadata, guard=guard)

    def wake(self, **options):
        with self._lock:
            self._core.wake(**options)
            return self

    def read_video(self, **options):
        return self._core.read_video(**options)

    def read_imu(self, **options):
        return self._core.read_imu(**options)

    def verify_control(self, **options):
        with self._lock:
            return self._core.verify_control(**options)

    def recover(self):
        with self._lock:
            return self._core.recover()

    def close(self):
        with self._lock:
            if self._unknown_shadow:
                raise StreamSessionError(
                    "Unknown retained shadow; refusing implicit cleanup or reboot",
                    self.report,
                )
            return self._core.close()

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


def build_resumable_stream(
    camera,
    *,
    decode=True,
    queue_size=1,
    imu_options=None,
    timeout=60,
    fast_sleep=False,
    retain_ecm=False,
):
    return ResumableUSBStream(
        camera,
        decode=decode,
        queue_size=queue_size,
        imu_options=imu_options,
        timeout=timeout,
        fast_sleep=fast_sleep,
        retain_ecm=retain_ecm,
    )
