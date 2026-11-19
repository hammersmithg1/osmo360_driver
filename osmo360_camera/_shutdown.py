"""Closed-capture stock Sleep for host shutdown; no preview or IMU is opened."""

from copy import deepcopy
from dataclasses import dataclass
from functools import partial
import math
import re
import secrets
import time

from . import _app
from ._prepared_attachment import export_prepared, restore_prepared
from ._prepared_plan import build_prepared_fast_plan
from ._standby_control import _sleep_observation_identity
from ._stream_defaults import build_stream_session
from ._stream_resume import _command, _discover, _json, _status_at, _publish_metadata, _read_metadata
from ._usb_charging import attached_camera_count
from ._usb_plan import _validate_runtime


def _closed(camera):
    stream = camera._usb_stream
    if (camera._imu is not None or camera._usb_preview is not None or camera._record_owned
            or stream is not None and stream.cleanup_safe is not True):
        raise RuntimeError('Shutdown requires verified closed capture owners')
    if stream is not None:
        _no_pending(stream._core.control)


def _no_pending(control):
    if (control.pending_command is not None or control.pending_task is not None
            or control._unknown_task or control.bootstrap.session.quarantined):
        raise RuntimeError('Unknown/pending camera task; shutdown cannot issue another START')


def _fresh_boot(control, remaining):
    _no_pending(control)
    output = '/blackbox/export_tmp/osmo_shutdown_' + secrets.token_hex(6) + '.json'
    program = ("import os,json;p=" + repr(output) + ";f=open(p,'x');"
               "v=dict(b=open('/proc/sys/kernel/random/boot_id').read().strip(),"
               "i=list(os.fstat(f.fileno())[1:3]));json.dump(v,f);f.close()")
    control._run(_command(program), timeout_ms=min(5000, int(remaining()*1000)))
    value = _json(control.bootstrap.pull_bytes(output, max_bytes=1024, timeout=min(10, remaining())))
    if (type(value) is not dict or set(value) != {'b', 'i'}
            or not isinstance(value['b'], str)
            or not re.fullmatch('[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', value['b'])
            or type(value['i']) is not list or len(value['i']) != 2
            or any(type(v) is not int or v <= 0 for v in value['i'])):
        raise ValueError('Fresh camera boot/inode receipt required')
    unlink = ("import os;p=" + repr(output) + "\nif list(os.stat(p)[1:3])!="
              + repr(value['i']) + ":raise ValueError('Export changed')\nos.unlink(p)")
    control._run(_command(unlink), timeout_ms=min(5000, int(remaining()*1000)))
    return value['b']


def _guard_cancelled(control, work, identifier, remaining):
    _no_pending(control)
    program = ('import usb_standby_owner as m;w=m.environment(m.base_module());c=m.configuration(w);i="'
               + identifier + '";g=m.read(w/("cmd-"+i)/"guard-finished.json")\n'
               'if not(m.correlated(g,c,dict(command_id=i,action="sleep")) and g.get("cancelled") is True'
               ' and all(g["closed"][k]["closed_verified_ebadf"] is True for k in ("alarm90","alarm240"))):'
               'raise ValueError("Guard not closed")')
    control._run(_command(program, work=work), timeout_ms=min(5000, int(remaining()*1000)))


def _apps_absent(camera, remaining):
    reply = camera._transport.request(0, 0x88, _app.query_payload(), receiver=0x28,
                                      timeout=min(2, remaining()))
    value = _app.decode_query_reply(reply.payload)
    if not value.all_apps_absent or value.count or value.own_records:
        raise RuntimeError('Shutdown Sleep requires all APP capture registrations absent')
    return value


@dataclass(frozen=True)
class _ClosedSleepProof:
    camera: object
    transport: object
    reader: object
    verified_at: float
    idle: object
    apps: object

    def validate(self, now):
        self.idle.validate(now)
        _closed(self.camera)
        if (not 0 <= now-self.verified_at <= 5 or self.camera._transport is not self.transport
                or self.transport._thread is not self.reader or not self.transport.connected
                or not self.apps.all_apps_absent or self.apps.count or self.apps.own_records):
            raise RuntimeError('Closed native-USB Sleep proof changed before the one input')


def prepare_shutdown(camera, *, expected_sleep=None, expected_boot_id=None, timeout=90):
    """Leave a verified native Sleep with no capture lease, then close local USB.

    A failed attempt is never replayed. Its controller stays attached to Camera
    for inspection, including any unknown task. Existing stable Sleep is read
    only. An idle awake camera receives exactly one stock QS Sleep input.
    """
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Finite positive shutdown timeout required')
    previous = getattr(camera, '_shutdown_report', None)
    if previous is not None:
        if previous.get('ready') is True:
            return deepcopy(previous)
        raise RuntimeError('Prior camera shutdown attempt failed; no input replay')
    _closed(camera)
    if attached_camera_count() != 1:
        raise RuntimeError('Shutdown requires exactly one supported attached Osmo 360')
    report = camera._shutdown_report = dict(ready=False, state='preparing', error='', sleep_input_attempted=False)
    deadline = time.monotonic() + timeout
    control = None

    def remaining():
        value = deadline-time.monotonic()
        if value < .1:
            raise TimeoutError('Camera shutdown deadline; no input replay')
        return value

    try:
        camera.connect(timeout=None)
        core = build_stream_session(camera, decode=False, timeout=remaining())
        control = core.control
        camera._shutdown_control = control
        core.idle_probe.__self__.ensure(deadline=deadline, resolve=True)
        boot = _fresh_boot(control, remaining)
        if expected_boot_id is not None and boot != expected_boot_id:
            raise RuntimeError('Camera boot changed before shutdown')
        work = _discover(control)
        remaining()
        current = _status_at(control, work, deadline=deadline) if work is not None else None
        if current is not None:
            receipt = dict(work=work, nonce=current['nonce'], owner=current['owner'], service=current['service'])
            core = build_stream_session(camera, decode=False, timeout=remaining(), existing=receipt,
                                        bootstrap=control.bootstrap)
            control = core.control
            camera._shutdown_control = control
            control.last = deepcopy(current)
        if expected_sleep is not None:
            expected_work = '/dev/shm/osmo-usb-standby-' + expected_sleep['nonce']
            if (work != expected_work or current is None or current.get('state') != 'SLEEP'
                    or _sleep_observation_identity(current, work, expected_sleep['command_id']) !=
                    _sleep_observation_identity(expected_sleep, work, expected_sleep['command_id'])):
                raise RuntimeError('Exact completed Sleep owner/command changed before shutdown')
        # A validated native-five-function report excludes a live ECM alias,
        # renewable preview plan, or an unrolled-back capture configuration.
        native = control.bootstrap.query_runtime(budget=remaining)
        _validate_runtime(native)
        report['native_usb_restored'] = True
        if current is None or current['state'] == 'AWAKE':
            idle = core.idle_probe(camera)
            _apps_absent(camera, remaining)
            if current is None:
                control.launch(recording_state=idle.recording_state, battery_present=idle.battery_present,
                               activation_state=idle.activation_state)
                work = control.work
            owner = dict(work=work, nonce=control.nonce, owner=control.owner, service=control.service)
            core.prepared_factory.preload_assets(camera, control, owner_receipt=owner)
            before = control.snapshot()
            number = before['last_sequence'] + 1
            if before['state'] != 'AWAKE' or number >= 0xffffffffffffffff:
                raise RuntimeError('Stable awake owner and next Sleep/Wake IDs required')
            backend = core.prepared_factory(camera, control, owner_receipt=owner,
                sleep_id=f'{number:016x}', wake_id=f'{number+1:016x}')
            backend.prepare()  # Unused RAM plan; never arm or start AV/lease.
            # Staging/INIT can be slow: refresh all evidence before the input.
            idle = core.idle_probe(camera)
            apps = _apps_absent(camera, remaining)
            proof = _ClosedSleepProof(camera, camera._transport, camera._transport._thread,
                                      time.monotonic(), idle, apps)
            proof.validate(time.monotonic())
            remaining()
            report['sleep_input_attempted'] = True
            submitted = control._submit('sleep', proof, deadline=deadline)
            if submitted['command_id'] != backend.sleep_id:
                raise RuntimeError('Prepared shutdown Sleep command generation changed')
            current = control.wait(submitted['command_id'], 'SLEEP', timeout=min(20, remaining()))
            metadata = export_prepared(backend, native_floor_ns=0)
            _publish_metadata(control, metadata, guard=remaining, deadline=deadline)
        else:
            metadata = _read_metadata(control, deadline=deadline)
            restore_prepared(camera, control, metadata,
                source_provider=core.prepared_factory.source_provider,
                plan_builder=partial(build_prepared_fast_plan, fast=True), decode=False,
                queue_size=1, imu_options=core.imu_options, deadline=deadline)
        report['resume_metadata_verified'] = True
        if current is None or current['state'] != 'SLEEP':
            raise RuntimeError('Native camera Sleep was not proven')
        identifier = current['command_id']
        control.last = deepcopy(current)
        observed = control.wait_sleep_snapshot(identifier, deadline=deadline)
        _guard_cancelled(control, work, identifier, remaining)
        final = control.wait_sleep_snapshot(identifier, deadline=deadline)
        if (_sleep_observation_identity(final, work, identifier) !=
                _sleep_observation_identity(observed, work, identifier)
                or _fresh_boot(control, remaining) != boot):
            raise RuntimeError('Camera boot/Sleep proof changed at shutdown completion')
        remaining()
        report.update(boot_id=boot, work=work, owner=deepcopy(final['owner']),
                      service=deepcopy(final['service']), command_id=identifier,
                      sleep=deepcopy(final), guard_cancelled=True)
        report['detach'] = control.detach()
        camera._transport.close()
        camera._usb_stream = None
        report.update(ready=True, state='sleeping', local_control_closed=True)
        return deepcopy(report)
    except BaseException as error:
        try:
            description = str(error)[:1000]
        except BaseException:
            description = type(error).__name__
        report.update(state='error', error=description)
        raise
