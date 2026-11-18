"""Optional, scoped battery charging policy through a verified native driver.

This is not a USB input-current limit. Readback is driver policy, not physical
proof of zero battery/USB current. No camera-side persistent hook is installed.
The caller owns the lease across capture sessions and restores it only after
all capture consumers have closed. An uncertain diagnostic task is never
bypassed by opening another controller.
"""

from copy import deepcopy
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import secrets
import shlex
import time
import zlib

from . import _app, _usb
from ._helper_install import _controller
from ._prepared_stream import upload
from .controls import CameraReplyError
from .transport import RequestTimeout


class USBChargingError(RuntimeError):
    def __init__(self, message, report=None):
        self.report = deepcopy(report or {})
        super().__init__(message)


def attached_camera_count():
    """Complete descriptor inventory; zero is absence, failures are unknown.

    No device handle/interface is opened. A partial or failed inventory never
    authorizes a webcam-only fallback.
    """
    usb = _usb.load_usb()
    context, devices = C.c_void_p(), C.POINTER(C.c_void_p)()
    initialized, count = False, -1
    try:
        _usb.check(usb, usb.libusb_init(C.byref(context)), 'Charging inventory init')
        initialized = True
        count = _usb.check(usb, usb.libusb_get_device_list(context, C.byref(devices)),
                           'Charging complete device inventory')
        matches = 0
        for index in range(count):
            raw = C.create_string_buffer(18)
            _usb.check(usb, usb.libusb_get_device_descriptor(devices[index], raw),
                       'Charging inventory descriptor')
            vendor, product = (int.from_bytes(raw.raw[8:10], 'little'),
                               int.from_bytes(raw.raw[10:12], 'little'))
            if vendor == 0x2CA3 and product != 0x0025:
                raise USBChargingError('DJI device is in an unsupported USB mode; absence is not proven')
            matches += (vendor, product) == (0x2CA3, 0x0025)
        return matches
    finally:
        if count >= 0:
            usb.libusb_free_device_list(devices, 1)
        if initialized:
            usb.libusb_exit(context)


def _command(program):
    compile(program, 'usb-charging-control', 'exec')
    command = 'python3 -B -c ' + shlex.quote(program)
    if not command.isascii() or len(command) > 500 or '\0' in command:
        raise ValueError('Charging diagnostic command exceeds500 ASCII bytes')
    return command


def _json(raw):
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError('Duplicate charging report key')
            value[key] = item
        return value
    if not isinstance(raw, bytes) or not 0 < len(raw) <= 16384:
        raise ValueError('Bounded charging report required')
    return json.loads(raw, object_pairs_hook=unique,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite charging report')))


def _unlink_command(path, inode):
    return _command('import os;p=' + repr(path) + ';s=os.stat(p);'
                    'assert [s.st_dev,s.st_ino]==' + repr(inode) + ';os.unlink(p)')


def _pending(control):
    session = control.bootstrap.session
    return (control.pending_command is not None or control.pending_task is not None
            or control._unknown_task or session.quarantined)


class USBChargingLease:
    """Explicit battery charging enable/disable with same-boot original restore.

    Construction is pure. ``with camera.usb_charging(False)`` prepares/restores
    this lease. Hold it outside the stream context so stop/Sleep does not turn
    charging back on while another camera still uses the host's USB budget.
    """
    def __init__(self, camera, enabled, *, timeout=45, clock=time.monotonic, sleep=time.sleep):
        if type(enabled) is not bool:
            raise ValueError('USB battery charging enabled must be bool')
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Charging timeout must be finite and positive')
        self.camera, self.enabled, self.timeout, self.clock = camera, enabled, timeout, clock
        self.sleep = sleep
        self.control = self.generation = self.boot_id = None
        self.receipt = None
        self.work = self.output = None
        self.staged = self.stage_attempted = self.uncertain = False
        self._idle_proof = None
        self._report = dict(requested=enabled, state='unprepared', ready=False,
                            physical_proof='unknown', policy_readback_is_cache=True,
                            error='', errors=[])

    @property
    def status(self):
        return deepcopy(self._report)

    def _error(self, message, *, report=None, uncertain=False):
        self.uncertain = self.uncertain or uncertain
        if report is not None:
            self._report.update(deepcopy(report))
        self._report.update(requested=self.enabled, ready=False, error=message)
        self._report['state'] = 'unknown' if self.uncertain else self._report.get('state', 'error')
        self._report.setdefault('errors', []).append(message)
        raise USBChargingError(message, self._report)

    def _signature(self):
        transport = self.camera._transport
        identity = transport._device_identity
        if (not transport.connected or transport._thread is None or type(identity) is not dict
                or any(type(identity.get(key)) is not int for key in ('bus', 'address'))):
            raise USBChargingError('Current charging USB reader generation is unavailable', self._report)
        return transport, transport._thread, identity['bus'], identity['address']

    def _assert_local_idle(self):
        stream = getattr(self.camera, '_usb_stream', None)
        if (self.camera._imu is not None or self.camera._usb_preview is not None
                or self.camera._record_owned
                or stream is not None and stream.owns_camera_resources):
            raise USBChargingError('USB charging writes require closed capture owners', self._report)

    def _guard(self, deadline, *, idle=False, restore=False):
        if self.clock() >= deadline:
            raise TimeoutError('USB charging operation deadline')
        if self.uncertain or self.control is None or _pending(self.control):
            raise USBChargingError('Unknown/pending charging diagnostic task; no new START', self._report)
        if self._signature() != self.generation or self.control.bootstrap.transport is not self.camera._transport:
            raise USBChargingError('Charging control generation changed during operation', self._report)
        if idle:
            self._assert_local_idle()
            if restore:
                # A completed capture cleanup may retain native Sleep. Restore
                # the already-owned policy using D1 without waking C1/video/IMU.
                return
            if (self._idle_proof is None or self._idle_proof['generation'] != self.generation
                    or not 0 <= self.clock()-self._idle_proof['at'] <= 5):
                raise USBChargingError('USB charging requires a fresh same-generation idle proof', self._report)

    def _idle_state(self, deadline):
        """Retry only the observed E1 GET rejection, never a policy operation."""
        started = self.clock()
        limit = min(deadline, started + 2)
        evidence = dict(started_at=started, deadline=limit, attempts=0,
                        observed_e1_rejections=0, history=[], outcome='pending')
        observations = self._report.setdefault('evidence', {}).setdefault('idle_get_readiness', [])
        observations.append(evidence)
        del observations[:-4]
        first_e1 = None
        try:
            while True:
                self._guard(deadline)
                self._assert_local_idle()
                remaining = limit - self.clock()
                if remaining <= 0:
                    raise TimeoutError('Read-only idle GET readiness deadline')
                evidence['attempts'] += 1
                try:
                    value = self.camera.state(timeout=remaining)
                except CameraReplyError as error:
                    if error.status != 0xE1:
                        raise
                    if first_e1 is None:
                        first_e1 = error
                    evidence['observed_e1_rejections'] += 1
                    evidence['history'].append(dict(at=self.clock(), status=0xE1,
                                                    payload_hex=error.payload[:16].hex()))
                    del evidence['history'][:-8]
                    self._guard(deadline)
                    self._assert_local_idle()
                    remaining = limit - self.clock()
                    if remaining <= 0:
                        raise TimeoutError('Read-only idle GET readiness deadline')
                    self.sleep(min(.05, remaining))
                    continue
                self._guard(deadline)
                self._assert_local_idle()
                if self.clock() >= limit:
                    raise TimeoutError('Read-only idle GET observation arrived after readiness deadline')
                evidence.update(outcome='response', finished_at=self.clock())
                return value
        except BaseException as error:
            evidence.update(outcome='error', finished_at=self.clock(),
                            error=(type(error).__name__ + ': ' + str(error))[:280])
            if first_e1 is not None and isinstance(error, TimeoutError):
                raise USBChargingError(str(error), self.status) from first_e1
            raise

    def _prove_idle(self, deadline):
        self._guard(deadline)
        self._assert_local_idle()
        try:
            state = self._idle_state(deadline)
            if state.recording_state != 0:
                raise USBChargingError('USB charging writes require fresh Record0', self._report)
            reply = self.camera._transport.request(0, 0x88, _app.query_payload(),
                                                  receiver=0x28, timeout=min(2, deadline-self.clock()))
            apps = _app.decode_query_reply(reply.payload)
            if not apps.all_apps_absent or apps.own_records or apps.count:
                raise USBChargingError('USB charging writes require no APP capture registration', self._report)
            evidence = dict(method='C1_Record0_and_APP_absent')
        except RequestTimeout:
            # An existing SDK Sleep can retain USB/D1 while C1 services sleep.
            # Only the source-pinned fresh native Sleep receipt is an alternate
            # idle proof; a missing/awake/unknown owner never authorizes SET.
            from ._stream_resume import _discover, _status_at
            self._guard(deadline)
            work = _discover(self.control)
            if work is None:
                raise USBChargingError('C1 unavailable without a verified native Sleep owner', self._report)
            snapshot = _status_at(self.control, work, deadline=deadline)
            if (snapshot.get('state') != 'SLEEP' or snapshot.get('owner_alive') is not True
                    or snapshot.get('requested_readiness', {}).get('system_power_level') not in (2, 3)
                    or snapshot.get('shadow', {}).get('value') != 'none'):
                raise USBChargingError('C1 unavailable; native Sleep idle proof is not valid', self._report)
            evidence = dict(method='source_pinned_native_Sleep', work=work,
                            owner=snapshot['owner'], service=snapshot['service'],
                            command_id=snapshot['command_id'])
        self._guard(deadline)
        self._idle_proof = dict(generation=self.generation, at=self.clock())
        self._report.setdefault('evidence', {})['idle'] = evidence

    def _run(self, command, deadline, *, idle=False, restore=False):
        self._guard(deadline, idle=idle, restore=restore)
        try:
            return self.control._run(command, timeout_ms=min(5000, max(100, int((deadline-self.clock())*1000))),
                                     pre_send=lambda: self._guard(deadline, idle=idle, restore=restore))
        except BaseException:
            # Do not create a fresh adapter after a possibly delivered START.
            if _pending(self.control):
                self.uncertain = True
            raise

    def _pull(self, path, deadline):
        self._guard(deadline)
        value = _json(self.control.bootstrap.pull_bytes(
            path, max_bytes=16384, timeout=min(10, deadline-self.clock())))
        self._guard(deadline)
        return value

    def _fresh_boot(self, deadline):
        nonce = secrets.token_hex(6)
        output = '/blackbox/export_tmp/osmo_charge_boot_' + nonce + '.json'
        program = ("import os,json;from pathlib import Path;d=Path('/blackbox/export_tmp');"
                   "assert Path('/blackbox').is_dir() and d.resolve()==d;d.mkdir(mode=448,exist_ok=True);"
                   "f=open(" + repr(output) + ",'x');s=os.fstat(f.fileno());json.dump(dict(boot="
                   "Path('/proc/sys/kernel/random/boot_id').read_text().strip(),i=[s.st_dev,s.st_ino]),f);f.close()")
        self._run(_command(program), deadline)
        result = self._pull(output, deadline)
        boot, inode = result.get('boot'), result.get('i')
        if (not isinstance(boot, str) or len(boot) != 36 or type(inode) is not list
                or len(inode) != 2 or any(type(n) is not int or n < 1 for n in inode)):
            raise ValueError('Malformed charging boot observation')
        self._run(_unlink_command(output, inode), deadline)
        return boot

    def _bind(self, deadline, control, verified_rebind):
        if self.uncertain or self.control is not None and _pending(self.control):
            self._error('Prior charging task is unknown; no replacement controller', uncertain=True)
        if not self.camera.connected:
            if self.generation is not None and not verified_rebind:
                self._error('Charging connection closed without a verified transition')
            # Open only. D1 readiness remains usable when ordinary C1 GETs are
            # unavailable in native Sleep; prepare() separately proves Record0.
            self.camera.connect(timeout=None)
        current = self._signature()
        changed = self.generation is not None and current != self.generation
        if changed and not verified_rebind:
            proof = getattr(self.camera, '_usb_charging_closed_transition', None)
            if (type(proof) is tuple and len(proof) == 5 and proof[1:] == current
                    and proof[0] is getattr(self.camera, '_usb_stream', None)
                    and proof[0].camera is self.camera and proof[0].cleanup_safe):
                verified_rebind = True
        if changed and not verified_rebind:
            self._error('Charging reader changed without a verified USB transition')
        if control is not None:
            if control.bootstrap.transport is not self.camera._transport or _pending(control):
                self._error('Current stream control has unknown/pending work')
            self.control = control
        elif self.control is None or changed or getattr(self.control, 'detached', False):
            self.control = _controller(self.camera, timeout=min(15, deadline-self.clock()))
        self.generation = current
        self._guard(deadline)
        if self.boot_id is None or changed:
            observed = self._fresh_boot(deadline)
            if self.boot_id is not None and observed != self.boot_id:
                previous = dict(boot_id=self.boot_id, receipt=deepcopy(self.receipt))
                self._report['retired_boot'] = previous
                self.receipt = None
                self.work = self.output = None
                self.staged = self.stage_attempted = False
            self.boot_id = observed
        self._report.setdefault('evidence', {})['host_generation'] = dict(bus=current[2], address=current[3])

    def _stage(self, deadline):
        if self.staged:
            return
        if self.stage_attempted:
            self._error('Charging source staging was uncertain; no implicit retry', uncertain=True)
        self.stage_attempted = True
        source = Path(__file__).with_name('_device').joinpath('usb_charging.py').read_bytes()
        if not source or not source.isascii() or b'\0' in source or len(source) > 32768:
            raise ValueError('Bounded ASCII native charging helper required')
        compile(source, 'usb_charging.py', 'exec')
        payload = zlib.compress(source, 9)
        if len(payload) > 8192:
            raise ValueError('Compressed charging helper exceeds8KiB')
        nonce = secrets.token_hex(6)
        self.work = '/dev/shm/osmo-charge-' + nonce
        self.output = '/blackbox/export_tmp/osmo_charge_' + nonce + '.json'
        self._run('umask 077 && mkdir ' + shlex.quote(self.work), deadline)
        upload(self.control, self.work + '/bundle.z', payload, guard=lambda: self._guard(deadline))
        self._packed_size, self._packed_sha = len(payload), hashlib.sha256(payload).hexdigest()
        self.staged = True

    def _native_command(self, operation, nonce):
        program = ('import zlib,hashlib;w=' + repr(self.work) + ";b=open(w+'/bundle.z','rb').read(8193);"
                   'assert len(b)==' + str(self._packed_size) + ' and hashlib.sha256(b).hexdigest()=='
                   + repr(self._packed_sha) + ";d={'__name__':'usb_charging_library'};exec(zlib.decompress(b),d);"
                   + "d['main'](w," + repr(operation) + ',' + repr(self.enabled) + ','
                   + repr(nonce) + ',' + repr(self.output) + ')')
        return _command(program)

    def _operate(self, operation, *, control=None, verified_rebind=False):
        deadline = self.clock() + self.timeout
        mutating = operation != 'check'
        mutation_attempted = valid_report = False
        try:
            if mutating:
                self._assert_local_idle()
            self._bind(deadline, control, verified_rebind)
            if operation == 'restore' and self.receipt is None:
                self._report.update(state='retired_boot' if self._report.get('retired_boot') else 'unprepared',
                                    ready=False, original_policy_restored=False)
                return self.status
            if operation == 'check' and self.receipt is None:
                self._error('Charging boot changed; active capture cannot adopt or reapply a policy')
            if mutating:
                if operation == 'prepare':
                    self._prove_idle(deadline)
                self._guard(deadline, idle=True, restore=operation == 'restore')
            self._stage(deadline)
            if operation == 'prepare':
                self._prove_idle(deadline)
            nonce = secrets.token_hex(8)
            command = self._native_command(operation, nonce)
            self._report.setdefault('evidence', {})['operation'] = dict(
                operation=operation, nonce=nonce, output=self.output, work=self.work, boot_id=self.boot_id)
            mutation_attempted = mutating
            self._run(command, deadline, idle=mutating,
                      restore=operation == 'restore')
            report = self._pull(self.output, deadline)
            if (type(report) is not dict or report.get('schema') != 1
                    or report.get('operation') != operation or report.get('operation_nonce') != nonce
                    or type(report.get('ok')) is not bool or type(report.get('ready')) is not bool
                    or report.get('boot_id', self.boot_id) != self.boot_id
                    or report['ok'] and report.get('boot_id') != self.boot_id
                    or type(report.get('errors')) is not list):
                raise ValueError('Stale/malformed native charging report')
            inode = report.get('i')
            if (type(inode) is not list or len(inode) != 2
                    or any(type(n) is not int or n < 1 for n in inode)):
                raise ValueError('Native charging report has no owned inode receipt')
            receipt = report.get('receipt')
            if receipt is not None:
                if self.receipt is not None and receipt != self.receipt:
                    raise ValueError('Original charging receipt changed')
                self.receipt = deepcopy(receipt)
            if report['ok'] and self.receipt is None:
                raise ValueError('Native charging success has no original policy receipt')
            valid_report = True
            host_evidence = self._report.get('evidence', {})
            self._report.update(deepcopy(report), requested=self.enabled, error='')
            self._report['evidence'] = dict(host_evidence, native=deepcopy(report.get('evidence', {})))
            # Never overwrite/replay an export. Delete only the exact file
            # whose current inode was returned by this completed operation.
            self._run(_unlink_command(self.output, inode), deadline)
            if not report['ok']:
                self._error('USB battery charging policy was not verified: ' +
                            '; '.join(map(str, report.get('errors', []))))
            return self.status
        except USBChargingError as error:
            if mutation_attempted and not valid_report:
                self.uncertain = True
            self._report.update(requested=self.enabled, ready=False, error=str(error))
            if self.uncertain:
                self._report['state'] = 'unknown'
            error.report = self.status
            raise
        except BaseException as error:
            if mutation_attempted and not valid_report:
                self.uncertain = True
            self._error(type(error).__name__ + ': ' + str(error), uncertain=self.uncertain)

    def prepare(self, *, control=None, verified_rebind=False):
        with self.camera._control_lock:
            return self._operate('prepare', control=control, verified_rebind=verified_rebind)

    def recheck(self, *, control=None, verified_rebind=False):
        with self.camera._control_lock:
            if self.receipt is None:
                self._error('Charging policy has no original same-boot receipt')
            return self._operate('check', control=control, verified_rebind=verified_rebind)

    def restore(self, *, control=None, verified_rebind=False):
        with self.camera._control_lock:
            if self.uncertain:
                self._error('Charging restoration is unknown after an uncertain task; no new START', uncertain=True)
            if self.receipt is None:
                return self.status
            return self._operate('restore', control=control, verified_rebind=verified_rebind)

    close = restore

    def __enter__(self):
        self.prepare()
        return self

    def __exit__(self, kind, error, traceback):
        try:
            self.restore()
        except Exception as cleanup:
            if error is None:
                raise
            if hasattr(error, 'add_note'):
                error.add_note('USB charging restore: ' + str(cleanup))
        return False
