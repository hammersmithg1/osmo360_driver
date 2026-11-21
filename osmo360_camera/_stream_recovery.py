"""One owned stream reboot, followed by verification-only completion.

No normal USB/DIAG rollback is allowed across the camera boot boundary. The
old D1 task is resolved before any new START, and old local capture producers
are quiesced before the single finalize request can be delivered.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
import shlex

from ._helper_install import _boot_id, package_profile, verify_installed
from ._standby_control import SOURCES, StandbyUncertain
from ._startup_recovery import _owned_boot
from ._stream_resume import _command


class StreamRecoveryError(RuntimeError):
    def __init__(self, message, report):
        self.report = deepcopy(report)
        super().__init__(message)


def _unique(values):
    result = []
    for value in values:
        if value is not None and not any(value is other for other in result):
            result.append(value)
    return tuple(result)


@dataclass(frozen=True)
class _VerifiedRebootProof:
    """Private capability for retiring only this old boot's host resources."""
    camera: object
    transport: object
    reader: object
    boot_id_before: str
    boot_id_after: str
    profile: str
    verified_at: float
    previews: tuple
    imus: tuple
    _coordinator: object

    def validate(self, camera=None):
        context = self._coordinator
        if (camera is not None and camera is not self.camera
                or context._proof is not self
                or context.camera is not self.camera
                or self.camera._transport is not self.transport
                or getattr(self.transport, '_thread', None) is not self.reader
                or self.reader is None or not self.transport.connected
                or not _boot_id(self.boot_id_before) or not _boot_id(self.boot_id_after)
                or self.boot_id_before == self.boot_id_after
                or context.report.get('new_boot_verified') is not True
                or context.report.get('disconnect_observed') is not True
                or self.profile != context.profile):
            raise RuntimeError('Verified different-boot local retirement proof changed')
        context._check_pins()
        return self

    def owns_preview(self, value):
        return any(value is item for item in self.previews)

    def owns_imu(self, value):
        return any(value is item for item in self.imus)


class StreamRecovery:
    """Retained by the core, including after a lost finalize receipt."""

    def __init__(self, core):
        self.core, self.camera, self.control = core, core.camera, core.control
        self.transport = self.camera._transport
        self.facade = getattr(self.camera, '_usb_stream', None)
        if self.facade is not None and self.facade is not core and getattr(self.facade, '_core', None) is not core:
            raise ValueError('Camera stream ownership differs from recovery core')
        if (not self.control.launch_attempted or self.control.owner is None
                or self.control.detached or self.control.finalize_attempted):
            raise ValueError('Known attached owner without an earlier finalize required')
        self.owner = core._owner_receipt()
        self.profile = package_profile()['manifest']['profile']
        self._proof = None
        self._old_reader = None
        self._pins = None
        self._subordinate_pins = ()
        self.previews = self.imus = self.videos = ()
        self.report = dict(state='RECOVERY_REQUIRED', reboot_attempted=False,
            disconnect_observed=False, new_boot_verified=False,
            boot_id_before=None, boot_id_after=None, old_boot_extinct=False,
            local_retired=False, profile=self.profile)

    def _remaining(self):
        value = self.deadline - self.core.clock()
        if value < .1:
            raise TimeoutError('Owned stream recovery deadline; no repeated finalize')
        return value

    def _diagnostics(self):
        session = self.control.bootstrap.session
        return dict(pending_command=self.control.pending_command,
            pending_task=self.control.pending_task, unknown_task=self.control._unknown_task,
            quarantined=getattr(session, 'quarantined', None),
            finalize_attempted=self.control.finalize_attempted)

    def _safe_tasks(self):
        self._remaining()
        session = self.control.bootstrap.session
        if (self.control.pending_task is not None or self.control._unknown_task
                or getattr(session, 'quarantined', None) is not False):
            raise StandbyUncertain('Unknown/running owned D1 task forbids recovery START')

    def _check_owner(self):
        self._remaining()
        if (self.camera._transport is not self.transport
                or self.core.control is not self.control
                or self.core._owner_receipt() != self.owner
                or self.control.detached
                or getattr(self.camera, '_usb_stream', None) is not self.facade):
            raise RuntimeError('Original camera/stream/control ownership changed')

    def _pin_resources(self):
        if self._pins is not None:
            self._check_pins()
            return
        prepared, retained = self.core._prepared, self.core._retained_backend
        for backend in _unique((prepared, retained)):
            if (backend.camera is not self.camera or backend.control is not self.control
                    or backend.owner_receipt != self.owner):
                raise RuntimeError('Foreign prepared backend prevents recovery')
        self.previews = _unique((self.core._preview,
            getattr(prepared, 'preview', None), getattr(retained, 'preview', None),
            getattr(retained, 'old_preview', None)))
        self.imus = _unique((self.core._imu, getattr(prepared, 'imu', None),
                             getattr(retained, 'imu', None)))
        self.videos = _unique(getattr(preview, '_stream', None) for preview in self.previews)
        subordinate = []
        for backend in _unique((prepared, retained)):
            for name in ('camera', 'control', 'preview', 'imu', 'old_preview'):
                subordinate.append((backend, name, getattr(backend, name, None)))
        for preview in self.previews:
            for name in ('camera', '_stream', '_lease', 'network'):
                subordinate.append((preview, name, getattr(preview, name, None)))
        for imu in self.imus:
            for name in ('_process', '_owned_process'):
                subordinate.append((imu, name, getattr(imu, name, None)))
        self._subordinate_pins = tuple(subordinate)
        self._pins = (self.core._preview, self.core._imu, prepared, retained,
                      self.camera._usb_preview, self.camera._imu)
        self._check_pins()
        for preview in self.previews:
            if preview.camera is not self.camera or not callable(getattr(preview, 'retire_after_reboot', None)):
                raise RuntimeError('Owned preview lacks host-only reboot retirement')
            video = getattr(preview, '_stream', None)
            if video is not None and not callable(getattr(video, 'close_local', None)):
                raise RuntimeError('Owned video lacks host-only quiescence')
        for imu in self.imus:
            if not all(callable(getattr(imu, name, None)) for name in ('quiesce_local', 'retire_after_reboot')):
                raise RuntimeError('Owned IMU lacks host-only quiescence/retirement')

    def _check_pins(self):
        if self._pins is None:
            return
        current = (self.core._preview, self.core._imu, self.core._prepared,
            self.core._retained_backend, self.camera._usb_preview, self.camera._imu)
        if (any(a is not b for a, b in zip(self._pins, current))
                or any(getattr(parent, name, None) is not value
                       for parent, name, value in self._subordinate_pins)
                or any(backend is not None and backend.owner_receipt != self.owner
                       for backend in self._pins[2:4])
                or self.camera._usb_preview is not None and not any(self.camera._usb_preview is p for p in self.previews)
                or self.camera._imu is not None and not any(self.camera._imu is p for p in self.imus)):
            raise RuntimeError('Owned capture resource identity changed during recovery')

    def _delivery_guard(self):
        self._check_owner()
        self._check_pins()
        self._safe_tasks()
        if (not self.transport.connected or getattr(self.transport, '_thread', None) is not self._old_reader):
            raise RuntimeError('Old USB generation changed before recovery delivery')

    def _run_old(self, command):
        budget = self._remaining()
        result = self.control._run(command, timeout_ms=min(5000, int(budget * 1000)),
                                   pre_send=self._delivery_guard)
        self._delivery_guard()
        return result

    def _join_workers(self):
        self.core._wait_preparation(timeout=self._remaining(), allow_error=True)
        thread = self.core._wake_thread
        if thread is not None:
            thread.join(self._remaining())
            if thread.is_alive():
                raise StandbyUncertain('Wake worker still active; no recovery START')

    def _prepare(self):
        self._check_owner()
        self._pin_resources()  # A retry must retain the first attempt's identities.
        if self.core.reconnect_control is not None:
            adapter = self.core.reconnect_control(self.camera, deadline=self.deadline)
            self.control.bind_bootstrap(adapter)
        self.control.resolve_pending(timeout=min(5, self._remaining()))
        self._safe_tasks()
        self._old_reader = getattr(self.transport, '_thread', None)
        if not self.transport.connected or self._old_reader is None:
            raise RuntimeError('Owned recovery requires a live control reader')
        for name, digest in SOURCES.items():
            self._run_old('[ "$(sha256sum ' + shlex.quote(self.control.work + '/' + name)
                          + ' | cut -d" " -f1)" = ' + digest + ' ]')
        self._run_old(_command('import usb_standby_owner as m;b=m.base_module();'
            'w=m.environment(b);m.configuration(w);b.RoleShadow(w).proof()', work=self.control.work))
        boot = _owned_boot(self.control, self.control.work,
            dict(owner=self.owner['owner'], service=self.owner['service']), self._remaining)
        self._delivery_guard()
        self.report['boot_id_before'] = boot
        self.core._set_state('RECOVERY_REQUIRED')
        # No mask restore or ordinary rollback: a native watcher might still
        # have future work. Kill/join only this exact owned host collector.
        for imu in self.imus:
            self._delivery_guard()
            result = imu.quiesce_local(timeout=self._remaining())
            if (type(result) is not dict or result.get('collector_exited') is not True
                    or result.get('workers_stopped') is not True):
                raise RuntimeError('IMU local quiescence is incomplete; no reboot')
        for video in self.videos:
            self._delivery_guard()
            result = video.close_local(timeout=self._remaining())
            if type(result) is not dict or result.get('local_cleanup_complete') is not True:
                raise RuntimeError('Video local quiescence is incomplete; no reboot')
        self._delivery_guard()

    def _request(self):
        def delivered():
            self._delivery_guard()
            self.report['reboot_attempted'] = True
        try:
            self.report['request'] = self.control.finalize(guard=delivered)
        except Exception as error:
            self.report['receipt_error'] = type(error).__name__ + ': ' + str(error)
            if not self.report['reboot_attempted']:
                raise
        self.core._set_state('RECOVERY_PENDING')
        self.report['state'] = 'RECOVERY_PENDING'
        self.report['reboot_requested'] = self.report.get('request', {}).get('reboot_requested')
        self.core.report['explicit_recovery'] = deepcopy(self.report)

    def _verify(self):
        self._check_owner()
        self._check_pins()
        if not self.report['disconnect_observed']:
            if not self.transport.wait_disconnected(min(30, self._remaining())):
                raise TimeoutError('Recovery reboot USB disappearance was not observed')
            self.report['disconnect_observed'] = True
            self._remaining()
            self.transport.close()
        if self.core.reconnect_control is None:
            raise RuntimeError('Recovery has no read-only reconnect capability')
        adapter = self.core.reconnect_control(self.camera, deadline=self.deadline)
        self.control.bind_bootstrap(adapter)
        self.report['prior_task_result'] = self.control.resolve_pending(timeout=min(5, self._remaining()))
        self._safe_tasks()
        verified = verify_installed(self.camera, self.profile, self.report['boot_id_before'],
            timeout=self._remaining(), control=self.control, guard=self._remaining)
        self._remaining()
        after = verified.get('boot_id')
        if (not _boot_id(after) or after == self.report['boot_id_before']
                or verified.get('autostart_hydration_verified') is not True):
            raise RuntimeError('Different-boot source/hydration proof is incomplete')
        if self.report['new_boot_verified'] and after != self.report['boot_id_after']:
            raise RuntimeError('Verified new camera boot changed during retirement retry')
        self.report.update(new_boot_verified=True, old_boot_extinct=True, boot_id_after=after)
        self._proof = _VerifiedRebootProof(self.camera, self.transport,
            getattr(self.transport, '_thread', None), self.report['boot_id_before'], after,
            self.profile, self.core.clock(), self.previews, self.imus, self)
        self._proof.validate(self.camera)

    def _retire(self):
        proof = self._proof
        proof.validate(self.camera)
        for imu in self.imus:
            self._remaining()
            imu.retire_after_reboot(proof, timeout=self._remaining())
        for preview in self.previews:
            self._remaining()
            preview.retire_after_reboot(proof, timeout=self._remaining())
        proof.validate(self.camera)
        # All old producers and local leases are gone. No native cleanup is
        # invoked here: the different boot already extinguished old RAM work.
        # Retire the controller before the non-throwing ownership commit. A
        # failed detach must not lose the references needed to retry safely.
        self.control.detach()
        self.camera._imu = self.camera._usb_preview = None
        self.camera._usb_stream = None
        self.camera._stream_startup = None
        self.core._preview = self.core._imu = self.core._prepared = None
        self.core._retained_backend = None
        self.core._primed_video.clear()
        self.core._primed_imu.clear()
        self.report.update(state='DETACHED', local_retired=True)
        self.core.report.update(explicit_recovery=deepcopy(self.report), cleanup_deferred=False,
            retained_shadow=False, native_restored=False, old_boot_extinct=True)
        self.core._set_state('DETACHED')

    def run(self, *, timeout, wait=True):
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Recovery timeout must be finite and positive')
        if self.report['state'] == 'DETACHED':
            return deepcopy(self.report)
        self.deadline = self.core.clock() + timeout
        try:
            self._join_workers()  # Workers may need Camera's control lock.
            if not self.camera._control_lock.acquire(timeout=self._remaining()):
                raise TimeoutError('Camera control lock remained busy during recovery')
            try:
                if not self.report['reboot_attempted']:
                    self._prepare()
                    self._request()
                if not wait:
                    return deepcopy(self.report)
                # A lost ACK, timeout or failed hydration never permits another
                # finalize. A later invocation continues only these proof steps.
                self._verify()
                self._retire()
            finally:
                self.camera._control_lock.release()
            self.report.pop('error', None)
            return deepcopy(self.report)
        except Exception as error:
            self.report['error'] = type(error).__name__ + ': ' + str(error)
            self.report['control'] = self._diagnostics()
            self.core.report['explicit_recovery'] = deepcopy(self.report)
            raise StreamRecoveryError(self.report['error'], self.report) from error
