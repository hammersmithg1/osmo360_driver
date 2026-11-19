"""Camera startup policy shared by ROS and non-ROS clients.

A retained native Sleep must be discovered before ordinary camera GETs or
helper installation: C1 can be asleep while the source-pinned D1 owner is
healthy. Discovery uses the same generation/quarantine rules as usb_stream.
An unknown owner/task is an error, never permission to install, key or reboot.
"""

from copy import deepcopy
import math
import time

from ._stream_defaults import build_stream_session
from ._stream_resume import _discover, _status_at
from ._standby_control import ExternallySleepingOwnerError
from ._startup_recovery import ExternalSleepRecovery


class StartupError(RuntimeError):
    def __init__(self, message, report):
        self.report = deepcopy(report)
        super().__init__(message)


class StreamStartup:
    def __init__(self, camera, *, clock=time.monotonic):
        self.camera, self.clock = camera, clock
        # Retain this exact diagnostic adapter after a failed observation.
        # Creating a new preflight object must not skip an unknown task.
        self._probe = None
        self._failed_helper_control = None
        self._verify_boot_after = None
        self._external_recovery = None
        self.report = {}

    def prepare(self, *, timeout=180, previous_boot_id=None):
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Startup timeout must be finite and positive')
        deadline = self.clock() + timeout
        report = self.report = dict(operation='prepare_stream', ready=False,
                                    mode='discovering', reboot_attempted=False)
        def remaining():
            value = deadline - self.clock()
            if value <= 0:
                raise TimeoutError('Camera startup preparation deadline; no operation replay')
            return value
        camera = self.camera
        try:
            with camera._control_lock:
                stream = getattr(camera, '_usb_stream', None)
                if (camera._imu is not None or camera._usb_preview is not None
                        or stream is not None and stream.owns_camera_resources):
                    raise RuntimeError('Camera startup requires closed capture owners')
                # Open only; a retained Sleep deliberately has no ordinary C1.
                camera.connect(timeout=None)
                remaining()
                if self._failed_helper_control is not None:
                    # Keep the exact helper adapter after a possibly delivered
                    # task. Only its RESULT can retire that uncertainty.
                    self._failed_helper_control.resolve_pending(timeout=min(5, remaining()))
                    remaining()
                    self._failed_helper_control = None
                if self._verify_boot_after is not None:
                    if previous_boot_id not in (None, self._verify_boot_after):
                        raise RuntimeError('Conflicting previous camera boot; no repeated reboot')
                    previous_boot_id = self._verify_boot_after
                    if self._external_recovery is not None:
                        report['recovery'] = deepcopy(self._external_recovery.report)
                        report['reboot_attempted'] = report['recovery']['reboot_attempted']
                if previous_boot_id is None:
                    if self._probe is None:
                        self._probe = build_stream_session(camera, decode=False)
                    manager = self._probe.idle_probe.__self__
                    manager.ensure(deadline=deadline, resolve=True)
                    remaining()
                    control = self._probe.control
                    work = _discover(control, deadline=deadline, clock=self.clock)
                    remaining()
                    if work is not None:
                        try:
                            observed = _status_at(control, work, deadline=deadline, clock=self.clock)
                        except ExternallySleepingOwnerError as error:
                            # Manual Sleep after a host crash has no SDK Sleep
                            # receipt or plan. Recover the exact owner once.
                            recovery = self._external_recovery = ExternalSleepRecovery(
                                camera, control, work, error.observation, clock=self.clock)
                            report['recovery'] = recovery.run(deadline=deadline)
                            report['reboot_attempted'] = True
                            previous_boot_id = self._verify_boot_after = report['recovery']['boot_id_before']
                            self._failed_helper_control = recovery.finalize_control
                        else:
                            remaining()
                            if observed['state'] not in ('AWAKE', 'SLEEP'):
                                raise RuntimeError('Retained owner transition is pending; no new input')
                            report.update(ready=True, mode='retained_owner',
                                          install_skipped=True, retained_owner=dict(
                                              work=work, state=observed['state'],
                                              owner=observed['owner'], service=observed['service']))
                            return deepcopy(report)
                report['mode'] = 'helpers'
                # Only a fully resolved read-only discovery with no retained
                # owner permits the idempotent installer. A known reboot stays
                # verify-only inside install_helpers(previous_boot_id=...).
                helpers = camera.install_helpers(timeout=remaining(),
                    **({} if previous_boot_id is None else {'previous_boot_id': previous_boot_id}))
                reboot_attempted = report['reboot_attempted'] or helpers.get('reboot_attempted') is True
                report.update(deepcopy(helpers), operation='prepare_stream', mode='helpers',
                              reboot_attempted=reboot_attempted)
                remaining()
                if (helpers.get('installed') is not True
                        or helpers.get('autostart_hydration_verified') is not True):
                    raise RuntimeError('Camera helper installation/autostart was not verified')
                if self._external_recovery is not None:
                    if helpers.get('boot_id') in (None, self._external_recovery.report.get('boot_id_before')):
                        raise RuntimeError('External-sleep recovery did not prove a different camera boot')
                    report['recovery']['new_boot_verified'] = True
                    report['recovery']['boot_id_after'] = helpers['boot_id']
                report['video_mode'] = camera.ensure_video_mode(guard=remaining)
                remaining()
                report['ready'] = True
                if self._external_recovery is not None:
                    self._probe = None  # Only after the old boot was verified extinct.
                    self._external_recovery = None
                    self._failed_helper_control = None
                self._verify_boot_after = None
                return deepcopy(report)
        except Exception as error:
            report.update(ready=False, error=type(error).__name__ + ': ' + str(error))
            detail = getattr(error, 'report', None)
            if isinstance(detail, dict):
                report['failure'] = deepcopy(detail)
                if detail.get('reboot_attempted') is True:
                    # An accepted reboot may have lost its reply. Subsequent
                    # preparation is verify-only for this exact previous boot.
                    before = detail.get('boot_id_before')
                    if before is not None:
                        self._verify_boot_after = before
                    else:
                        # No identity means no safe automatic reboot replay.
                        self._verify_boot_after = 'unknown previous boot'
            helper_control = getattr(error, 'helper_control', None)
            if helper_control is not None:
                self._failed_helper_control = helper_control
            raise StartupError('Camera startup preparation failed: ' + str(error), report) from error
