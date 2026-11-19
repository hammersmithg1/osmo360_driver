"""One explicit reboot of an exact owner externally put to sleep.

The cached AWAKE receipt is never rewritten into Sleep. Existing pinned native
finalize performs recovery; host success separately requires USB disappearance
and a later verify-only new-boot helper proof. Unknown tasks forbid delivery.
"""

from copy import deepcopy
import json
import re
import secrets
import time

from ._standby_control import StandbyControl, ExternallySleepingOwnerError, parse_snapshot
from ._stream_resume import _command, _status_at, WORK
from ._helper_install import _boot_id
from ._amt import wait_ready as wait_amt_ready


class StartupRecoveryError(RuntimeError):
    def __init__(self, message, report, control):
        self.report, self.helper_control = deepcopy(report), control
        super().__init__(message)


def _identity(value, work):
    matched = re.fullmatch(WORK, work)
    if matched is None:
        raise ValueError('Exact standby owner path required for recovery')
    try:
        parse_snapshot(json.dumps(value).encode(), nonce=matched[1])
    except ExternallySleepingOwnerError as proof:
        value = proof.observation
    else:
        raise ValueError('Only a fully verified externally sleeping AWAKE owner may recover')
    shadow = {key: item for key, item in value['shadow'].items() if key != 'value'}
    return (value['nonce'], value['owner'], value['service'], value['last_sequence'],
            value.get('command_id'), value.get('last_result'), shadow)


def _safe(control):
    session = control.bootstrap.session
    if (getattr(control, 'pending_command', False) is not None
            or getattr(control, 'pending_task', False) is not None
            or getattr(control, '_unknown_task', None) is not False
            or getattr(session, 'quarantined', None) is not False):
        raise RuntimeError('Unknown or pending task forbids external-sleep recovery')


def _owned_boot(control, work, observed, remaining):
    def run(command):
        budget = remaining()
        if budget < .1:
            raise TimeoutError('Insufficient recovery boot diagnostic task budget')
        result = control._run(command, timeout_ms=min(5000, int(budget*1000)), pre_send=remaining)
        remaining()
        return result
    output = '/blackbox/export_tmp/osmo_startup_' + secrets.token_hex(6) + '.json'
    # The same source-pinned native module verifies root tmpfs/work/nonce and
    # config boot_id against the running kernel before exporting this receipt.
    program = ("import usb_standby_owner as m,json,os;w=m.environment(m.base_module());"
        "c=m.configuration(w);f=open(" + repr(output) + ",'x');"
        "json.dump([c['boot_id'],c['owner'],c['sys_service'],list(os.fstat(f.fileno())[1:3])],f);f.close()")
    run(_command(program, work=work))
    value = json.loads(control.bootstrap.pull_bytes(output, max_bytes=16384, timeout=min(10, remaining())))
    remaining()
    if (type(value) is not list or len(value) != 4 or not _boot_id(value[0])
            or value[1] != observed['owner'] or value[2] != observed['service']
            or type(value[3]) is not list or len(value[3]) != 2
            or any(type(n) is not int or n < 1 for n in value[3])):
        raise ValueError('Same-boot owner/service recovery receipt is invalid')
    program = ("import os;p=" + repr(output) + ";assert list(os.stat(p)[1:3])=="
               + repr(value[3]) + ';os.unlink(p)')
    run(_command(program))
    remaining()
    return value[0]


class ExternalSleepRecovery:
    def __init__(self, camera, control, work, observed, *, clock=time.monotonic):
        self.camera, self.control, self.work = camera, control, work
        self.observed, self.clock = deepcopy(observed), clock
        self.finalize_control = None
        self.report = dict(reason='cached_AWAKE_native_sleep', reboot_attempted=False,
                           disconnect_observed=False, new_boot_verified=False)

    def run(self, *, deadline):
        report = self.report
        def remaining():
            value = deadline - self.clock()
            if value <= 0:
                raise TimeoutError('External-sleep recovery deadline; no repeated reboot')
            return value
        try:
            if report['reboot_attempted']:
                raise RuntimeError('External-sleep reboot already attempted; verify new boot only')
            expected = _identity(self.observed, self.work)
            _safe(self.control)
            # Re-observe the same source/owner/terminal command immediately
            # before recovery. A transition or changed identity is not adopted.
            try:
                _status_at(self.control, self.work, deadline=deadline, clock=self.clock)
            except ExternallySleepingOwnerError as error:
                observed = error.observation
            else:
                raise RuntimeError('Camera state changed before external-sleep recovery')
            if (_identity(observed, self.work) != expected
                    or observed['at']['boottime'] < self.observed['at']['boottime']):
                raise RuntimeError('Camera owner changed before external-sleep recovery')
            remaining()
            _safe(self.control)
            report['boot_id_before'] = _owned_boot(self.control, self.work, observed, remaining)
            _safe(self.control)
            receipt = dict(work=self.work, nonce=observed['nonce'], owner=observed['owner'],
                           service=observed['service'])
            attached = self.finalize_control = StandbyControl(self.control.bootstrap,
                source_provider=self.control.sources, control_lock=self.camera._control_lock,
                existing=receipt)
            attached.last = deepcopy(observed)
            def delivery():
                remaining()
                _safe(self.control)
                _safe(attached)
                report['reboot_attempted'] = True
            try:
                report['request'] = attached.finalize(guard=delivery)
            except Exception as error:
                report['receipt_error'] = type(error).__name__ + ': ' + str(error)
                if not report['reboot_attempted']:
                    raise
                # Native may reboot before D1's receipt reaches the host.
                # Observe disappearance; never send finalize a second time.
            if not self.camera._transport.wait_disconnected(min(30, remaining())):
                raise RuntimeError('Recovery reboot USB disappearance was not observed')
            report['disconnect_observed'] = True
            remaining()
            self.camera._transport.close()
            self.camera.connect(timeout=min(45, remaining()))
            remaining()
            # A possibly delivered old task is retired only with its exact
            # RESULT. The following installer will verify the different boot.
            report['control_readiness'] = wait_amt_ready(self.camera._transport,
                                                        timeout=min(15, remaining()))
            remaining()
            report['prior_task_result'] = attached.resolve_pending(timeout=min(5, remaining()))
            remaining()
            return deepcopy(report)
        except Exception as error:
            report['error'] = type(error).__name__ + ': ' + str(error)
            raise StartupRecoveryError(str(error), report,
                                       self.finalize_control or self.control) from error
