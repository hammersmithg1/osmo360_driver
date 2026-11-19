"""Bounded synchronous client for the driver's serialized command worker.

The worker remains the only owner of camera operations. A caller timeout or
signal never authorizes a second USB task, a new owner, or a replay. Cancellation
waits for an already accepted transaction to finish before returning control to
the caller, so application cleanup can use the same owner normally.
"""

from copy import deepcopy
import math
import time


class CameraCommandError(RuntimeError):
    def __init__(self, message, status):
        self.status = deepcopy(status)
        super().__init__(message)


class CameraCommandCancelled(CameraCommandError):
    pass


def _deadline(timeout, clock, cancelled):
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Camera command timeout must be finite and positive')
    if cancelled is not None and not callable(cancelled):
        raise TypeError('cancelled must be callable or None')
    return clock() + timeout


def _wait_idle(owner, deadline, cancelled, clock, sleep):
    while True:
        state = owner.status()
        remaining = deadline - clock()
        if remaining <= 0:
            raise CameraCommandError('Camera command completion deadline; operation was not replayed', state)
        if not state.get('busy'):
            if cancelled is not None and cancelled():
                raise CameraCommandCancelled('Camera command cancelled after completion', state)
            return state
        sleep(min(.05, remaining))


def wait_idle(owner, *, timeout=150, cancelled=None,
              clock=time.monotonic, sleep=time.sleep):
    """Observe an accepted transaction to completion without issuing commands.

    Return its status, including any error. The caller may need to finish an
    application policy update before asking the same owner to prepare shutdown.
    Cancellation waits for that accepted transaction; timeout never replays it.
    """
    deadline = _deadline(timeout, clock, cancelled)
    return _wait_idle(owner, deadline, cancelled, clock, sleep)


def run_command(owner, action, settings=None, *, timeout=240, cancelled=None,
                clock=time.monotonic, sleep=time.sleep):
    """Issue once, then observe its completion with an explicit host deadline.

    A timeout leaves the accepted operation under the same worker's ownership.
    It must not be interpreted as proof that the command did not reach the
    camera. No command, cleanup, reconnect or reboot is retried here.
    """
    deadline = _deadline(timeout, clock, cancelled)
    if cancelled is not None and cancelled():
        raise CameraCommandCancelled('Camera command cancelled before delivery', owner.status())
    owner.command(action, settings)
    state = _wait_idle(owner, deadline, None, clock, sleep)
    if state.get('error'):
        raise CameraCommandError(state['error'], state)
    if cancelled is not None and cancelled():
        raise CameraCommandCancelled('Camera command cancelled after completion', state)
    return state


def prepare_shutdown(owner, *, timeout=150, cancelled=None,
                     clock=time.monotonic, sleep=time.sleep):
    """Wait for existing work, then prove camera-only shutdown using one budget.

    Application admission, motor stop, recording drain, webcam coordination and
    OS poweroff remain the caller's responsibility. This function never powers
    off the host or opens another camera owner after an uncertain operation.
    """
    deadline = _deadline(timeout, clock, cancelled)
    _wait_idle(owner, deadline, cancelled, clock, sleep)
    remaining = deadline - clock()
    if remaining <= 0:
        raise CameraCommandError('Camera shutdown deadline before delivery', owner.status())
    state = run_command(owner, 'prepare_shutdown', timeout=remaining,
                        cancelled=cancelled, clock=clock, sleep=sleep)
    proof = state.get('shutdown')
    if not isinstance(proof, dict) or proof.get('ready') is not True:
        raise CameraCommandError('Camera shutdown was not verified; keep host power on', state)
    return state


def prepare(owner, **options):
    """Verify this owner's requested charging policy without starting capture."""
    requested = owner.usb_charging
    state = run_command(owner, 'prepare_power', **options)
    power = state.get('power', {})
    if (power.get('error') or power.get('ready') is not True
            or 'requested' not in power or power['requested'] is not requested):
        raise CameraCommandError('USB charging preparation failed: ' +
            (power.get('error') or 'requested policy was not verified'), state)
    return state


def start(owner, **options):
    """Start using the same owner; success requires fresh video and IMU."""
    timeout = options.get('timeout', 240)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Camera command timeout must be finite and positive')
    clock, sleep = options.get('clock', time.monotonic), options.get('sleep', time.sleep)
    cancelled = options.get('cancelled')
    deadline = clock() + timeout
    state = run_command(owner, 'start', **options)
    while True:
        if state.get('error') or not state.get('enabled') or state.get('state') != 'streaming':
            raise CameraCommandError(state.get('error') or 'Camera start completed without an active stream', state)
        if cancelled is not None and cancelled():
            raise CameraCommandCancelled('Camera start cancelled after command completion', state)
        remaining = deadline - clock()
        if remaining <= 0:
            raise CameraCommandError('Fresh video/IMU deadline; start was not replayed', state)
        if all(type(state.get(field)) in (int, float) and math.isfinite(state[field])
               and 0 <= state[field] < 2 for field in ('frame_age_s', 'imu_age_s')):
            return state
        # Starting the native session precedes Python's first reader callback.
        # Observe those callbacks under the original budget rather than
        # reporting a false failed start or issuing another command.
        sleep(min(.05, remaining))
        state = owner.status()
