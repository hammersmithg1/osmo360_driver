"""Public installation/removal lifecycle, including one verified warm reboot.

The device backend owns persistent files. This layer owns local producers and
the transition to a new camera boot. A lost reboot receipt never retries the
command. Success includes a fresh, read-only post-boot proof.
"""

from copy import deepcopy
import math
import time

from . import _helper_install as backend
from ._standby_control import StandbyControl, SOURCES


class HelperLifecycleError(RuntimeError):
    def __init__(self, message, *, report, control=None):
        self.report = deepcopy(report)
        self.helper_control = control
        super().__init__(message)


def _budget(timeout, clock):
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Helper operation timeout must be finite and positive")
    end = clock() + timeout

    def remaining():
        value = end - clock()
        if value <= 0:
            raise TimeoutError("Helper lifecycle deadline; no repeated reboot")
        return value

    return remaining


def _idle(camera):
    if camera._usb_preview is not None or camera._imu is not None:
        raise RuntimeError("Close preview and IMU before this helper operation")
    stream = getattr(camera, "_usb_stream", None)
    if stream is not None and stream.owns_camera_resources:
        raise RuntimeError("Close the USB stream before installing camera helpers")


def _reboot(camera, receipt, remaining, report):
    """Keep the backend's exact task session until the old boot disappears."""
    control = receipt.pop("_control", None)
    owner = receipt.get("owner_receipt")
    if owner is None:
        report["reboot_attempted"] = True
        value = camera.reboot(timeout=remaining())
        report["reboot"] = dict(
            disconnect_observed=True,
            elapsed=getattr(value, "reconnect_elapsed", None),
            method="stock_usb",
        )
        return
    if (
        control is None
        or control.lock is not camera._control_lock
        or control.bootstrap.transport is not camera._transport
    ):
        raise RuntimeError(
            "Owned standby reboot requires the exact completed diagnostic session"
        )
    control.resolve_pending(timeout=min(5, remaining()))
    remaining()
    if camera.state(timeout=min(2, remaining())).recording_state != 0:
        raise RuntimeError("Owned standby reboot requires a fresh Record0 observation")
    remaining()
    attached = StandbyControl(
        control.bootstrap,
        source_provider=lambda: {
            name: (backend.DEVICE / name).read_bytes() for name in SOURCES
        },
        control_lock=camera._control_lock,
        existing=owner,
    )
    report["reboot_attempted"] = True
    try:
        report["reboot_request"] = attached.finalize(guard=remaining)
    except Exception as error:
        # A lost receipt cannot authorize another power command. Actual USB
        # disappearance plus the subsequent new boot can still prove success.
        report["reboot_receipt_error"] = repr(error)
    if not camera._transport.wait_disconnected(min(20, remaining())):
        raise RuntimeError(
            "Owner reboot disappearance was not observed; no command retry"
        )
    report["reboot"] = dict(disconnect_observed=True, method="owned_standby_finalize")
    camera._transport.close()
    camera.connect(timeout=min(45, remaining()))


def install(camera, *, timeout=180, previous_boot_id=None, clock=time.monotonic):
    remaining = _budget(timeout, clock)
    if previous_boot_id is not None and not backend._boot_id(previous_boot_id):
        raise ValueError('Verified previous helper boot identity required')
    report = dict(operation="install_helpers", installed=False, reboot_attempted=False)
    control = None
    with camera._control_lock:
        _idle(camera)
        camera.connect(timeout=min(20, remaining()))
        if camera.state().recording_state != 0:
            raise RuntimeError("Installing helpers requires Record0")
        try:
            if previous_boot_id is not None:
                profile = backend.package_profile()['manifest']['profile']
                report.update(verification_only=True, boot_id_before=previous_boot_id, profile=profile)
                verified = backend.verify_boot_cache(camera, profile,
                    previous_boot_id=previous_boot_id, timeout=remaining(), guard=remaining)
                remaining()
                report.update(installed=True, already_installed=True, profile=profile,
                    autostart_hydration_verified=True, verification=verified,
                    boot_id_before=previous_boot_id, boot_id=verified['boot_id'],
                    hydration_wait=verified['hydration_wait'], verification_only=True)
                return report
            receipt = backend.install(camera, timeout=remaining(), guard=remaining)
            remaining()
            control = receipt.get("_control")
            public = {
                key: value for key, value in receipt.items() if not key.startswith("_")
            }
            report["installation"] = deepcopy(public)
            report['already_installed'] = receipt.get('already_installed') is True
            report['boot_id_before'] = receipt['boot_id']
            cache = receipt.get("boot_cache", {})
            if receipt.get('already_installed') is True:
                if (type(cache) is not dict or type(cache.get('hydrated')) is not bool
                        or cache.get('boot_id') != receipt.get('boot_id')
                        or cache.get('profile') != receipt.get('profile')
                        or cache.get('phase') not in (None, 'pending')):
                    raise RuntimeError('Installed helper cache identity is unverified; no reboot')
                expected_work = '/dev/shm/osmo360-sdk-cache-' + receipt['profile'][:12]
                if (cache.get('work') is not None and cache['work'] != expected_work
                        or cache['hydrated'] and (cache.get('mode') != 'bytes-only'
                            or cache.get('phase') is not None or cache.get('work') != expected_work)
                        or not cache['hydrated'] and (cache.get('mode') is not None
                            or cache.get('phase') == 'pending' and cache.get('work') != expected_work
                            or cache.get('phase') is None and cache.get('work') is not None)):
                    raise RuntimeError('Installed helper cache path/mode is unverified; no reboot')
            ready = (
                receipt.get("already_installed") is True
                and cache.get("hydrated") is True
                and cache.get("boot_id") == receipt.get("boot_id")
                and cache.get("profile") == receipt.get("profile")
                and cache.get("mode") == "bytes-only"
            )
            if ready:
                remaining()
                report.update(
                    installed=True,
                    already_installed=True,
                    autostart_hydration_verified=True,
                    profile=receipt["profile"],
                    boot_id=receipt['boot_id'],
                )
                return report
            pending = (receipt.get('already_installed') is True
                and cache.get('hydrated') is False and cache.get('phase') == 'pending'
                and cache.get('boot_id') == receipt.get('boot_id')
                and cache.get('profile') == receipt.get('profile'))
            if pending:
                verified = backend.verify_boot_cache(camera, receipt['profile'],
                    current_boot_id=receipt['boot_id'], timeout=remaining(), control=control, guard=remaining)
                remaining()
                report.update(installed=True, autostart_hydration_verified=True,
                    profile=receipt['profile'], boot_id=verified['boot_id'],
                    verification=verified, hydration_wait=verified['hydration_wait'])
                return report
            remaining()
            _reboot(camera, receipt, remaining, report)
            verified = backend.verify_installed(
                camera,
                receipt["profile"],
                receipt["previous_boot_id"],
                timeout=remaining(),
                guard=remaining,
            )
            remaining()
            report.update(
                installed=True,
                profile=receipt["profile"],
                autostart_hydration_verified=True,
                verification=verified,
                boot_id=verified['boot_id'],
                hydration_wait=verified.get('hydration_wait'),
            )
            return report
        except Exception as error:
            report["error"] = repr(error)
            raise HelperLifecycleError(
                "Camera helper installation was not fully verified",
                report=report,
                control=getattr(error, "helper_control", control),
            ) from error


def remove(camera, *, timeout=180, clock=time.monotonic):
    remaining = _budget(timeout, clock)
    report = dict(operation="remove_helpers", removed=False, reboot_attempted=False)
    control = None
    # Camera.close joins stream workers outside its control lock. It also
    # restores our IMU mask before any USB rollback. Preserve that ordering.
    if any(
        getattr(camera, name, None) is not None
        for name in ("_usb_stream", "_usb_preview", "_imu")
    ):
        camera.close()
        remaining()
    with camera._control_lock:
        _idle(camera)
        camera.connect(timeout=min(20, remaining()))
        if camera.state().recording_state != 0:
            raise RuntimeError("Removing helpers requires Record0")
        try:
            receipt = backend.remove(camera, timeout=remaining(), guard=remaining)
            remaining()
            control = receipt.get("_control")
            report["persistent_removal"] = deepcopy(
                {
                    key: value
                    for key, value in receipt.items()
                    if not key.startswith("_")
                }
            )
            if (
                receipt.get("requires_reboot") is not True
                or receipt.get("removed") is not True
            ):
                raise RuntimeError("Persistent removal must finish before reboot")
            remaining()
            _reboot(camera, receipt, remaining, report)
            verified = backend.verify_removed(
                camera,
                receipt["previous_boot_id"],
                timeout=remaining(),
                guard=remaining,
            )
            remaining()
            report.update(removed=True, installed=False, verification=verified)
            return report
        except Exception as error:
            report["error"] = repr(error)
            raise HelperLifecycleError(
                "Camera helper removal was not fully verified",
                report=report,
                control=getattr(error, "helper_control", control),
            ) from error
