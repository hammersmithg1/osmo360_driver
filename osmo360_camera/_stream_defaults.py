"""Default assembly of the source-pinned USB stream/standby components.

Building the object validates only host options and packaged source bytes.
Device access begins at USBStreamSession.start(). A diagnostic adapter belongs
to one USB reader generation; read-only recovery never skips an old unknown
task by merely constructing another adapter.
"""

from functools import partial
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
import time

from . import controls
from ._amt import D1Session, wait_ready as wait_amt_ready
from ._prepared_plan import build_prepared_fast_plan
from ._prepared_stream import PreparedAssets, PreparedStream, SOURCES as WATCHER_SOURCES
from ._retained_stream import (
    RetainedAssets,
    RetainedStream,
    SOURCES as RETAINED_SOURCES,
)
from ._standby_control import StandbyControl, SOURCES as OWNER_SOURCES
from ._helper_install import InstalledSources
from ._stream_session import USBStreamSession, _IdleEvidence
from ._usb_bootstrap import USBBootstrap
from .imu import IMUStream
from .usb_preview import USBPreview
from .video import VideoStream
from .transport import USBError, CameraDisconnected, RequestTimeout

DEVICE = Path(__file__).with_name("_device")
ACTIVATION_QUERY = bytes.fromhex("310000000000")
BATTERY_QUERY = '[ "$(cat /sys/class/power_supply/battery/present)" = 1 ]'


def _sources(allowlist):
    """Only exact package filenames, verified before any camera-side change."""
    result = {}
    for name, expected in allowlist.items():
        data = (DEVICE / name).read_bytes()
        if (
            not data
            or not data.isascii()
            or b"\0" in data
            or hashlib.sha256(data).hexdigest() != expected
        ):
            raise ValueError("Packaged camera source digest mismatch: " + name)
        result[name] = data
    return result


def _bootstrap(transport):
    # Constructors only: no USB open, readiness probe or native task START.
    return USBBootstrap(transport, session=D1Session(transport, reuse_owned_id=True))


class _ControlGeneration:
    def __init__(
        self, camera, control, timeout, *, clock=time.monotonic, sleep=time.sleep
    ):
        self.camera, self.control, self.timeout = camera, control, timeout
        self.clock, self.sleep = clock, sleep
        self.generation = None
        self.amt_ready = False

    def _signature(self):
        transport = self.camera._transport
        if not transport.connected:
            raise CameraDisconnected("Caller-owned control transport is not connected")
        identity = transport._device_identity
        thread = transport._thread
        if (
            thread is None
            or type(identity) is not dict
            or any(
                type(identity.get(key)) is not int or not 0 <= identity[key] <= 255
                for key in ("bus", "address")
            )
        ):
            raise USBError("Opened camera reader/bus/address generation is unavailable")
        # Retain the actual Thread reference, rather than a recyclable id().
        return thread, identity["bus"], identity["address"]

    def _remaining(self, deadline):
        value = deadline - self.clock()
        if value <= 0:
            raise RequestTimeout("Read-only control readiness deadline")
        return value

    def ensure(self, *, deadline, resolve):
        """Called under Camera's shared control lock; never opens a handle."""
        current = self._signature()
        if (
            self.generation is None
            or current[0] is not self.generation[0]
            or current[1:] != self.generation[1:]
        ):
            # bind_bootstrap first records the old session quarantine. A new
            # adapter itself is not permission to start a diagnostic task.
            self.control.bind_bootstrap(_bootstrap(self.camera._transport))
            self.generation, self.amt_ready = current, False
        if not self.amt_ready:
            wait_amt_ready(
                self.camera._transport, timeout=min(60, self._remaining(deadline))
            )
            self._remaining(deadline)  # A late reply is not readiness.
            self.amt_ready = True
        if resolve:
            self.control.resolve_pending(timeout=min(5, self._remaining(deadline)))
            self._remaining(deadline)
        return self.control.bootstrap

    def reconnect(self, camera, *, deadline=None):
        """Only open + missing-task RESULT probes; no state/START/QS/mode SET."""
        if camera is not self.camera:
            raise ValueError("Reconnect callback belongs to another Camera")
        if deadline is None:
            deadline = self.clock() + min(20, self.timeout)
        elif type(deadline) not in (int, float) or not math.isfinite(deadline):
            raise ValueError('Read-only reconnect requires a finite absolute deadline')
        else:
            deadline = min(deadline, self.clock() + min(20, self.timeout))
        with camera._control_lock:
            while True:
                try:
                    self._remaining(deadline)
                    if not camera._transport.connected:
                        camera._transport.open()
                    return self.ensure(deadline=deadline, resolve=False)
                except USBError:
                    if self.clock() >= deadline:
                        raise
                    # Retain a healthy open handle across readiness timeouts;
                    # USBTransport.open releases only a failed/stale generation.
                    self.sleep(min(0.05, self._remaining(deadline)))

    def _retry_readonly(self, query, *, deadline):
        """Wait for ordinary GET readiness on exactly this open generation.

        A timed-out GET is safe to query again; it is not a possibly delivered
        SET/QS/diagnostic START. Never open, rotate a task session or adopt a new
        USB reader here. Pending diagnostic tasks remain separately guarded.
        """
        transport, expected = self.control.bootstrap.transport, self.generation
        if expected is None:
            raise USBError('Ordinary readiness requires a proven control generation')

        def same_generation():
            current = self._signature()
            if (self.camera._transport is not transport
                    or current[0] is not expected[0] or current[1:] != expected[1:]):
                raise CameraDisconnected('Control generation changed during ordinary readiness')

        while True:
            self._remaining(deadline)
            same_generation()
            try:
                value = query(min(2, self._remaining(deadline)))
            except (RequestTimeout, controls.CameraReplyError) as error:
                if isinstance(error, controls.CameraReplyError) and error.status != 0xE3:
                    raise
                same_generation()
                if self.clock() >= deadline:
                    raise
                self.sleep(min(0.05, self._remaining(deadline)))
                continue
            same_generation()
            self._remaining(deadline)
            return value

    def _activation(self, timeout):
        raw = self.camera._transport.request(
            0, 0x32, ACTIVATION_QUERY, receiver=0x88, timeout=timeout
        ).payload
        if not raw:
            raise controls.ControlDecodeError("Empty activation response")
        if raw[0]:
            raise controls.CameraReplyError(raw[0], raw)
        if len(raw) != 60 or raw[1] != 1:
            raise controls.ControlDecodeError(
                "Activated raw1/status0 exact60-byte response required"
            )
        return 1

    def idle(self, camera):
        if camera is not self.camera:
            raise ValueError("Idle observer belongs to another Camera")
        with camera._control_lock:
            deadline = self.clock() + self.timeout
            self.ensure(deadline=deadline, resolve=True)
            activation = self._retry_readonly(
                self._activation, deadline=min(deadline, self.clock() + 10)
            )

            def record0():
                state = self._retry_readonly(
                    lambda remaining: camera.state(timeout=remaining),
                    deadline=min(deadline, self.clock() + 10),
                )
                if state.recording_state != 0:
                    raise ValueError("USB stream setup requires idle Record0")
                return state

            # A not-yet-ready service may answer Record1 after a GET timeout.
            # Reject it before even the fixed read-only diagnostic task START.
            record0()
            # This is a fixed read-only test, not a power/setting operation.
            # _run also rechecks any uncertainty before its one task START.
            self.control._run(
                BATTERY_QUERY,
                timeout_ms=max(100, min(5000, int(self._remaining(deadline) * 1000))),
            )
            self._remaining(deadline)
            # Record0 LAST: long uploads/readiness must not stale the entry proof.
            state = record0()
            return _IdleEvidence(self.clock(), state.recording_state, activation, True)


def _capture_factories(on_access_unit):
    """Bind the raw admission callback to every initial/wake video producer."""
    if on_access_unit is None:
        return {}
    if not callable(on_access_unit):
        raise ValueError("on_access_unit must be callable or None")
    return dict(preview_factory=partial(USBPreview, on_access_unit=on_access_unit),
                video_factory=partial(VideoStream, on_access_unit=on_access_unit))


class _PreparedFactory:
    def __init__(self, source_provider, *, decode, queue_size, imu_options, on_access_unit=None):
        self.source_provider = source_provider
        self.capture_factories = _capture_factories(on_access_unit)
        self.decode, self.queue_size, self.imu_options = (
            decode,
            queue_size,
            dict(imu_options),
        )
        self.assets = None

    def preload_assets(self, camera, control, *, owner_receipt):
        if self.assets is None:
            self.assets = PreparedAssets(control, owner_receipt, self.source_provider)
        elif (
            self.assets.control is not control
            or self.assets.owner_receipt != owner_receipt
        ):
            raise ValueError("Prepared source cache belongs to another owner")
        self.assets.preload_assets()
        return dict(
            work=self.assets.work, staged=self.assets.staged, immutable_sources=True
        )

    def __call__(self, camera, control, *, owner_receipt, sleep_id, wake_id):
        return PreparedStream(
            camera,
            control,
            owner_receipt=owner_receipt,
            sleep_id=sleep_id,
            wake_id=wake_id,
            source_provider=self.source_provider,
            assets=self.assets,
            plan_builder=partial(build_prepared_fast_plan, fast=True),
            decode=self.decode,
            queue_size=self.queue_size,
            imu_options=self.imu_options,
            **self.capture_factories,
        )


class _RetainedFactory:
    def __init__(self, source_provider, *, decode, queue_size, imu_options, on_access_unit=None):
        self.source_provider = source_provider
        self.capture_factories = _capture_factories(on_access_unit)
        self.decode, self.queue_size, self.imu_options = (
            decode,
            queue_size,
            dict(imu_options),
        )
        self.assets = None

    def preload_assets(self, camera, control, *, owner_receipt):
        if self.assets is None:
            self.assets = RetainedAssets(control, owner_receipt, self.source_provider)
        elif (
            self.assets.control is not control
            or self.assets.owner_receipt != owner_receipt
        ):
            raise ValueError("Retained source cache belongs to another owner")
        self.assets.preload_assets()
        return dict(
            work=self.assets.work, staged=self.assets.staged, immutable_sources=True
        )

    def __call__(self, camera, control, *, owner_receipt, sleep_id, wake_id, preview):
        return RetainedStream(
            camera,
            control,
            owner_receipt=owner_receipt,
            sleep_id=sleep_id,
            wake_id=wake_id,
            preview=preview,
            assets=self.assets,
            decode=self.decode,
            queue_size=self.queue_size,
            imu_options=self.imu_options,
            **self.capture_factories,
        )


def build_stream_session(
    camera,
    *,
    decode=True,
    queue_size=1,
    imu_options=None,
    timeout=60,
    existing=None,
    bootstrap=None,
    fast_sleep=False,
    retain_ecm=False,
    on_access_unit=None,
):
    """Pure host assembly. Actual device work begins in session.start()."""
    if (
        type(decode) is not bool
        or type(queue_size) is not int
        or not 1 <= queue_size <= 256
    ):
        raise ValueError("decode must be bool; video queue_size must be1..256")
    if type(fast_sleep) is not bool:
        raise ValueError("fast_sleep must be a bool")
    if type(retain_ecm) is not bool or retain_ecm and not fast_sleep:
        raise ValueError("retain_ecm requires fast_sleep=True")
    if retain_ecm and sys.platform != "linux":
        raise ValueError(
            "retain_ecm currently supports Linux unmanaged USB networking only"
        )
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    if imu_options is not None and type(imu_options) is not dict:
        raise TypeError("imu_options must be a dictionary or None")
    _capture_factories(on_access_unit)  # Validate before opening any device.
    options = dict(read_buffer_bytes=65536, queue_size=16384)
    options.update(imu_options or {})
    for key in ("startup_timeout", "stop_timeout"):
        if key in options and (
            type(options[key]) not in (int, float)
            or not math.isfinite(options[key])
            or options[key] <= 0
        ):
            raise ValueError("IMU timeouts must be finite and positive")
    IMUStream(**options)  # Constructor is pure: validate unknown/invalid options now.
    if decode and importlib.util.find_spec("av") is None:
        raise ImportError(
            "Decoded video requires requirements-video.txt, or decode=False"
        )
    owner_sources, watcher_sources = _sources(OWNER_SOURCES), _sources(WATCHER_SOURCES)
    retained_sources = _sources(RETAINED_SOURCES) if retain_ecm else None
    if bootstrap is not None and (
        existing is None or bootstrap.transport is not camera._transport
    ):
        raise ValueError(
            "Existing owner requires its completed adapter on this transport"
        )
    control = StandbyControl(
        bootstrap if bootstrap is not None else _bootstrap(camera._transport),
        source_provider=lambda: dict(owner_sources),
        control_lock=camera._control_lock,
        adapter_factory=_bootstrap,
        existing=existing,
        source_stager=InstalledSources(),
    )
    manager = _ControlGeneration(camera, control, timeout)
    control.readonly_reconnect = partial(manager.reconnect, camera)
    if bootstrap is not None:
        # The resume path already completed RO AMT readiness/discovery on this
        # exact reader. Retain that task session instead of silently replacing it.
        manager.generation, manager.amt_ready = manager._signature(), True
    prepared = _PreparedFactory(
        lambda: dict(watcher_sources),
        decode=decode,
        queue_size=queue_size,
        imu_options=options,
        on_access_unit=on_access_unit,
    )
    retained = (
        _RetainedFactory(
            lambda: dict(retained_sources),
            decode=decode,
            queue_size=queue_size,
            imu_options=options,
            on_access_unit=on_access_unit,
        )
        if retain_ecm
        else None
    )
    return USBStreamSession(
        camera,
        control=control,
        prepared_factory=prepared,
        idle_probe=manager.idle,
        reconnect_control=manager.reconnect,
        preview_options=dict(decode=decode, queue_size=queue_size, timeout=timeout,
                             **({"on_access_unit": on_access_unit} if on_access_unit else {})),
        imu_options=options,
        fast_sleep=fast_sleep,
        retain_ecm=retain_ecm,
        retained_factory=retained,
    )
