"""Owned USB live preview: RAM bootstrap, ECM lease, decoded frames, rollback."""

from contextlib import contextmanager
from dataclasses import asdict
import importlib.util
import math
import time

from ._amt import D1Session, wait_ready as wait_amt_ready
from ._preview_lease import PreviewLease
from ._usb_bootstrap import USBBootstrap
from ._usb_plan import build_usb_plan
from . import _host_network
from .imu import IMUStopError
from .video import VideoStream


class USBPreviewError(RuntimeError):
    def __init__(self, message, report):
        self.report = report
        super().__init__(message)


class USBPreview:
    """Use through ``with cam.usb_preview() as video: frame = video.read(5)``.

    Setup temporarily re-enumerates USB at high speed and adds native ECM.
    Camera storage is used only for one small, verified runtime report, which
    is deleted. Video/IMU samples are never recorded by this class.
    Stop restores the original USB functions/speed and verifies the result.
    The camera-side lease also restores USB if the host application disappears.
    """

    def __init__(
        self, camera, *, decode=True, queue_size=1, timeout=60, network_guard=None
    ):
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("timeout must be finite and positive")
        if (
            type(decode) is not bool
            or type(queue_size) is not int
            or not 1 <= queue_size <= 256
        ):
            raise ValueError("decode must be bool; queue_size must be1..256")
        self.camera, self.decode, self.queue_size = camera, decode, queue_size
        self.timeout = timeout
        self.network = (
            network_guard
            if network_guard is not None
            else _host_network.preview_guard()
        )
        self._stream = self._lease = self._runtime = self._plan = self._baseline = None
        self._started = self._closed = self._scheduled = False
        self.report = dict(restored=False, errors=[])

    @property
    def stats(self):
        return dict(
            video=self._stream.stats if self._stream else None,
            lease=dict(self._lease.stats) if self._lease else None,
            healthy=self._lease.healthy if self._lease else False,
            restored=self.report["restored"],
        )

    @property
    def calibration(self):
        """Raw factory calibration metadata; no assumed OpenCV/IMU axes model."""
        return self._stream.calibration if self._stream else None

    def _stop_owned_imu(self, *, allow_restored_fault=False):
        imu = self.camera._imu
        if imu is not None:
            try:
                report = imu.stop()
            except IMUStopError as error:
                if not allow_restored_fault:
                    raise
                # An exited collector may have restored its original mask on
                # a replacement USB handle while still reporting the stream
                # failure. Mask restoration alone is not a quiet endpoint:
                # verified drain and collector exit must precede lease CLOSE.
                try:
                    report = imu.report
                    returncode = imu.stats.get("collector_returncode")
                except Exception as proof_error:
                    self.report["imu_stop_proof_error"] = str(proof_error)
                    raise error from proof_error
                self.report["imu_stop"] = report
                self.report["imu_collector_returncode"] = returncode
                if (
                    type(returncode) is not int
                    or not isinstance(report, dict)
                    or report.get("restored") is not True
                    or report.get("quiescent") is not True
                ):
                    raise
                self.report["imu_stop_error"] = type(error).__name__ + ": " + str(error)
                self.report["errors"].append("IMU: " + self.report["imu_stop_error"])
                self.camera._imu = None
                return error
            self.report["imu_stop"] = report
            if (report is None and imu.pid is not None) or (
                report is not None
                and (
                    report.get("restored") is not True
                    or report.get("quiescent") is not True
                    or report.get("errors")
                )
            ):
                raise USBPreviewError(
                    "Owned IMU mask restoration and DIAG quiescence were not verified",
                    self.report,
                )
            self.camera._imu = None

    def _cleanup_host_network(self, primary_error=None):
        # Custom passive guards retain the original three-method protocol.
        # Packaged Linux owners implement this additional explicit capability.
        cleanup = getattr(self.network, "cleanup_owned", None)
        if cleanup is None:
            return
        try:
            self.report["host_network_cleanup"] = cleanup()
        except BaseException as error:
            self.report["host_network_cleanup_error"] = (
                type(error).__name__ + ": " + str(error)
            )
            if primary_error is None:
                raise
            if hasattr(primary_error, "add_note"):
                primary_error.add_note("Owned USB host network cleanup: " + str(error))

    def start(self):
        if self._closed:
            raise USBPreviewError("USBPreview is single-use", self.report)
        if self._started:
            return self
        if self.decode and importlib.util.find_spec("av") is None:
            raise USBPreviewError(
                "Decoded video requires requirements-video.txt; or use decode=False",
                self.report,
            )
        self._started = True
        started = time.monotonic()
        deadline = started + self.timeout
        self.report["phase_seconds"] = {}

        @contextmanager
        def phase(name):
            began = time.monotonic()
            try:
                yield
            finally:
                self.report["phase_seconds"][name] = time.monotonic() - began

        def remaining():
            value = deadline - time.monotonic()
            if value <= 0:
                raise USBPreviewError("USB preview setup deadline", self.report)
            return value

        with self.camera._control_lock:
            try:
                with phase("preflight"):
                    self.camera.connect()
                    before = self.camera.state()
                    self.report["state_before"] = dict(
                        asdict(before), recording_state=before.recording_state
                    )
                    if before.recording_state != 0:
                        raise USBPreviewError(
                            "USB preview setup requires idle Record0", self.report
                        )
                    self.report["charging_only"] = (
                        self.camera._transport.select_charging_only()
                    )
                    self._baseline = self.network.preflight()
                with phase("amt_ready"):
                    self.report["diagnostic_ready"] = wait_amt_ready(
                        self.camera._transport, timeout=min(20, remaining())
                    )
                with phase("runtime_query"):
                    session = D1Session(self.camera._transport, reuse_owned_id=True)
                    bootstrap = USBBootstrap(self.camera._transport, session=session)
                    self._runtime = bootstrap.query_runtime()
                    self.report["runtime"] = asdict(self._runtime)
                    self.report["runtime_query"] = bootstrap.last_report
                with phase("plan_upload"):
                    self._plan = build_usb_plan(self._runtime)
                    self.report["work_dir"] = self._plan["work_dir"]
                    self.report["script_sha256"] = self._plan["sha256"]
                    for command in self._plan["commands"]:
                        remaining()
                        bootstrap.run_checked(command)
                with phase("apply"):
                    self._stop_owned_imu()
                    if self.camera.state().recording_state != 0:
                        raise USBPreviewError(
                            "Camera left Record0 during setup", self.report
                        )
                    remaining()
                    # One launch only. A lost response cannot justify a retry.
                    self._scheduled = True
                    bootstrap.run_checked(self._plan["apply_command"])
                    self.camera._transport.close()
                with phase("network"):
                    guard = self.network.wait_ready(
                        self._baseline, self._runtime.host_addr, timeout=remaining()
                    )
                    self.report["network"] = guard
                with phase("lease"):
                    self._lease = PreviewLease(
                        dict(self._plan["lease"], interface=guard["ncm_interface"])
                    )
                    self._lease.start(timeout=min(12, remaining()))
                with phase("reconnect"):
                    self._reconnect(remaining())
                with phase("first_frame"):
                    self._stream = VideoStream(
                        decode=self.decode,
                        queue_size=self.queue_size,
                        host=self._plan["lease"]["camera_ip"],
                        local_address=self._plan["lease"]["host_ip"],
                        interface=guard["ncm_interface"],
                    )
                    self._stream.start()
                    while self._stream.stats["published_frames"] == 0:
                        remaining()
                        if self._stream.stats["error"] or self._stream.stats["stopped"]:
                            raise USBPreviewError(
                                "Preview failed: " + str(self._stream.stats["error"]),
                                self.report,
                            )
                        if not self._lease.healthy:
                            raise USBPreviewError(
                                "Preview lease became unhealthy", self.report
                            )
                        time.sleep(0.01)
                self.report["setup_seconds"] = time.monotonic() - started
                return self
            except BaseException as error:
                self.report["errors"].append(f"Setup: {type(error).__name__}: {error}")
                try:
                    self.stop()
                except Exception as cleanup:
                    self.report["errors"].append("Setup cleanup: " + repr(cleanup))
                    if hasattr(error, "add_note"):
                        error.add_note("USB preview cleanup: " + str(cleanup))
                raise

    def _reconnect(self, timeout, *, budget=None):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            if budget is not None:
                budget()
            try:
                self.camera._transport.open()
                state = self.camera.state(
                    timeout=min(2, max(0.01, deadline - time.monotonic()))
                )
            except Exception as error:
                last = error
                self.camera._transport.close()
                time.sleep(0.1)
            else:
                if budget is not None:
                    budget()
                return state
        raise USBPreviewError(
            "Camera control did not reconnect: " + str(last), self.report
        )

    def read(self, timeout=None):
        if self._stream is None:
            raise USBPreviewError(
                "Start USB preview before reading frames", self.report
            )
        if not self._lease.healthy and not self._closed:
            raise USBPreviewError("USB preview heartbeat is unhealthy", self.report)
        return self._stream.read(timeout)

    def stop(self, *, budget=None):
        if budget is not None and not callable(budget):
            raise TypeError("Optional stop budget must be callable")

        def check():
            if budget is not None:
                budget()

        def reconnect(timeout):
            return (
                self._reconnect(timeout, budget=budget)
                if budget is not None
                else self._reconnect(timeout)
            )

        check()
        if self._closed:
            return self.report
        with self.camera._control_lock:
            check()
            errors = []
            imu_error = None
            if self._stream:
                try:
                    self._stream.stop()
                except Exception as error:
                    errors.append("Video: " + str(error))
                finally:
                    # Failed metadata validation makes stop() raise. Preserve
                    # its diagnostic snapshot while still restoring native USB.
                    try:
                        self.report["video"] = dict(self._stream.stats)
                    except Exception as error:
                        errors.append("Video statistics: " + str(error))
            # Keep heartbeats until mask restoration and endpoint drain are proven.
            if self._scheduled:
                imu_error = self._stop_owned_imu(allow_restored_fault=True)
                check()  # Keep heartbeat/IP if no budget remains before close.
                # A pre-CLOSE deadline is not a failed CLOSE attempt. Keep it
                # outside restoration cleanup so the live lease and IP remain.
                close_sent = False
                if self._lease:
                    check()
                try:
                    if self._lease:
                        try:
                            self.report["lease_close"] = self._lease.stop()
                            close_sent = self.report["lease_close"] is not None
                        except Exception as error:
                            self.report["lease_close_error"] = str(error)
                    if not close_sent:
                        # A failed setup may have no usable IP link. The same
                        # control interface can schedule our reviewed rollback.
                        try:
                            check()
                            reconnect(5)
                            check()
                            session = D1Session(
                                self.camera._transport, reuse_owned_id=True
                            )
                            session.run(self._plan["rollback_command"])
                        except Exception as error:
                            self.report["rollback_schedule_error"] = str(error)
                    restoration_error = None
                    try:
                        try:
                            if self.camera._transport.connected:
                                check()
                                self.report["disconnect_observed"] = (
                                    self.camera._transport.wait_disconnected(15)
                                )
                        finally:
                            self.camera._transport.close()
                        check()
                        state = reconnect(45)
                        session = D1Session(self.camera._transport, reuse_owned_id=True)
                        bootstrap = USBBootstrap(
                            self.camera._transport, session=session
                        )
                        restored = (
                            bootstrap.query_runtime(budget=budget)
                            if budget is not None
                            else bootstrap.query_runtime()
                        )
                        check()
                        if (
                            restored.udc != self._runtime.udc
                            or restored.original_links != self._runtime.original_links
                            or restored.original_max_speed
                            != self._runtime.original_max_speed
                            or (restored.dev_addr, restored.host_addr)
                            != (self._runtime.dev_addr, self._runtime.host_addr)
                        ):
                            raise USBPreviewError(
                                "Original USB runtime was not restored", self.report
                            )
                    except BaseException as error:
                        restoration_error = error
                        raise
                    finally:
                        # IMU restoration and lease/rollback attempts precede this
                        # scope. Never drop IP while an owned IMU still needs it.
                        self._cleanup_host_network(restoration_error)
                    self.report["network_restored"] = self.network.check_restored(
                        self._baseline
                    )
                    self.report["state_after"] = dict(
                        asdict(state), recording_state=state.recording_state
                    )
                    check()
                    self.report["restored"] = True
                except Exception as error:
                    if imu_error is None:
                        raise
                    detail = (
                        "USB preview cleanup: "
                        + type(error).__name__
                        + ": "
                        + str(error)
                    )
                    self.report["errors"].append(detail)
                    if hasattr(imu_error, "add_note"):
                        imu_error.add_note(detail)
                    raise imu_error from error
            self._closed = True
            if getattr(self.camera, "_usb_preview", None) is self:
                self.camera._usb_preview = None
            if errors:
                self.report["errors"].extend(errors)
            if imu_error is not None:
                if errors and hasattr(imu_error, "add_note"):
                    imu_error.add_note("USB preview cleanup: " + "; ".join(errors))
                raise imu_error
            if errors:
                raise USBPreviewError("; ".join(errors), self.report)
            return self.report

    close = stop

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        except Exception as error:
            if exc is None:
                raise
            if hasattr(exc, "add_note"):
                exc.add_note("USB preview shutdown: " + str(error))
        return False

    def __iter__(self):
        return self

    def __next__(self):
        return self.read()
