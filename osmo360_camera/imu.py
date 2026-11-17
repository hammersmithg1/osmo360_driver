"""Live, unfused gyro/accel over the proven USB DIAG collector.

The camera's SSC XYZ values are already physical units: accel in m/s², gyro
in rad/s, including factory calibration. They are not quaternions or ADC
counts. Axes remain in the native SSC frame. Sensor ticks use a 19.2MHz clock;
their offset to video/host clocks and camera-body axes still need calibration.

This adapter owns a private collector subprocess.
It enables only the existing ssc-raw profile, pairs exact hardware timestamps,
and streams indefinitely without recording samples. A unique local JSON report
preserves the original DIAG mask and verifies restoration when stopping.

Enabling the DIAG log does not create a camera-side sensor subscription. Idle
streaming without recording has been verified after the firmware update;
start() itself confirms the mask, not receipt of samples in every camera state.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable

from ._imu_wire import BINARY_PREFIX, TICK_HZ, unpack_sample

READY_LINE = "Sensor log mask enabled; capture started"


class IMUStreamError(RuntimeError):
    """Collector/transport failure; inspect report_path and output_tail."""


class IMUStartError(IMUStreamError):
    pass


class IMUStopError(IMUStreamError):
    """Clean collector exit and DIAG-mask restoration could not be verified."""


@dataclass(frozen=True)
class IMUSample:
    timestamp_ticks: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    host_receive_monotonic_ns: int
    accel_status: int
    gyro_status: int
    accel_suid: str = ""
    gyro_suid: str = ""
    tick_hz: int = TICK_HZ

    @property
    def timestamp_s(self) -> float:
        """Sensor-clock seconds, not UTC or host monotonic time."""
        return self.timestamp_ticks / self.tick_hz

    @property
    def timestamp_ns(self) -> int:
        """Camera clock in nanoseconds; no host/UTC offset is applied."""
        return self.timestamp_ticks * 1_000_000_000 // self.tick_hz

    @property
    def accel(self) -> tuple[float, float, float]:
        """Native SSC XYZ in m/s²."""
        return self.ax, self.ay, self.az

    @property
    def gyro(self) -> tuple[float, float, float]:
        """Native SSC XYZ in rad/s."""
        return self.gx, self.gy, self.gz

    def to_dict(self) -> dict:
        return {
            **asdict(self),
            "sensor_type": "imu",
            "accel_unit": "m/s^2",
            "gyro_unit": "rad/s",
        }

    @classmethod
    def from_datagram(cls, data: bytes) -> IMUSample:
        if isinstance(data, (bytes, bytearray)) and data.startswith(BINARY_PREFIX):
            return cls(**unpack_sample(data))
        record = json.loads(data)
        if (
            not isinstance(record, dict)
            or record.get("sensor_type") != "imu"
            or record.get("tick_hz") != TICK_HZ
            or record.get("accel_unit") != "m/s^2"
            or record.get("gyro_unit") != "rad/s"
        ):
            raise ValueError("Expected paired RAW IMU in SI units at 19.2MHz ticks")
        numbers = {}
        for key in (
            "timestamp_ticks",
            "host_receive_monotonic_ns",
            "accel_status",
            "gyro_status",
        ):
            value = record.get(key)
            if type(value) is not int or value < 0:
                raise ValueError(f"Invalid {key}")
            numbers[key] = value
        for key in ("ax", "ay", "az", "gx", "gy", "gz"):
            value = record.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid {key}")
            numbers[key] = float(value)
        suids = {key: record.get(key, "") for key in ("accel_suid", "gyro_suid")}
        if not all(isinstance(value, str) for value in suids.values()):
            raise ValueError("Invalid sensor SUID")
        return cls(**numbers, **suids)


class IMUStream:
    """Context manager, iterator and optional callback for live USB samples.

    ``with IMUStream() as imu: sample = imu.read(timeout=2)``

    The sensor's native rate is about8kHz; this class does not reconfigure or
    resample it. Slow consumers discard the oldest queued sample to keep
    latency bounded. ``stats['queue_drops']`` counts iterator-queue losses;
    a callback has its own bounded queue and ``callback_drops`` counter.
    These counters cannot measure kernel UDP loss. Tick discontinuities and
    the source collector's UDP-drop count remain separately visible.
    ``max_native_interval_ticks`` measures the largest positive interval
    between received native timestamps, independent of host arrival timing.
    Its ``max_native_interval_ns`` conversion rounds down to whole nanoseconds;
    both are None until a positive interval has been received.
    Callbacks run on a daemon thread and should return promptly; stop does not
    wait indefinitely for application callback code that is blocked.

    A stream is single-use. Call stop/close or use a context manager. Stop
    sends SIGINT once and waits for the collector's mask-restoring finally.
    It never force-kills a collector that may still be restoring the camera.
    start() confirms the log mask, not receipt of the first sample.

    ``read_buffer_bytes`` controls only host USB reads: a multiple of 1024
    from 1024 through 65536. The 4096-byte default reduces host batching;
    it does not change the sensor sample rate or camera-side configuration.
    The owned collector uses a private lossless binary UDP format internally;
    standalone collector UDP and sample files retain their default JSON format.
    """

    def __init__(
        self,
        *,
        queue_size: int = 4096,
        read_buffer_bytes: int = 4096,
        callback: Callable[[IMUSample], None] | None = None,
        report_dir: str | Path | None = None,
        startup_timeout: float = 12,
        stop_timeout: float = 25,
    ):
        if type(queue_size) is not int or queue_size < 1:
            raise ValueError("queue_size must be a positive integer")
        if (
            type(read_buffer_bytes) is not int
            or not 1024 <= read_buffer_bytes <= 65536
            or read_buffer_bytes % 1024
        ):
            raise ValueError(
                "read_buffer_bytes must be a multiple of 1024 in 1024..65536"
            )
        if callback is not None and not callable(callback):
            raise TypeError("callback must be callable")
        if not all(math.isfinite(v) and v > 0 for v in (startup_timeout, stop_timeout)):
            raise ValueError("timeouts must be finite and positive")
        self.queue_size = queue_size
        self.read_buffer_bytes = read_buffer_bytes
        self.callback = callback
        self.report_dir = Path(report_dir) if report_dir is not None else None
        self.startup_timeout, self.stop_timeout = startup_timeout, stop_timeout
        self.report_path: Path | None = None
        self._report = None
        self._process = None
        self._socket = None
        self._queue = queue.Queue(maxsize=queue_size)
        self._callbacks = queue.Queue(maxsize=queue_size) if callback else None
        self._ready = threading.Event()
        self._receiver_done = threading.Event()
        self._stop_requested = False
        self._closed = False
        self._started = False
        self._receiver_error = None
        self._stop_error = None
        self._threads = []
        self._tail = deque(maxlen=20)
        self._lock = threading.Lock()
        self._stats = dict(
            received=0,
            queue_drops=0,
            malformed_datagrams=0,
            callback_drops=0,
            callback_errors=0,
            first_timestamp_ticks=None,
            last_timestamp_ticks=None,
            nonincreasing_ticks=0,
            gaps_over_5ms=0,
            max_native_interval_ticks=None,
        )

    @property
    def udp_port(self) -> int | None:
        return self._port if self._started else None

    @property
    def pid(self) -> int | None:
        return self._process.pid if self._process else None

    @property
    def report(self) -> dict | None:
        if self.report_path and self.report_path.exists():
            try:
                self._report = json.loads(self.report_path.read_text())
            except (OSError, ValueError):
                pass  # Source writes a snapshot during startup, then on exit.
        return deepcopy(self._report)

    @property
    def output_tail(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(self._tail)

    @property
    def stats(self) -> dict:
        with self._lock:
            result = dict(self._stats)
        first, last = result["first_timestamp_ticks"], result["last_timestamp_ticks"]
        result["observed_rate_hz"] = (
            (result["received"] - 1) * TICK_HZ / (last - first)
            if first is not None and last > first
            else None
        )
        interval = result["max_native_interval_ticks"]
        result["max_native_interval_ns"] = (
            interval * 1_000_000_000 // TICK_HZ if interval is not None else None
        )
        result["queued"] = self._queue.qsize()
        result["collector_returncode"] = self._process.poll() if self._process else None
        return result

    def start(self) -> IMUStream:
        if self._closed or self._stop_requested:
            raise IMUStreamError("An IMUStream is single-use; create a new instance")
        if self._started:
            return self
        if self.report_dir is not None:
            self.report_dir.mkdir(parents=True, exist_ok=True)
        run_dir = Path(tempfile.mkdtemp(prefix="osmo360-imu-", dir=self.report_dir))
        prefix = run_dir / "capture"
        self.report_path = prefix.with_suffix(".json")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.bind(("127.0.0.1", 0))
            sock.settimeout(0.1)
            self._port = sock.getsockname()[1]
            command = [
                sys.executable,
                "-u",
                "-m",
                "osmo360_camera._diag_collector",
                "--profile",
                "ssc-raw",
                "--paired",
                "--seconds",
                "0",
                "--prefix",
                str(prefix),
                "--udp-port",
                str(self._port),
                "--udp-format",
                "binary",
                "--read-buffer-bytes",
                str(self.read_buffer_bytes),
                "--no-packet-jsonl",
                "--no-raw",
            ]
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except BaseException:
            sock.close()
            self._closed = True
            raise
        self._socket, self._started = sock, True
        for name, target in [
            ("osmo-imu-output", self._read_output),
            ("osmo-imu-udp", self._receive),
        ]:
            thread = threading.Thread(name=name, target=target, daemon=True)
            self._threads.append(thread)
            thread.start()
        if self.callback:
            thread = threading.Thread(
                name="osmo-imu-callback", target=self._dispatch_callbacks, daemon=True
            )
            self._threads.append(thread)
            thread.start()
        deadline = time.monotonic() + self.startup_timeout
        try:
            while not self._ready.wait(0.02):
                if self._process.poll() is not None:
                    raise IMUStartError(
                        self._failure("IMU collector exited before startup")
                    )
                if time.monotonic() >= deadline:
                    raise IMUStartError(
                        self._failure("Timed out enabling the DIAG sensor mask")
                    )
            if self._process.poll() is not None:
                raise IMUStartError(
                    self._failure("IMU collector exited during startup")
                )
        except BaseException as error:
            try:
                self.stop()
            except IMUStopError as cleanup_error:
                if hasattr(error, "add_note"):
                    error.add_note(str(cleanup_error))
            raise
        return self

    def _failure(self, message):
        tail = "\n".join(self.output_tail[-5:])
        return f"{message}; report={self.report_path}" + (f"\n{tail}" if tail else "")

    def _read_output(self):
        try:
            for line in self._process.stdout:
                line = line.rstrip()
                with self._lock:
                    self._tail.append(line[:1000])
                if line == READY_LINE:
                    self._ready.set()
        finally:
            self._process.stdout.close()
            self._process.wait()  # Reap even after a caller's stop timeout.

    def _put_recent(self, target, sample, drop_key):
        try:
            target.put_nowait(sample)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass  # A consumer just made space; no sample was discarded.
            else:
                with self._lock:
                    self._stats[drop_key] += 1
            target.put_nowait(sample)  # This thread is the only producer.

    def _receive(self):
        try:
            while True:
                try:
                    data = self._socket.recv(65536)
                except socket.timeout:
                    if self._process.poll() is not None:
                        break
                    continue
                try:
                    sample = IMUSample.from_datagram(data)
                except (ValueError, TypeError, KeyError, OverflowError):
                    with self._lock:
                        self._stats["malformed_datagrams"] += 1
                    continue
                with self._lock:
                    last = self._stats["last_timestamp_ticks"]
                    if last is not None:
                        delta = sample.timestamp_ticks - last
                        self._stats["nonincreasing_ticks"] += delta <= 0
                        self._stats["gaps_over_5ms"] += delta > TICK_HZ * 0.005
                        largest = self._stats["max_native_interval_ticks"]
                        if delta > 0 and (largest is None or delta > largest):
                            self._stats["max_native_interval_ticks"] = delta
                    else:
                        self._stats["first_timestamp_ticks"] = sample.timestamp_ticks
                    self._stats["last_timestamp_ticks"] = sample.timestamp_ticks
                    self._stats["received"] += 1
                self._put_recent(self._queue, sample, "queue_drops")
                if self._callbacks is not None:
                    self._put_recent(self._callbacks, sample, "callback_drops")
        except OSError as error:
            self._receiver_error = str(error)
            if self._process.poll() is None and not self._stop_requested:
                self._stop_requested = True
                self._process.send_signal(signal.SIGINT)
        finally:
            self._socket.close()
            self._receiver_done.set()

    def _dispatch_callbacks(self):
        while not self._receiver_done.is_set() or not self._callbacks.empty():
            try:
                sample = self._callbacks.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.callback(sample)
            except Exception as error:
                with self._lock:
                    self._stats["callback_errors"] += 1
                    self._stats["last_callback_error"] = (
                        f"{type(error).__name__}: {error}"
                    )

    def read(self, timeout: float | None = None) -> IMUSample:
        """Read one sample; TimeoutError on timeout, StopIteration after clean stop."""
        if not self._started:
            raise IMUStreamError("Start the IMUStream before reading")
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            raise ValueError("timeout must be finite and nonnegative, or None")
        deadline = time.monotonic() + timeout if timeout is not None else math.inf
        while True:
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                pass
            if self._receiver_done.is_set():
                if self._receiver_error:
                    raise IMUStreamError(
                        self._failure(
                            f"IMU UDP receiver failed: {self._receiver_error}"
                        )
                    )
                code = self._process.poll()
                if code not in (None, 0):
                    raise IMUStreamError(
                        self._failure(f"IMU collector exited with code {code}")
                    )
                if not self._stop_requested:
                    raise IMUStreamError(
                        self._failure("IMU collector stopped unexpectedly")
                    )
                raise StopIteration
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("No IMU sample before timeout")
            try:
                return self._queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                pass

    def samples(self, timeout: float | None = None):
        """Iterate samples; an optional timeout applies to each sample."""
        while True:
            try:
                yield self.read(timeout=timeout)
            except StopIteration:
                return

    def __iter__(self):
        return self

    def __next__(self):
        return self.read()

    def stop(self, timeout: float | None = None) -> dict | None:
        """SIGINT the owned collector and verify its original mask was restored.

        On timeout the process is deliberately left alive to finish restoring;
        call stop again to wait longer. No SIGTERM/SIGKILL escalation occurs.
        """
        if self._closed:
            if self._stop_error:
                raise IMUStopError(self._stop_error)
            return self.report
        if not self._started:
            self._closed = True
            return None
        if timeout is None:
            timeout = self.stop_timeout
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("stop timeout must be finite and positive")
        if not self._stop_requested:
            self._stop_requested = True
            if self._process.poll() is None:
                try:
                    self._process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
        try:
            code = self._process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            raise IMUStopError(
                self._failure(
                    f"Collector PID {self.pid} is still restoring; left running, retry stop()"
                )
            ) from error
        # UDP worker drains already received datagrams before its next timeout.
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1)
        self._closed = True
        report = self.report
        if (
            code != 0
            or not report
            or report.get("restored") is not True
            or report.get("errors")
        ):
            self._stop_error = self._failure(
                f"DIAG restoration/clean exit not verified (returncode={code})"
            )
            raise IMUStopError(self._stop_error)
        return report

    close = stop

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False
