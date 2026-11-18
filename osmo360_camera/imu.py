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
import platform
from pathlib import Path
import queue
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable

from ._imu_queue import RecentSamples
from ._imu_wire import BINARY_PREFIX, PACKET, TICK_HZ, unpack_sample

READY_LINE = "Sensor log mask enabled; capture started"
LIVE_PREFIX = 'OSMO_IMU_STATUS '
_LIVE_EVENTS = {'progress', 'mask_verified', 'first_ssc', 'first_pair',
                'fatal_read', 'cleanup_started', 'drain_verified', 'drain_failed',
                'collector_finished'}
_LIVE_PHASES = {'starting', 'capture', 'cleanup', 'finished'}
_LIVE_COUNTS = {'usb_reads', 'usb_bytes', 'usb_timeouts', 'ssc_packets',
                'accel_samples', 'gyro_samples', 'paired_samples',
                'sample_decode_errors', 'udp_sent', 'udp_drops', 'bad_crc', 'unpaired_pending'}
_LIVE_TIMES = {'mask_verified_ns', 'first_ssc_ns', 'first_pair_ns', 'fatal_read_ns'}
_UDP_RECEIVE_BYTES = 4 * 1024 * 1024
_DROP_COUNT = struct.Struct('=I')  # Linux ancillary unsigned 32-bit, native byte order.
_MAX_BATCH_SAMPLES = 64


def _linux_socket_option(name, generic_number):
    """Use native constants, or verified asm-generic numbers on known Linux ABIs.

    Linux include/uapi/asm-generic/socket.h defines RCVBUFFORCE=33/RXQ_OVFL=40.
    Other operating systems and Linux ABIs with different socket numbers must
    never receive these numeric fallbacks.
    """
    if sys.platform != 'linux':
        return None
    native = getattr(socket, name, None)
    if type(native) is int:
        return native
    if platform.machine().lower() in {
        'aarch64', 'arm64', 'armv6l', 'armv7l', 'x86_64', 'amd64', 'i386', 'i686',
    }:
        return generic_number
    return None


def _kernel_drop_report(ancillary, flags, option):
    """Read an observed RXQ_OVFL counter; absent control data is unknown, not zero.

    The uint32 value describes drops since this socket's creation as observed
    when the received datagram was queued. It can lag current losses and wraps
    at 32 bits. Invalid/truncated control data cannot replace a known counter.
    """
    if flags & socket.MSG_CTRUNC:
        return None, True
    found, value = False, None
    try:
        for level, kind, data in ancillary:
            if level != socket.SOL_SOCKET or kind != option:
                continue
            if found or len(data) != _DROP_COUNT.size:
                return None, True
            value, = _DROP_COUNT.unpack(data)
            found = True
    except (TypeError, ValueError, struct.error):
        return None, True
    return value, False


def _collector_status(line):
    """Accept only the bounded numeric schema, never collector raw output."""
    if not line.startswith(LIVE_PREFIX) or len(line) > 4096:
        return None
    try:
        value = json.loads(line[len(LIVE_PREFIX):])
    except (ValueError, TypeError, RecursionError):
        return None
    required = _LIVE_COUNTS | _LIVE_TIMES | {
        'schema', 'event', 'phase', 'monotonic_ns', 'last_read_status',
        'fatal_read_status', 'fatal_read_phase'}
    if (type(value) is not dict or not required <= value.keys()
            or value.keys() - required - {'restored', 'quiescent', 'error_count'}
            or type(value['schema']) is not int or value['schema'] != 1
            or type(value['event']) is not str or type(value['phase']) is not str
            or value['event'] not in _LIVE_EVENTS or value['phase'] not in _LIVE_PHASES):
        return None

    def uint(number):
        return type(number) is int and 0 <= number <= 0xffffffffffffffff

    if not uint(value['monotonic_ns']) or any(not uint(value[key]) for key in _LIVE_COUNTS):
        return None
    if any(value[key] is not None and not uint(value[key]) for key in _LIVE_TIMES):
        return None
    for key in ('last_read_status', 'fatal_read_status'):
        number = value[key]
        if number is not None and (type(number) is not int or not -0x80000000 <= number < 0x80000000):
            return None
    if value['fatal_read_phase'] is not None and (
            type(value['fatal_read_phase']) is not str or value['fatal_read_phase'] not in _LIVE_PHASES):
        return None
    if ('restored' in value and type(value['restored']) is not bool
            or 'quiescent' in value and type(value['quiescent']) is not bool
            or 'error_count' in value and not uint(value['error_count'])):
        return None
    return value


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
    Linux ``kernel_drops`` separately exposes an observed SO_RXQ_OVFL uint32
    counter; None means no kernel report is available, rather than zero loss.
    It can lag current losses and wraps at 32 bits. Other platforms leave it
    unavailable. Tick discontinuities and collector send failures remain
    separately visible. ``udp_socket`` reports the requested/effective receive
    buffer and any option errno; no global sysctl or privileges are changed.
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
    The owned collector batches up to64 unchanged private binary samples per
    datagram, flushing the partial batch at each USB read. Native ticks and the
    original USB receive time are retained on every sample; no resampling or
    timer-delayed batching occurs. Small queues cap the batch to their sample
    capacity. Kernel overflow counters remain counts of datagrams, not samples.
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
        self._batch_size = min(_MAX_BATCH_SAMPLES, queue_size)
        self.read_buffer_bytes = read_buffer_bytes
        self.callback = callback
        self.report_dir = Path(report_dir) if report_dir is not None else None
        self.startup_timeout, self.stop_timeout = startup_timeout, stop_timeout
        self.report_path: Path | None = None
        self._report = None
        self._process = None
        self._socket = None
        self._rxq_option = None
        self._socket_report = dict(
            requested_rcvbuf_bytes=_UDP_RECEIVE_BYTES, effective_rcvbuf_bytes=None,
            buffer_method=None, force_attempted=False, force_errno=None,
            buffer_errno=None, readback_errno=None, rxq_overflow=False,
            rxq_overflow_errno=None, kernel_drop_source=None, kernel_drop_counter_bits=None,
            private_batch_max_samples=self._batch_size, kernel_drop_unit=None,
        )
        self._queue = RecentSamples(queue_size)
        self._callbacks = RecentSamples(queue_size) if callback else None
        self._ready = threading.Event()
        self._receiver_done = threading.Event()
        self._stop_requested = False
        self._closed = False
        self._started = False
        self._receiver_error = None
        self._stop_error = None
        self._threads = []
        self._tail = deque(maxlen=20)
        self._collector_live = None
        self._collector_live_at = None
        self._collector_events = deque(maxlen=12)
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
            first_host_receive_monotonic_ns=None,
            last_host_receive_monotonic_ns=None,
            kernel_drops=None,
            kernel_drop_telemetry_errors=0,
            truncated_datagrams=0,
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
            result['udp_socket'] = dict(self._socket_report)
            result['collector_live'] = dict(self._collector_live) if self._collector_live else None
            result['collector_live_age_s'] = (max(0, time.monotonic() - self._collector_live_at)
                                               if self._collector_live_at is not None else None)
            result['collector_events'] = [dict(event) for event in self._collector_events]
            # stop() has already read this final host-local report. Keep its
            # separate mask/drain proof visible even when IMUStopError stops
            # the outer lifecycle before it can publish last_imu_stop.
            final = self._report if self._closed else None
            result['cleanup'] = ({key: deepcopy(final.get(key))
                                  for key in ('restored', 'quiescent', 'drain')}
                                 if isinstance(final, dict) else None)
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

    def _configure_udp_socket(self, sock):
        report = self._socket_report
        force = _linux_socket_option('SO_RCVBUFFORCE', 33)
        if force is not None:
            report['force_attempted'] = True
            try:
                # Same bounded 4 MiB request as before, beyond rmem_max only if
                # the process already has CAP_NET_ADMIN. Never obtain rights.
                sock.setsockopt(socket.SOL_SOCKET, force, _UDP_RECEIVE_BYTES)
                report['buffer_method'] = 'SO_RCVBUFFORCE'
            except OSError as error:
                report['force_errno'] = error.errno
        if report['buffer_method'] is None:
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _UDP_RECEIVE_BYTES)
                report['buffer_method'] = 'SO_RCVBUF'
            except OSError as error:
                report['buffer_errno'] = error.errno
                raise  # Preserve the existing required-option startup failure.
        try:
            # Linux includes its doubled bookkeeping allowance in this value.
            report['effective_rcvbuf_bytes'] = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        except OSError as error:
            report['readback_errno'] = error.errno
        overflow = _linux_socket_option('SO_RXQ_OVFL', 40)
        if (overflow is not None and callable(getattr(sock, 'recvmsg', None))
                and hasattr(socket, 'CMSG_SPACE') and hasattr(socket, 'MSG_CTRUNC')
                and hasattr(socket, 'MSG_TRUNC')):
            try:
                sock.setsockopt(socket.SOL_SOCKET, overflow, 1)
            except OSError as error:
                report['rxq_overflow_errno'] = error.errno
            else:
                self._rxq_option = overflow
                report.update(rxq_overflow=True, kernel_drop_source='SO_RXQ_OVFL',
                              kernel_drop_counter_bits=32, kernel_drop_unit='datagrams')

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
            self._configure_udp_socket(sock)
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
                "--udp-batch-size",
                str(self._batch_size),
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
                live = _collector_status(line)
                with self._lock:
                    self._tail.append(line[:1000])
                    if live is not None and (self._collector_live is None
                            or live['monotonic_ns'] >= self._collector_live['monotonic_ns']):
                        self._collector_live = live
                        self._collector_live_at = time.monotonic()
                        if live['event'] != 'progress':
                            self._collector_events.append(live)
                if line == READY_LINE:
                    self._ready.set()
        finally:
            self._process.stdout.close()
            self._process.wait()  # Reap even after a caller's stop timeout.

    def _put_recent(self, target, sample, drop_key):
        self._put_many_recent(target, (sample,), drop_key)

    def _put_many_recent(self, target, samples, drop_key):
        # Queue condition is released before the stats lock. stats() likewise
        # releases its lock before qsize(): neither path nests these locks.
        dropped = target.put_many_recent(samples)
        if dropped:
            with self._lock:
                self._stats[drop_key] += dropped

    def _samples_from_datagram(self, data):
        if isinstance(data, (bytes, bytearray)) and data.startswith(BINARY_PREFIX):
            if not len(data) or len(data) % PACKET.size or len(data) // PACKET.size > self._batch_size:
                raise ValueError('Invalid owned IMU batch length')
            # Complete validation precedes any sample/stat/queue admission.
            return tuple(IMUSample.from_datagram(data[offset:offset + PACKET.size])
                         for offset in range(0, len(data), PACKET.size))
        return (IMUSample.from_datagram(data),)  # Legacy JSON remains one sample.

    def _receive(self):
        overflow = self._rxq_option
        try:
            while True:
                drops, invalid_control, flags = None, False, 0
                try:
                    if overflow is None:
                        data = self._socket.recv(65536)
                    else:
                        data, ancillary, flags, _ = self._socket.recvmsg(
                            65536, socket.CMSG_SPACE(_DROP_COUNT.size)
                        )
                        drops, invalid_control = _kernel_drop_report(ancillary, flags, overflow)
                except socket.timeout:
                    if self._process.poll() is not None:
                        break
                    continue
                try:
                    if flags & getattr(socket, 'MSG_TRUNC', 0):
                        raise ValueError('Truncated IMU datagram')
                    samples = self._samples_from_datagram(data)
                except (ValueError, TypeError, KeyError, OverflowError):
                    with self._lock:
                        self._record_kernel_report(drops, invalid_control)
                        self._stats['truncated_datagrams'] += bool(flags & getattr(socket, 'MSG_TRUNC', 0))
                        self._stats["malformed_datagrams"] += 1
                    continue
                with self._lock:
                    self._record_kernel_report(drops, invalid_control)
                    last = self._stats["last_timestamp_ticks"]
                    largest = self._stats["max_native_interval_ticks"]
                    nonincreasing, gaps = 0, 0
                    if last is None:
                        self._stats["first_timestamp_ticks"] = samples[0].timestamp_ticks
                        self._stats['first_host_receive_monotonic_ns'] = samples[0].host_receive_monotonic_ns
                    for sample in samples:
                        if last is not None:
                            delta = sample.timestamp_ticks - last
                            nonincreasing += delta <= 0
                            gaps += delta > TICK_HZ * 0.005
                            if delta > 0 and (largest is None or delta > largest):
                                largest = delta
                        last = sample.timestamp_ticks
                    self._stats['nonincreasing_ticks'] += nonincreasing
                    self._stats['gaps_over_5ms'] += gaps
                    self._stats['max_native_interval_ticks'] = largest
                    self._stats["last_timestamp_ticks"] = sample.timestamp_ticks
                    self._stats['last_host_receive_monotonic_ns'] = sample.host_receive_monotonic_ns
                    self._stats["received"] += len(samples)
                self._put_many_recent(self._queue, samples, "queue_drops")
                if self._callbacks is not None:
                    self._put_many_recent(self._callbacks, samples, "callback_drops")
        except OSError as error:
            self._receiver_error = str(error)
            if self._process.poll() is None and not self._stop_requested:
                self._stop_requested = True
                self._process.send_signal(signal.SIGINT)
        finally:
            self._socket.close()
            self._receiver_done.set()

    def _record_kernel_report(self, drops, invalid_control):
        """Called under the existing receive-stat lock, including bad payloads."""
        if drops is not None:
            self._stats['kernel_drops'] = drops
        self._stats['kernel_drop_telemetry_errors'] += invalid_control

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
        """Restore the mask, drain this DIAG reader, then release its interface.

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
            or report.get("quiescent") is not True
            or report.get("errors")
        ):
            self._stop_error = self._failure(
                f"DIAG restoration/quiescence/clean exit not verified (returncode={code})"
            )
            raise IMUStopError(self._stop_error)
        return report

    close = stop

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False
