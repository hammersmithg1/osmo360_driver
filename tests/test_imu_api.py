"""Offline adapter tests: real localhost UDP, mocked collector process/USB."""

import json
import io
from pathlib import Path
import queue
import signal
import socket
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import _diag_collector as diag_sensors

from osmo360_camera.imu import (
    IMUSample,
    IMUStartError,
    IMUStopError,
    IMUStream,
    IMUStreamError,
    READY_LINE,
    TICK_HZ,
)
from osmo360_camera._imu_wire import pack_sample


def datagram(tick=19_200_000, **changes):
    record = dict(
        sensor_type="imu",
        timestamp_ticks=tick,
        tick_hz=TICK_HZ,
        ax=1,
        ay=2,
        az=9.81,
        gx=0.1,
        gy=0.2,
        gz=0.3,
        host_receive_monotonic_ns=123456789,
        accel_status=3,
        gyro_status=3,
        accel_suid="accel",
        gyro_suid="gyro",
        accel_unit="m/s^2",
        gyro_unit="rad/s",
    )
    record.update(changes)
    return json.dumps(record).encode()


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("Timed out waiting for adapter thread")
        time.sleep(0.005)


class FakeOutput:
    def __init__(self):
        self.lines = queue.Queue()

    def __iter__(self):
        while True:
            line = self.lines.get()
            if line is None:
                return
            yield line

    def close(self):
        pass


class FakeCollector:
    """Only the adapter's process contract, never launches diag_sensors/USB."""

    next_pid = 10000

    def __init__(self, command, *, ready=True, finish_on_signal=True, restored=True):
        self.command = command
        self.prefix = Path(command[command.index("--prefix") + 1])
        self.port = int(command[command.index("--udp-port") + 1])
        self.stdout = FakeOutput()
        self.signals = []
        self.done = threading.Event()
        self.finish_on_signal = finish_on_signal
        self.restored = restored
        self.returncode = None
        FakeCollector.next_pid += 1
        self.pid = FakeCollector.next_pid
        if ready:
            self.stdout.lines.put(READY_LINE + "\n")

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if not self.done.wait(timeout):
            raise subprocess.TimeoutExpired(self.command, timeout)
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)
        if self.finish_on_signal:
            self.finish()

    def finish(self, code=0, errors=None):
        report = dict(
            restored=self.restored,
            enabled_mask_verified=True,
            errors=errors or [],
            udp_drops=0,
        )
        self.prefix.with_suffix(".json").write_text(json.dumps(report))
        self.returncode = code
        self.done.set()
        self.stdout.lines.put(None)


class IMUNativeIntervalTests(unittest.TestCase):
    """Replay actual datagrams through the receiver without USB or socket binds."""

    def replay(self, values):
        class ReplaySocket:
            def __init__(self):
                self.values = iter(values)

            def recv(self, size):
                try:
                    return next(self.values)
                except StopIteration:
                    raise socket.timeout from None

            def close(self):
                pass

        stream = IMUStream()
        stream._socket = ReplaySocket()
        stream._process = SimpleNamespace(poll=lambda: 0)
        stream._receive()
        return stream

    def test_host_jitter_and_batched_arrival_do_not_become_native_gaps(self):
        stream = self.replay(
            [
                datagram(TICK_HZ, host_receive_monotonic_ns=1),
                datagram(TICK_HZ + 2400, host_receive_monotonic_ns=5_000_000_001),
                datagram(TICK_HZ + 4800, host_receive_monotonic_ns=5_000_000_001),
                datagram(TICK_HZ + 7200, host_receive_monotonic_ns=5_000_000_002),
            ]
        )
        self.assertEqual(stream.stats["max_native_interval_ticks"], 2400)
        self.assertEqual(stream.stats["max_native_interval_ns"], 125_000)
        self.assertEqual(stream.stats["gaps_over_5ms"], 0)
        self.assertEqual(stream.stats["received"], 4)

        # A real native gap remains visible even when all packets arrive together.
        stream = self.replay(
            [
                datagram(tick, host_receive_monotonic_ns=99)
                for tick in (0, 2400, 194_401, 196_801)
            ]
        )
        self.assertEqual(stream.stats["max_native_interval_ticks"], 192_001)
        self.assertEqual(stream.stats["max_native_interval_ns"], 10_000_052)
        self.assertEqual(stream.stats["gaps_over_5ms"], 1)

    def test_no_positive_interval_has_no_maximum(self):
        for ticks in ((), (1000,), (1000, 1000, 999)):
            with self.subTest(ticks=ticks):
                stream = self.replay([datagram(tick) for tick in ticks])
                self.assertIsNone(stream.stats["max_native_interval_ticks"])
                self.assertIsNone(stream.stats["max_native_interval_ns"])
                self.assertEqual(
                    stream.stats["nonincreasing_ticks"], max(0, len(ticks) - 1)
                )

    def test_nonincreasing_ticks_are_counted_and_do_not_replace_positive_maximum(self):
        stream = self.replay(
            [datagram(tick) for tick in (10_000, 10_000, 9000, 9005, 9005, 8995, 8997)]
        )
        self.assertEqual(stream.stats["max_native_interval_ticks"], 5)
        self.assertEqual(stream.stats["max_native_interval_ns"], 260)
        self.assertEqual(stream.stats["nonincreasing_ticks"], 4)
        self.assertEqual(stream.stats["received"], 7)


class IMUAdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.collectors = []
        self.fake_options = {}

        def factory(command, **kwargs):
            self.assertTrue(kwargs["start_new_session"])
            self.assertEqual(kwargs["stderr"], subprocess.STDOUT)
            self.assertNotIn("cwd", kwargs)
            collector = FakeCollector(command, **self.fake_options)
            self.collectors.append(collector)
            return collector

        self.patcher = patch("osmo360_camera.imu.subprocess.Popen", side_effect=factory)
        self.patcher.start()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        for collector in self.collectors:
            if collector.poll() is None:
                collector.finish()
        self.sender.close()
        self.patcher.stop()
        self.temp.cleanup()

    def stream(self, **kwargs):
        return IMUStream(report_dir=self.temp.name, **kwargs)

    def send(self, stream, value):
        self.sender.sendto(value, ("127.0.0.1", stream.udp_port))

    def test_default_reports_use_private_temporary_directory_outside_package(self):
        with patch("tempfile.tempdir", self.temp.name), IMUStream() as stream:
            self.assertIsNone(stream.report_dir)
            self.assertEqual(stream.report_path.parent.parent, Path(self.temp.name))
            self.assertTrue(stream.report_path.parent.name.startswith("osmo360-imu-"))

    def test_paired_si_and_clock_preserved_and_sigint_restores(self):
        with self.stream() as stream:
            command = self.collectors[-1].command
            self.assertEqual(
                command[1:4], ["-u", "-m", "osmo360_camera._diag_collector"]
            )
            self.assertEqual(command[command.index("--seconds") + 1], "0")
            self.assertEqual(command[command.index("--profile") + 1], "ssc-raw")
            self.assertEqual(command[command.index("--read-buffer-bytes") + 1], "4096")
            self.assertEqual(command[command.index("--udp-format") + 1], "binary")
            self.assertIn("--paired", command)
            self.assertIn("--no-raw", command)
            self.assertIn("--no-packet-jsonl", command)
            self.assertNotIn("--samples-jsonl", command)
            self.send(stream, datagram())
            sample = next(stream)
            self.assertIsInstance(sample, IMUSample)
            self.assertEqual(sample.timestamp_ticks, TICK_HZ)
            self.assertEqual(sample.timestamp_s, 1)
            self.assertEqual(sample.accel, (1, 2, 9.81))
            self.assertEqual(sample.gyro, (0.1, 0.2, 0.3))
            self.assertEqual(sample.host_receive_monotonic_ns, 123456789)
            self.assertEqual(sample.to_dict()["gyro_unit"], "rad/s")
        self.assertEqual(self.collectors[-1].signals, [signal.SIGINT])
        self.assertTrue(stream.report["restored"])
        self.assertEqual(stream.stop(), stream.report)

    def test_host_read_buffer_boundaries_and_invalid_values(self):
        for size in (1024, 65536):
            with self.subTest(size=size), self.stream(read_buffer_bytes=size):
                command = self.collectors[-1].command
                self.assertEqual(
                    command[command.index("--read-buffer-bytes") + 1], str(size)
                )
        for size in (True, 4096.0, "4096", 0, 1023, 1025, 65537):
            with self.subTest(size=size), self.assertRaises(ValueError):
                self.stream(read_buffer_bytes=size)

    def test_collector_rejects_invalid_read_buffer_before_usb(self):
        with patch.object(diag_sensors.osmo_usb, "load_usb") as load_usb:
            for size in ("0", "1023", "1025", "65537", "bad"):
                with (
                    self.subTest(size=size),
                    patch(
                        "sys.argv",
                        [
                            "diag_sensors.py",
                            "--prefix",
                            "unused",
                            "--read-buffer-bytes",
                            size,
                        ],
                    ),
                    patch("sys.stderr", new_callable=io.StringIO),
                    self.assertRaises(SystemExit) as error,
                ):
                    diag_sensors.main()
                self.assertEqual(error.exception.code, 2)
            load_usb.assert_not_called()

    def test_bounded_queue_keeps_latest_and_counts_actual_evictions(self):
        with self.stream(queue_size=2) as stream:
            for i in range(8):
                self.send(stream, datagram(TICK_HZ + i * 2400))
            wait_for(lambda: stream.stats["queue_drops"] == 6)
            self.assertEqual(
                [stream.read(0.2).timestamp_ticks for _ in range(2)],
                [TICK_HZ + 6 * 2400, TICK_HZ + 7 * 2400],
            )
            self.assertAlmostEqual(stream.stats["observed_rate_hz"], 8000)
            with self.assertRaises(TimeoutError):
                stream.read(0.01)

    def test_bad_units_nonfinite_and_unpaired_data_are_rejected(self):
        with self.stream() as stream:
            for value in (
                b"{bad",
                datagram(gyro_unit="degrees/s"),
                datagram(gx=float("nan")),
                datagram(sensor_type="gyro"),
            ):
                self.send(stream, value)
            self.send(stream, datagram())
            self.assertEqual(stream.read(1).timestamp_ticks, TICK_HZ)
            self.assertEqual(stream.stats["malformed_datagrams"], 4)
            self.assertEqual(stream.stats["received"], 1)

    def test_binary_receiver_preserves_samples_and_counts_malformed_packets(self):
        record = json.loads(datagram())
        record.update(
            accel_suid="00112233445566778899aabbccddeeff",
            gyro_suid="ffeeddccbbaa99887766554433221100",
        )
        packet = pack_sample(record)
        with self.stream() as stream:
            self.send(stream, packet[:-1])
            self.send(stream, packet[:7] + b"\x02" + packet[8:])
            self.send(stream, packet)
            sample = stream.read(1)
            self.assertEqual(sample.to_dict(), record)
            self.assertEqual(stream.stats["malformed_datagrams"], 2)
            self.assertEqual(stream.stats["received"], 1)

    def test_random_ports_and_unique_reports(self):
        with self.stream() as first, self.stream() as second:
            self.assertNotEqual(first.udp_port, second.udp_port)
            self.assertNotEqual(first.report_path, second.report_path)
            self.send(first, datagram())
            self.send(second, datagram(TICK_HZ + 1))
            self.assertNotEqual(
                first.read(1).timestamp_ticks, second.read(1).timestamp_ticks
            )

    def test_callback_queue_does_not_block_udp_receiver(self):
        entered, release = threading.Event(), threading.Event()
        ticks = []

        def callback(sample):
            ticks.append(sample.timestamp_ticks)
            if len(ticks) == 1:
                entered.set()
                release.wait(2)
            else:
                raise ValueError("callback failure is reported")

        try:
            with self.stream(queue_size=2, callback=callback) as stream:
                self.send(stream, datagram())
                self.assertTrue(entered.wait(1))
                for i in range(1, 8):
                    self.send(stream, datagram(TICK_HZ + i * 2400))
                wait_for(lambda: stream.stats["received"] == 8)
                wait_for(lambda: stream.stats["callback_drops"] == 5)
                release.set()
                wait_for(lambda: stream.stats["callback_errors"] == 2)
                self.assertEqual(
                    ticks, [TICK_HZ, TICK_HZ + 6 * 2400, TICK_HZ + 7 * 2400]
                )
        finally:
            release.set()

    def test_stop_timeout_never_force_kills_or_interrupts_restore_twice(self):
        self.fake_options["finish_on_signal"] = False
        stream = self.stream().start()
        with self.assertRaises(IMUStopError):
            stream.stop(timeout=0.01)
        collector = self.collectors[-1]
        self.assertIsNone(collector.poll())
        self.assertEqual(collector.signals, [signal.SIGINT])
        collector.finish()
        self.assertTrue(stream.stop()["restored"])
        self.assertEqual(collector.signals, [signal.SIGINT])

    def test_startup_timeout_still_stops_owned_collector(self):
        self.fake_options["ready"] = False
        stream = self.stream(startup_timeout=0.03)
        with self.assertRaises(IMUStartError):
            stream.start()
        self.assertEqual(self.collectors[-1].signals, [signal.SIGINT])
        self.assertTrue(stream.report["restored"])

    def test_unrestored_mask_is_a_stop_error(self):
        self.fake_options["restored"] = False
        stream = self.stream().start()
        with self.assertRaises(IMUStopError):
            stream.stop()
        self.assertFalse(stream.report["restored"])
        with self.assertRaises(IMUStopError):
            stream.stop()  # Idempotence must not turn a failed restore into success.

    def test_unexpected_collector_exit_is_not_a_clean_iterator_end(self):
        stream = self.stream().start()
        self.collectors[-1].finish(code=1, errors=["USB disconnected"])
        with self.assertRaises(IMUStreamError):
            stream.read(timeout=1)
        with self.assertRaises(IMUStopError):
            stream.stop()

    def test_iterator_ends_after_clean_stop(self):
        stream = self.stream().start()
        stream.stop()
        self.assertEqual(list(stream.samples()), [])
        with self.assertRaises(IMUStreamError):
            stream.start()


if __name__ == "__main__":
    unittest.main()
