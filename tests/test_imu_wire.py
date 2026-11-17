"""Private binary IMU channel: lossless values and unchanged external JSON."""

import argparse
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from osmo360_camera import _diag_collector as diag_sensors
from osmo360_camera._imu_wire import PACKET, pack_sample, unpack_sample
from osmo360_camera.imu import IMUSample, TICK_HZ


def sample_record(**changes):
    record = dict(
        sensor_type="imu",
        timestamp_ticks=(1 << 64) - 1,
        tick_hz=TICK_HZ,
        host_receive_monotonic_ns=(1 << 64) - 2,
        ax=-0.0,
        ay=1.0000000000000002,
        az=-3.141592653589793,
        gx=1e-300,
        gy=-1e300,
        gz=0.125,
        accel_status=(1 << 32) - 1,
        gyro_status=3,
        accel_suid="00112233445566778899aabbccddeeff",
        gyro_suid="ffeeddccbbaa99887766554433221100",
        accel_unit="m/s^2",
        gyro_unit="rad/s",
    )
    record.update(changes)
    return record


class IMUWireTests(unittest.TestCase):
    def test_binary_roundtrip_retains_every_float_bit_and_raw_suid_byte(self):
        record = sample_record()
        packet = pack_sample(record)
        self.assertEqual(len(packet), 112)
        self.assertEqual(packet[:8], b"OSMOIMU\x01")
        self.assertEqual(
            packet[-32:], bytes.fromhex(record["accel_suid"] + record["gyro_suid"])
        )
        # This literal wire layout is independent of the helper's Struct object.
        expected = struct.pack(
            "<8sQQ6dII16s16s",
            b"OSMOIMU\x01",
            record["timestamp_ticks"],
            record["host_receive_monotonic_ns"],
            *(record[k] for k in ("ax", "ay", "az", "gx", "gy", "gz")),
            record["accel_status"],
            record["gyro_status"],
            bytes.fromhex(record["accel_suid"]),
            bytes.fromhex(record["gyro_suid"]),
        )
        self.assertEqual(packet, expected)
        sample = IMUSample.from_datagram(packet)
        self.assertEqual(sample.to_dict(), record)
        for name in ("ax", "ay", "az", "gx", "gy", "gz"):
            self.assertEqual(
                struct.pack("<d", getattr(sample, name)),
                struct.pack("<d", record[name]),
            )
        self.assertEqual(math.copysign(1, sample.ax), -1)

    def test_magic_version_length_and_nonfinite_payloads_are_rejected(self):
        packet = pack_sample(sample_record())
        for malformed in (
            b"",
            packet[:7],
            packet[:-1],
            packet + b"\0",
            b"X" + packet[1:],
            packet[:7] + b"\x02" + packet[8:],
        ):
            with self.subTest(malformed=malformed[:8]), self.assertRaises(ValueError):
                IMUSample.from_datagram(malformed)
        for axis in range(6):
            for value in (float("nan"), float("inf"), -float("inf")):
                fields = list(PACKET.unpack(packet))
                fields[3 + axis] = value
                with (
                    self.subTest(axis=axis, value=value),
                    self.assertRaises(ValueError),
                ):
                    IMUSample.from_datagram(PACKET.pack(*fields))
        with self.assertRaises(ValueError):
            unpack_sample(b"X" + packet[1:])

    def test_integer_boundaries_and_invalid_types_cannot_be_truncated(self):
        for name, limit in (
            ("timestamp_ticks", 1 << 64),
            ("host_receive_monotonic_ns", 1 << 64),
            ("accel_status", 1 << 32),
            ("gyro_status", 1 << 32),
        ):
            for value in (-1, limit, True, 1.0, "1", None):
                with (
                    self.subTest(name=name, value=value),
                    self.assertRaises(ValueError),
                ):
                    pack_sample(sample_record(**{name: value}))
            for value in (0, limit - 1):
                self.assertEqual(
                    unpack_sample(pack_sample(sample_record(**{name: value})))[name],
                    value,
                )
        for name, value in (
            ("sensor_type", "gyro"),
            ("tick_hz", 200),
            ("accel_unit", "g"),
            ("gyro_unit", "degrees/s"),
        ):
            with self.subTest(name=name), self.assertRaises(ValueError):
                pack_sample(sample_record(**{name: value}))

    def test_axis_and_suid_validation_is_strict(self):
        for value in (True, None, "1", float("nan"), float("inf"), 10**1000):
            with self.subTest(value=str(value)[:20]), self.assertRaises(ValueError):
                pack_sample(sample_record(ax=value))
        for value in ("", "00" * 15, "00" * 17, "x" * 32, " " * 32, None, 0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                pack_sample(sample_record(accel_suid=value))

    def test_binary_only_never_serializes_or_parses_json(self):
        record = sample_record()
        with patch.object(
            diag_sensors.json, "dumps", side_effect=AssertionError("No JSON encode")
        ):
            line, packet = diag_sensors._sample_outputs(record, udp_format="binary")
        self.assertIsNone(line)
        with patch(
            "osmo360_camera.imu.json.loads",
            side_effect=AssertionError("No JSON decode"),
        ):
            self.assertEqual(
                IMUSample.from_datagram(packet).timestamp_ticks,
                record["timestamp_ticks"],
            )

    def test_json_udp_and_sample_file_keep_the_existing_line_contract(self):
        record = sample_record()
        expected = json.dumps(record, separators=(",", ":")) + "\n"
        line, packet = diag_sensors._sample_outputs(record, udp_format="json")
        self.assertEqual((line, packet), (expected, expected.encode()))
        self.assertEqual(IMUSample.from_datagram(packet).to_dict(), record)
        self.assertEqual(
            IMUSample.from_datagram(expected).to_dict(), record
        )  # Prior JSON str also works.
        line, binary = diag_sensors._sample_outputs(
            record, write_json=True, udp_format="binary"
        )
        self.assertEqual(line, expected)
        self.assertEqual(IMUSample.from_datagram(binary).to_dict(), record)
        self.assertEqual(
            diag_sensors._sample_outputs(record, write_json=True), (expected, None)
        )
        # Standalone unpaired RAW JSON has not acquired binary's paired-field restrictions.
        unpaired = dict(sensor_type="gyro", x=1.0, y=2.0, z=3.0)
        self.assertEqual(
            diag_sensors._sample_outputs(unpaired, udp_format="json")[1],
            b'{"sensor_type":"gyro","x":1.0,"y":2.0,"z":3.0}\n',
        )

    def test_cli_default_is_still_json_and_invalid_binary_fails_before_usb(self):
        parsed = []
        original = argparse.ArgumentParser.parse_args

        def observe(parser, *args, **kwargs):
            result = original(parser, *args, **kwargs)
            parsed.append(result)
            return result

        with tempfile.TemporaryDirectory() as folder:
            prefix = str(Path(folder) / "capture")
            with (
                patch(
                    "sys.argv",
                    ["diag_sensors.py", "--prefix", prefix, "--udp-port", "5600"],
                ),
                patch.object(argparse.ArgumentParser, "parse_args", observe),
                patch.object(
                    diag_sensors.osmo_usb,
                    "load_usb",
                    side_effect=RuntimeError("No USB"),
                ),
            ):
                with self.assertRaisesRegex(RuntimeError, "No USB"):
                    diag_sensors.main()
            self.assertEqual(parsed[0].udp_format, "json")
            for args in (
                ["--udp-format", "binary", "--udp-port", "5600"],
                ["--udp-format", "binary", "--paired"],
            ):
                with (
                    patch("sys.argv", ["diag_sensors.py", "--prefix", prefix, *args]),
                    patch("sys.stderr", new_callable=io.StringIO),
                    patch.object(diag_sensors.osmo_usb, "load_usb") as usb,
                ):
                    with self.assertRaises(SystemExit) as error:
                        diag_sensors.main()
                    self.assertEqual(error.exception.code, 2)
                    usb.assert_not_called()


if __name__ == "__main__":
    unittest.main()
