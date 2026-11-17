"""Exercise the production collector's SSC decoder without USB or recordings."""

import math
import struct
import unittest

from osmo360_camera import _diag_collector as collector


def varint(value):
    data = bytearray()
    while value >= 128:
        data.append((value & 127) | 128)
        value >>= 7
    return bytes(data) + bytes([value])


def field(tag, wire, value):
    prefix = varint((tag << 3) | wire)
    if wire == 0:
        return prefix + varint(value)
    return prefix + (varint(len(value)) if wire == 2 else b"") + value


def fixed(tag, value, size=8):
    return field(tag, 1 if size == 8 else 5, value.to_bytes(size, "little"))


def packet(sensor, samples, *, code=0x19CB, kind="raw"):
    uid = fixed(1, 0x1122334455667788) + fixed(2, 0x99AABBCCDDEEFF00)
    message = (
        fixed(2, 12345) + field(3, 2, uid) + field(4, 2, sensor.encode()) + fixed(5, 42)
    )
    if kind == "raw":
        batch = b"".join(
            field(
                1,
                2,
                field(1, 0, 1)
                + fixed(2, tick)
                + field(3, 2, struct.pack("<3f", *xyz))
                + field(4, 0, 3),
            )
            for tick, xyz in samples
        )
        message += field(103, 2, batch)
    else:
        tick, xyz = samples[0]
        payload = field(1, 2, struct.pack("<3f", *xyz)) + field(2, 0, 3)
        event = fixed(1, 1025, 4) + fixed(2, tick)
        if kind == "api":
            message += field(101, 2, event + field(100, 2, field(1, 2, payload)))
        else:
            message += field(
                102, 2, field(101, 2, field(2, 2, event + field(3, 2, payload)))
            )
    header = bytearray(16)
    header[0] = 0x10
    struct.pack_into("<HH", header, 4, 12 + len(message), code)
    return bytes(header) + message


class SSCDecoderTests(unittest.TestCase):
    def test_collector_preserves_native_tick_precision_axes_status_and_sensor_identity(
        self,
    ):
        tick = (1 << 53) + 1
        for sensor, xyz in (
            ("accel", (1.25, -2.5, 9.75)),
            ("gyro", (0.125, -0.25, 0.5)),
        ):
            raw = packet(sensor, [(tick, xyz), (tick + 2400, xyz)])
            values = collector.decode_diag_packet(raw)
            self.assertEqual(
                [v["timestamp_ticks"] for v in values], [tick, tick + 2400]
            )
            for value in values:
                self.assertEqual((value["x"], value["y"], value["z"]), xyz)
                self.assertEqual(value["sensor_type"], sensor)
                self.assertEqual(value["kind"], "raw")
                self.assertEqual(value["status"], 3)
                self.assertEqual(value["batch_type"], 1)
                self.assertEqual(value["suid"], "99aabbccddeeff001122334455667788")
                self.assertEqual(value["instance_id"], 42)

    def test_api_copies_remain_labelled_for_collector_filtering(self):
        for kind in ("api", "client"):
            (value,) = collector.decode_diag_packet(
                packet("gyro", [(7, (0.125, 0.25, 0.5))], kind=kind)
            )
            self.assertEqual(value["kind"], "api")
            self.assertEqual(value["timestamp_ticks"], 7)

    def test_unrelated_packets_are_ignored_and_malformed_sensor_packets_rejected(self):
        self.assertEqual(collector.decode_diag_packet(b"\x73"), [])
        self.assertEqual(
            collector.decode_diag_packet(packet("gyro", [], code=0x19CD)), []
        )
        raw = packet("gyro", [(19_200_000, (0.0, 1.0, 2.0))])
        with self.assertRaises(ValueError):
            collector.decode_diag_packet(raw[:-1])
        with self.assertRaises(ValueError):
            collector.decode_diag_packet(packet("gyro", [(1, (math.nan, 1.0, 2.0))]))


if __name__ == "__main__":
    unittest.main()
