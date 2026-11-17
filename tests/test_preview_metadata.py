"""Native preview SEI goldens and versioned-prefix boundaries; no camera I/O."""

import json
from pathlib import Path
import struct
from unittest import TestCase

from osmo360_camera.vendor.preview_metadata import records_from_video


FIXTURE = json.loads(
    (Path(__file__).parent / "fixtures" / "preview_sei_native.json").read_text()
)


def encoded_size(value):
    return bytes([255]) * (value // 255) + bytes([value % 255])


def sei(payload):
    """Standard SEI envelope for testing exact captured or malformed TLV bytes."""
    rbsp = encoded_size(245) + encoded_size(len(payload)) + payload + b"\x80"
    escaped = bytearray()
    zeros = 0
    for byte in rbsp:
        if zeros >= 2 and byte <= 3:
            escaped.append(3)
            zeros = 0
        escaped.append(byte)
        zeros = zeros + 1 if byte == 0 else 0
    return b"\0\0\0\1\x06" + escaped


def record(payload):
    return next(records_from_video(sei(payload)))


class PreviewSEITests(TestCase):
    def test_old_capture_keeps_original_timestamps_quaternions_and_calibration(self):
        old = FIXTURE["old"]
        row = record(bytes.fromhex(old["raw_tlv_hex"]))
        for key, value in old["expected"].items():
            self.assertEqual(row[key], value, key)
        self.assertEqual(row["quaternions"][0], old["first_quaternion"])
        self.assertEqual(len(row["quaternions"]), row["sample_count"])
        self.assertNotIn("motion_extension_hex", row)
        self.assertEqual(row["motion_version"], 1)
        self.assertEqual(row["motion_declared_body_length"], row["motion_body_length"])

    def test_current_original_sei_nals_use_v1_prefix_and_preserve_union_tail(self):
        for golden in FIXTURE["current"]:
            with self.subTest(frame=golden["name"]):
                row = next(
                    records_from_video(b"\0\0\0\1" + bytes.fromhex(golden["nal_hex"]))
                )
                for key, value in golden["expected"].items():
                    self.assertEqual(row[key], value, key)
                self.assertEqual(row["motion_version"], 1)
                self.assertEqual(
                    row["motion_body_length"] - row["motion_declared_body_length"], 12
                )
                self.assertEqual(row["motion_extension_hex"], bytes(12).hex())
                self.assertEqual(row["quaternions"][0], golden["first_quaternion"])
                self.assertEqual(len(row["quaternions"]), row["sample_count"])
                self.assertNotIn("raw_gyro", row)
                if golden["name"] == "first":
                    self.assertEqual(row["calibration_version"], 1)
                    self.assertEqual(row["calibration_declared_body_length"], 219)
                    self.assertEqual(row["calibration_body_length"], 287)
                    self.assertEqual(len(bytes.fromhex(row["calibration_hex"])), 287)
                    self.assertEqual(row["calibration_extension_hex"], bytes(68).hex())

    def test_unknown_calibration_version_and_length_do_not_hide_motion_timestamp(self):
        row = next(
            records_from_video(
                b"\0\0\0\1" + bytes.fromhex(FIXTURE["current"][0]["nal_hex"])
            )
        )
        payload = bytearray.fromhex(row["raw_tlv_hex"])
        self.assertEqual(struct.unpack_from("<H", payload)[0], 13)
        payload[4] = 99
        struct.pack_into("<I", payload, 5, 0xFFFFFFFF)
        actual = record(payload)
        self.assertEqual(actual["frame_clock_ns_raw"], row["frame_clock_ns_raw"])
        self.assertEqual(actual["calibration_version"], 99)
        self.assertEqual(actual["calibration_declared_body_length"], 0xFFFFFFFF)
        self.assertEqual(actual["calibration_hex"], row["calibration_hex"])
        self.assertEqual(actual["calibration_value_hex"], payload[4 : 4 + 292].hex())

    def test_unknown_motion_version_preserves_raw_without_guessing_a_timestamp(self):
        old = bytes.fromhex(FIXTURE["old"]["raw_tlv_hex"])
        first_length = struct.unpack_from("<H", old, 2)[0]
        payload = bytearray(old[4 + first_length :])
        self.assertEqual(struct.unpack_from("<H", payload)[0], 12)
        payload[4] = 2
        row = record(payload)
        self.assertEqual(row["motion_version"], 2)
        self.assertEqual(row["motion_decode_error"], "unsupported_version")
        self.assertEqual(row["raw_tlv_hex"], payload.hex())
        self.assertNotIn("frame_clock_ns_raw", row)
        self.assertNotIn("quaternions", row)

    def test_motion_extension_is_not_interpreted_as_more_quaternions(self):
        old = bytes.fromhex(FIXTURE["old"]["raw_tlv_hex"])
        first_length = struct.unpack_from("<H", old, 2)[0]
        payload = bytearray(old[4 + first_length :])
        length = struct.unpack_from("<H", payload, 2)[0]
        tail = bytes.fromhex("aabbccdd01020304")
        struct.pack_into("<H", payload, 2, length + len(tail))
        payload += tail
        row = record(payload)
        self.assertEqual(row["sample_count"], 33)
        self.assertEqual(len(row["quaternions"]), 33)
        self.assertEqual(row["motion_extension_hex"], tail.hex())

    def test_invalid_motion_length_and_count_are_still_rejected(self):
        old = bytes.fromhex(FIXTURE["old"]["raw_tlv_hex"])
        first_length = struct.unpack_from("<H", old, 2)[0]
        valid = bytearray(old[4 + first_length :])
        too_long = bytearray(valid)
        struct.pack_into("<I", too_long, 5, 0xFFFFFFFF)
        wrong_count = bytearray(valid)
        struct.pack_into("<I", wrong_count, 9 + 36, 34)
        short_outer = bytearray(valid)
        struct.pack_into("<H", short_outer, 2, len(short_outer))
        for payload in (too_long, wrong_count, short_outer):
            with (
                self.subTest(payload=payload[:12].hex()),
                self.assertRaises(ValueError),
            ):
                record(payload)


class ResumeEmptyMotionTests(TestCase):
    """The native fallback retains a frame clock without fusion samples."""

    @staticmethod
    def native_body():
        # The header-only body has no fusion payload. Exposure and the frame
        # clock still come from independent image metadata fields.
        body = bytearray(68)
        struct.pack_into("<IIQ", body, 12, 1, 100, 198155615260)
        return body

    @staticmethod
    def payload(body):
        return struct.pack("<HHBI", 12, len(body) + 5, 1, 0) + body

    def assert_header_only(self, row, body, *, clock=True):
        self.assertEqual(row["motion_version"], 1)
        self.assertEqual(row["motion_declared_body_length"], 0)
        self.assertEqual(row["motion_body_length"], 68)
        self.assertEqual(row["motion_body_hex"], bytes(body).hex())
        self.assertEqual(row["sample_count"], 0)
        self.assertEqual(row["imu_clock_us_raw"], 0)
        self.assertTrue(row["motion_header_only"])
        self.assertFalse(row["motion_samples_available"])
        self.assertFalse(row["imu_clock_available"])
        self.assertEqual(row["frame_clock_available"], clock)
        self.assertNotIn("quaternions", row)

    def test_exact_native_resume_nal(self):
        golden = json.loads(
            (
                Path(__file__).parent / "fixtures" / "preview_sei_resume_empty.json"
            ).read_text()
        )
        nal = b"\0\0\0\1" + bytes.fromhex(golden["nal_hex"])
        row = next(records_from_video(nal))
        for key, value in golden["expected"].items():
            self.assertEqual(row[key], value, key)
        self.assert_header_only(row, self.native_body())
        self.assertEqual(row["frame_clock_ns_raw"], 198155615260)
        self.assertEqual(len(bytes.fromhex(row["calibration_hex"])), 287)

    def test_independently_packed_exposure_pair_can_vary(self):
        for numerator, denominator in ((0, 0), (1, 100), (333, 7), (0xFFFFFFFF, 1)):
            with self.subTest(exposure=(numerator, denominator)):
                body = self.native_body()
                struct.pack_into("<II", body, 12, numerator, denominator)
                row = record(self.payload(body))
                self.assert_header_only(row, body)
                self.assertEqual(row["frame_clock_ns_raw"], 198155615260)
                self.assertEqual(
                    (row["unknown_u32_12"], row["unknown_u32_16"]),
                    (numerator, denominator),
                )

    def test_zero_frame_clock_is_unavailable_instead_of_a_zero_timestamp(self):
        body = self.native_body()
        struct.pack_into("<Q", body, 20, 0)
        row = record(self.payload(body))
        self.assert_header_only(row, body, clock=False)
        self.assertNotIn("frame_clock_ns_raw", row)

    def test_declared_zero_rejects_nearby_storage_lengths(self):
        body = self.native_body()
        for variant in (body[:-1], body + b"\0"):
            with self.subTest(body_length=len(variant)), self.assertRaises(ValueError):
                record(self.payload(variant))

    def test_declared_zero_rejects_every_nonzero_reserved_or_fusion_byte(self):
        for offset in (*range(12), *range(28, 68)):
            body = self.native_body()
            body[offset] = 1
            with (
                self.subTest(changed_body_offset=offset),
                self.assertRaises(ValueError),
            ):
                record(self.payload(body))
