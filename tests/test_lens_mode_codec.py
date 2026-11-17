"""Exact type003F codec checks; no USB, camera state changes or reboot."""

from enum import IntEnum
import unittest

from osmo360_camera import controls as c


# Captured lens-mode GET reply from firmware10.00.25.29.
CAPTURED_GET = bytes.fromhex("0000013f000101")


class LensModeCodecTests(unittest.TestCase):
    def test_actual_captured_get_and_readonly_query(self):
        self.assertIs(c.decode_lens_mode(CAPTURED_GET), c.LensMode.PANORAMIC)
        self.assertEqual(c.encode_lens_mode_query(), bytes.fromhex("00013f00"))
        self.assertEqual(int(c.CameraCommand.PARAM_OPTION), 0x8E)

    def test_exact_set_packets_for_both_modes(self):
        self.assertEqual(
            c.encode_lens_mode(c.LensMode.SINGLE_LENS), bytes.fromhex("01013f000100")
        )
        self.assertEqual(
            c.encode_lens_mode(c.LensMode.PANORAMIC), bytes.fromhex("01013f000101")
        )
        self.assertIsNone(c.decode_set_ack(b"\0"))

    def test_single_lens_and_standard_byteslike_reply_inputs(self):
        self.assertIs(
            c.decode_lens_mode(CAPTURED_GET[:-1] + b"\0"), c.LensMode.SINGLE_LENS
        )
        for payload in (bytearray(CAPTURED_GET), memoryview(CAPTURED_GET)):
            with self.subTest(payload_type=type(payload).__name__):
                self.assertIs(c.decode_lens_mode(payload), c.LensMode.PANORAMIC)

    def test_set_requires_explicit_lens_enum_and_rejects_unknown_values(self):
        class OtherMode(IntEnum):
            ZERO = 0

        for value in (0, 1, True, False, 1.0, "1", None, OtherMode.ZERO):
            with self.subTest(value=value), self.assertRaises(TypeError):
                c.encode_lens_mode(value)
        for value in (-1, 2, 255, 256):
            with self.subTest(value=value), self.assertRaises(ValueError):
                c.LensMode(value)

    def test_get_rejects_empty_truncated_and_trailing_data(self):
        for payload in (b"", b"\0", CAPTURED_GET[:-1], CAPTURED_GET + b"\0"):
            with (
                self.subTest(payload=payload.hex()),
                self.assertRaises(c.ControlDecodeError),
            ):
                c.decode_lens_mode(payload)
        with self.assertRaises(TypeError):
            c.decode_lens_mode(CAPTURED_GET.hex())

    def test_get_requires_exact_operation_count_type_and_length(self):
        # Status must be followed by GET0, count1, type003F, length1.
        for index, value in ((1, 1), (2, 0), (2, 2), (3, 0x3E), (4, 1), (5, 0), (5, 2)):
            payload = bytearray(CAPTURED_GET)
            payload[index] = value
            with (
                self.subTest(index=index, value=value),
                self.assertRaises(c.ControlDecodeError),
            ):
                c.decode_lens_mode(payload)

    def test_get_rejection_status_is_retained_before_layout_validation(self):
        for payload in (b"\xd6", b"\xe3" + CAPTURED_GET[1:], b"\xdfbad"):
            with (
                self.subTest(payload=payload.hex()),
                self.assertRaises(c.CameraReplyError) as error,
            ):
                c.decode_lens_mode(payload)
            self.assertEqual(error.exception.status, payload[0])
            self.assertEqual(error.exception.payload, payload)

    def test_unknown_mode_is_decode_error_instead_of_an_untyped_integer(self):
        for value in (2, 255):
            with self.subTest(value=value), self.assertRaises(c.ControlDecodeError):
                c.decode_lens_mode(CAPTURED_GET[:-1] + bytes((value,)))


if __name__ == "__main__":
    unittest.main()
