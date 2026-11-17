"""Pure read-only power-query codec; no camera or transport is used."""

import unittest

from osmo360_camera.controls import CameraReplyError, ControlDecodeError
from osmo360_camera.power import build_system_mode_request, decode_system_mode_response


# Captured native USB system-mode reply, including its nonzero padding.
ACTUAL = bytes.fromhex(
    "007374616e646172645f7374617465202d206c69766576696577000000000000000100000000000000000000000000000000000000000000000000000000000000"
)


class PowerCodecTest(unittest.TestCase):
    def test_actual_system_mode_and_nonzero_padding(self):
        self.assertEqual(build_system_mode_request(), b"\x86\0\0")
        result = decode_system_mode_response(ACTUAL)
        self.assertEqual((result.state, result.scene), ("standard_state", "liveview"))
        self.assertEqual(result.raw, ACTUAL[1:])
        self.assertIn(1, result.raw[result.raw.index(0) :])

    def test_unknown_state_scene_names_are_retained(self):
        result = decode_system_mode_response(
            b"\0" + b"future_state - future_scene\0".ljust(64, b"\0")
        )
        self.assertEqual((result.state, result.scene), ("future_state", "future_scene"))

    def test_malformed_and_rejected_responses(self):
        with self.assertRaises(CameraReplyError) as error:
            decode_system_mode_response(b"\xe0")
        self.assertEqual(error.exception.status, 0xE0)
        for payload in (
            b"",
            ACTUAL[:-1],
            ACTUAL + b"\0",
            b"\0" + b"x" * 64,
            b"\0" + b"invalid\0".ljust(64, b"\0"),
            b"\0" + b"state - sc\xffene\0".ljust(64, b"\0"),
        ):
            with (
                self.subTest(payload=payload.hex()),
                self.assertRaises(ControlDecodeError),
            ):
                decode_system_mode_response(payload)


if __name__ == "__main__":
    unittest.main()
