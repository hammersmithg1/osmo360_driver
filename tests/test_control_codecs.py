"""Offline protocol checks. No USB module, camera access or state mutation."""

import json
from fractions import Fraction
from pathlib import Path
import struct
import unittest

from osmo360_camera import controls as c


FIXTURE = json.loads(
    (Path(__file__).parent / "fixtures/camera_controls_native.json").read_text()
)


def packet(name):
    return bytes.fromhex(FIXTURE[name]["payload_hex"])


class NativeControlsTest(unittest.TestCase):
    def test_actual_readonly_get_snapshot(self):
        data = FIXTURE["current_gets"]
        video = c.decode_video_format(bytes.fromhex(data["video_format"]))
        self.assertEqual(c.encode_video_format(video).hex(), data["video_format"][2:])
        exposure = c.decode_exposure_mode(bytes.fromhex(data["exposure_mode"]))
        self.assertEqual(exposure.known_mode, c.ExposureMode.PROGRAM)
        self.assertEqual(
            c.encode_exposure_mode(exposure).hex(), data["exposure_mode"][2:]
        )
        wb = c.decode_white_balance(bytes.fromhex(data["white_balance"]))
        self.assertEqual(wb, c.WhiteBalance(0, 20, True))
        self.assertEqual(wb.kelvin, 2000)
        self.assertEqual(c.encode_white_balance(wb).hex(), data["white_balance"][2:])
        self.assertTrue(c.decode_iso(bytes.fromhex(data["iso"])).automatic)
        self.assertEqual(
            c.decode_color(bytes.fromhex(data["color"])).known_mode,
            c.ColorMode.NORMAL_ENHANCE,
        )
        with self.assertRaises(c.CameraReplyError) as error:
            c.decode_shutter(bytes.fromhex(data["shutter"]))
        self.assertEqual(error.exception.status, 0xDF)

    def test_real_capture_settings(self):
        settings = c.decode_capture_parameters(packet("capture_parameters"))
        self.assertEqual(settings.video_format, c.VideoFormat(111, 3, 0, 0, 112))
        self.assertEqual(settings.video_format.known_resolution, c.Resolution.SQUARE_4K)
        self.assertEqual(settings.video_format.fps, Fraction(30000, 1001))
        self.assertEqual(settings.exposure_mode.known_mode, c.ExposureMode.PROGRAM)
        self.assertEqual(settings.white_balance, c.WhiteBalance(0, 28))
        self.assertEqual(settings.white_balance.kelvin, 2800)
        self.assertEqual(settings.color.known_mode, c.ColorMode.NORMAL_ENHANCE)
        self.assertTrue(settings.iso.automatic)
        self.assertIsNone(settings.shutter.seconds)
        self.assertEqual(len(settings.raw), 104)

        # The legacy GET codecs share these exact packed field layouts. This
        # verifies captured bytes, without claiming separate GETs were captured.
        raw = settings.raw
        self.assertEqual(
            c.decode_video_format(b"\0" + raw[13:18]), settings.video_format
        )
        self.assertEqual(
            c.decode_exposure_mode(b"\0" + raw[20:22]), settings.exposure_mode
        )
        self.assertEqual(
            c.decode_white_balance(b"\0" + raw[23:25]), settings.white_balance
        )
        self.assertEqual(c.decode_color(b"\0" + raw[31:32]), settings.color)
        self.assertEqual(c.encode_video_format(settings.video_format), raw[13:18])

    def test_real_profile_readback_and_rejection(self):
        profile = c.decode_mode_profile(packet("mode_profile"))
        # Current camera has a value beyond the bundled header's old MAX=0x37.
        self.assertEqual(profile.profile, 0x38)
        self.assertEqual(c.encode_mode_profile(profile), b"\x38")
        self.assertEqual(
            c.encode_profile_query().hex(), FIXTURE["mode_profile"]["request_hex"]
        )
        with self.assertRaises(c.CameraReplyError) as error:
            c.decode_set_ack(packet("rejected_profile"))
        self.assertEqual(error.exception.status, 0xE3)
        self.assertEqual(error.exception.payload, b"\xe3")

    def test_subscription_correlation_and_bounds(self):
        value = packet("mode_profile")
        for bad in (value[:4], value[:-1], value + b"\0"):
            with (
                self.subTest(payload=bad.hex()),
                self.assertRaises(c.ControlDecodeError),
            ):
                c.decode_mode_profile(bad)
        with self.assertRaises(c.ControlDecodeError):
            c.decode_mode_profile(value, key=0x4262)
        with self.assertRaises(c.ControlDecodeError):
            c.decode_mode_profile(value, subscribe_id=1)
        with self.assertRaises(c.ControlDecodeError):
            c.decode_mode_profile(value[:2] + b"\x03" + value[3:])
        with self.assertRaises(c.ControlDecodeError):
            c.decode_mode_profile(value + value[5:])
        with self.assertRaises(c.ControlDecodeError):
            c.decode_capture_parameters(value)

    def test_all_readers_reject_status_and_malformed_length(self):
        for decode, size in (
            (c.decode_video_format, 5),
            (c.decode_exposure_mode, 2),
            (c.decode_white_balance, 2),
            (c.decode_color, 1),
            (c.decode_iso, 1),
            (c.decode_shutter, 4),
        ):
            with self.subTest(reader=decode.__name__):
                with self.assertRaises(c.CameraReplyError):
                    decode(b"\xd6")
                with self.assertRaises(c.ControlDecodeError):
                    decode(b"")
                with self.assertRaises(c.ControlDecodeError):
                    decode(b"\0" * size)
        with self.assertRaises(c.ControlDecodeError):
            c.decode_set_ack(b"\0\0")
        self.assertIsNone(c.decode_set_ack(b"\0"))

    def test_unknown_enum_and_reserved_bits_survive_restore(self):
        video = c.decode_video_format(bytes.fromhex("00feddfcfae9"))
        self.assertIsNone(video.known_resolution)
        self.assertIsNone(video.fps)
        self.assertEqual(c.encode_video_format(video).hex(), "feddfcfae9")
        exposure = c.decode_exposure_mode(b"\0\xfe\xa3")
        self.assertIsNone(exposure.known_mode)
        self.assertEqual(c.encode_exposure_mode(exposure), b"\xfe\xa3")
        color = c.decode_color(b"\0\xf2")
        self.assertIsNone(color.known_mode)
        self.assertEqual(c.encode_color(color), b"\xf2")

    def test_wb_variants_and_explicit_hardware_units(self):
        wb = c.WhiteBalance.manual(5600)
        self.assertEqual(c.encode_white_balance(wb), bytes((6, 56)))
        extended = b"\0" + struct.pack("<BI", 6, 56)
        parsed = c.decode_white_balance(extended)
        self.assertTrue(parsed.extended)
        self.assertEqual(parsed.kelvin, 5600)
        self.assertEqual(c.encode_white_balance(parsed), extended[1:])
        for invalid in (1900, 10001, 5550):
            with self.assertRaises(ValueError):
                c.WhiteBalance.manual(invalid)
        self.assertEqual(c.IsoSetting.manual(6400).code, 9)
        self.assertEqual(c.decode_iso(b"\0\x09").iso, 6400)
        with self.assertRaises(ValueError):
            c.IsoSetting.manual(123)

    def test_shutter_wire_flag_decimal_and_v2_fields(self):
        # Synthetic native-layout packets: bit15=reciprocal and decimal is
        # printed as decimal digits, not an exponent or fixed 1/100 scale.
        shutter = c.decode_shutter(bytes.fromhex("0001648000"))
        self.assertEqual(shutter.shutter.seconds, Fraction(1, 100))
        self.assertEqual(c.encode_shutter(shutter).hex(), "01648000")
        value = c.ShutterSpeed(1, c.ShutterValue(2, False, 5), 2, 0xE000)
        self.assertEqual(value.shutter.seconds, Fraction(5, 2))
        self.assertEqual(c.decode_shutter(b"\0" + c.encode_shutter(value)), value)

    def test_wire_ranges_and_no_implicit_hdr_capability(self):
        for make in (
            lambda: c.ColorSetting(256),
            lambda: c.ModeProfileSetting(-1),
            lambda: c.VideoFormat(0, True, 0, 0, 0),
            lambda: c.ShutterValue(32768, False),
            lambda: c.encode_camera_info_query(65536),
        ):
            with self.assertRaises((ValueError, TypeError)):
                make()
        with self.assertRaises(TypeError):
            c.encode_color(61)
        caps = c.ColorCapabilities((c.ColorMode.NORMAL_ENHANCE, c.ColorMode.D_LOG_M))
        self.assertTrue(caps.supports_d_log_m)
        self.assertFalse(caps.supports_hlg)
        self.assertFalse(hasattr(caps, "supports_hdr_video"))
        self.assertEqual(c.VideoFormat(111, c.FrameRate.FPS30_TRUE, 0, 0, 0).fps, 30)


if __name__ == "__main__":
    unittest.main()
