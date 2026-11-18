"""Camera facade behavior with a fake transport; no USB access or recording.

The fake accepts real encoded command payloads and supplies ACK/readback
responses independently, so an ACK alone cannot make a setting test pass.
"""

from collections import deque
from dataclasses import replace
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import controls as c
from osmo360_camera.camera import Camera, CameraError, SettingApplyError
from osmo360_camera.transport import CameraDisconnected, RequestTimeout


class FakeClock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeTransport:
    def __init__(self):
        self.connected = False
        self.link_speed = "fake10Gb/s"
        self.requests = []
        self.open_calls = self.close_calls = 0
        self.open_error = self.close_error = None
        self.recording_state = 0
        self.handlers = {}
        self.state_outcomes = deque()
        self.handlers[(2, 0x70)] = self._state
        self.handlers[(2, 2)] = self._record

    def open(self):
        self.open_calls += 1
        if self.open_error:
            raise self.open_error
        self.connected = True
        return self

    def close(self):
        self.close_calls += 1
        self.connected = False
        if self.close_error:
            raise self.close_error

    def request(self, command_set, command, payload=b"", *, receiver=1):
        self.requests.append((command_set, command, payload, receiver))
        if not self.connected:
            raise CameraDisconnected("fake transport closed")
        result = self.handlers[(command_set, command)](payload)
        if isinstance(result, Exception):
            raise result
        return SimpleNamespace(payload=result)

    def _state(self, payload):
        assert payload == b""
        state = (
            self.state_outcomes.popleft()
            if self.state_outcomes
            else self.recording_state
        )
        if isinstance(state, Exception):
            raise state
        return b"\0" + (2 | state << 6).to_bytes(4, "little") + b"\x38"

    def _record(self, payload):
        assert payload in (b"\0", b"\1")
        self.recording_state = 2 if payload == b"\1" else 0
        return b"\0"

    def record_writes(self, value=None):
        return [
            r
            for r in self.requests
            if r[:2] == (2, 2) and (value is None or r[2] == value)
        ]


class CameraFacadeTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeTransport()
        self.clock = FakeClock()
        # Guard against accidentally opening the actual transport in a test.
        self.usb_guard = patch(
            "osmo360_camera.camera.USBTransport",
            side_effect=AssertionError("USB is forbidden in facade tests"),
        )
        self.usb_guard.start()
        self.clock_patch = patch("osmo360_camera.camera.time", self.clock)
        self.clock_patch.start()

    def tearDown(self):
        self.clock_patch.stop()
        self.usb_guard.stop()

    def camera(self):
        return Camera(transport=self.fake).connect()

    def color_camera(self, *, previous=63):
        self.color = previous

        def get(payload):
            self.assertEqual(payload, b"")
            return b"\0" + bytes([self.color])

        def set_value(payload):
            self.assertEqual(len(payload), 1)
            self.color = payload[0]
            return b"\0"

        self.fake.handlers[(2, int(c.CameraCommand.GET_COLOR))] = get
        self.fake.handlers[(2, int(c.CameraCommand.SET_COLOR))] = set_value
        return self.camera()

    def test_context_uses_injected_transport_and_closes_it(self):
        with Camera(transport=self.fake) as camera:
            self.assertTrue(camera.connected)
            self.assertEqual(camera.link_speed, "fake10Gb/s")
        self.assertFalse(camera.connected)
        self.assertEqual((self.fake.open_calls, self.fake.close_calls), (1, 1))
        self.assertEqual(self.fake.requests, [])

    def lens_camera(self, *, ignore_single=False):
        self.lens = c.LensMode.PANORAMIC
        self.native_profile = c.ModeProfile.VIDEO_PANO

        def typed_parameter(payload):
            if payload == bytes.fromhex("00013f00"):
                # Actual capture post-update-lens-mode-get-1.json for PANO1.
                return bytes.fromhex("0000013f0001") + bytes([self.lens])
            if payload in (
                bytes.fromhex("01013f000100"),
                bytes.fromhex("01013f000101"),
            ):
                requested = c.LensMode(payload[-1])
                if not (ignore_single and requested == c.LensMode.SINGLE_LENS):
                    self.lens = requested
                    # Hardware selected PHOTO_NORMAL(5), not VIDEO_NORMAL(1).
                    self.native_profile = (
                        5
                        if requested == c.LensMode.SINGLE_LENS
                        else c.ModeProfile.VIDEO_PANO
                    )
                return b"\0"
            raise AssertionError(
                f"Unexpected lens typed-parameter payload: {payload.hex()}"
            )

        self.fake.handlers[(2, 0x8E)] = typed_parameter
        return self.camera()

    def test_lens_mode_get_and_typed_set_route_readback_and_auto_profile(self):
        camera = self.lens_camera()
        self.assertEqual(camera.lens_mode(), c.LensMode.PANORAMIC)
        self.assertEqual(self.fake.requests, [(2, 0x8E, bytes.fromhex("00013f00"), 1)])
        self.fake.requests.clear()
        self.assertEqual(
            camera.set_lens_mode(c.LensMode.SINGLE_LENS), c.LensMode.SINGLE_LENS
        )
        self.assertEqual(self.native_profile, 5)
        self.assertEqual(
            [(r[1], r[2]) for r in self.fake.requests],
            [
                (0x70, b""),
                (0x8E, bytes.fromhex("00013f00")),
                (0x8E, bytes.fromhex("01013f000100")),
                (0x8E, bytes.fromhex("00013f00")),
            ],
        )
        self.assertTrue(all(r[0] == 2 and r[3] == 1 for r in self.fake.requests))

    def test_lens_mode_ack_without_matching_readback_rolls_back(self):
        camera = self.lens_camera(ignore_single=True)
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_lens_mode(c.LensMode.SINGLE_LENS)
        error = caught.exception
        self.assertEqual(error.previous, c.LensMode.PANORAMIC)
        self.assertEqual(error.actual, c.LensMode.PANORAMIC)
        self.assertTrue(error.restored)
        self.assertEqual(self.lens, c.LensMode.PANORAMIC)
        self.assertEqual(
            [r[2] for r in self.fake.requests if r[1] == 0x8E and r[2][:1] == b"\1"],
            [bytes.fromhex("01013f000100"), bytes.fromhex("01013f000101")],
        )

    def test_lens_mode_requires_explicit_enum_before_any_usb_request(self):
        camera = self.camera()
        for invalid in (0, 1, True, "pano", None, c.ModeProfile.VIDEO_NORMAL):
            with self.subTest(invalid=invalid), self.assertRaises(TypeError):
                camera.set_lens_mode(invalid)
        self.assertEqual(self.fake.requests, [])

    def test_lens_mode_record_guard_blocks_all_nonidle_states(self):
        camera = self.camera()
        for state in (1, 2, 3):
            self.fake.recording_state = state
            with self.subTest(state=state), self.assertRaises(CameraError):
                camera.set_lens_mode(c.LensMode.SINGLE_LENS)
        self.assertTrue(all(r[1] == 0x70 for r in self.fake.requests))

    def test_apply_requires_ack_and_actual_readback(self):
        camera = self.color_camera()
        target = c.ColorSetting(int(c.ColorMode.D_LOG_M))
        self.assertEqual(camera.set_color(c.ColorMode.D_LOG_M), target)
        self.assertEqual(
            [(r[1], r[2]) for r in self.fake.requests],
            [(0x43, b""), (0x42, b"\x3d"), (0x43, b"")],
        )
        self.assertTrue(all(r[0] == 2 and r[3] == 1 for r in self.fake.requests))

    def test_apply_waits_for_asynchronous_readback(self):
        camera = self.color_camera()
        reads = deque([63, 63, 63, 61])  # previous, two stale reads, then applied.
        self.fake.handlers[(2, 0x43)] = lambda _: b"\0" + bytes([reads.popleft()])
        self.assertEqual(camera.set_color(c.ColorMode.D_LOG_M), c.ColorSetting(61))
        self.assertAlmostEqual(self.clock.now, 0.1)
        self.assertEqual([r[2] for r in self.fake.requests if r[1] == 0x42], [b"\x3d"])

    def test_ack_rejection_rolls_back_and_preserves_error_details(self):
        camera = self.color_camera()

        def setter(payload):
            self.color = payload[0]
            return b"\xe3" if payload == b"\x3d" else b"\0"

        self.fake.handlers[(2, 0x42)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_color(c.ColorMode.D_LOG_M)
        error = caught.exception
        self.assertEqual(error.previous, c.ColorSetting(63))
        self.assertIsNone(error.actual)
        self.assertTrue(error.restored)
        self.assertIsInstance(error.__cause__, c.CameraReplyError)
        self.assertEqual(error.__cause__.status, 0xE3)
        self.assertEqual(self.color, 63)

    def test_readback_mismatch_restores_exact_previous_unknown_value(self):
        camera = self.color_camera(previous=242)  # preserve unknown wire enum.

        def setter(payload):
            self.color = 60 if payload == b"\x3d" else payload[0]
            return b"\0"

        self.fake.handlers[(2, 0x42)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_color(c.ColorMode.D_LOG_M)
        self.assertEqual(caught.exception.previous, c.ColorSetting(242))
        self.assertEqual(caught.exception.actual, c.ColorSetting(60))
        self.assertTrue(caught.exception.restored)
        self.assertEqual(
            [r[2] for r in self.fake.requests if r[1] == 0x42], [b"\x3d", b"\xf2"]
        )
        self.assertEqual(self.color, 242)

    def test_failed_rollback_is_reported_as_unrestored(self):
        camera = self.color_camera()

        def setter(payload):
            if payload == b"\x3f":
                raise RequestTimeout("lost rollback ACK")
            self.color = 60
            return b"\0"

        self.fake.handlers[(2, 0x42)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_color(c.ColorMode.D_LOG_M)
        self.assertFalse(caught.exception.restored)
        self.assertEqual(caught.exception.actual, c.ColorSetting(60))

    def test_lost_set_ack_also_restores_previous_value(self):
        camera = self.color_camera()

        def setter(payload):
            self.color = payload[0]
            if payload == b"\x3d":
                raise RequestTimeout("lost SET ACK after camera applied it")
            return b"\0"

        self.fake.handlers[(2, 0x42)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_color(c.ColorMode.D_LOG_M)
        self.assertTrue(caught.exception.restored)
        self.assertEqual(self.color, 63)
        self.assertIsInstance(caught.exception.__cause__, RequestTimeout)

    def test_keyboard_interrupt_after_set_restores_baseline_and_reraises_original(self):
        camera = self.color_camera()
        interrupt = KeyboardInterrupt("Ctrl-C after SET reached the camera")
        reads = 0

        def getter(_):
            nonlocal reads
            reads += 1
            if reads == 2:
                raise interrupt  # Baseline GET succeeded; SET was already applied.
            return b"\0" + bytes([self.color])

        self.fake.handlers[(2, 0x43)] = getter
        with self.assertRaises(KeyboardInterrupt) as caught:
            camera.set_color(c.ColorMode.D_LOG_M)
        self.assertIs(caught.exception, interrupt)
        self.assertEqual(self.color, 63)
        self.assertEqual(
            [(r[1], r[2]) for r in self.fake.requests],
            [(0x43, b""), (0x42, b"\x3d"), (0x43, b""), (0x42, b"\x3f"), (0x43, b"")],
        )

    def test_invalid_setting_does_not_write_to_camera(self):
        camera = self.color_camera()
        with self.assertRaises(TypeError):
            camera.set_color(61)  # must be explicit ColorMode or ColorSetting.
        self.assertFalse(any(r[1] == 0x42 for r in self.fake.requests))

    def test_missing_shutter_previous_df_can_apply_without_invented_rollback(self):
        target = c.ShutterSpeed(1, c.ShutterValue(100, True))
        shutter = None

        def getter(_):
            return b"\xdf" if shutter is None else b"\0" + c.encode_shutter(shutter)

        def setter(payload):
            nonlocal shutter
            shutter = c.decode_shutter(b"\0" + payload)
            return b"\0"

        self.fake.handlers[(2, 0x29)] = getter
        self.fake.handlers[(2, 0x28)] = setter
        camera = self.camera()
        self.assertEqual(camera.set_shutter(target), target)
        shutter = None
        self.fake.handlers[(2, 0x28)] = lambda _: b"\xe3"
        before = len(self.fake.requests)
        with self.assertRaises(SettingApplyError) as caught:
            camera.set_shutter(target)
        self.assertIsNone(caught.exception.previous)
        self.assertIsNone(caught.exception.restored)
        self.assertEqual(sum(r[1] == 0x28 for r in self.fake.requests[before:]), 1)

    def test_non_df_getter_error_prevents_shutter_write(self):
        self.fake.handlers[(2, 0x29)] = lambda _: b"\xe0"
        camera = self.camera()
        with self.assertRaises(c.CameraReplyError):
            camera.set_shutter(c.ShutterSpeed(1, c.ShutterValue(100, True)))
        self.assertFalse(any(r[1] == 0x28 for r in self.fake.requests))

    def test_manual_white_balance_accepts_retail_extended_get_layout(self):
        # Real retail GET snapshot is 00 00 14000000 (mode0, CT20, extended).
        # Firmware accepts the legacy SET too, but GET remains the extended
        # layout. Layout metadata must not turn correct Kelvin readback into
        # a mismatch/rollback of an otherwise successful user setting.
        current = c.WhiteBalance(0, 20, True)

        def getter(_):
            return b"\0" + c.encode_white_balance(current)

        def setter(payload):
            nonlocal current
            current = replace(c.decode_white_balance(b"\0" + payload), extended=True)
            return b"\0"

        self.fake.handlers[(2, 0x2D)] = getter
        self.fake.handlers[(2, 0x2C)] = setter
        camera = self.camera()
        actual = camera.set_white_balance(c.WhiteBalance.manual(5600))
        self.assertEqual((actual.mode, actual.kelvin), (6, 5600))
        self.assertEqual(sum(r[1] == 0x2C for r in self.fake.requests), 1)

    def test_auto_white_balance_accepts_saved_temperature_without_manual_rollback(self):
        current = c.WhiteBalance(c.WhiteBalanceMode.MANUAL, 50, True)

        def getter(_):
            return b"\0" + c.encode_white_balance(current)

        def setter(payload):
            nonlocal current
            requested = c.decode_white_balance(b"\0" + payload)
            # Actual native behavior: AUTO changes only the mode; CT remains50.
            current = replace(current, mode=requested.mode)
            return b"\0"

        self.fake.handlers[(2, 0x2D)] = getter
        self.fake.handlers[(2, 0x2C)] = setter
        actual = self.camera().set_white_balance(
            c.WhiteBalance(c.WhiteBalanceMode.AUTO, 20)
        )
        self.assertEqual(
            (actual.mode, actual.temperature_code), (c.WhiteBalanceMode.AUTO, 50)
        )
        writes = [r[2] for r in self.fake.requests if r[1] == 0x2C]
        self.assertEqual(len(writes), 1)
        self.assertEqual(
            c.decode_white_balance(b"\0" + writes[0]).mode, c.WhiteBalanceMode.AUTO
        )

    def test_manual_white_balance_wrong_temperature_fails_and_rolls_back_to_auto(self):
        current = c.WhiteBalance(c.WhiteBalanceMode.AUTO, 20, True)

        def getter(_):
            return b"\0" + c.encode_white_balance(current)

        def setter(payload):
            nonlocal current
            requested = c.decode_white_balance(b"\0" + payload)
            if requested.mode == c.WhiteBalanceMode.MANUAL:
                current = replace(current, mode=requested.mode, temperature_code=45)
            else:
                current = replace(current, mode=requested.mode)
            return b"\0"

        self.fake.handlers[(2, 0x2D)] = getter
        self.fake.handlers[(2, 0x2C)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            self.camera().set_white_balance(c.WhiteBalance.manual(5000))
        error = caught.exception
        self.assertEqual(
            (error.actual.mode, error.actual.temperature_code),
            (c.WhiteBalanceMode.MANUAL, 45),
        )
        self.assertTrue(error.restored)
        self.assertEqual(current.mode, c.WhiteBalanceMode.AUTO)
        writes = [
            c.decode_white_balance(b"\0" + r[2])
            for r in self.fake.requests
            if r[1] == 0x2C
        ]
        self.assertEqual(
            [w.mode for w in writes],
            [c.WhiteBalanceMode.MANUAL, c.WhiteBalanceMode.AUTO],
        )

    def test_legacy_shutter_set_accepts_v2_get_without_changing_requested_time(self):
        target = c.ShutterSpeed(1, c.ShutterValue(100, True))
        current = None

        def getter(_):
            return b"\xdf" if current is None else b"\0" + c.encode_shutter(current)

        def setter(payload):
            nonlocal current
            current = replace(
                c.decode_shutter(b"\0" + payload),
                shutter_type=2,
                parameter_flags=0xE000,
            )
            return b"\0"

        self.fake.handlers[(2, 0x29)] = getter
        self.fake.handlers[(2, 0x28)] = setter
        camera = self.camera()
        actual = camera.set_shutter(target)
        self.assertEqual(actual.shutter.seconds, target.shutter.seconds)
        self.assertEqual((actual.shutter_type, actual.parameter_flags), (2, 0xE000))
        self.assertEqual(sum(r[1] == 0x28 for r in self.fake.requests), 1)

    def test_explicit_v2_shutter_metadata_still_requires_matching_readback(self):
        target = c.ShutterSpeed(1, c.ShutterValue(100, True), 2, 0xE000)
        current = None

        def getter(_):
            return b"\xdf" if current is None else b"\0" + c.encode_shutter(current)

        def setter(payload):
            nonlocal current
            current = replace(c.decode_shutter(b"\0" + payload), shutter_type=3)
            return b"\0"

        self.fake.handlers[(2, 0x29)] = getter
        self.fake.handlers[(2, 0x28)] = setter
        with self.assertRaises(SettingApplyError) as caught:
            self.camera().set_shutter(target)
        self.assertEqual(
            caught.exception.actual.shutter.seconds, target.shutter.seconds
        )
        self.assertEqual(caught.exception.actual.shutter_type, 3)
        self.assertIsNone(caught.exception.previous)
        self.assertIsNone(caught.exception.restored)

    def test_frame_rate_uses_explicit_enum_and_preserves_other_format_bytes(self):
        current = c.VideoFormat(111, 3, 0xFC, 0xFA, 0xE9)

        def getter(_):
            return b"\0" + c.encode_video_format(current)

        def setter(payload):
            nonlocal current
            current = c.decode_video_format(b"\0" + payload)
            return b"\0"

        self.fake.handlers[(2, 0x19)] = getter
        self.fake.handlers[(2, 0x18)] = setter
        camera = self.camera()
        with self.assertRaises(TypeError):
            camera.set_frame_rate(30)
        self.assertEqual(self.fake.requests, [])
        actual = camera.set_frame_rate(c.FrameRate.FPS30_TRUE)
        self.assertEqual(actual, c.VideoFormat(111, 14, 0xFC, 0xFA, 0xE9))
        self.assertEqual(
            [r[2] for r in self.fake.requests if r[1] == 0x18],
            [bytes.fromhex("6f0efcfae9")],
        )

    def test_apply_transactions_do_not_interleave_between_set_and_readback(self):
        camera = self.color_camera()
        entered, release, second_started = (threading.Event() for _ in range(3))
        errors = []

        def setter(payload):
            if payload == b"\x3d":
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test did not release first SET")
            self.color = payload[0]
            return b"\0"

        self.fake.handlers[(2, 0x42)] = setter

        def apply(mode, started=None):
            if started:
                started.set()
            try:
                camera.set_color(mode)
            except BaseException as error:
                errors.append(error)

        first = threading.Thread(target=apply, args=(c.ColorMode.D_LOG_M,))
        second = threading.Thread(target=apply, args=(c.ColorMode.HLG, second_started))
        first.start()
        try:
            self.assertTrue(entered.wait(1))
            second.start()
            self.assertTrue(second_started.wait(1))
            # Give the second caller a chance to acquire the transaction lock.
            # It must not issue even its "previous value" GET while SET1 waits.
            threading.Event().wait(0.03)
            self.assertEqual(
                [(r[1], r[2]) for r in self.fake.requests],
                [(0x43, b""), (0x42, b"\x3d")],
            )
        finally:
            release.set()
            first.join(2)
            if second.ident is not None:
                second.join(2)
        self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(
            [(r[1], r[2]) for r in self.fake.requests],
            [
                (0x43, b""),
                (0x42, b"\x3d"),
                (0x43, b""),
                (0x43, b""),
                (0x42, b"\x3c"),
                (0x43, b""),
            ],
        )

    def test_close_stops_only_recording_owned_by_this_camera(self):
        camera = self.camera()
        self.assertTrue(camera.start_recording().recording)
        camera.close()
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual([r[2] for r in self.fake.record_writes()], [b"\1", b"\0"])
        self.assertFalse(self.fake.connected)

        external = FakeTransport()
        external.recording_state = 2
        with Camera(transport=external):
            pass
        self.assertEqual(external.recording_state, 2)
        self.assertEqual(external.record_writes(), [])

    def test_start_refuses_nonidle_recording_without_taking_ownership(self):
        self.fake.recording_state = 2
        camera = self.camera()
        with self.assertRaises(CameraError):
            camera.start_recording()
        camera.close()
        self.assertEqual(self.fake.record_writes(), [])
        self.assertEqual(self.fake.recording_state, 2)

    def test_lost_start_ack_attempts_stop_and_reraises_start_error(self):
        camera = self.camera()

        def recording(payload):
            self.fake._record(payload)
            if payload == b"\1":
                raise RequestTimeout("START ACK lost after recording began")
            return b"\0"

        self.fake.handlers[(2, 2)] = recording
        with self.assertRaises(RequestTimeout):
            camera.start_recording()
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual(len(self.fake.record_writes(b"\0")), 1)
        camera.close()
        self.assertEqual(len(self.fake.record_writes(b"\0")), 1)

    def test_start_preserves_original_error_when_cleanup_stop_also_fails(self):
        camera = self.camera()
        start_error = RequestTimeout("original START ACK lost")

        def recording(payload):
            if payload == b"\1":
                self.fake.recording_state = 2
                raise start_error
            return b"\xe3"

        self.fake.handlers[(2, 2)] = recording
        with self.assertRaises(RequestTimeout) as caught:
            camera.start_recording()
        self.assertIs(caught.exception, start_error)
        if hasattr(start_error, "add_note"):
            self.assertTrue(any("STOP" in note for note in start_error.__notes__))
        self.assertEqual(len(self.fake.record_writes(b"\0")), 1)
        self.fake.handlers[(2, 2)] = self.fake._record
        camera.close()  # Ownership survived the failed cleanup; retry succeeds.
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual(len(self.fake.record_writes(b"\0")), 2)

    def test_keyboard_interrupt_during_start_still_attempts_stop(self):
        camera = self.camera()

        def recording(payload):
            self.fake._record(payload)
            if payload == b"\1":
                raise KeyboardInterrupt("Ctrl-C during START")
            return b"\0"

        self.fake.handlers[(2, 2)] = recording
        with self.assertRaises(KeyboardInterrupt):
            camera.start_recording()
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual([r[2] for r in self.fake.record_writes()], [b"\1", b"\0"])

    def test_start_transition_timeout_also_stops_owned_recording(self):
        camera = self.camera()

        def recording(payload):
            self.fake.recording_state = 1 if payload == b"\1" else 0
            return b"\0"

        self.fake.handlers[(2, 2)] = recording
        with self.assertRaises(CameraError):
            camera.start_recording()
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual([r[2] for r in self.fake.record_writes()], [b"\1", b"\0"])

    def test_failed_stop_ack_retains_ownership_for_close_retry(self):
        camera = self.camera()
        camera.start_recording()
        self.fake.handlers[(2, 2)] = lambda _: b"\xe3"
        with self.assertRaises(c.CameraReplyError):
            camera.stop_recording()
        self.assertEqual(self.fake.recording_state, 2)
        self.fake.handlers[(2, 2)] = self.fake._record
        camera.close()
        self.assertEqual(self.fake.recording_state, 0)
        self.assertEqual(len(self.fake.record_writes(b"\0")), 2)

    def test_owned_stop_is_attempted_even_when_state_readback_times_out(self):
        camera = self.camera()
        camera.start_recording()
        # Commands still reach the camera, but GET_STATE responses are lost.
        # An owned recording must receive a best-effort STOP before USB closes,
        # even though final state verification will necessarily report failure.
        self.fake.handlers[(2, 0x70)] = lambda _: RequestTimeout("GET_STATE timed out")
        with self.assertRaises(CameraError):
            camera.close()
        self.assertEqual(len(self.fake.record_writes(b"\0")), 1)
        self.assertEqual(self.fake.recording_state, 0)
        self.assertFalse(self.fake.connected)

    def test_close_attempts_imu_and_usb_cleanup_after_recording_error(self):
        camera = self.camera()
        camera.start_recording()
        self.fake.handlers[(2, 2)] = lambda _: b"\xe3"
        imu = Mock()
        imu.stop.side_effect = RuntimeError("mask restoration failed")
        camera._imu = imu  # Own a fake stream without spawning any process.
        self.fake.close_error = RuntimeError("USB cleanup failed")
        with self.assertRaises(CameraError) as caught:
            camera.close()
        self.assertIn("STOP:", str(caught.exception))
        self.assertIn("IMU:", str(caught.exception))
        self.assertIn("USB:", str(caught.exception))
        imu.stop.assert_called_once()
        self.assertEqual(self.fake.close_calls, 1)

    def test_connect_failure_propagates_without_read_or_write(self):
        self.fake.open_error = CameraDisconnected("camera absent")
        camera = Camera(transport=self.fake)
        with self.assertRaises(CameraDisconnected):
            camera.connect()
        self.assertFalse(camera.connected)
        self.assertEqual(self.fake.requests, [])

    def test_successful_close_releases_owned_imu_before_reconnect(self):
        first, second = Mock(), Mock()
        for imu in (first, second):
            imu.pid = None
            imu.stop.return_value = None
        with patch("osmo360_camera.camera.IMUStream", side_effect=[first, second]):
            camera = self.camera()
            self.assertIs(camera.imu(queue_size=10), first)
            with self.assertRaises(CameraError):
                camera.imu()
            camera.close()
            first.stop.assert_called_once()
            camera.connect()
            self.assertIs(camera.imu(), second)
            camera.close()
            second.stop.assert_called_once()

    def test_close_retains_collector_when_custom_stop_only_reports_restored_mask(self):
        for proof in ({"restored": True, "errors": []},
                      {"restored": True, "quiescent": False, "errors": []}):
            with self.subTest(proof=proof):
                camera = self.camera()
                imu = camera._imu = Mock(pid=123)
                imu.stop.return_value = proof
                with self.assertRaisesRegex(CameraError, "quiescence"):
                    camera.close()
                self.assertIs(camera._imu, imu)
                self.assertFalse(self.fake.connected)


if __name__ == "__main__":
    unittest.main()
