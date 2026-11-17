"""Fake-network lifecycle, metadata boundaries and synthetic H.264 decoding."""

import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import socket
import struct
import threading
from types import SimpleNamespace
from unittest import TestCase, mock, skipUnless

from osmo360_camera.video import VideoStream, VideoStreamError, _decoder_bytes
from osmo360_camera.vendor.preview_metadata import records_from_video
from osmo360_camera.vendor import datalink, util
from test_preview_metadata import sei


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / "tests/fixtures/preview_sei_native.json").read_text())
RAW = bytes.fromhex(FIXTURE["current"][0]["nal_hex"])
RAW = b"\0\0\0\1" + RAW
STAMP = FIXTURE["current"][0]["expected"]["frame_clock_ns_raw"]


class FakeLink:
    def __init__(
        self,
        *,
        on_video,
        units=(),
        error=None,
        stubborn=False,
        stop_error=None,
        **kwargs,
    ):
        self.on_video = on_video
        self.units = units
        self.error = error
        self.stubborn = stubborn
        self.stop_error = stop_error
        self.stats = {"packets": 0}
        self.fed = threading.Event()
        self.release = threading.Event()

    def run(self):
        try:
            for raw in self.units:
                self.on_video(raw, 0.0)
                self.stats["packets"] += 1
            if self.error:
                raise self.error
        finally:
            self.fed.set()
        self.release.wait(15)

    def stop(self):
        if not self.stubborn:
            self.release.set()
        if self.stop_error:
            raise self.stop_error


class FakeImage:
    def __init__(self, packet):
        self.pts, self.time_base = packet.pts, packet.time_base
        self.calls = 0
        self.array = object()

    def to_ndarray(self, **kwargs):
        self.calls += 1
        if kwargs != {"format": "bgr24"}:
            raise AssertionError(kwargs)
        return self.array


class FakeCodec:
    def __init__(self):
        self.inputs = []
        self.thread_count = None

    def decode(self, packet):
        self.inputs.append(packet)
        return [FakeImage(packet)]


class FakePacket:
    def __init__(self, data):
        self.data = data


class VideoTests(TestCase):
    def link_patch(self, **kwargs):
        links = []

        def create(**options):
            link = FakeLink(**options, **kwargs)
            links.append(link)
            return link

        return links, mock.patch("osmo360_camera.video.Datalink", side_effect=create)

    def test_raw_mode_context_queue_and_optional_decoder(self):
        links, patch = self.link_patch(units=[RAW, RAW, RAW])
        with (
            patch,
            mock.patch("osmo360_camera.video.poke"),
            mock.patch.dict(sys.modules, {"av": None}),
        ):
            with VideoStream(decode=False, queue_size=2) as stream:
                self.assertTrue(links[0].fed.wait(2))
                self.assertEqual(stream.stats["dropped_frames"], 1)
                frame = next(stream)
                self.assertEqual(frame.timestamp_ns, STAMP)
                self.assertGreater(frame.host_receive_monotonic_ns, 0)
                self.assertEqual(frame.raw_h264, RAW)
                self.assertIsNone(frame.image)
                with self.assertRaises(VideoStreamError):
                    _ = frame.bgr
            self.assertEqual(stream.read().timestamp_ns, STAMP)
            with self.assertRaises(StopIteration):
                stream.read()
            with self.assertRaises(VideoStreamError):
                stream.start()

    def test_decoder_receives_every_access_unit_before_decoded_queue_drops(self):
        codec = FakeCodec()
        fake_av = SimpleNamespace(
            CodecContext=SimpleNamespace(create=lambda *_: codec), Packet=FakePacket
        )
        links, patch = self.link_patch(units=[RAW] * 10)
        with (
            patch,
            mock.patch("osmo360_camera.video.poke"),
            mock.patch.dict(sys.modules, {"av": fake_av}),
        ):
            stream = VideoStream(queue_size=2).start()
            self.assertTrue(links[0].fed.wait(2))
            self.assertEqual(len(codec.inputs), 10)
            self.assertEqual(codec.thread_count, 1)
            self.assertEqual(stream.stats["decoded_frames"], 10)
            self.assertEqual(stream.stats["dropped_frames"], 8)
            frame = stream.read(0)
            self.assertEqual(frame.image.calls, 0)
            self.assertIs(frame.bgr, frame.bgr)
            self.assertEqual(frame.image.calls, 1)
            self.assertEqual(frame.raw_h264, RAW)
            self.assertEqual(frame.metadata["frame_clock_ns_raw"], frame.timestamp_ns)
            self.assertNotIn("quaternions", frame.metadata)
            self.assertTrue(
                all(
                    p.pts == STAMP and p.time_base.denominator == 1_000_000_000
                    for p in codec.inputs
                )
            )
            self.assertTrue(
                all(not list(records_from_video(p.data)) for p in codec.inputs)
            )
            stream.stop()

    def test_header_only_capture_keeps_native_timestamp_through_decoder_mapping(self):
        # The post-resume diagnostic retained the exact SEI, not image pixels.
        # Exercise its metadata/PTS mapping with the fake decoder.
        golden = json.loads(
            (ROOT / "tests/fixtures/preview_sei_resume_empty.json").read_text()
        )
        raw = b"\0\0\0\1" + bytes.fromhex(golden["nal_hex"])
        codec = FakeCodec()
        fake_av = SimpleNamespace(
            CodecContext=SimpleNamespace(create=lambda *_: codec), Packet=FakePacket
        )
        links, patch = self.link_patch(units=[raw])
        with (
            patch,
            mock.patch("osmo360_camera.video.poke"),
            mock.patch.dict(sys.modules, {"av": fake_av}),
        ):
            with VideoStream() as stream:
                self.assertTrue(links[0].fed.wait(2))
                frame = stream.read(0)
                self.assertEqual(frame.timestamp_ns, 198155615260)
                self.assertEqual(frame.image.pts, frame.timestamp_ns)
                self.assertEqual(frame.image.time_base.denominator, 1_000_000_000)
                self.assertEqual(
                    frame.metadata["frame_clock_ns_raw"], frame.timestamp_ns
                )
                self.assertTrue(frame.metadata["motion_header_only"])
                self.assertFalse(frame.metadata["motion_samples_available"])
                self.assertFalse(frame.metadata["imu_clock_available"])
                self.assertNotIn("quaternions", frame.metadata)
                self.assertEqual(frame.raw_h264, raw)
                self.assertEqual(stream.stats["metadata_errors"], 0)
                self.assertEqual(stream.stats["missing_timestamps"], 0)

    def test_missing_clock_evidence_distinguishes_absent_sei_and_explicit_zero_clock(self):
        picture = b'\0\0\1\x41\x12\x34'
        body = bytearray(68)
        struct.pack_into('<II', body, 12, 1, 100)
        zero = sei(struct.pack('<HHBI', 12, 5 + len(body), 1, 0) + body) + picture
        codec = FakeCodec()
        stream = VideoStream(queue_size=2)
        stream._started = True  # Isolate inbound processing; no network thread.
        stream._decoder, stream._av = codec, SimpleNamespace(Packet=FakePacket)
        stream._link = SimpleNamespace(stats={'truncated_pictures': 1, 'video_packets': 20})
        stream._on_video(picture, 0.0)
        stream._on_video(zero, 0.0)
        first, second = stream.read(0), stream.read(0)
        self.assertIsNone(first.timestamp_ns)
        self.assertIsNone(second.timestamp_ns)
        self.assertEqual((first.raw_h264, second.raw_h264), (picture, zero))
        self.assertEqual(len(codec.inputs), 2)
        stats = stream.stats
        self.assertEqual(stats['metadata_errors'], 0)
        self.assertEqual(stats['missing_timestamps'], 2)
        missing = stats['missing_timestamp']
        self.assertEqual(missing['reason'], 'frame_clock_unavailable')
        self.assertEqual(missing['nal_types'], [6, 1])
        self.assertEqual(missing['access_unit_sha256'], hashlib.sha256(zero).hexdigest())
        self.assertFalse(missing['metadata_records'][0]['frame_clock_available'])
        self.assertEqual(missing['network']['truncated_pictures'], 1)
        self.assertGreater(missing['received_monotonic_ns'], 0)
        self.assertEqual(stats['first_missing_timestamp']['reason'], 'no_f5_sei')
        missing['metadata_records'][0]['raw_tlv_hex'] = 'caller mutation'
        self.assertNotEqual(stream.stats['missing_timestamp']['metadata_records'][0]['raw_tlv_hex'], 'caller mutation')

    def test_unknown_motion_version_diagnostic_is_bounded_without_inventing_clock(self):
        body = b'\x55' * 12000
        raw = sei(struct.pack('<HHBI', 12, 5 + len(body), 2, len(body)) + body)
        stream = VideoStream(decode=False)
        stream._started = True  # Isolate inbound processing; no network thread.
        stream._on_video(raw, 0.0)
        self.assertIsNone(stream.read(0).timestamp_ns)
        diagnostic = stream.stats['missing_timestamp']
        self.assertEqual(diagnostic['reason'], 'unsupported_motion_version')
        entry = diagnostic['f5_sei'][0]
        self.assertTrue(entry['truncated'])
        self.assertLessEqual(len(entry['nal_hex']), 16384)
        self.assertEqual(entry['nal_sha256'], hashlib.sha256(raw[4:]).hexdigest())
        self.assertTrue(diagnostic['metadata_records'][0]['raw_tlv_truncated'])
        self.assertLessEqual(len(diagnostic['metadata_records'][0]['raw_tlv_hex']), 8192)
        self.assertEqual(stream.stats['metadata_errors'], 0)

    def test_calibration_survives_raw_queue_eviction_and_caller_mutation(self):
        last = b"\0\0\0\1" + bytes.fromhex(FIXTURE["current"][-1]["nal_hex"])
        links, patch = self.link_patch(units=[RAW, last])
        with patch, mock.patch("osmo360_camera.video.poke"):
            stream = VideoStream(decode=False, queue_size=1)
            self.assertIsNone(stream.calibration)
            stream.start()
            self.assertTrue(links[0].fed.wait(2))
            frame = stream.read(0)
            self.assertNotIn("calibration_hex", frame.metadata)
            calibration = stream.calibration
            self.assertEqual(calibration["calibration_version"], 1)
            self.assertEqual(calibration["calibration_declared_body_length"], 219)
            calibration["calibration_hex"] = "caller changed copy"
            self.assertNotEqual(
                stream.calibration["calibration_hex"], calibration["calibration_hex"]
            )
            stream.stop()
            self.assertEqual(stream.calibration["calibration_body_length"], 287)

    def test_only_first_calibration_is_cached_and_frame_metadata_is_its_own_record(
        self,
    ):
        records = [
            {
                "frame_clock_ns_raw": 1,
                "header_hex": "first",
                "calibration_hex": "aa",
                "calibration_version": 1,
                "quaternions": [[1, 0, 0, 0]],
            },
            {
                "frame_clock_ns_raw": 2,
                "header_hex": "second",
                "calibration_hex": "bb",
                "calibration_version": 99,
                "quaternions": [[0, 1, 0, 0]],
            },
        ]
        links, patch = self.link_patch(units=[RAW, RAW])
        with (
            patch,
            mock.patch("osmo360_camera.video.poke"),
            mock.patch(
                "osmo360_camera.video.records_from_video",
                side_effect=[[r] for r in records],
            ) as parse,
        ):
            stream = VideoStream(decode=False, queue_size=2).start()
            self.assertTrue(links[0].fed.wait(2))
            first, second = stream.read(0), stream.read(0)
            self.assertEqual(parse.call_count, 2)
            self.assertEqual(first.metadata["frame_clock_ns_raw"], 1)
            self.assertEqual(second.metadata["frame_clock_ns_raw"], 2)
            self.assertEqual(second.metadata["calibration_version"], 99)
            second.metadata["calibration_hex"] = "caller changed frame"
            self.assertEqual(
                stream.calibration, {"calibration_hex": "aa", "calibration_version": 1}
            )
            self.assertNotIn("quaternions", first.metadata)
            stream.stop()

    def test_strip_preserves_other_nals_and_original_sei_is_retained(self):
        aud = b"\0\0\1\x09\xf0"
        timing = b"\0\0\0\1\x06\x01\x01\x00\x80"
        picture = b"\0\0\1\x41\x12\x34"
        self.assertEqual(
            _decoder_bytes(aud + RAW + timing + picture), aud + timing + picture
        )
        self.assertEqual(_decoder_bytes(picture), picture)

    def test_read_timeout_and_stop_wakes_waiter(self):
        links, patch = self.link_patch()
        with patch, mock.patch("osmo360_camera.video.poke"):
            stream = VideoStream(decode=False).start()
            self.assertTrue(links[0].fed.wait(2))
            with self.assertRaises(TimeoutError):
                stream.read(0.01)
            results = []

            def read():
                try:
                    stream.read()
                except StopIteration:
                    results.append("stopped")

            waiter = threading.Thread(target=read)
            waiter.start()
            stream.stop()
            waiter.join(2)
            self.assertFalse(waiter.is_alive())
            self.assertEqual(results, ["stopped"])
            self.assertTrue(stream.stop()["stopped"])

    def test_network_and_stop_errors_surface(self):
        for kwargs in (
            {"error": OSError("network failed")},
            {"stop_error": OSError("close failed")},
        ):
            links, patch = self.link_patch(**kwargs)
            with (
                self.subTest(kwargs=kwargs),
                patch,
                mock.patch("osmo360_camera.video.poke"),
            ):
                stream = VideoStream(decode=False).start()
                self.assertTrue(links[0].fed.wait(2))
                if "error" in kwargs:
                    with self.assertRaisesRegex(VideoStreamError, "network failed"):
                        stream.read(2)
                with self.assertRaisesRegex(VideoStreamError, "failed"):
                    stream.stop()
                self.assertTrue(stream.stats["stopped"])

    def test_stop_timeout_keeps_state_and_can_be_retried(self):
        links, patch = self.link_patch(stubborn=True)
        with patch, mock.patch("osmo360_camera.video.poke"):
            stream = VideoStream(decode=False).start()
            self.assertTrue(links[0].fed.wait(2))
            try:
                with self.assertRaisesRegex(VideoStreamError, "did not stop"):
                    stream.stop(0.01)
            finally:
                links[0].release.set()
            self.assertTrue(stream.stop()["stopped"])

    def test_invalid_metadata_fails_instead_of_inventing_a_timestamp(self):
        malformed = b"\0\0\1\x06\xf5\x04\x0c\x00\xff\x7f\x80"
        links, patch = self.link_patch(units=[malformed])
        with patch, mock.patch("osmo360_camera.video.poke"):
            stream = VideoStream(decode=False).start()
            self.assertTrue(links[0].fed.wait(2))
            with self.assertRaisesRegex(VideoStreamError, "timestamp metadata"):
                stream.read(2)
            self.assertEqual(stream.stats["metadata_errors"], 1)
            self.assertEqual(stream.stats["error_cause"], "ValueError: Неполный TLV 12")
            self.assertIn("Неполный TLV 12", stream.stats["error"])
            with self.assertRaises(VideoStreamError):
                stream.stop()

    def test_metadata_diagnostic_isolates_exact_bad_sei_and_keeps_validation_strict(
        self,
    ):
        bad = b"\x06\xf5\x04\x0c\x00\xff\x7f\x80"
        picture = b"\0\0\1\x41\x12\x34"
        unit = RAW + b"\0\0\1" + bad + picture
        stream = VideoStream(decode=False)
        with self.assertRaises(VideoStreamError) as caught:
            stream._on_video(unit, 0.0)
        self.assertIsInstance(caught.exception.__cause__, ValueError)
        self.assertEqual(str(caught.exception.__cause__), "Неполный TLV 12")
        diagnostic = stream.stats["metadata_error"]
        self.assertEqual(diagnostic["access_unit_bytes"], len(unit))
        self.assertEqual(
            diagnostic["access_unit_sha256"], hashlib.sha256(unit).hexdigest()
        )
        self.assertEqual(diagnostic["offending_sei"]["nal_offset"], len(RAW))
        self.assertEqual(diagnostic["offending_sei"]["nal_hex"], bad.hex())
        self.assertFalse(diagnostic["offending_sei"]["truncated"])
        self.assertEqual(
            diagnostic["offending_sei"]["nal_sha256"], hashlib.sha256(bad).hexdigest()
        )
        self.assertNotIn(picture.hex(), diagnostic["offending_sei"]["nal_hex"])
        self.assertEqual(
            (stream.stats["published_frames"], stream.stats["missing_timestamps"]),
            (0, 0),
        )
        diagnostic["offending_sei"]["nal_hex"] = "caller mutation"
        self.assertEqual(
            stream.stats["metadata_error"]["offending_sei"]["nal_hex"], bad.hex()
        )

    def test_conflicting_clocks_retain_both_valid_seis_without_accepting_frame(self):
        last = b"\0\0\0\1" + bytes.fromhex(FIXTURE["current"][-1]["nal_hex"])
        stream = VideoStream(decode=False)
        with self.assertRaisesRegex(VideoStreamError, "Multiple frame timestamps"):
            stream._on_video(RAW + last, 0.0)
        diagnostic = stream.stats["metadata_error"]
        self.assertNotIn("offending_sei", diagnostic)
        entries = diagnostic["timestamp_sei"]
        self.assertEqual(len(entries), 2)
        self.assertEqual(entries[0]["frame_clocks_ns_raw"], [STAMP])
        self.assertNotEqual(
            entries[0]["frame_clocks_ns_raw"], entries[1]["frame_clocks_ns_raw"]
        )
        self.assertEqual(stream.stats["published_frames"], 0)

    def test_oversized_sei_diagnostic_marks_truncation_and_never_masks_original_error(
        self,
    ):
        nal = b"\x06\xf5\x01" + b"\x55" * 9000 + b"\x80"
        stream = VideoStream(decode=False)
        with self.assertRaises(VideoStreamError) as caught:
            stream._on_video(b"\0\0\1" + nal, 0.0)
        self.assertIsInstance(caught.exception.__cause__, ValueError)
        entry = stream.stats["metadata_error"]["timestamp_sei"][0]
        self.assertTrue(entry["truncated"])
        self.assertEqual(entry["nal_bytes"], len(nal))
        self.assertEqual(entry["nal_hex"], nal[:8192].hex())
        self.assertEqual(entry["nal_sha256"], hashlib.sha256(nal).hexdigest())
        self.assertEqual(stream.stats["published_frames"], 0)
        with mock.patch(
            "osmo360_camera.video._metadata_diagnostic",
            side_effect=RuntimeError("diagnostic failed"),
        ):
            with self.assertRaises(VideoStreamError) as caught:
                stream._on_video(b"\0\0\1" + nal, 0.0)
        self.assertIsInstance(caught.exception.__cause__, ValueError)
        self.assertIn("Неизвестный формат длины SEI", str(caught.exception.__cause__))
        self.assertIn(
            "diagnostic failed", stream.stats["metadata_error"]["diagnostic_error"]
        )

    def test_validation_and_not_started_read(self):
        for kwargs in (
            {"queue_size": 0},
            {"queue_size": True},
            {"decode": 1},
            {"host": ""},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                VideoStream(**kwargs)
        stream = VideoStream()
        with self.assertRaises(VideoStreamError):
            stream.read(0)
        stream.stop()
        with self.assertRaises(VideoStreamError):
            stream.start()

    def test_facade_binding_options_reach_udp_and_tcp(self):
        links, patch = self.link_patch()
        with patch as factory, mock.patch("osmo360_camera.video.poke") as poke:
            stream = VideoStream(
                decode=False, local_address="192.168.253.2", interface="en9"
            ).start()
            self.assertTrue(links[0].fed.wait(2))
            self.assertEqual(factory.call_args.kwargs["local_address"], "192.168.253.2")
            self.assertEqual(factory.call_args.kwargs["interface"], "en9")
            poke.assert_called_once_with(
                "192.168.253.1", local_address="192.168.253.2", interface="en9"
            )
            stream.stop()

    @skipUnless(importlib.util.find_spec("av"), "Optional PyAV dependency is absent")
    def test_real_decoder_maps_native_sei_and_decodes_before_queue_eviction(self):
        # Encode generated gray images, never replay private camera pixels.
        # Native SEI is a small protocol golden independent of image content.
        import av
        from fractions import Fraction

        try:
            encoder = av.CodecContext.create("libx264", "w")
        except av.codec.codec.UnknownCodecError:
            self.skipTest("PyAV has no libx264 encoder for synthetic H.264 input")
        encoder.width, encoder.height = 32, 16
        encoder.pix_fmt = "yuv420p"
        encoder.time_base = Fraction(1, 30)
        encoder.options = {
            "preset": "ultrafast",
            "tune": "zerolatency",
            "x264-params": "keyint=1:bframes=0",
        }
        packets = []
        for index in range(8):
            image = av.VideoFrame(32, 16, "yuv420p")
            image.pts, image.time_base = index, encoder.time_base
            for plane_index, plane in enumerate(image.planes):
                value = 16 + index * 16 if plane_index == 0 else 128
                plane.update(bytes((value,)) * plane.buffer_size)
            packets.extend(encoder.encode(image))
        packets.extend(encoder.encode(None))
        self.assertEqual(len(packets), 8)
        units = [RAW + bytes(packet) for packet in packets]
        links, patch = self.link_patch(units=units)
        with patch, mock.patch("osmo360_camera.video.poke"):
            stream = VideoStream(queue_size=2).start()
            try:
                self.assertTrue(links[0].fed.wait(5))
                stats = stream.stats
                self.assertEqual(stats["encoded_units"], 8)
                self.assertEqual(stats["decoded_frames"], 8)
                self.assertEqual(stats["dropped_frames"], 6)
                self.assertEqual(stats["queued_frames"], 2)
                self.assertEqual(stats["missing_timestamps"], 0)
                self.assertEqual(stats["metadata_errors"], 0)
                calibration = stream.calibration
                self.assertEqual(calibration["calibration_version"], 1)
                self.assertEqual(calibration["calibration_body_length"], 287)
                for frame in (stream.read(0), stream.read(0)):
                    self.assertEqual(frame.timestamp_ns, STAMP)
                    self.assertEqual(frame.bgr.shape, (16, 32, 3))
                    self.assertEqual(frame.image.pts, frame.timestamp_ns)
                    self.assertEqual(frame.image.time_base.denominator, 1_000_000_000)
                    self.assertEqual(frame.metadata["frame_clock_ns_raw"], STAMP)
                    self.assertTrue(list(records_from_video(frame.raw_h264)))
                    self.assertNotIn("quaternions", frame.metadata)
            finally:
                stream.stop()


class ExplicitBindingTests(TestCase):
    def test_partial_assembled_picture_exposes_loss_before_video_callback(self):
        # Exercise the actual assembler's loss path without opening a socket.
        link = datalink.Datalink.__new__(datalink.Datalink)
        link.stats = {}
        snapshots = []
        link.on_video = lambda raw, first: snapshots.append(dict(link.stats))
        link.frames = datalink._FrameAssembler(link._deliver_video)
        link.frames._synced = True
        link.frames._data.extend(b'\0\0\1\x41\x12\x34')
        with mock.patch.object(datalink, 'log'):
            link.frames.finish(short=True)
        self.assertEqual(snapshots, [{'truncated_pictures': 1, 'keyframes': 0}])
        self.assertEqual(link.stats['truncated_pictures'], link.frames.truncated)

    def test_darwin_interface_option_precedes_explicit_source_bind(self):
        sock = mock.Mock()
        with (
            mock.patch.object(util.sys, "platform", "darwin"),
            mock.patch.object(socket, "if_nametoindex", return_value=62),
        ):
            util.bind_socket(sock, "192.168.253.2", "en9")
        self.assertEqual(
            sock.method_calls,
            [
                mock.call.setsockopt(
                    socket.IPPROTO_IP, getattr(socket, "IP_BOUND_IF", 25), 62
                ),
                mock.call.bind(("192.168.253.2", 0)),
            ],
        )

    def test_darwin_prefers_exported_constant_and_linux_uses_its_own_constant(self):
        for platform, option_name, option, value in (
            ("darwin", "IP_BOUND_IF", 9191, 62),
            ("linux", "SO_BINDTODEVICE", 8181, b"en9\0"),
        ):
            sock = mock.Mock()
            with (
                self.subTest(platform=platform),
                mock.patch.object(util.sys, "platform", platform),
                mock.patch.object(socket, option_name, option, create=True),
                mock.patch.object(socket, "if_nametoindex", return_value=62),
            ):
                util.bind_socket(sock, "192.168.253.2", "en9")
            level = socket.IPPROTO_IP if platform == "darwin" else socket.SOL_SOCKET
            sock.setsockopt.assert_called_once_with(level, option, value)
            sock.bind.assert_called_once_with(("192.168.253.2", 0))

    def test_denied_or_unsupported_interface_never_falls_back_to_unbound(self):
        for platform, error in (
            ("darwin", PermissionError("denied")),
            ("unsupported", None),
        ):
            sock = mock.Mock()
            sock.setsockopt.side_effect = error
            with (
                self.subTest(platform=platform),
                mock.patch.object(util.sys, "platform", platform),
                mock.patch.object(socket, "if_nametoindex", return_value=62),
            ):
                with self.assertRaises(OSError):
                    util.bind_socket(sock, "192.168.253.2", "en9")
            sock.bind.assert_not_called()

    def test_bound_tcp_order_is_interface_source_connect_send(self):
        sock = mock.MagicMock()
        sock.__enter__.return_value = sock
        with (
            mock.patch.object(datalink.socket, "socket", return_value=sock),
            mock.patch.object(datalink.socket, "create_connection") as fallback,
            mock.patch.object(util.sys, "platform", "darwin"),
            mock.patch.object(socket, "if_nametoindex", return_value=62),
            mock.patch.object(datalink.time, "sleep"),
            mock.patch.object(datalink, "log"),
        ):
            datalink.poke(
                "192.168.253.1", local_address="192.168.253.2", interface="en9"
            )
        calls = [c[0] for c in sock.method_calls]
        self.assertEqual(
            calls[:5], ["settimeout", "setsockopt", "bind", "connect", "sendall"]
        )
        sock.bind.assert_called_once_with(("192.168.253.2", 0))
        sock.connect.assert_called_once_with(("192.168.253.1", 7001))
        fallback.assert_not_called()

    def test_udp_constructor_closes_after_binding_error(self):
        sock = mock.Mock()
        with (
            mock.patch.object(datalink.socket, "socket", return_value=sock),
            mock.patch.object(util.sys, "platform", "unsupported"),
            mock.patch.object(socket, "if_nametoindex", return_value=62),
        ):
            with self.assertRaises(OSError):
                datalink.Datalink(local_address="192.168.253.2", interface="en9")
        sock.close.assert_called_once()
        sock.bind.assert_not_called()

    def test_tcp_binding_error_closes_without_connect_or_fallback(self):
        sock = mock.Mock()
        with (
            mock.patch.object(datalink.socket, "socket", return_value=sock),
            mock.patch.object(datalink.socket, "create_connection") as fallback,
            mock.patch.object(util.sys, "platform", "unsupported"),
            mock.patch.object(socket, "if_nametoindex", return_value=62),
        ):
            with self.assertRaises(OSError):
                datalink.poke(
                    "192.168.253.1", local_address="192.168.253.2", interface="en9"
                )
        sock.close.assert_called_once()
        sock.connect.assert_not_called()
        fallback.assert_not_called()
