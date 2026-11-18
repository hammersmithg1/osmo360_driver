"""Offline real PyAV/OpenCV checks for preview/archival separation."""

from fractions import Fraction
import importlib.util
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera.video import VideoFrame
from osmo360_driver.jpeg import PreviewEncoder


@unittest.skipUnless(all(importlib.util.find_spec(name) for name in ('av', 'cv2', 'numpy')),
                     'Optional PyAV/OpenCV/NumPy dependencies are absent')
class PreviewTests(unittest.TestCase):
    def setUp(self):
        import av
        import cv2
        import numpy as np
        self.av, self.cv2, self.np = av, cv2, np
        self.encoder = PreviewEncoder()

    def frame(self, shift=0):
        # Detailed, deterministic colour content; no USB or native decoder.
        np = self.np
        y, x = np.indices((1024, 2048), dtype=np.uint16)
        pixels = np.stack(((x + y + shift) % 256, (3*x + shift) % 256,
                           (5*y + shift) % 256), axis=-1).astype(np.uint8)
        image = self.av.VideoFrame.from_ndarray(pixels, format='bgr24').reformat(format='yuv420p')
        image.pts, image.time_base = 81234567890123 + shift, Fraction(1, 1_000_000_000)
        return VideoFrame(100 + shift, 200 + shift, image, b'original H264', {'native': b'\x00\xff'})

    def fingerprint(self, frame):
        return (frame.timestamp_ns, frame.host_receive_monotonic_ns, frame.raw_h264,
                dict(frame.metadata), frame.image.pts, frame.image.time_base,
                tuple((name, getattr(frame.image, name)) for name in
                      ('colorspace', 'color_range', 'color_primaries', 'color_trc')
                      if hasattr(frame.image, name)),
                tuple(bytes(plane) for plane in frame.image.planes))

    def decode(self, jpeg):
        result = self.cv2.imdecode(self.np.frombuffer(jpeg, self.np.uint8), self.cv2.IMREAD_COLOR)
        self.assertIsNotNone(result)
        return result

    def encode(self, bgr, quality):
        ok, encoded = self.cv2.imencode('.jpg', bgr, [self.cv2.IMWRITE_JPEG_QUALITY, quality])
        self.assertTrue(ok)
        return encoded.tobytes()

    def test_live_only_downscales_native_without_full_bgr_or_timestamp_mutation(self):
        first = self.frame()
        before = self.fingerprint(first)
        jpegs = self.encoder.for_frame(first)
        preview = jpegs.preview()
        self.assertIs(first._bgr, None)
        self.assertIs(jpegs._full_bgr, None)
        self.assertIs(jpegs._full_jpeg, None)
        self.assertEqual(self.decode(preview).shape, (512, 1024, 3))
        self.assertIs(jpegs.preview(), preview)
        self.assertEqual(self.fingerprint(first), before)
        reformatter = self.encoder.reformatter
        second = self.frame(32)
        self.assertNotEqual(self.encoder.for_frame(second).preview(), preview)
        self.assertIs(self.encoder.reformatter, reformatter)
        self.assertIs(second._bgr, None)

    def test_archive_after_preview_is_byte_identical_to_legacy_full_jpeg92(self):
        frame = self.frame()
        before = self.fingerprint(frame)
        expected = self.encode(frame.image.to_ndarray(format='bgr24'), 92)
        jpegs = self.encoder.for_frame(frame)
        jpegs.preview()
        with patch.object(self.encoder, 'encode', wraps=self.encoder.encode) as encode:
            actual = jpegs.full()
            self.assertIs(jpegs.full(), actual)
            encode.assert_called_once()
            self.assertEqual(encode.call_args.args[1], 92)
        self.assertEqual(actual, expected)
        self.assertEqual(self.decode(actual).shape, (1024, 2048, 3))
        self.assertIs(jpegs._full_bgr, frame._bgr)
        self.assertEqual(self.fingerprint(frame), before)

    def test_archive_first_reuses_full_bgr_for_small_preview_and_keeps_quality75(self):
        frame = self.frame()
        jpegs = self.encoder.for_frame(frame)
        full = jpegs.full()
        expected_small = self.cv2.resize(frame.bgr, (1024, 512), interpolation=self.cv2.INTER_AREA)
        with patch.object(self.encoder, 'small_bgr', side_effect=AssertionError('Do not convert native twice')):
            preview = jpegs.preview()
        self.assertEqual(preview, self.encode(expected_small, 75))
        self.assertEqual(full, self.encode(frame.bgr, 92))
        self.assertLess(len(preview), len(full))
        self.assertIs(self.encoder.reformatter, None)

    def test_existing_public_bgr_cache_is_reused_without_forcing_full_jpeg(self):
        frame = self.frame()
        bgr = frame.bgr
        with patch.object(self.encoder, 'small_bgr', side_effect=AssertionError('BGR is already available')):
            jpegs = self.encoder.for_frame(frame)
            self.assertEqual(self.decode(jpegs.preview()).shape, (512, 1024, 3))
        self.assertIs(frame.bgr, bgr)
        self.assertIs(jpegs._full_jpeg, None)

    def test_bgr_only_adapter_keeps_legacy_full_path_and_small_live_view(self):
        bgr = self.frame().bgr
        jpegs = self.encoder.for_frame(SimpleNamespace(bgr=bgr))
        preview = jpegs.preview()
        self.assertEqual(self.decode(preview).shape, (512, 1024, 3))
        self.assertEqual(jpegs.full(), self.encode(bgr, 92))

    def test_failed_jpeg_encoding_remains_an_error(self):
        with patch.object(self.cv2, 'imencode', return_value=(False, None)):
            with self.assertRaisesRegex(RuntimeError, 'JPEG Osmo'):
                self.encoder.for_frame(self.frame()).preview()
