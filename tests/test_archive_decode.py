"""Real codec round trips preserve acquisition times, including B-frame order."""
from fractions import Fraction
import unittest

try:
    import av
except ImportError:
    av = None

from osmo360_driver.archive import decode_h264_records


@unittest.skipUnless(av, 'PyAV required for codec integration')
class ArchiveDecodeTests(unittest.TestCase):
    def records(self, count=24):
        encoder = av.CodecContext.create('libx264', 'w')
        encoder.width, encoder.height, encoder.pix_fmt = 64, 48, 'yuv420p'
        encoder.time_base = Fraction(1, 30)
        encoder.options = {'preset': 'ultrafast', 'x264-params': 'keyint=6:bframes=2:repeat-headers=1'}
        packets = []
        for i in range(count):
            frame = av.VideoFrame(64, 48, 'yuv420p')
            for plane in frame.planes:
                plane.update(bytes([64 + i]) * plane.buffer_size)
            frame.pts, frame.time_base = i, Fraction(1, 30)
            packets.extend(encoder.encode(frame))
        packets.extend(encoder.encode(None))
        origin = 1_795_098_000_000_000_000
        return [(bytes(packet), origin + packet.pts * 33_333_333) for packet in packets]

    def test_all_frames_and_exact_times_survive_reordered_h264(self):
        records = self.records()
        expected = sorted(stamp for _, stamp in records)
        frames = list(decode_h264_records(records))
        self.assertEqual([stamp for _, stamp in frames], expected)
        self.assertEqual(len(frames), 24)
        self.assertTrue(all((f.width, f.height) == (64, 48) for f, _ in frames))

    def test_truncated_non_keyframe_start_is_not_silently_called_valid(self):
        with self.assertRaises(Exception):
            list(decode_h264_records(self.records()[2:4]))

    def test_invalid_timestamp_is_rejected(self):
        with self.assertRaises(ValueError):
            list(decode_h264_records([(b'', -1)]))
