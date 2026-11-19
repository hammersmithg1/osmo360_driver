"""Archival AUs survive stalled/overloaded preview; no device is opened."""
import json
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

from osmo360_camera.video import VideoStream, VideoFrame
from osmo360_driver.capture_archive import ArchiveVideo
from osmo360_driver.acquisition import raw_video_acquisition, materialize_acquisition
from test_video_stream import FakeLink, FakeCodec, FakePacket, clocked_unit
from test_ros_topics import load_fake_ros, FakeNode

HEADERS = b'\x00\x00\x00\x01\x67abc\x00\x00\x00\x01\x68def'
IDR = b'\x00\x00\x00\x01\x65\x88'
P = b'\x00\x00\x00\x01\x41\x88'


def frame(stamp, *, key=False, size=1):
    return VideoFrame(stamp, stamp + 1000, None, b'x' * size, {
        'h264_keyframe': key, 'h264_codec_headers_complete': True,
        'h264_codec_headers_hex': HEADERS.hex(),
    })


class RawCaptureTests(unittest.TestCase):
    def test_stalled_preview_decoder_never_blocks_or_drops_raw_admission(self):
        release = threading.Event()
        entered = threading.Event()
        raw = []
        archived = []
        archive = ArchiveVideo(lambda f, stamp: archived.append((f.raw_h264, stamp)))
        boundary = threading.Event()
        admitted = threading.Event()

        def receive(frame):
            raw.append(frame)
            archive.append(frame, frame.timestamp_ns)
            if len(raw) == 1:
                boundary.set()
                if not admitted.wait(3):
                    raise TimeoutError('test archive admission')

        codec = FakeCodec()
        original = codec.decode

        def slow(packet):
            entered.set()
            if not release.wait(3):
                raise RuntimeError('test decoder stalled')
            return original(packet)
        codec.decode = slow
        units = [clocked_unit(1000000 + n, (HEADERS + IDR) if n == 0 else P)
                 for n in range(1000)]
        link = None

        def factory(**kwargs):
            nonlocal link
            link = FakeLink(**kwargs, units=units)
            return link
        av = SimpleNamespace(CodecContext=SimpleNamespace(create=lambda *_: codec), Packet=FakePacket)
        with mock.patch('osmo360_camera.video.Datalink', side_effect=factory), \
             mock.patch('osmo360_camera.video.poke'), mock.patch.dict(sys.modules, {'av': av}):
            stream = VideoStream(queue_size=1, on_access_unit=receive).start()
            try:
                self.assertTrue(boundary.wait(2))
                archive.begin(lambda: self.fail('first IDR is already cached'))
                admitted.set()
                self.assertTrue(link.fed.wait(2), 'network producer waited for decoder')
                self.assertEqual([f.raw_h264 for f in raw], units)
                self.assertEqual([payload for payload, _ in archived], units)
                self.assertEqual([stamp for _, stamp in archived], list(range(1000000, 1001000)))
                self.assertEqual([f.timestamp_ns for f in raw], list(range(1000000, 1001000)))
                self.assertTrue(all(f.image is None for f in raw))
                self.assertEqual(stream.stats['raw_callback_units'], 1000)
                self.assertGreater(stream.stats['preview_decoder']['dropped_units'], 0)
                self.assertLessEqual(stream.stats['preview_decoder']['queued_units'], 8)
                self.assertEqual(stream.stats['raw_callback_errors'], 0)
                self.assertTrue(raw[0].metadata['h264_keyframe'])
                self.assertEqual(raw[-1].metadata['h264_codec_headers_hex'], HEADERS.hex())
            finally:
                admitted.set()
                release.set()
                stream.stop()

    def test_raw_admission_runs_before_any_decoder_and_losing_output_queue(self):
        saved = []
        stream = VideoStream(decode=False, queue_size=1, on_access_unit=saved.append)
        for number in range(20):
            stream._on_video(clocked_unit(number + 1, HEADERS + IDR), 0)
        self.assertEqual(len(saved), 20)
        self.assertEqual(stream.stats['dropped_frames'], 19)
        self.assertEqual([x.timestamp_ns for x in saved], list(range(1, 21)))

    def test_paused_raw_units_cannot_seed_a_new_capture_epoch(self):
        saved = []
        stream = VideoStream(decode=False, on_access_unit=saved.append)
        stream._publication_paused = True
        stream._on_video(clocked_unit(1, HEADERS + IDR), 0)
        self.assertEqual(saved, [])
        self.assertEqual(stream.stats['raw_suppressed_units'], 1)
        stream._publication_paused = False
        stream._raw_publication_epoch += 1
        stream._on_video(clocked_unit(2, HEADERS + IDR), 0)
        self.assertEqual([f.timestamp_ns for f in saved], [2])

    def test_pause_waits_for_an_inflight_raw_callback_before_returning_barrier(self):
        entered, release, paused = threading.Event(), threading.Event(), threading.Event()
        def sink(_):
            entered.set()
            self.assertTrue(release.wait(2))
        stream = VideoStream(decode=False, on_access_unit=sink)
        stream._started = True
        stream._link = SimpleNamespace(pause=mock.Mock(return_value=True), stats={})
        producer = threading.Thread(target=stream._on_video, args=(clocked_unit(1, HEADERS + IDR), 0))
        producer.start()
        self.assertTrue(entered.wait(1))
        results = []
        def pause():
            results.append(stream.pause(timeout=2))
            paused.set()
        waiter = threading.Thread(target=pause)
        waiter.start()
        self.assertFalse(paused.wait(.02))
        stream._link.pause.assert_not_called()
        release.set()
        producer.join(1)
        waiter.join(1)
        self.assertEqual(results, [True])
        self.assertEqual(stream.stats['raw_callbacks_inflight'], 0)
        stream._on_video(clocked_unit(2, HEADERS + IDR), 0)
        self.assertEqual(stream.stats['raw_suppressed_units'], 1)

    def test_raw_callback_error_is_not_silently_counted_as_success(self):
        def fail(_):
            raise BufferError('archive queue full')
        stream = VideoStream(decode=False, on_access_unit=fail)
        with self.assertRaisesRegex(BufferError, 'queue full'):
            stream._on_video(clocked_unit(1, HEADERS + IDR), 0)
        self.assertEqual(stream.stats['raw_callback_units'], 0)
        self.assertEqual(stream.stats['raw_callback_errors'], 1)

    def test_conflicting_native_clock_metadata_fails_before_claiming_raw_admission(self):
        saved = []
        stream = VideoStream(decode=False, on_access_unit=saved.append)
        from osmo360_camera.video import VideoStreamError
        with self.assertRaisesRegex(VideoStreamError, 'Multiple frame timestamps'):
            stream._on_video(clocked_unit(1, IDR) + clocked_unit(2, IDR), 0)
        self.assertEqual(saved, [])
        self.assertEqual(stream.stats['metadata_errors'], 1)
        self.assertEqual(stream.stats['raw_callback_units'], 0)

    def test_raw_ros_admission_is_immutable_and_never_encodes_jpeg_or_serializes_ros(self):
        topics_type, created = load_fake_ros()
        saved = []
        topics = topics_type(FakeNode(), acquisition_sink=lambda *args: saved.append(args),
                             raw_video_archive=True)
        source = frame(10, key=True)
        topics.video_raw(source, 100)
        topics.video_lazy(source, mock.Mock(side_effect=AssertionError('JPEG is not archival')), 100)
        self.assertEqual(created, {})
        self.assertEqual(len(saved), 1)
        dto = saved[0][1]
        self.assertEqual(dto[:6], ('video_raw', 100, 10, 1010, 0, b'x'))
        types = topics.message_types
        entries = materialize_acquisition(dto, message_types=types)
        self.assertEqual({row[0] for row in entries}, {
            '/osmo360/video/h264', '/osmo360/video/timing', '/osmo360/video/metadata'})
        self.assertEqual(len(entries), 3)
        source.metadata['h264_keyframe'] = False
        self.assertTrue(json.loads(dto[-1])['h264_keyframe'])
        source.timestamp_ns = None
        self.assertEqual(len(materialize_acquisition(raw_video_acquisition(source, 101, 1),
                                                   message_types=types)), 3)


class ArchiveBoundaryTests(unittest.TestCase):
    def test_cached_idr_preroll_then_live_is_one_ordered_sequence(self):
        saved = []
        archive = ArchiveVideo(lambda f, t: saved.append((f.timestamp_ns, t)))
        archive.append(frame(1, key=True), 101)
        archive.append(frame(2), 102)
        self.assertEqual(saved, [])
        result = archive.begin(lambda: self.fail('valid cache does not need another IDR'))
        archive.append(frame(3), 103)
        self.assertEqual(saved, [(1, 101), (2, 102), (3, 103)])
        self.assertTrue(result['independently_decodable'])
        self.assertEqual(result['first_stamp_ns'], 101)
        self.assertEqual(result['preroll_frames'], 2)
        self.assertEqual(result['codec_headers_hex'], HEADERS.hex())
        archive.end()
        archive.append(frame(4), 104)
        self.assertEqual(len(saved), 3)

    def test_bounded_cache_requests_and_uses_a_new_idr_without_fake_timestamps(self):
        saved = []
        archive = ArchiveVideo(lambda f, t: saved.append((f.timestamp_ns, t)), max_bytes=3)
        archive.append(frame(1, key=True, size=2), 101)
        archive.append(frame(2, size=2), 102)
        self.assertFalse(archive.status()['decodable_boundary'])
        self.assertEqual(archive.status()['cached_bytes'], 0)
        result = archive.begin(lambda: archive.append(frame(3, key=True), 103))
        self.assertEqual(saved, [(3, 103)])
        self.assertEqual(result['first_native_ns'], 3)
        archive.reset()
        self.assertFalse(archive.status()['active'])
        self.assertEqual(archive.status()['cached_bytes'], 0)

    def test_end_is_a_barrier_for_inflight_sink_before_a_next_session(self):
        entered, release, sealing, sealed = [threading.Event() for _ in range(4)]
        destination = ['first']
        saved = []
        def sink(source, stamp):
            if source.timestamp_ns == 2:
                entered.set()
                self.assertTrue(release.wait(2))
            saved.append((destination[0], source.timestamp_ns))
        archive = ArchiveVideo(sink)
        archive.append(frame(1, key=True), 101)
        archive.begin(lambda: None)
        producer = threading.Thread(target=archive.append, args=(frame(2), 102))
        producer.start()
        self.assertTrue(entered.wait(1))
        def end():
            sealing.set()
            archive.end()
            sealed.set()
        waiter = threading.Thread(target=end)
        waiter.start()
        self.assertTrue(sealing.wait(1))
        self.assertFalse(sealed.wait(.02))
        release.set()
        producer.join(1)
        waiter.join(1)
        self.assertTrue(sealed.is_set())
        destination[0] = 'second'
        archive.append(frame(3, key=True), 103)
        archive.begin(lambda: None)
        archive.append(frame(4), 104)
        self.assertEqual(saved, [('first', 1), ('first', 2), ('second', 3), ('second', 4)])

    def test_no_idr_or_incomplete_headers_fails_archive_start_explicitly(self):
        archive = ArchiveVideo(lambda *_: self.fail('no undecodable preroll'))
        incomplete = frame(1, key=True)
        incomplete.metadata['h264_codec_headers_complete'] = False
        archive.append(incomplete, 101)
        with self.assertRaisesRegex(TimeoutError, 'IDR boundary'):
            archive.begin(lambda: None, timeout=.001)
        self.assertFalse(archive.status()['active'])


class RuntimeRawTests(unittest.TestCase):
    def owner(self, sink):
        from osmo360_driver.runtime import OsmoCamera
        with mock.patch('osmo360_driver.runtime.threading.Thread'):
            owner = OsmoCamera(on_video_raw=sink, on_error=mock.Mock(), host_offset_ns=10000)
        return owner

    def test_encoded_pts_reordering_and_raw_preview_share_one_clock_offset(self):
        owner = self.owner(lambda *_: None)
        owner.raw_video(frame(100, key=True))
        first = owner.clock.map_unordered(100, 1100)
        owner.raw_video(frame(80))  # Decode order is allowed to differ from PTS.
        self.assertEqual(owner.clock.map_unordered(80, 2000), first - 20)
        self.assertEqual(owner.clock.stamp(100, 3000, 'video'), first)
        self.assertEqual(owner.clock.stamp(90, 4000, 'imu'), first - 10)

    def test_sink_failure_notifies_source_and_does_not_call_usb_lifecycle_stats(self):
        def fail(*_):
            raise BufferError('raw archive cannot admit AU')
        owner = self.owner(fail)
        owner.raw_video(frame(100, key=True))
        with self.assertRaisesRegex(BufferError, 'cannot admit'):
            owner.archive.begin(lambda: None)
        self.assertFalse(owner.archive.status()['active'])
        # A running sink failure must also fail the owning capture immediately.
        owner.archive.sink = lambda *_: None
        owner.archive.begin(lambda: None)
        owner.archive.sink = fail
        owner.failure_stats = mock.Mock(side_effect=AssertionError('network callback must not query USB'))
        with self.assertRaises(BufferError):
            owner.raw_video(frame(101))
        owner.on_error.assert_called_once_with('raw archive cannot admit AU')
        owner.failure_stats.assert_not_called()
        self.assertTrue(owner.reader_stop.is_set())
        self.assertFalse(owner.enabled)
        self.assertIn('cannot admit', owner.capture_stats['failure']['capture']['message'])

    def test_missing_native_timestamp_is_counted_once_across_raw_and_preview(self):
        owner = self.owner(lambda *_: None)
        source = frame(1, key=True)
        source.timestamp_ns = None
        owner.raw_video(source)
        self.assertEqual(owner.timestamp_missing_frames, 1)
        owner.on_video_lazy = mock.Mock()
        def read_video(**_):
            owner.reader_stop.set()
            return source
        owner.stream = SimpleNamespace(read_video=read_video, stats={})
        encoder = SimpleNamespace(for_frame=lambda _: SimpleNamespace(
            full=lambda: b'full', preview=lambda: b'preview'))
        with mock.patch('osmo360_driver.runtime.PreviewEncoder', return_value=encoder):
            owner.video_loop()
        self.assertEqual(owner.timestamp_missing_frames, 1)
        owner.on_video_lazy.assert_called_once()

    def test_configure_raw_callback_is_allowed_only_before_producer_exists(self):
        owner = self.owner(lambda *_: None)
        sink = mock.Mock()
        owner.configure_raw_archive(sink)
        self.assertIs(owner.archive.sink, sink)
        for enabled, stream in ((True, None), (False, object())):
            owner.enabled, owner.stream = enabled, stream
            with self.assertRaisesRegex(ValueError, 'before starting capture'):
                owner.configure_raw_archive(lambda *_: None)
