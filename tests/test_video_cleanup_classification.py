"""Historical capture faults never substitute for local/native cleanup proof."""

from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera.video import VideoCaptureStoppedError, VideoStream, VideoStreamError
from osmo360_camera.usb_preview import USBPreviewError
from osmo360_camera.imu import IMUStopError
from osmo360_camera._stream_recovery import _VerifiedRebootProof
from test_usb_preview import FakeIMU, FaultedIMU, harness, runtime
from test_video_stream import FakeLink


BAD_AU = b'\0\0\1\x06\xf5\x04\x0c\x00\xff\x7f\x80'


@contextmanager
def failed_video():
    """Run the real producer/parser; replace only its network transport."""
    links = []

    def create(**options):
        link = FakeLink(**options, units=[BAD_AU])
        links.append(link)
        return link

    with patch('osmo360_camera.video.Datalink', side_effect=create), patch('osmo360_camera.video.poke'):
        stream = VideoStream(decode=False).start()
        try:
            if not links[0].fed.wait(2):
                raise AssertionError('Fake transport did not feed its malformed AU')
            stream._thread.join(2)
            if stream._thread.is_alive():
                raise AssertionError('Failed video producer remained alive')
            yield stream, links[0]
        finally:
            links[0].release.set()
            links[0].stop_error = None
            stream._preview_decoder = None
            try:
                stream.stop()
            except VideoStreamError:
                pass


def reboot_proof(*, preview=None, imu=None):
    camera = preview.camera if preview is not None else SimpleNamespace()
    transport = getattr(camera, '_transport', None) or SimpleNamespace()
    transport.connected, transport._thread = True, object()
    camera._transport = transport
    coordinator = SimpleNamespace(camera=camera, profile='verified-source-profile',
        report=dict(new_boot_verified=True, disconnect_observed=True), _check_pins=Mock())
    proof = _VerifiedRebootProof(camera, transport, transport._thread,
        '11111111-1111-1111-1111-111111111111', '22222222-2222-2222-2222-222222222222',
        coordinator.profile, 10., (preview,) if preview is not None else (),
        (imu,) if imu is not None else (), coordinator)
    coordinator._proof = proof
    return proof


class VideoStopClassificationTests(unittest.TestCase):
    def test_real_capture_error_still_surfaces_with_exact_owner_local_receipt(self):
        with failed_video() as (stream, _):
            with self.assertRaisesRegex(VideoStreamError, 'timestamp metadata'):
                stream.read(0)
            with self.assertRaises(VideoCaptureStoppedError) as caught:
                stream.stop()
            error = caught.exception
            self.assertIs(error.stream, stream)
            self.assertIs(error.capture_error, stream._capture_error)
            self.assertIs(error.__cause__, stream._capture_error)
            self.assertEqual(error.cleanup_receipt, dict(link_stop_succeeded=True,
                video_worker_stopped=True, decoder_worker_stopped=True, local_cleanup_complete=True))
            error.cleanup_receipt['local_cleanup_complete'] = False
            self.assertTrue(error.cleanup_receipt['local_cleanup_complete'])
            self.assertEqual(stream.stats['metadata_errors'], 1)
            self.assertTrue(stream.stats['stopped'])

    def test_current_link_stop_failure_is_never_a_historical_capture_receipt(self):
        with failed_video() as (stream, link):
            link.stop_error = RuntimeError('current socket close failed')
            with self.assertRaises(VideoStreamError) as caught:
                stream.stop()
            self.assertNotIsInstance(caught.exception, VideoCaptureStoppedError)
            self.assertIs(caught.exception.__cause__, link.stop_error)
            self.assertIn('socket close failed', str(caught.exception))

    def test_live_worker_timeout_cannot_claim_local_cleanup(self):
        links = []

        def create(**options):
            link = FakeLink(**options, stubborn=True)
            links.append(link)
            return link

        with patch('osmo360_camera.video.Datalink', side_effect=create), patch('osmo360_camera.video.poke'):
            stream = VideoStream(decode=False).start()
            self.assertTrue(links[0].fed.wait(2))
            try:
                with self.assertRaisesRegex(VideoStreamError, 'did not stop') as caught:
                    stream.close_local(timeout=.01)
                self.assertNotIsInstance(caught.exception, VideoCaptureStoppedError)
                self.assertIsNone(stream._local_close_receipt)
            finally:
                links[0].release.set()
                stream._thread.join(2)
                stream.stop()

    def test_decoder_failure_or_live_decoder_cannot_claim_local_cleanup(self):
        for throws in (True, False):
            with self.subTest(throws=throws), failed_video() as (stream, _):
                decoder = SimpleNamespace(thread=SimpleNamespace(is_alive=lambda: not throws),
                    stop=Mock(side_effect=TimeoutError('decoder stop failed') if throws else None))
                stream._preview_decoder = decoder
                with self.assertRaises((TimeoutError, VideoStreamError)) as caught:
                    stream.close_local(timeout=.2)
                self.assertNotIsInstance(caught.exception, VideoCaptureStoppedError)
                self.assertIsNone(stream._local_close_receipt)

    def test_host_only_close_keeps_capture_cause_and_is_idempotent(self):
        with failed_video() as (stream, link):
            with patch.object(link, 'stop', wraps=link.stop) as stop:
                receipt = stream.close_local(timeout=.2)
                self.assertTrue(receipt['local_cleanup_complete'])
                self.assertTrue(receipt['host_only'])
                self.assertFalse(receipt['camera_restoration_performed'])
                self.assertIn('timestamp metadata', receipt['capture_failure']['message'])
                receipt['capture_failure']['message'] = 'mutated consumer copy'
                self.assertNotEqual(stream.close_local(timeout=.2)['capture_failure']['message'],
                                    'mutated consumer copy')
                self.assertEqual(stop.call_count, 1)
                with self.assertRaises(ValueError):
                    stream.close_local(timeout=0)


class USBHistoricalCaptureTests(unittest.TestCase):
    def test_real_producer_error_and_proven_native_restore_close_successfully(self):
        with harness() as case, failed_video() as (stream, _):
            case.preview.start()
            case.preview._stream = stream
            report = case.preview.stop()
            self.assertTrue(report['restored'])
            self.assertTrue(report['network_restored']['ok'])
            self.assertEqual(report['errors'], [])
            self.assertIn('timestamp metadata', report['video_capture_failure']['message'])
            self.assertTrue(report['video_capture_failure']['local_cleanup']['local_cleanup_complete'])
            self.assertEqual(report['video']['metadata_errors'], 1)
            self.assertIsNone(case.camera._usb_preview)
            self.assertEqual(case.events.count('lease.close'), 1)
            self.assertIs(case.preview.stop(), report)

    def test_unknown_generic_stop_error_is_not_accepted_from_stopped_stats(self):
        with harness() as case:
            case.preview.start()
            case.state.stream.stop = Mock(side_effect=RuntimeError('unknown shutdown error'))
            with self.assertRaisesRegex(USBPreviewError, 'unknown shutdown error'):
                case.preview.stop()
            self.assertNotIn('video_capture_failure', case.preview.report)
            self.assertIn('Video: unknown shutdown error', case.preview.report['errors'])

    def test_typed_receipt_for_another_stream_cannot_authorize_cleanup(self):
        with harness() as case, failed_video() as (other, _):
            case.preview.start()
            with self.assertRaises(VideoCaptureStoppedError) as capture:
                other.stop()
            case.state.stream.stop = Mock(side_effect=capture.exception)
            with self.assertRaises(USBPreviewError):
                case.preview.stop()
            self.assertNotIn('video_capture_failure', case.preview.report)

    def test_current_link_failure_still_throws_after_native_restore(self):
        with harness() as case, failed_video() as (stream, link):
            case.preview.start()
            case.preview._stream = stream
            link.stop_error = RuntimeError('socket close failed now')
            with self.assertRaisesRegex(USBPreviewError, 'socket close failed now'):
                case.preview.stop()
            self.assertNotIn('video_capture_failure', case.preview.report)

    def test_historical_video_error_does_not_weaken_imu_mask_or_drain_proof(self):
        for mode in ('mask', 'drain', 'historical_imu_fault'):
            with self.subTest(mode=mode), harness() as case, failed_video() as (stream, _):
                case.preview.start()
                case.preview._stream = stream
                if mode == 'historical_imu_fault':
                    imu = case.camera._imu = FaultedIMU(case.events)
                    with self.assertRaises(IMUStopError) as caught:
                        case.preview.stop()
                    self.assertIs(caught.exception, imu.error)
                    self.assertTrue(case.preview.report['restored'])
                else:
                    imu = case.camera._imu = FakeIMU(case.events, fail=mode == 'mask')
                    imu.quiescent = mode != 'drain'
                    with self.assertRaises(USBPreviewError):
                        case.preview.stop()
                    self.assertNotIn('lease.close', case.events)
                    self.assertFalse(case.preview.report['restored'])
                    self.assertIs(case.camera._imu, imu)

    def test_historical_capture_error_cannot_hide_native_or_host_restore_failure(self):
        for options in (
            dict(restored_runtime=replace(runtime(), host_addr='02:00:00:00:00:01')),
            dict(failure='restore_network'), dict(failure='cleanup_network'),
        ):
            with self.subTest(options=options), harness(**options) as case, failed_video() as (stream, _):
                case.preview.start()
                case.preview._stream = stream
                with self.assertRaises((USBPreviewError, RuntimeError)):
                    case.preview.stop()
                self.assertFalse(case.preview.report['restored'])
                self.assertIs(case.camera._usb_preview, case.preview)

    def test_historical_error_and_failed_rollback_still_throw(self):
        with harness(failure='lease_close') as case, failed_video() as (stream, _):
            case.preview.start()
            case.preview._stream = stream
            with patch.object(case.preview, '_reconnect', side_effect=RuntimeError('rollback reconnect failed')):
                with self.assertRaisesRegex(RuntimeError, 'rollback reconnect failed'):
                    case.preview.stop()
            self.assertFalse(case.preview.report['restored'])


class USBHostRetirementTests(unittest.TestCase):
    def test_verified_new_boot_releases_only_host_lease_network_and_video(self):
        with harness() as case, failed_video() as (stream, _):
            case.preview.start()
            case.preview._stream = stream
            proof = reboot_proof(preview=case.preview)
            lease = case.preview._lease = SimpleNamespace(stop=Mock(return_value=None))
            case.preview._stop_owned_imu = Mock(side_effect=AssertionError('old DIAG restoration'))
            case.preview._reconnect = Mock(side_effect=AssertionError('old USB rollback'))
            imu = case.camera._imu = object()
            before_queries = case.state.runtime_queries
            report = case.preview.retire_after_reboot(proof, timeout=.5)
            self.assertTrue(report['host_only'])
            self.assertFalse(report['native_cleanup_replayed'])
            self.assertFalse(case.preview.report['restored'])
            self.assertTrue(case.preview._closed)
            self.assertIs(case.camera._usb_preview, case.preview)
            self.assertIs(case.camera._imu, imu)
            lease.stop.assert_called_once_with(close_camera=False)
            case.preview._stop_owned_imu.assert_not_called()
            case.preview._reconnect.assert_not_called()
            self.assertEqual(case.state.runtime_queries, before_queries)
            self.assertNotIn('session:rollback_once', case.events)
            self.assertFalse(case.state.host_alias)
            self.assertEqual(case.preview.retire_after_reboot(proof, timeout=.5), report)
            self.assertEqual(lease.stop.call_count, 1)

    def test_missing_foreign_or_changed_generation_proof_does_nothing(self):
        with harness() as case:
            case.preview.start()
            local = Mock(side_effect=AssertionError('local close must not run'))
            case.preview._stream = SimpleNamespace(close_local=local)
            for proof in (object(), reboot_proof()):
                with self.subTest(proof=type(proof).__name__), self.assertRaises((USBPreviewError, RuntimeError)):
                    case.preview.retire_after_reboot(proof, timeout=.5)
            proof = reboot_proof(preview=case.preview)
            case.camera._transport._thread = object()
            with self.assertRaises(RuntimeError):
                case.preview.retire_after_reboot(proof, timeout=.5)
            local.assert_not_called()
            self.assertNotIn('lease.close', case.events)
            self.assertNotIn('network.cleanup', case.events)

    def test_current_local_failure_prevents_host_lease_and_network_release(self):
        with harness() as case:
            case.preview.start()
            case.preview._stream = SimpleNamespace(close_local=Mock(side_effect=RuntimeError('local worker still alive')))
            with self.assertRaisesRegex(RuntimeError, 'still alive'):
                case.preview.retire_after_reboot(reboot_proof(preview=case.preview), timeout=.5)
            self.assertNotIn('lease.close', case.events)
            self.assertNotIn('network.cleanup', case.events)
            self.assertFalse(case.preview._closed)


if __name__ == '__main__':
    unittest.main()
