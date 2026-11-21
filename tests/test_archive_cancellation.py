"""A pending IDR boundary cannot survive end/reset into another capture epoch."""

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_driver.capture_archive import ArchiveBeginCancelled, ArchiveVideo


def frame(stamp, *, key=False):
    return SimpleNamespace(timestamp_ns=stamp, raw_h264=str(stamp).encode(), metadata={
        'h264_keyframe': key, 'h264_codec_headers_complete': True,
        'h264_codec_headers_hex': '0000000167aa0000000168bb',
    })


class ArchiveCancellationTests(unittest.TestCase):
    def setUp(self):
        self.saved = []
        self.archive = ArchiveVideo(lambda value, stamp: self.saved.append((value.timestamp_ns, stamp)))
        self.waiters = []
        self.addCleanup(self.finish_waiters)

    def finish_waiters(self):
        self.archive.end()
        # Also release a broken implementation's cache waiter, so a failing
        # regression cannot leave a thread alive until its long timeout.
        self.archive.append(frame(999, key=True), 999)
        for thread in self.waiters:
            thread.join(1)

    def begin_waiter(self, request=None, *, timeout=5):
        requested, done = threading.Event(), threading.Event()
        results, errors = [], []

        def keyframe():
            requested.set()
            if request is not None:
                request()

        def run():
            try:
                results.append(self.archive.begin(keyframe, timeout=timeout))
            except BaseException as error:
                errors.append(error)
            finally:
                done.set()

        thread = threading.Thread(target=run, name='test-archive-boundary', daemon=True)
        self.waiters.append(thread)
        thread.start()
        self.assertTrue(requested.wait(1), 'begin did not request its initial IDR')
        return SimpleNamespace(thread=thread, done=done, results=results, errors=errors)

    def test_end_and_reset_cancel_old_waiter_before_replacement_idr(self):
        for operation in ('end', 'reset'):
            with self.subTest(operation=operation):
                waiter = self.begin_waiter()
                self.assertTrue(self.archive.status()['starting'])
                self.assertFalse(self.archive.status()['active'])
                getattr(self.archive, operation)()
                self.archive.append(frame(100, key=True), 1100)
                self.assertTrue(waiter.done.wait(1))
                self.assertEqual(waiter.results, [])
                self.assertEqual(len(waiter.errors), 1)
                self.assertIsInstance(waiter.errors[0], ArchiveBeginCancelled)
                self.assertFalse(self.archive.status()['active'])
                self.assertFalse(self.archive.status()['starting'])
                self.assertEqual(self.archive.status()['sessions'], 0)
                self.assertEqual(self.saved, [])
                self.archive.reset()

    def test_cancel_wakes_waiter_without_needing_a_new_frame_or_timeout(self):
        waiter = self.begin_waiter(timeout=5)
        self.archive.end()
        self.assertTrue(waiter.done.wait(1), 'end must notify the IDR condition waiter')
        self.assertIsInstance(waiter.errors[0], ArchiveBeginCancelled)
        self.assertFalse(self.archive.status()['decodable_boundary'])
        self.assertEqual(self.saved, [])

    def test_cancel_during_keyframe_request_is_checked_before_using_new_cache(self):
        release = threading.Event()
        self.addCleanup(release.set)
        waiter = self.begin_waiter(lambda: self.assertTrue(release.wait(2)))
        self.archive.reset()
        self.archive.append(frame(200, key=True), 1200)
        release.set()
        self.assertTrue(waiter.done.wait(1))
        self.assertIsInstance(waiter.errors[0], ArchiveBeginCancelled)
        self.assertFalse(self.archive.status()['active'])
        self.assertEqual(self.saved, [])

    def test_cancelled_old_request_cannot_clear_or_arm_a_new_pending_begin(self):
        release_old = threading.Event()
        self.addCleanup(release_old.set)
        old = self.begin_waiter(lambda: self.assertTrue(release_old.wait(2)))
        self.archive.end()
        current = self.begin_waiter()
        release_old.set()
        self.assertTrue(old.done.wait(1))
        self.assertIsInstance(old.errors[0], ArchiveBeginCancelled)
        self.assertTrue(self.archive.status()['starting'])
        self.archive.append(frame(300, key=True), 1300)
        self.assertTrue(current.done.wait(1))
        self.assertEqual(current.errors, [])
        self.assertTrue(current.results[0]['active'])
        self.assertFalse(current.results[0]['starting'])
        self.assertEqual(self.saved, [(300, 1300)])
        self.assertEqual(self.archive.status()['sessions'], 1)

    def test_same_epoch_idr_then_live_remains_one_ordered_sequence(self):
        waiter = self.begin_waiter()
        self.archive.append(frame(400, key=True), 1400)
        self.assertTrue(waiter.done.wait(1))
        self.archive.append(frame(401), 1401)
        self.assertEqual(waiter.errors, [])
        self.assertEqual(self.saved, [(400, 1400), (401, 1401)])
        self.assertEqual(self.archive.status()['units'], 2)
        self.assertEqual(self.archive.status()['sessions'], 1)
        self.archive.end()
        self.archive.append(frame(402), 1402)
        self.assertEqual(self.saved, [(400, 1400), (401, 1401)])

    def test_second_pending_begin_is_rejected_without_an_extra_encoder_request(self):
        waiter = self.begin_waiter()
        request = Mock()
        with self.assertRaisesRegex(ValueError, 'already waiting'):
            self.archive.begin(request)
        request.assert_not_called()
        self.archive.end()
        self.assertTrue(waiter.done.wait(1))
        self.assertIsInstance(waiter.errors[0], ArchiveBeginCancelled)

    def test_request_failure_or_timeout_releases_pending_token_for_a_later_begin(self):
        with self.assertRaisesRegex(RuntimeError, 'keyframe request failed'):
            self.archive.begin(Mock(side_effect=RuntimeError('keyframe request failed')))
        self.assertFalse(self.archive.status()['starting'])
        with self.assertRaises(TimeoutError):
            self.archive.begin(lambda: None, timeout=.01)
        self.assertFalse(self.archive.status()['starting'])
        self.archive.append(frame(500, key=True), 1500)
        self.assertTrue(self.archive.begin(Mock(side_effect=AssertionError('IDR already cached')))['active'])
        self.assertEqual(self.saved, [(500, 1500)])

    def test_reset_drops_old_cache_and_new_begin_uses_only_new_epoch_preroll(self):
        self.archive.append(frame(1, key=True), 1001)
        self.archive.append(frame(2), 1002)
        self.archive.reset()
        waiter = self.begin_waiter()
        self.archive.append(frame(600, key=True), 1600)
        self.assertTrue(waiter.done.wait(1))
        self.assertEqual(waiter.errors, [])
        self.assertEqual(self.saved, [(600, 1600)])
        self.assertEqual(waiter.results[0]['first_native_ns'], 600)


if __name__ == '__main__':
    unittest.main()
