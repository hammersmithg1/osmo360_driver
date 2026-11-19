"""Normal close retires its exact APP before IMU/USB/power-policy teardown."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import _app
from osmo360_camera._stream_session import StreamSessionError
from osmo360_camera._usb_charging import USBChargingLease, USBChargingError
import test_stream_session as fixtures


FOREIGN = bytes.fromhex('120044000123004f5448455200000003')
MIXED = fixtures.PRESENT[:4] + b'\x02' + fixtures.PRESENT[5:] + FOREIGN[5:]


class CloseAppTests(unittest.TestCase):
    def setUp(self):
        self.env = fixtures.Environment()
        self.session = self.env.session().start()
        self.env.events.clear()

    def test_live_close_retires_app_before_mask_usb_and_detach(self):
        result = self.session.close()
        events = self.env.events
        for before, after in (('pause', 'app.unregister'), ('app.unregister', 'imu.stop'),
                              ('imu.stop', 'preview.stop'), ('preview.stop', 'owner.detach')):
            self.assertLess(events.index(before), events.index(after))
        self.assertEqual(events.count('app.unregister'), 1)
        self.assertFalse(self.env.present)
        self.assertTrue(self.session.cleanup_safe)
        self.assertEqual(result['app_teardown'][-1]['after'], fixtures.ABSENT.hex())
        self.assertEqual(self.env.message.payload, _app.registration_payload(unregister=True))

    def test_canonical_already_absent_close_never_sends_unregister(self):
        self.env.present = False
        self.session.close()
        self.assertNotIn('app.unregister', self.env.events)
        self.assertEqual(self.session.report['app_teardown'][-1]['attempts'], 0)
        self.assertTrue(self.session.cleanup_safe)

    def test_real_charging_app_guard_rejects_live_registration_and_passes_after_normal_close(self):
        lease = USBChargingLease.__new__(USBChargingLease)
        lease.camera, lease.clock, lease.generation = self.env.camera, self.env.clock, object()
        lease._report = {}
        # Isolate APP retirement: native Record0/generation have separate tests.
        lease._guard = lease._assert_local_idle = Mock()
        lease._idle_state = Mock(return_value=SimpleNamespace(recording_state=0))
        with self.assertRaisesRegex(USBChargingError, 'APP capture registration'):
            lease._prove_idle(10)
        self.session.close()
        lease._prove_idle(10)
        self.assertEqual(lease._report['evidence']['idle']['method'], 'C1_Record0_and_APP_absent')
        self.assertEqual(self.env.events.count('app.unregister'), 1)

    def test_foreign_only_or_mixed_registration_fails_before_any_unregister_or_usb_stop(self):
        for payload in (FOREIGN, MIXED):
            with self.subTest(payload=payload.hex()):
                env = fixtures.Environment()
                session = env.session().start()
                env.camera._transport.request = Mock(return_value=SimpleNamespace(payload=payload))
                env.events.clear()
                with self.assertRaises(StreamSessionError) as caught:
                    session.close()
                self.assertIn('foreign', str(caught.exception.__cause__))
                for forbidden in ('app.unregister', 'imu.stop', 'preview.stop', 'owner.detach'):
                    self.assertNotIn(forbidden, env.events)
                self.assertFalse(session.cleanup_safe)

    def test_pause_failure_preserves_live_owners_without_unregister_or_rollback(self):
        self.session._preview.pause_ok = False
        with self.assertRaises(StreamSessionError):
            self.session.close()
        self.assertIn('resume', self.env.events)
        for forbidden in ('app.unregister', 'imu.stop', 'preview.stop', 'owner.detach'):
            self.assertNotIn(forbidden, self.env.events)

    def test_active_camera_alias_change_blocks_app_query_and_write_before_mutation(self):
        self.env.camera._usb_preview = object()
        with self.assertRaises(StreamSessionError) as caught:
            self.session.close()
        self.assertIn('preview changed', str(caught.exception.__cause__))
        for forbidden in ('pause', 'app.query', 'app.unregister', 'imu.stop', 'preview.stop', 'owner.detach'):
            self.assertNotIn(forbidden, self.env.events)

    def test_lost_unregister_reply_uses_query_proof_and_never_replays_input(self):
        write = self.env.camera._transport._write
        def lost(message):
            write(message)
            raise TimeoutError('Write completion unavailable')
        self.env.camera._transport._write = lost
        self.session.close()
        self.assertEqual(self.env.events.count('app.unregister'), 1)
        self.assertTrue(self.session.cleanup_safe)
        self.assertIn('delivery_error', self.session.report['app_teardown'][-1])

    def test_unapplied_unregister_timeout_does_not_retry_during_recovery_close(self):
        writes = []
        self.env.camera._transport._write = lambda message: writes.append(message)
        with self.assertRaises(StreamSessionError):
            self.session.close()
        self.assertEqual(len(writes), 1)
        self.assertEqual(self.session.state, 'RECOVERY_REQUIRED')
        self.session.close()
        self.assertEqual(len(writes), 1)
        self.assertFalse(self.session.cleanup_safe)  # Deferred cleanup is not erased.

    def test_closed_video_worker_and_socket_are_a_valid_nonlive_teardown_barrier(self):
        stream = self.session._preview
        stream._thread = SimpleNamespace(is_alive=lambda: False)
        stream._link = SimpleNamespace(stopped=SimpleNamespace(is_set=lambda: True),
                                       _sock=SimpleNamespace(fileno=lambda: -1))
        stream.pause = Mock(side_effect=AssertionError('A stopped stream cannot be paused'))
        self.session.close()
        stream.pause.assert_not_called()
        self.assertEqual(self.env.events.count('app.unregister'), 1)

    def test_closed_flag_alone_never_replaces_worker_and_socket_proof(self):
        stream = self.session._preview
        stream._thread = SimpleNamespace(is_alive=lambda: False)
        stream._link = SimpleNamespace(stopped=SimpleNamespace(is_set=lambda: True),
                                       _sock=SimpleNamespace(fileno=lambda: 12))
        stream.pause = Mock(side_effect=RuntimeError('Socket is still owned'))
        with self.assertRaises(StreamSessionError):
            self.session.close()
        self.assertNotIn('app.unregister', self.env.events)

    def test_sleep_already_retired_app_and_stable_close_never_unregisters_again(self):
        self.session.sleep()
        before = self.env.events.count('app.unregister')
        self.session.close()
        self.assertEqual(self.env.events.count('app.unregister'), before)


if __name__ == '__main__':
    unittest.main()
