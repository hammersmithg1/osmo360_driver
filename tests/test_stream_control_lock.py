"""Slow C1 proofs serialize lifecycle work while actual data reads continue."""

from threading import Event, Lock, Thread
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import _stream_session as m
from osmo360_camera.transport import RequestTimeout
from test_stream_session import Environment


class ControlLockTests(unittest.TestCase):
    def awake(self):
        env = Environment()
        session = env.session().start()
        session.sleep()
        session.wake()
        transport = env.camera._transport
        transport.connected, transport._thread = True, object()
        transport._device_identity = dict(bus=1, address=7)
        env.control.bootstrap = SimpleNamespace(transport=transport)
        env.control.lock = env.camera._control_lock
        env.control.bind_bootstrap = lambda value: setattr(env.control, 'bootstrap', value)
        session._primed_video.clear()
        session._primed_imu.clear()
        counter, lock = [1_000_000_000], Lock()
        def read(**options):
            with lock:
                counter[0] += 1
                return SimpleNamespace(timestamp_ns=counter[0], host_receive_monotonic_ns=counter[0])
        session._preview.read = session._imu.read = read
        env.present = True  # The new fake preview owns a fresh APP registration.
        env.events.clear()
        return env, session

    def thread(self, function):
        done, result, errors = Event(), [], []
        def run():
            try: result.append(function())
            except BaseException as error: errors.append(error)
            finally: done.set()
        thread = Thread(target=run, daemon=True)
        thread.start()
        self.addCleanup(lambda: thread.join(2))
        return thread, done, result, errors

    def blocked(self, env, session, *, stage='reconnect', failure=None):
        entered, release = Event(), Event()
        self.addCleanup(release.set)
        def block():
            entered.set()
            if not release.wait(2): raise TimeoutError('test proof release missing')
            if failure is not None: raise failure
        if stage == 'reconnect':
            def reconnect(camera):
                block()
                return env.control.bootstrap
            session.reconnect_control = Mock(side_effect=reconnect)
        else:
            session.reconnect_control = Mock(return_value=env.control.bootstrap)
            wait = env.control.wait
            def status(*args, **options):
                block()
                return wait(*args, **options)
            env.control.wait = Mock(side_effect=status)
        running = self.thread(session.verify_control)
        self.assertTrue(entered.wait(1))
        return release, running

    def assert_data_flows(self, session):
        readers = [self.thread(lambda read=read: [read(timeout=.01) for _ in range(20)])
                   for read in (session.read_video, session.read_imu)]
        for thread, done, results, errors in readers:
            self.assertTrue(done.wait(.5), 'a C1 proof/lifecycle waiter blocked the data lock')
            self.assertEqual(errors, [])
            self.assertEqual(len(results[0]), 20)
        stats = self.thread(lambda: session.stats)
        self.assertTrue(stats[1].wait(.5))
        self.assertEqual(stats[3], [])

    def test_slow_reconnect_keeps_video_imu_and_stats_available(self):
        env, session = self.awake()
        release, proof = self.blocked(env, session)
        self.assert_data_flows(session)
        self.assertFalse(proof[1].is_set())
        self.assertFalse(session.stats['control_verified'])
        release.set()
        self.assertTrue(proof[1].wait(1))
        self.assertEqual(proof[3], [])
        self.assertTrue(session.stats['control_verified'])
        self.assertEqual(env.events.count('owner.wait.AWAKE'), 1)

    def test_reconnect_or_status_timeout_preserves_live_capture_and_unverified_state(self):
        for stage in ('reconnect', 'status'):
            with self.subTest(stage=stage):
                env, session = self.awake()
                preview, imu = session._preview, session._imu
                failure = RequestTimeout('C1 proof timed out')
                release, proof = self.blocked(env, session, stage=stage, failure=failure)
                self.assert_data_flows(session)
                release.set()
                self.assertTrue(proof[1].wait(1))
                self.assertEqual(proof[3], [failure])
                self.assertFalse(session.stats['control_verified'])
                self.assertEqual(session.state, 'AWAKE')
                self.assertIs(session._preview, preview)
                self.assertIs(session._imu, imu)
                self.assertNotIn('late_control_proof', session.report)
                self.assertNotIn('preview.stop', env.events)
                self.assertNotIn('imu.stop', env.events)
                self.assert_data_flows(session)

    def test_second_proof_serializes_without_duplicate_reconnect_or_data_block(self):
        env, session = self.awake()
        release, first = self.blocked(env, session)
        second = self.thread(session.verify_control)
        self.assertFalse(second[1].wait(.03))
        self.assert_data_flows(session)
        release.set()
        for running in (first, second):
            self.assertTrue(running[1].wait(1))
            self.assertEqual(running[3], [])
        session.reconnect_control.assert_called_once_with(env.camera)
        self.assertEqual(env.events.count('owner.wait.AWAKE'), 1)

    def test_close_waits_before_data_lock_then_commits_after_proof_without_deadlock(self):
        env, session = self.awake()
        release, proof = self.blocked(env, session)
        closing = self.thread(session.close)
        self.assertFalse(closing[1].wait(.03))
        self.assert_data_flows(session)
        release.set()
        for running in (proof, closing):
            self.assertTrue(running[1].wait(1), 'lifecycle/data lock inversion')
            self.assertEqual(running[3], [])
        self.assertEqual(session.state, 'DETACHED')
        self.assertLess(env.events.index('owner.wait.AWAKE'), env.events.index('preview.stop'))
        session.reconnect_control.assert_called_once()
        with self.assertRaises(m.StreamSessionError): session.verify_control()

    def test_sleep_waits_for_existing_proof_without_holding_data_lock_or_replaying_wake(self):
        env, session = self.awake()
        release, proof = self.blocked(env, session)
        sleeping = self.thread(session.sleep)
        self.assertFalse(sleeping[1].wait(.03))
        self.assert_data_flows(session)
        release.set()
        for running in (proof, sleeping):
            self.assertTrue(running[1].wait(1))
            self.assertEqual(running[3], [])
        self.assertEqual(session.state, 'SLEEP')
        self.assertEqual(env.events.count('owner.sleep'), 1)
        self.assertEqual(env.events.count('owner.wait.AWAKE'), 1)
        self.assertNotIn('owner.wake', env.events)
        session.reconnect_control.assert_called_once()

    def test_session_or_control_identity_change_cannot_publish_late_proof(self):
        for change in ('wake', 'control', 'owner', 'capture', 'transport'):
            with self.subTest(change=change):
                env, session = self.awake()
                release, proof = self.blocked(env, session)
                if change == 'wake': session._wake_id = '000000000000ffff'
                elif change == 'control': session.control = object()
                elif change == 'owner': env.control.owner = dict(pid=999, start='9')
                elif change == 'capture': session._preview = object()
                else: env.camera._transport = object()
                release.set()
                self.assertTrue(proof[1].wait(1))
                self.assertEqual(len(proof[3]), 1)
                self.assertIsInstance(proof[3][0], m.StreamSessionError)
                self.assertFalse(session._late_verified)
                self.assertNotIn('late_control_proof', session.report)
                self.assertNotIn('owner.wait.AWAKE', env.events)

    def test_reader_or_usb_generation_change_during_status_blocks_proof_commit(self):
        for change in ('reader', 'address', 'disconnect'):
            with self.subTest(change=change):
                env, session = self.awake()
                release, proof = self.blocked(env, session, stage='status')
                transport = env.camera._transport
                if change == 'reader': transport._thread = object()
                elif change == 'address': transport._device_identity['address'] = 8
                else: transport.connected = False
                self.assert_data_flows(session)
                release.set()
                self.assertTrue(proof[1].wait(1))
                self.assertEqual(len(proof[3]), 1)
                self.assertIsInstance(proof[3][0], m.StreamSessionError)
                self.assertFalse(session.stats['control_verified'])
                self.assertNotIn('late_control_proof', session.report)


if __name__ == '__main__':
    unittest.main()
