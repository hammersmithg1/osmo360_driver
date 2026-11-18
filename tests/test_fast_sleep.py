"""Optional local-first Sleep and its single background owner; no device I/O."""

import threading
import types
import unittest
from unittest.mock import Mock

from osmo360_camera import _stream_session as m
from osmo360_camera.imu import IMUStream
from test_stream_session import Environment, Capture, Prepared


class LocalStream:
    def __init__(self, env):
        self.env, self.alive, self.closed = env, True, False
        self.stats = dict(stopped=False)
        self._thread = types.SimpleNamespace(is_alive=lambda: self.alive)
        self._link = types.SimpleNamespace(
            stopped=types.SimpleNamespace(is_set=lambda: self.closed),
            _sock=types.SimpleNamespace(fileno=lambda: -1 if self.closed else 3),
        )

    def pause(self, **options):
        self.env.events.append("pause")
        return True

    def resume(self, **options):
        self.env.events.append("resume")

    def stop(self):
        self.env.events.append("video.local_stop")
        self.alive, self.closed, self.stats["stopped"] = False, True, True
        return dict(stopped=True)


class Preview(Capture):
    def __init__(self, env):
        super().__init__(env, "preview")
        self._stream, self._scheduled = LocalStream(env), True
        self._plan = dict(
            work_dir="/dev/shm/osmo-ncm-offline", lease=dict(nonce="ab" * 16)
        )
        self._lease = types.SimpleNamespace(
            healthy=True, last_ack_monotonic=0, nonce=bytes.fromhex("ab" * 16)
        )
        self.entered, self.release = threading.Event(), threading.Event()
        self.stop_error = None

    def stop(self, **options):
        self.env.events.append("usb.full_stop")
        self.entered.set()
        if not self.release.wait(2):
            raise TimeoutError("fake USB stop release missing")
        if self.stop_error:
            raise self.stop_error
        self.report = dict(restored=True, errors=[])
        self.env.camera._usb_preview = None
        return self.report


class IMU(Capture):
    def __init__(self, env):
        super().__init__(env, "imu")
        self.pid, self._closed = 321, False
        self._process = types.SimpleNamespace(poll=lambda: 0 if self._closed else None)
        self._threads = []
        self._receiver_done = threading.Event()
        self._socket = types.SimpleNamespace(fileno=lambda: -1 if self._closed else 3)

    def stop(self, **options):
        self._closed = True
        self._receiver_done.set()
        return super().stop(**options)


class SleepingPrepared(Prepared):
    def prepare_after_sleep(self, *, guard=None, deadline=None):
        if guard:
            guard()
        self.env.events.append("prepared.after_sleep")
        self.report["prepared"] = True
        return self


class FastSleepTests(unittest.TestCase):
    def setup_session(self):
        env = Environment()

        def preview(**options):
            value = Preview(env)
            env.camera._usb_preview = value
            return value

        def imu(**options):
            value = IMU(env)
            env.camera._imu = value
            return value

        env.camera.usb_preview, env.camera.imu = preview, imu
        control = env.control
        control.lock, control.clock = env.camera._control_lock, env.clock
        control.bootstrap = types.SimpleNamespace(transport=env.camera._transport)
        control.resolve_pending = lambda: env.events.append("resolve_pending")

        def submit(action, proof):
            self.assertEqual(action, "sleep")
            self.assertFalse(env.camera._usb_preview.report["restored"])
            return control._submit_sleep(proof)

        control._submit = submit
        created = []

        def factory(camera, control, **options):
            value = SleepingPrepared(env, **options)
            created.append(value)
            return value

        session = m.USBStreamSession(
            env.camera,
            control=control,
            prepared_factory=factory,
            idle_probe=lambda camera: m._IdleEvidence(env.clock(), 0, 1, True),
            clock=env.clock,
            sleep=env.clock.sleep,
            fast_sleep=True,
        ).start()
        old = session._preview

        def release():
            old.release.set()
            if session._prepare_thread:
                session._prepare_thread.join(3)

        self.addCleanup(release)
        return env, session, old, created

    def test_native_sleep_returns_while_usb_stop_is_blocked_then_publishes_ready_last(
        self,
    ):
        env, session, old, created = self.setup_session()
        callback = Mock(
            side_effect=lambda backend, floor, **options: (
                env.events.append("publish.READY") or dict(published=True)
            )
        )
        session._prepare_ready_callback = callback
        value = session.sleep()
        self.assertEqual(value["state"], "SLEEP")
        self.assertTrue(old.entered.wait(1))
        self.assertFalse(session._prepare_done.is_set())
        self.assertTrue(session.report["preparing_wake"])
        self.assertEqual(sum(event == "owner.sleep" for event in env.events), 1)
        callback.assert_not_called()
        old.release.set()
        session._wait_preparation(timeout=2)
        self.assertEqual(session.state, "SLEEP")
        self.assertFalse(session.report["preparing_wake"])
        self.assertLess(
            env.events.index("owner.wait.SLEEP"), env.events.index("usb.full_stop")
        )
        self.assertLess(
            env.events.index("usb.full_stop"), env.events.index("prepared.after_sleep")
        )
        self.assertLess(
            env.events.index("prepared.after_sleep"), env.events.index("publish.READY")
        )
        self.assertIs(session._prepared, created[0])
        self.assertEqual(session.report["prepared_receipt"], dict(published=True))

    def test_actual_imu_stop_retains_completed_pid_and_still_allows_native_sleep(self):
        env, session, old, created = self.setup_session()
        # Use the real stop implementation, with a fake Popen only. It keeps
        # the completed process for stats; clearing its PID is not its ABI.
        imu = IMUStream()
        outcome = {"code": None}

        def wait(**options):
            outcome["code"] = 0
            return 0

        imu._process = types.SimpleNamespace(
            pid=4321,
            poll=lambda: outcome["code"],
            wait=wait,
            send_signal=lambda signal: None,
        )
        imu._started = True
        imu._report = dict(restored=True, quiescent=True, errors=[])
        imu._receiver_done.set()
        imu._socket = types.SimpleNamespace(fileno=lambda: -1)
        session._imu = env.camera._imu = imu
        session.sleep()
        self.assertEqual(imu.pid, 4321)
        self.assertEqual(imu.stats["collector_returncode"], 0)
        self.assertEqual(env.events.count("owner.sleep"), 1)
        self.assertTrue(old.entered.wait(1))

    def test_immediate_close_waits_without_core_or_control_lock_and_never_reboots(self):
        env, session, old, created = self.setup_session()
        session.sleep()
        self.assertTrue(old.entered.wait(1))
        finished, errors = threading.Event(), []

        def close():
            try:
                session.close()
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()

        thread = threading.Thread(target=close)
        thread.start()
        self.assertFalse(finished.wait(0.03))
        acquired = session._lock.acquire(timeout=0.2)
        self.assertTrue(acquired)
        if acquired:
            session._lock.release()
        old.release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(session.state, "DETACHED")
        self.assertNotIn("owner.finalize", env.events)

    def test_immediate_wake_waits_for_preparation_before_one_native_wake(self):
        env, session, old, created = self.setup_session()
        session.sleep()
        errors, done = [], threading.Event()

        def wake():
            try:
                session.wake(timeout=3)
            except BaseException as error:
                errors.append(error)
            finally:
                done.set()

        thread = threading.Thread(target=wake)
        thread.start()
        self.assertFalse(done.wait(0.03))
        self.assertNotIn("owner.wake", env.events)
        old.release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(session.state, "AWAKE")
        self.assertLess(
            env.events.index("prepared.after_sleep"), env.events.index("owner.wake")
        )

    def test_worker_failure_keeps_owner_and_blocks_wake_close_without_second_input(
        self,
    ):
        env, session, old, created = self.setup_session()
        old.stop_error = RuntimeError("native USB restore failed")
        session.sleep()
        old.release.set()
        with self.assertRaises(m.StreamSessionError):
            session._wait_preparation(timeout=2)
        self.assertEqual(session.state, "RECOVERY_REQUIRED")
        with self.assertRaises(m.StreamSessionError):
            session.wake(timeout=1)
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertTrue(session.owns_camera_resources)
        self.assertFalse(session.cleanup_safe)
        self.assertNotIn("owner.wake", env.events)
        self.assertNotIn("owner.detach", env.events)
        self.assertNotIn("owner.finalize", env.events)

    def test_foreign_collector_created_before_worker_lock_prevents_usb_rollback(self):
        env, session, old, created = self.setup_session()
        foreign = object()
        with env.camera._control_lock:
            session.sleep()
            env.camera._imu = foreign
        with self.assertRaises(m.StreamSessionError):
            session._wait_preparation(timeout=2)
        self.assertIs(env.camera._imu, foreign)
        self.assertNotIn("usb.full_stop", env.events)
        self.assertEqual(env.events.count("owner.sleep"), 1)
        self.assertEqual(session.state, "RECOVERY_REQUIRED")
        self.assertTrue(session.owns_camera_resources)

    def test_publication_failure_never_claims_ready_or_retries_the_callback(self):
        env, session, old, created = self.setup_session()
        callback = Mock(side_effect=RuntimeError("unknown READY append"))
        session._prepare_ready_callback = callback
        session.sleep()
        old.release.set()
        with self.assertRaises(m.StreamSessionError):
            session._wait_preparation(timeout=2)
        callback.assert_called_once()
        self.assertNotIn("prepared_receipt", session.report)
        self.assertEqual(session.state, "RECOVERY_REQUIRED")

    def test_thread_start_failure_never_joins_unstarted_thread_and_explicit_recover_still_works(
        self,
    ):
        env, session, old, created = self.setup_session()
        with __import__("unittest").mock.patch.object(
            m.threading.Thread, "start", side_effect=RuntimeError("no thread")
        ):
            with self.assertRaises(RuntimeError):
                session.sleep()
        self.assertIsNone(session._prepare_thread)
        self.assertTrue(session._prepare_done.is_set())
        with self.assertRaises(m.StreamSessionError):
            session.close()
        result = session.recover()
        self.assertTrue(result["reboot_requested"])
        self.assertEqual(env.events.count("owner.sleep"), 1)
        self.assertEqual(env.events.count("owner.finalize"), 1)


if __name__ == "__main__":
    unittest.main()
