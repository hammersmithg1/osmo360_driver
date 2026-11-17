"""First-fresh-video sequencing and cleanup; no USB, sockets or camera calls."""

import types
import unittest
from unittest.mock import Mock

from osmo360_camera import _retained_stream as retained
from osmo360_camera import _stream_session as core
from osmo360_camera._prepared_stream import FreshnessWitness
import test_retained_stream as retained_fixtures
from test_stream_session import Environment, Prepared


class Deferred(Prepared):
    start_imu_after_video = retained.RetainedStream.start_imu_after_video

    def __init__(self, env, **options):
        super().__init__(env, **options)
        self.camera = env.camera
        self.closed = self.attach_attempted = self.imu_started = False
        self.fail_start = self.no_video = self.late_start = False

    def attach_data_first(self, **options):
        preview, imu = super().attach_data_first(**options)
        self.witness = FreshnessWitness(
            options["native_floor_ns"], options["wake_host_ns"]
        )
        self.attach_attempted = True
        self.camera._usb_preview, self.camera._imu = preview, imu
        preview._lease = types.SimpleNamespace(healthy=True)
        original_read = imu.read

        def read(**kwargs):
            if not self.imu_started:
                raise AssertionError("Unstarted IMU must not be read")
            self.env.events.append("deferred.imu.read")
            return original_read(**kwargs)

        imu.read = read

        def start():
            self.env.events.append("deferred.imu.start")
            if not self.imu_started:
                raise AssertionError("Start attempt must be owned before call")
            if self.late_start:
                self.env.clock.now += 10
            if self.fail_start:
                imu.mask_ok = False
                raise OSError("Partial collector start with unknown mask")
            return imu

        imu.start = start
        if self.no_video:
            preview.items.clear()
        else:
            # First frame has a fresh native clock but belongs to an old host epoch.
            preview.items.appendleft(
                types.SimpleNamespace(
                    timestamp_ns=options["native_floor_ns"] + 8,
                    host_receive_monotonic_ns=options["wake_host_ns"] - 1,
                )
            )
        return preview, imu

    def stop(self):
        self.env.events.append("deferred.stop")
        if self.camera._imu is not None:
            self.camera._imu.stop()
            self.camera._imu = None
        return super().stop()


class VideoFirstTests(unittest.TestCase):
    def session(self, *, deferred=True):
        env = Environment()

        def factory(camera, control, **options):
            env.prepared = (Deferred if deferred else Prepared)(env, **options)
            return env.prepared

        session = core.USBStreamSession(
            env.camera,
            control=env.control,
            prepared_factory=factory,
            idle_probe=lambda camera: core._IdleEvidence(env.clock(), 0, 1, True),
            clock=env.clock,
            sleep=env.clock.sleep,
        )
        session.start()
        session.sleep()
        env.events.clear()
        return env, session, env.prepared

    def test_actual_backend_attaches_video_and_unstarted_owned_imu_without_diag(self):
        fixture = retained_fixtures.RetainedTests()
        fixture.setUp()
        value = fixture.armed()
        value.__class__ = retained.RetainedStream
        value._completion = Mock(return_value=dict(ok=True, ncm_interface="usb1"))
        preview, imu = value.attach_data_first(native_floor_ns=1, wake_host_ns=2)
        self.assertIn("video_start", fixture.events)
        self.assertNotIn("imu_start", fixture.events)
        self.assertIs(fixture.camera._imu, imu)
        self.assertFalse(value.imu_started)
        self.assertTrue(value.report["imu_start_pending"])
        self.assertFalse(value.report["data_started"])

    def test_native_and_host_stale_frames_do_not_start_or_read_imu_fresh_frame_is_preserved(
        self,
    ):
        env, session, value = self.session()
        observe = value.start_imu_after_video
        starts = []

        def start(frame):
            self.assertIs(frame, value.fresh_video)
            self.assertEqual(value.witness.rejected_video, 2)
            starts.append(frame)
            return observe(frame)

        value.start_imu_after_video = start
        session.wake(timeout=3)
        self.assertEqual(starts, [value.fresh_video])
        self.assertEqual(env.events.count("deferred.imu.start"), 1)
        self.assertLess(
            env.events.index("deferred.imu.start"),
            env.events.index("deferred.imu.read"),
        )
        self.assertIs(session.read_video(), value.fresh_video)
        self.assertIs(session.read_imu(), value.fresh_imu)
        self.assertTrue(value.report["media_ready_proven"])
        self.assertEqual(env.events.count("owner.wake"), 1)

    def test_no_first_fresh_video_never_starts_diag_and_known_lease_can_clean_up(self):
        env, session, value = self.session()
        value.no_video = True
        with self.assertRaises(TimeoutError):
            session.wake(timeout=0.3)
        self.assertIsNone(session._imu)
        self.assertNotIn("deferred.imu.start", env.events)
        self.assertNotIn("deferred.imu.read", env.events)
        session.close()
        self.assertEqual(session.state, "DETACHED")
        self.assertIn("preview.stop", env.events)
        self.assertIn("owner.detach", env.events)

    def test_partial_start_marks_attempt_before_error_and_unknown_mask_keeps_preview(
        self,
    ):
        env, session, value = self.session()
        value.fail_start = True
        with self.assertRaisesRegex(OSError, "Partial collector"):
            session.wake(timeout=3)
        self.assertTrue(value.imu_started)
        self.assertIs(session._imu, value.imu)
        self.assertIsNotNone(value.witness.first_video)
        with self.assertRaises(core.StreamSessionError):
            session.close()
        self.assertEqual(session.state, "RECOVERY_REQUIRED")
        self.assertNotIn("preview.stop", env.events)
        self.assertNotIn("owner.detach", env.events)
        with self.assertRaises(retained.PreparedStreamError):
            value.start_imu_after_video(value.fresh_video)
        self.assertEqual(env.events.count("deferred.imu.start"), 1)

    def test_foreign_producer_or_lost_lease_is_rejected_before_one_start_attempt(self):
        for field in ("_imu", "_usb_preview", "lease"):
            with self.subTest(field=field):
                env, session, value = self.session()

                def reject(frame):
                    if field == "lease":
                        value.preview._lease.healthy = False
                    else:
                        setattr(env.camera, field, object())
                    return Deferred.start_imu_after_video(value, frame)

                value.start_imu_after_video = reject
                with self.assertRaises(retained.PreparedStreamError):
                    session.wake(timeout=3)
                self.assertFalse(value.imu_started)
                self.assertNotIn("deferred.imu.start", env.events)

    def test_late_blocking_start_never_returns_joint_ready_or_retries(self):
        env, session, value = self.session()
        value.late_start = True
        with self.assertRaises(TimeoutError):
            session.wake(timeout=0.5)
        self.assertEqual(env.events.count("deferred.imu.start"), 1)
        self.assertFalse(value.report.get("media_ready_proven", False))
        self.assertEqual(session.state, "RECOVERY_REQUIRED")

    def test_generic_prepared_backend_keeps_original_eager_contract(self):
        env, session, value = self.session(deferred=False)
        session.wake(timeout=3)
        self.assertIs(session.read_video(), value.fresh_video)
        self.assertIs(session.read_imu(), value.fresh_imu)
        self.assertEqual(session.state, "AWAKE")
        self.assertNotIn("deferred.imu.start", env.events)

    def test_starter_requires_the_exact_first_accepted_frame_and_cannot_repeat(self):
        env, session, value = self.session()
        env.wake_submitted.set()
        value.attach_data_first(timeout=1, native_floor_ns=100, wake_host_ns=200)
        frame = value.fresh_video
        with self.assertRaises(retained.PreparedStreamError):
            value.start_imu_after_video(frame)
        self.assertTrue(value.witness.observe_video(frame))
        other = types.SimpleNamespace(
            timestamp_ns=frame.timestamp_ns + 1,
            host_receive_monotonic_ns=frame.host_receive_monotonic_ns,
        )
        with self.assertRaises(retained.PreparedStreamError):
            value.start_imu_after_video(other)
        self.assertNotIn("deferred.imu.start", env.events)
        self.assertIs(value.start_imu_after_video(frame), value.imu)
        with self.assertRaises(retained.PreparedStreamError):
            value.start_imu_after_video(frame)
        self.assertEqual(env.events.count("deferred.imu.start"), 1)


if __name__ == "__main__":
    unittest.main()
