"""Retained-ECM core lifecycle fakes; no USB, sockets or camera commands."""

import threading
import types
import unittest
from unittest.mock import Mock

from osmo360_camera import _stream_session as m
from osmo360_camera._prepared_stream import FreshnessWitness, UnknownFutureApply
from test_stream_session import Environment, Prepared
from test_fast_sleep import Preview, IMU, SleepingPrepared


class Retained(Prepared):
    def __init__(self, env, **options):
        super().__init__(env, **options)
        self.sleep_id, self.wake_id = options["sleep_id"], options["wake_id"]
        self.arm_attempted = False
        self.attach_attempted = False
        self.arm_error = None
        self.old_preview = options["preview"]
        self.camera, self.control = env.camera, env.control
        self.owner_receipt = options['owner_receipt']

    def prepare(self):
        self.env.events.append("retained.prepare")
        if self.env.control.last["state"] != "AWAKE":
            raise AssertionError("Not AWAKE")
        return self

    def arm_watcher(self, *, deadline=None):
        self.arm_deadline = deadline
        self.env.events.append("retained.arm")
        if self.env.control.last["state"] != "SLEEP":
            raise AssertionError("Not SLEEP")
        self.arm_attempted = self.possible_future_apply = True
        if self.arm_error:
            raise self.arm_error
        self.old_preview._lease.stop(close_camera=False)

    def attach_data_first(self, **options):
        preview, imu = super().attach_data_first(**options)
        self.possible_future_apply = False
        self.env.camera._usb_preview, self.env.camera._imu = preview, imu
        return preview, imu


class RetainedFactory:
    def __init__(self, env):
        self.env, self.created, self.staged = env, [], False

    def preload_assets(self, camera, control, **options):
        self.env.events.append("retained.preload")
        if control.last["state"] != "AWAKE":
            raise AssertionError("Staged while asleep")
        self.staged = True
        return dict(staged=True)

    def __call__(self, camera, control, **options):
        if not self.staged:
            raise AssertionError("Retained cache missing")
        value = Retained(self.env, **options)
        self.created.append(value)
        return value


class RetainedSessionTests(unittest.TestCase):
    def setup_session(self):
        env = Environment()
        previews, prepared = [], []

        def preview(**options):
            value = Preview(env)
            value._lease.stop = Mock(
                side_effect=lambda **kw: env.events.append("lease.local_stop")
            )
            previews.append(value)
            env.camera._usb_preview = value
            return value

        def imu(**options):
            value = IMU(env)
            env.camera._imu = value
            return value

        env.camera.usb_preview, env.camera.imu = preview, imu
        env.control.lock, env.control.clock = env.camera._control_lock, env.clock
        env.control.bootstrap = types.SimpleNamespace(transport=env.camera._transport,
            session=types.SimpleNamespace(quarantined=False))
        env.control.pending_command = env.control.pending_task = None
        env.control._unknown_task = False
        env.control.resolve_pending = lambda: env.events.append("resolve_pending")
        env.control._submit = lambda action, proof: env.control._submit_sleep(proof)

        def factory(camera, control, **options):
            value = SleepingPrepared(env, **options)
            prepared.append(value)
            return value

        retained = RetainedFactory(env)
        session = m.USBStreamSession(
            env.camera,
            control=env.control,
            prepared_factory=factory,
            retained_factory=retained,
            retain_ecm=True,
            fast_sleep=True,
            idle_probe=lambda camera: m._IdleEvidence(env.clock(), 0, 1, True),
            clock=env.clock,
            sleep=env.clock.sleep,
        ).start()

        def release():
            for value in previews:
                value.release.set()
            if session._prepare_thread:
                session._prepare_thread.join(3)

        self.addCleanup(release)
        return env, session, previews[0], retained, prepared

    def test_retained_sleep_stops_producers_but_preserves_plan_lease_and_has_no_worker(
        self,
    ):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        self.assertEqual(session.state, "SLEEP")
        self.assertIs(session._preview, old)
        self.assertIsNone(session._prepare_thread)
        self.assertTrue(session._prepare_done.is_set())
        self.assertFalse(session.report["preparing_wake"])
        self.assertEqual(prepared, [])
        self.assertNotIn("usb.full_stop", env.events)
        old._lease.stop.assert_not_called()
        self.assertNotIn("retained.arm", env.events)
        self.assertLess(
            env.events.index("retained.preload"), env.events.index("preview.start")
        )
        self.assertLess(
            env.events.index("retained.prepare"), env.events.index("owner.sleep")
        )
        self.assertLess(
            env.events.index("imu.stop"), env.events.index("video.local_stop")
        )
        self.assertLess(
            env.events.index("video.local_stop"), env.events.index("owner.sleep")
        )

    def test_wake_arms_near_wake_before_one_input_then_returns_fresh_objects(self):
        env, session, old, factory, _ = self.setup_session()
        old_imu = session._imu
        session.sleep()
        env.clock.sleep(300)
        self.assertNotIn("retained.arm", env.events)
        session.wake(timeout=5)
        self.assertEqual(env.events.count("owner.wake"), 1)
        self.assertLess(
            env.events.index("retained.arm"), env.events.index("owner.wake")
        )
        self.assertLess(
            env.events.index("lease.local_stop"), env.events.index("owner.wake")
        )
        old._lease.stop.assert_called_once_with(close_camera=False)
        self.assertIsNot(session._preview, old)
        self.assertIsNot(session._imu, old_imu)
        self.assertTrue(session._prepared.report["media_ready_proven"])
        self.assertGreater(session.read_video().timestamp_ns, 300_000_000_000)
        self.assertGreater(session.read_imu().timestamp_ns, 300_000_000_000)

    def test_delayed_sleep_observation_uses_original_deadline_and_full_freshness_dwell(self):
        env, session, old, factory, _ = self.setup_session()
        session.sleep()
        baseline = session._native_floor_ns
        backend = session._prepared
        arm = backend.arm_watcher
        def delayed(*, deadline):
            self.assertEqual(deadline, 15)
            env.clock.sleep(2)
            return arm(deadline=deadline)
        backend.arm_watcher = delayed
        backend.attach_data_first = Mock(wraps=backend.attach_data_first)
        env.clock.sleep(10)
        session.wake(timeout=5)
        options = backend.attach_data_first.call_args.kwargs
        self.assertEqual(options['timeout'], 3)
        self.assertEqual(options['native_floor_ns'], baseline + 12_000_000_000)
        self.assertEqual(backend.arm_deadline, 15)
        self.assertEqual(env.events.count('owner.wake'), 1)

    def unarmed_failure(self):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        backend = session._prepared
        def fail_before_arm(**options):
            raise ValueError('fresh Sleep observation did not settle')
        backend.arm_watcher = fail_before_arm
        with self.assertRaises(ValueError):
            session.wake(timeout=5)
        self.assertEqual(session.state, 'RECOVERY_REQUIRED')
        self.assertIsNotNone(session._unarmed_wake_failure)
        self.assertFalse(backend.arm_attempted)
        self.assertFalse(backend.possible_future_apply)
        self.assertNotIn('owner.wake', env.events)
        return env, session, old, backend

    def test_close_of_proven_unarmed_wake_failure_publishes_normal_resume_without_any_key(self):
        env, session, old, backend = self.unarmed_failure()
        old.release.set()
        session._prepare_ready_callback = lambda *args, **options: env.events.append('publish.READY') or dict(schema=1)
        observe = Mock(wraps=env.control.wait_sleep_snapshot)
        env.control.wait_sleep_snapshot = observe
        result = session.close()
        self.assertEqual(session.state, 'DETACHED')
        self.assertEqual(result['unarmed_wake_cleanup']['verified_sleep_id'], backend.sleep_id)
        self.assertTrue(all(call.kwargs['deadline'] == 90 for call in observe.call_args_list))
        self.assertEqual(env.events.count('publish.READY'), 1)
        self.assertNotIn('owner.wake', env.events)
        self.assertFalse(backend.arm_attempted)
        self.assertFalse(backend.possible_future_apply)
        self.assertIsNone(session._unarmed_wake_failure)
        self.assertTrue(any(error.startswith('Wake:') for error in result['errors']))

    def test_generic_recovery_state_without_new_prearm_provenance_is_never_prepared(self):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        session._set_state('RECOVERY_REQUIRED')
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertIsNone(session._unarmed_wake_failure)
        self.assertNotIn('prepared.after_sleep', env.events)
        self.assertNotIn('owner.wake', env.events)
        old._lease.stop.assert_not_called()

    def test_unarmed_cleanup_rejects_changed_owner_or_new_unknown_task_without_preparation(self):
        for change in ('owner', 'pending', 'unknown', 'quarantine', 'arm', 'attach', 'missing_session', 'missing_unknown'):
            with self.subTest(change=change):
                env, session, old, backend = self.unarmed_failure()
                if change == 'owner': env.control.owner = dict(pid=999, start='1')
                elif change == 'pending': env.control.pending_command = backend.wake_id
                elif change == 'unknown': env.control._unknown_task = True
                elif change == 'quarantine':
                    env.control.bootstrap.session = types.SimpleNamespace(quarantined=True)
                elif change == 'arm': backend.arm_attempted = True
                elif change == 'attach': backend.attach_attempted = True
                elif change == 'missing_session': del env.control.bootstrap.session
                else: del env.control._unknown_task
                with self.assertRaises(m.StreamSessionError):
                    session.close()
                self.assertEqual(session.state, 'RECOVERY_REQUIRED')
                self.assertNotIn('prepared.after_sleep', env.events)
                self.assertNotIn('owner.wake', env.events)
                old._lease.stop.assert_not_called()

    def test_unarmed_cleanup_requires_fresh_same_sleep_receipt_after_observation(self):
        env, session, old, backend = self.unarmed_failure()
        env.control.wait_sleep_snapshot = Mock(return_value=dict(state='SLEEP',
            command_id=backend.sleep_id, last_result=dict(changed=True)))
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertEqual(session.state, 'RECOVERY_REQUIRED')
        self.assertNotIn('prepared.after_sleep', env.events)
        old._lease.stop.assert_not_called()

    def test_unarmed_cleanup_observation_and_worker_share_one_original_preparation_deadline(self):
        env, session, old, backend = self.unarmed_failure()
        env.clock.sleep(10)
        old.release.set()
        observe = env.control.wait_sleep_snapshot
        first = [True]
        def slow_observation(*args, **options):
            self.assertEqual(options['deadline'], 100)
            if first[0]:
                first[0] = False
                env.clock.sleep(80)
            return observe(*args, **options)
        env.control.wait_sleep_snapshot = slow_observation
        factory = session.prepared_factory
        def late_candidate(*args, **options):
            candidate = factory(*args, **options)
            def late(*, guard, deadline):
                self.assertEqual(deadline, 100)
                env.clock.sleep(11)
                guard()
            candidate.prepare_after_sleep = late
            return candidate
        session.prepared_factory = late_candidate
        publish = Mock()
        session._prepare_ready_callback = publish
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertEqual(session.state, 'RECOVERY_REQUIRED')
        self.assertEqual(env.clock(), 101)
        publish.assert_not_called()
        self.assertNotIn('owner.wake', env.events)

    def test_stable_sleep_close_prepares_schema_one_ready_without_holding_core_lock_while_joining(
        self,
    ):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        backend = session._prepared
        callback_lock = []

        def publish(candidate, floor, **options):
            acquired = session._lock.acquire(blocking=False)
            callback_lock.append(acquired)
            if acquired:
                session._lock.release()
            env.events.append("publish.READY")
            return dict(schema=1)

        session._prepare_ready_callback = publish
        errors = []
        finished = threading.Event()

        def close():
            try:
                session.close()
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()

        thread = threading.Thread(target=close)
        thread.start()
        self.assertTrue(old.entered.wait(1))
        self.assertFalse(finished.wait(0.02))
        acquired = session._lock.acquire(timeout=0.2)
        self.assertTrue(acquired)
        if acquired:
            session._lock.release()
        old.release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(callback_lock, [True])
        self.assertEqual(session.state, "DETACHED")
        self.assertIsNone(session._retained_backend)
        self.assertIs(session._prepared, prepared[0])
        self.assertIsNot(session._prepared, backend)
        self.assertLess(
            env.events.index("usb.full_stop"), env.events.index("prepared.after_sleep")
        )
        self.assertLess(
            env.events.index("publish.READY"), env.events.index("owner.detach")
        )
        self.assertNotIn("owner.wake", env.events)
        self.assertNotIn("owner.finalize", env.events)

    def test_unknown_arm_cannot_trigger_normal_close_or_full_preparation(self):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        factory.created[0].arm_error = OSError("lost helper launch reply")
        with self.assertRaises(OSError):
            session.wake(timeout=5)
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertTrue(session.owns_camera_resources)
        self.assertFalse(session.cleanup_safe)
        self.assertEqual(prepared, [])
        self.assertNotIn("usb.full_stop", env.events)
        self.assertNotIn("owner.wake", env.events)
        self.assertNotIn("owner.detach", env.events)
        self.assertNotIn("owner.finalize", env.events)

    def test_failed_wake_stats_keep_backend_diagnostics_and_original_cause_chain(self):
        env, session, old, factory, _ = self.setup_session()
        session.sleep()
        backend = factory.created[0]

        def fail_attach(**options):
            self.assertTrue(env.wake_submitted.wait(0.5))
            try:
                try:
                    raise OSError('Final USB generation lost carrier')
                except OSError as error:
                    raise TimeoutError('Retained completion/data deadline') from error
            except TimeoutError as error:
                backend.report.update(
                    recovery_required=True, errors=[repr(error)],
                    network_generations=[{'ncm_interface': 'enx-camera', 'ok': False}],
                )
                raise UnknownFutureApply('No final reset/lease proof') from error

        backend.attach_data_first = fail_attach
        with self.assertRaises(UnknownFutureApply):
            session.wake(timeout=5)
        stats = session.stats
        self.assertEqual(stats['state'], 'RECOVERY_REQUIRED')
        failure = stats['report']['wake_failure']
        self.assertEqual(failure['prepared'], backend.report)
        self.assertEqual(stats['report']['cycles'][-1]['prepared'], backend.report)
        self.assertEqual(failure['errors'], [
            "UnknownFutureApply('No final reset/lease proof')",
            "TimeoutError('Retained completion/data deadline')",
            "OSError('Final USB generation lost carrier')",
        ])
        self.assertEqual(env.events.count('owner.wake'), 1)
        # Preserve postmortem evidence independently of live backend changes and
        # independently of callers editing their returned stats JSON.
        backend.report['errors'].append('later cleanup error')
        failure['prepared']['errors'].append('caller edit')
        frozen = session.stats['report']['wake_failure']['prepared']['errors']
        self.assertEqual(frozen, ["TimeoutError('Retained completion/data deadline')"])

    def test_failed_joint_wake_keeps_latest_producer_evidence_and_startup_stages(self):
        env, session, old, factory, _ = self.setup_session()
        session.sleep()
        backend = factory.created[0]
        attach = backend.attach_data_first

        def attach_without_imu(**options):
            preview, imu = attach(**options)
            backend.witness = FreshnessWitness(options['native_floor_ns'], options['wake_host_ns'])
            preview.stats = {'video': {'latest_video': {
                'timestamp_ns': options['native_floor_ns'] + 500,
                'host_receive_monotonic_ns': options['wake_host_ns'] + 500,
                'motion': {'sample_count': 33, 'imu_clock_us_raw': 123},
            }}}
            imu.items.clear()
            imu.stats = {'received': 0, 'last_timestamp_ticks': None,
                         'last_host_receive_monotonic_ns': None, 'collector_returncode': None}
            backend.start_imu_after_video = lambda frame: env.events.append('fresh.imu.start')
            return preview, imu

        backend.attach_data_first = attach_without_imu
        with self.assertRaises(TimeoutError):
            session.wake(timeout=1)
        failure = session.stats['report']['wake_failure']['prepared']
        witness = failure['freshness']
        self.assertIsNotNone(witness['first_video'])
        self.assertIsNone(witness['first_imu'])
        self.assertEqual(witness['latest_video']['motion']['sample_count'], 33)
        self.assertGreater(witness['latest_video']['timestamp_ns'], witness['first_video']['timestamp_ns'])
        self.assertIsNone(witness['latest_imu'])
        self.assertFalse(witness['joint_ready'])
        stages = failure['wake_startup']['stages']
        self.assertEqual([row['stage'] for row in stages], [
            'watcher_arm_started', 'watcher_armed', 'wake_thread_started',
            'data_attached', 'first_video_accepted', 'imu_start_started',
            'imu_mask_ready', 'failed',
        ])
        self.assertEqual(failure['wake_startup']['failed_after'], 'imu_mask_ready')
        self.assertTrue(all(type(row['host_monotonic_ns']) is int for row in stages))
        self.assertEqual(env.events.count('owner.wake'), 1)
        self.assertEqual(env.events.count('fresh.imu.start'), 1)
        self.assertNotIn('owner.wait.AWAKE', env.events)

    def test_next_accepted_sleep_failure_retires_previous_warm_cleanup_proof(self):
        env, session, old, factory, prepared = self.setup_session()
        session.sleep()
        session.wake(timeout=5)
        previous = session._retained_backend
        self.assertTrue(previous.lease_proven)
        previous.stop = Mock(
            side_effect=AssertionError("Old cleanup proof must not be reused")
        )
        # Keep the second awake generation faithful to the real local-worker ABI.
        current_preview, current_imu = Preview(env), IMU(env)
        current_preview._lease.stop = Mock()
        session._preview = env.camera._usb_preview = previous.preview = current_preview
        session._imu = env.camera._imu = previous.imu = current_imu
        env.present = True  # Fresh awake Datalink registered its own APP again.
        self.addCleanup(current_preview.release.set)
        wait = env.control.wait

        def wait_lost(identifier, target, **options):
            if target == "SLEEP":
                env.control.pending_command = identifier
                raise TimeoutError("Accepted next Sleep status is unreadable")
            return wait(identifier, target, **options)

        env.control.wait = wait_lost
        with self.assertRaises(TimeoutError):
            session.sleep()
        current = factory.created[-1]
        self.assertIs(session._prepared, current)
        self.assertIs(session._retained_backend, current)
        self.assertIsNot(current, previous)
        self.assertFalse(current.lease_proven)
        self.assertEqual(current.sleep_id, "0000000000000003")
        with self.assertRaises(m.StreamSessionError):
            session.close()
        previous.stop.assert_not_called()
        current_preview._lease.stop.assert_not_called()
        self.assertNotIn("usb.full_stop", env.events)
        self.assertEqual(prepared, [])
        self.assertEqual(env.events.count("owner.sleep"), 2)
        self.assertEqual(env.events.count("owner.wake"), 1)
        self.assertNotIn("owner.detach", env.events)
        self.assertNotIn("owner.finalize", env.events)

    def test_classic_resumed_core_stages_retained_cache_while_awake_before_next_sleep(
        self,
    ):
        env, session, old, factory, _ = self.setup_session()
        # A restored schema1 READY core enters AWAKE through wake(), not start().
        # Its new retained factory therefore has no preloaded assets yet.
        factory.staged = False
        env.events.clear()
        session.sleep()
        self.assertTrue(factory.staged)
        self.assertLess(
            env.events.index("retained.preload"), env.events.index("retained.prepare")
        )
        self.assertLess(
            env.events.index("retained.prepare"), env.events.index("owner.sleep")
        )

    def test_retained_flags_reject_invalid_combinations_before_any_device_work(self):
        env = Environment()
        for fast, retain, factory in (
            (False, True, lambda: None),
            (True, 1, lambda: None),
            (True, True, None),
        ):
            with self.subTest(fast=fast, retain=retain), self.assertRaises(ValueError):
                m.USBStreamSession(
                    env.camera,
                    control=env.control,
                    prepared_factory=env.prepared_factory,
                    idle_probe=lambda camera: None,
                    fast_sleep=fast,
                    retain_ecm=retain,
                    retained_factory=factory,
                )
        self.assertEqual(env.events, [])


if __name__ == "__main__":
    unittest.main()
