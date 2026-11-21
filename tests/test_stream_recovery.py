"""Owned reboot ordering, uncertainty and old-generation retirement proofs."""

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera._stream_recovery import StreamRecoveryError, _VerifiedRebootProof
from osmo360_camera._stream_session import USBStreamSession
from osmo360_camera._stream_resume import ResumableUSBStream


BEFORE = '11111111-1111-1111-1111-111111111111'
AFTER = '22222222-2222-2222-2222-222222222222'


class Environment:
    def __init__(self):
        self.now, self.events = 0., []
        self.disconnect, self.finalize_error, self.verify_error = True, None, None
        self.result_running = self.fail_video = self.fail_imu = False
        self.proof_error = self.retirement_error = None
        self.after = AFTER
        self.transport = SimpleNamespace(connected=True, _thread=object())
        self.transport.wait_disconnected = self.wait_disconnected
        self.transport.close = self.close
        self.camera = SimpleNamespace(_transport=self.transport, _control_lock=threading.RLock(),
                                      _usb_stream=None, _imu=None, _usb_preview=None)
        self.control = SimpleNamespace(launch_attempted=True, finalize_attempted=False,
            detached=False, work='/dev/shm/osmo-usb-standby-aabbccddeeff', nonce='aabbccddeeff',
            owner=dict(pid=7, start='3'), service=dict(pid=8, start='4'), pending_task=None,
            pending_command=None, _unknown_task=False)
        self.control.bootstrap = SimpleNamespace(session=self.session())
        self.control.bind_bootstrap = self.bind
        self.control.resolve_pending = self.resolve
        self.control._run = self.run
        self.control.finalize = self.finalize
        self.control.detach = self.detach
        self.core = USBStreamSession(self.camera, control=self.control, prepared_factory=None,
            idle_probe=None, reconnect_control=self.reconnect, clock=lambda: self.now)
        self.core._set_state('RECOVERY_REQUIRED')
        self.facade = self.camera._usb_stream = SimpleNamespace(_core=self.core)
        self.video = SimpleNamespace(close_local=self.close_video)
        self.preview = SimpleNamespace(camera=self.camera, _stream=self.video,
                                       retire_after_reboot=self.retire_preview)
        self.imu = SimpleNamespace(quiesce_local=self.quiesce_imu,
                                  retire_after_reboot=self.retire_imu,
                                  stop=lambda **kw: self.fail('Old mask restore called'))
        self.core._preview = self.camera._usb_preview = self.preview
        self.core._imu = self.camera._imu = self.imu
        self.patches = [patch('osmo360_camera._stream_recovery.package_profile',
                             return_value={'manifest': {'profile': 'profile'}}),
                        patch('osmo360_camera._stream_recovery._owned_boot', side_effect=self.owned_boot),
                        patch('osmo360_camera._stream_recovery.verify_installed', side_effect=self.verify)]

    def fail(self, message):
        raise AssertionError(message)

    def session(self):
        return SimpleNamespace(quarantined=False, owned_task_id=37)

    def __enter__(self):
        for p in self.patches:
            p.start()
        return self

    def __exit__(self, *args):
        for p in reversed(self.patches):
            p.stop()

    def bind(self, adapter):
        if self.control.bootstrap.session.quarantined:
            self.control.pending_task = self.control.bootstrap.session.owned_task_id
        self.control.bootstrap = adapter

    def reconnect(self, camera, *, deadline):
        assert camera is self.camera and self.now < deadline
        self.events.append('reconnect')
        if not self.transport.connected:
            self.transport.connected = True
            self.transport._thread = object()
        return self.control.bootstrap

    def resolve(self, *, timeout):
        self.events.append('RESULT')
        if self.control._unknown_task or self.result_running:
            raise RuntimeError('Exact original task remains unknown/running')
        task = self.control.pending_task
        self.control.pending_task = None
        self.control.bootstrap = SimpleNamespace(session=self.session())
        return dict(task_id=task, terminal_or_missing=True)

    def run(self, command, *, timeout_ms, pre_send):
        pre_send()
        self.events.append('proof:' + command)
        if self.proof_error:
            raise self.proof_error
        return dict(completed=True, success=True)

    def owned_boot(self, control, work, observed, remaining):
        assert control is self.control and observed['owner'] == self.control.owner
        assert remaining() > 0
        self.events.append('old_boot')
        return BEFORE

    def quiesce_imu(self, *, timeout):
        self.events.append('imu.quiesce_host_only')
        if self.fail_imu:
            raise RuntimeError('Collector still alive')
        return dict(collector_exited=True, workers_stopped=True, mask_restored=False)

    def close_video(self, *, timeout):
        self.events.append('video.close_local')
        if self.fail_video:
            raise RuntimeError('Video callback is still mutating')
        return dict(local_cleanup_complete=True)

    def finalize(self, *, guard):
        if self.control.finalize_attempted:
            self.fail('Second finalize')
        guard()
        self.control.finalize_attempted = True
        self.events.append('finalize')
        if self.finalize_error:
            self.control.bootstrap.session.quarantined = True
            self.control.pending_task = 37
            raise self.finalize_error
        return dict(reboot_requested=True, restored=False)

    def wait_disconnected(self, timeout):
        self.events.append('disconnect')
        return self.disconnect

    def close(self):
        self.events.append('transport.close')
        self.transport.connected = False

    def verify(self, camera, profile, previous_boot_id, *, timeout, control, guard):
        assert camera is self.camera and control is self.control and profile == 'profile'
        assert previous_boot_id == BEFORE
        guard()
        self.events.append('verify_new_boot_only')
        if self.verify_error:
            raise self.verify_error
        return dict(boot_id=self.after, autostart_hydration_verified=True)

    def retire_imu(self, proof, *, timeout):
        assert isinstance(proof, _VerifiedRebootProof)
        proof.validate()
        assert proof.owns_imu(self.imu)
        self.events.append('imu.retire_host_only')
        if self.retirement_error:
            raise self.retirement_error
        return dict(collector_exited=True, workers_stopped=True)

    def retire_preview(self, proof, *, timeout):
        proof.validate(self.camera)
        assert proof.owns_preview(self.preview)
        self.events.append('preview.retire_host_only')
        return dict(host_only=True)

    def detach(self):
        self.events.append('detach')
        self.control.detached = True
        return dict(retained_shadow=True)


class StreamRecoveryTests(unittest.TestCase):
    def test_full_proof_order_and_no_old_native_cleanup(self):
        with Environment() as env:
            result = env.core.recover_and_wait(timeout=120)
            self.assertEqual(result['state'], 'DETACHED')
            self.assertEqual((result['boot_id_before'], result['boot_id_after']), (BEFORE, AFTER))
            self.assertTrue(result['new_boot_verified'])
            self.assertFalse(env.core.report['native_restored'])
            self.assertIsNone(env.camera._usb_stream)
            self.assertIsNone(env.camera._imu)
            self.assertIsNone(env.camera._usb_preview)
            order = ['RESULT', 'old_boot', 'imu.quiesce_host_only', 'video.close_local',
                     'finalize', 'disconnect', 'verify_new_boot_only',
                     'imu.retire_host_only', 'preview.retire_host_only', 'detach']
            positions = [env.events.index(event) for event in order]
            self.assertEqual(positions, sorted(positions))
            self.assertTrue(any('sha256sum' in event for event in env.events))
            before = list(env.events)
            self.assertEqual(env.core.recover_and_wait(), result)
            self.assertEqual(env.events, before)

    def test_running_or_unknown_original_task_cannot_start_or_kill_capture(self):
        for unknown in (False, True):
            with self.subTest(unknown=unknown), Environment() as env:
                env.result_running = not unknown
                env.control._unknown_task = unknown
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertNotIn('finalize', env.events)
                self.assertFalse(any(e.startswith('proof:') or e.startswith('imu.') for e in env.events))
                self.assertIs(env.camera._usb_stream, env.facade)

    def test_blocked_then_completed_task_allows_single_explicit_recovery(self):
        with Environment() as env:
            env.control.pending_task, env.result_running = 37, True
            with self.assertRaises(StreamRecoveryError):
                env.core.recover_and_wait()
            self.assertFalse(env.control.finalize_attempted)
            env.result_running = False
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)

    def test_alive_preparation_or_wake_worker_prevents_all_control(self):
        for kind in ('prepare', 'wake'):
            with self.subTest(kind=kind), Environment() as env:
                thread = SimpleNamespace(join=lambda timeout: None, is_alive=lambda: True)
                if kind == 'prepare':
                    env.core._prepare_thread = thread
                    env.core._prepare_done.clear()
                else:
                    env.core._wake_thread = thread
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertEqual(env.events, [])

    def test_failed_or_finished_preparation_error_can_recover(self):
        for finished in (False, True):
            with self.subTest(finished=finished), Environment() as env:
                env.core._prepare_thread = (SimpleNamespace(join=lambda timeout: None,
                    is_alive=lambda: False) if finished else None)
                env.core._prepare_error = RuntimeError('Worker could not start/failed')
                env.core._prepare_done.set()
                self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
                self.assertEqual(env.events.count('finalize'), 1)
                events = list(env.events)
                self.assertEqual(env.core.close()['state'], 'DETACHED')
                self.assertEqual(env.events, events)

    def test_unquiesced_collector_or_video_blocks_finalize(self):
        for kind in ('imu', 'video'):
            with self.subTest(kind=kind), Environment() as env:
                env.fail_imu, env.fail_video = kind == 'imu', kind == 'video'
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertNotIn('finalize', env.events)
                self.assertIs(env.camera._usb_stream, env.facade)

    def test_lost_finalize_ack_then_retry_is_verification_only(self):
        with Environment() as env:
            env.finalize_error, env.disconnect = OSError('ACK lost'), False
            with self.assertRaises(StreamRecoveryError) as got:
                env.core.recover_and_wait()
            self.assertTrue(got.exception.report['reboot_attempted'])
            self.assertTrue(got.exception.report['control']['finalize_attempted'])
            self.assertIs(env.camera._usb_stream, env.facade)
            env.disconnect = True
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)
            self.assertEqual(env.events.count('old_boot'), 1)

    def test_legacy_request_then_wait_shares_once_context(self):
        with Environment() as env:
            result = env.core.recover()
            self.assertEqual(result['state'], 'RECOVERY_PENDING')
            self.assertTrue(result['reboot_requested'])
            self.assertNotIn('verify_new_boot_only', env.events)
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)

    def test_legacy_request_loses_ack_then_wait_never_sends_a_second_finalize(self):
        with Environment() as env:
            env.finalize_error = OSError('Accepted finalize lost ACK')
            requested = env.core.recover()
            self.assertEqual(requested['state'], 'RECOVERY_PENDING')
            self.assertTrue(requested['reboot_attempted'])
            self.assertIsNone(requested['reboot_requested'])
            self.assertIn('ACK', requested['receipt_error'])
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)

    def test_request_only_close_preserves_coordinator_pins_and_can_finish(self):
        with Environment() as env:
            env.core.recover()
            events = list(env.events)
            with self.assertRaisesRegex(RuntimeError, 'continue recover_and_wait'):
                env.core.close()
            self.assertEqual(env.events, events)
            self.assertIs(env.core._preview, env.preview)
            self.assertIs(env.core._imu, env.imu)
            self.assertIs(env.camera._usb_stream, env.facade)
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)

    def test_resumable_wrapper_retires_unknown_shadow_only_after_proof(self):
        with Environment() as env:
            wrapper = ResumableUSBStream.__new__(ResumableUSBStream)
            wrapper._core, wrapper._lock = env.core, threading.RLock()
            wrapper._unknown_shadow = True
            env.camera._usb_stream = wrapper
            self.assertTrue(wrapper.owns_camera_resources)
            self.assertEqual(wrapper.recover_and_wait()['state'], 'DETACHED')
            self.assertFalse(wrapper.owns_camera_resources)
            self.assertTrue(wrapper.cleanup_safe)

    def test_same_boot_or_failed_hydration_never_releases_ownership_or_refinalizes(self):
        for failure in ('sameboot', 'verify'):
            with self.subTest(failure=failure), Environment() as env:
                env.after = BEFORE if failure == 'sameboot' else AFTER
                env.verify_error = TimeoutError('Not hydrated') if failure == 'verify' else None
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertIs(env.camera._usb_stream, env.facade)
                self.assertNotIn('imu.retire_host_only', env.events)
                env.after, env.verify_error = AFTER, None
                self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
                self.assertEqual(env.events.count('finalize'), 1)

    def test_deadline_during_original_result_prevents_late_start(self):
        with Environment() as env:
            original = env.control.resolve_pending
            def late(**options):
                result = original(**options)
                env.now = 121
                return result
            env.control.resolve_pending = late
            with self.assertRaises(StreamRecoveryError):
                env.core.recover_and_wait(timeout=120)
            self.assertNotIn('finalize', env.events)
            self.assertFalse(any(event.startswith('proof:') for event in env.events))

    def test_foreign_capture_or_owner_change_prevents_delivery(self):
        for changed in ('preview', 'owner'):
            with self.subTest(changed=changed), Environment() as env:
                if changed == 'preview':
                    env.camera._usb_preview = object()
                else:
                    old = env.control.resolve_pending
                    def replace(**options):
                        value = old(**options)
                        env.control.owner = dict(pid=99, start='9')
                        return value
                    env.control.resolve_pending = replace
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertNotIn('finalize', env.events)

    def test_failed_source_or_configuration_proof_never_quiesces_or_finalizes(self):
        for failed in ('source', 'config'):
            with self.subTest(failed=failed), Environment() as env:
                if failed == 'source':
                    env.proof_error = ValueError('Source digest changed')
                else:
                    env.patches[1].stop()
                    env.patches[1] = patch('osmo360_camera._stream_recovery._owned_boot',
                                           side_effect=ValueError('Owner/config boot changed'))
                    env.patches[1].start()
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                self.assertFalse(any(event.startswith(('imu.', 'video.')) for event in env.events))
                self.assertNotIn('finalize', env.events)

    def test_late_quiescence_cannot_deliver_finalize_but_later_call_can_retry(self):
        with Environment() as env:
            close = env.video.close_local
            def late(**options):
                result = close(**options)
                env.now = 121
                return result
            env.video.close_local = late
            with self.assertRaises(StreamRecoveryError):
                env.core.recover_and_wait(timeout=120)
            self.assertFalse(env.control.finalize_attempted)
            self.assertNotIn('finalize', env.events)
            env.video.close_local = close
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)

    def test_retirement_failure_retains_exact_new_boot_and_never_refinalizes(self):
        with Environment() as env:
            env.retirement_error = RuntimeError('Host lease cleanup incomplete')
            with self.assertRaises(StreamRecoveryError) as got:
                env.core.recover_and_wait()
            self.assertTrue(got.exception.report['new_boot_verified'])
            self.assertIs(env.camera._usb_stream, env.facade)
            env.retirement_error = None
            env.after = '33333333-3333-3333-3333-333333333333'
            with self.assertRaisesRegex(StreamRecoveryError, 'changed during retirement retry'):
                env.core.recover_and_wait()
            self.assertIs(env.camera._usb_stream, env.facade)
            env.after = AFTER
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)
            self.assertEqual(env.events.count('old_boot'), 1)

    def test_pre_finalize_retry_cannot_adopt_replacement_capture_objects(self):
        for replaced in ('imu', 'video', 'backend_preview', 'backend_receipt'):
            with self.subTest(replaced=replaced), Environment() as env:
                backend = env.core._prepared = SimpleNamespace(camera=env.camera,
                    control=env.control, owner_receipt=env.core._owner_receipt(),
                    preview=env.preview, imu=env.imu)
                env.fail_video = True
                with self.assertRaises(StreamRecoveryError):
                    env.core.recover_and_wait()
                first_events = list(env.events)
                if replaced == 'imu':
                    env.core._imu = env.camera._imu = SimpleNamespace(
                        quiesce_local=lambda **kw: env.fail('Replacement collector was killed'))
                elif replaced == 'video':
                    env.preview._stream = SimpleNamespace(
                        close_local=lambda **kw: env.fail('Replacement video was closed'))
                elif replaced == 'backend_preview':
                    backend.preview = object()
                else:
                    backend.owner_receipt = dict(env.core._owner_receipt(), owner=dict(pid=99, start='9'))
                env.fail_video = False
                with self.assertRaisesRegex(StreamRecoveryError, 'identity changed'):
                    env.core.recover_and_wait()
                self.assertEqual(env.events, first_events)
                self.assertFalse(env.control.finalize_attempted)

    def test_detach_failure_before_commit_preserves_resources_for_verify_only_retry(self):
        with Environment() as env:
            detach = env.control.detach
            def fail():
                raise RuntimeError('Local detach not completed')
            env.control.detach = fail
            with self.assertRaisesRegex(StreamRecoveryError, 'detach not completed'):
                env.core.recover_and_wait()
            self.assertIs(env.camera._usb_stream, env.facade)
            self.assertIs(env.core._preview, env.preview)
            self.assertIs(env.core._imu, env.imu)
            env.control.detach = detach
            self.assertEqual(env.core.recover_and_wait()['state'], 'DETACHED')
            self.assertEqual(env.events.count('finalize'), 1)


if __name__ == '__main__':
    unittest.main()
