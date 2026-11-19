"""Public shutdown uses stock Sleep and resumable RAM data, never AV capture."""

from contextlib import ExitStack
from copy import deepcopy
import json
import shlex
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import Camera
from osmo360_camera import _shutdown as m
from osmo360_camera import _stream_resume as resume
from osmo360_camera._stream_session import _IdleEvidence
from test_prepared_stream import runtime
from test_standby_control import snapshot, NONCE

BOOT = '11111111-1111-1111-1111-111111111111'
WORK = '/dev/shm/osmo-usb-standby-' + NONCE


class Manager:
    def __init__(self, events):
        self.events = events
        self.clock, self.timeout = time.monotonic, 90

    def ensure(self, **options):
        self.events.append('readiness')

    def idle(self, camera):
        self.events.append('idle')
        return _IdleEvidence(time.monotonic(), 0, 1, True)


class ShutdownCameraTests(unittest.TestCase):
    def setUp(self):
        self.events, self.metadata, self.cores = [], [], []
        self.camera = Camera(transport=SimpleNamespace(connected=True, _thread=object(),
            close=Mock(side_effect=lambda: self.events.append('local_close'))))
        self.camera.connect = Mock(side_effect=lambda **_: self.events.append('connect') or self.camera)
        self.current = snapshot('SLEEP', 1)
        self.control = SimpleNamespace(pending_command=None, pending_task=None, _unknown_task=False,
            work=WORK, nonce=NONCE, owner=self.current['owner'], service=self.current['service'],
            last=deepcopy(self.current),
            bootstrap=SimpleNamespace(session=SimpleNamespace(quarantined=False),
                                      query_runtime=Mock(return_value=runtime())))
        self.control._run = Mock(side_effect=lambda cmd, **_: self.events.append('guard_verified'))
        self.control.snapshot = Mock(side_effect=lambda: deepcopy(self.current))
        self.control.wait_sleep_snapshot = Mock(side_effect=lambda *_, **__: deepcopy(self.current))
        self.control.detach = Mock(return_value=dict(pending_task=None, unknown_task=False))
        self.control.launch = Mock(side_effect=lambda **_: self.events.append('owner_launch'))
        self.control._submit = Mock(side_effect=self.submit)
        self.control.wait = Mock(side_effect=lambda *_, **__: deepcopy(self.current))
        self.prepared = SimpleNamespace(sleep_id='0000000000000001', assets=object(),
            prepare=Mock(side_effect=lambda: self.events.append('stage_unused_plan')))
        self.factory = Mock(return_value=self.prepared)
        self.factory.preload_assets = Mock(side_effect=lambda *_, **__: self.events.append('cache_sources'))
        self.factory.source_provider = lambda: {}
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(m, 'attached_camera_count', return_value=1))
        self.stack.enter_context(patch.object(m, 'build_stream_session', side_effect=self.core))
        self.stack.enter_context(patch.object(m, '_fresh_boot', return_value=BOOT))
        self.discovery = self.stack.enter_context(patch.object(m, '_discover', return_value=WORK))
        self.stack.enter_context(patch.object(m, '_status_at', side_effect=lambda *_, **__: deepcopy(self.current)))
        self.apps = self.stack.enter_context(patch.object(m, '_apps_absent', return_value=SimpleNamespace(
            all_apps_absent=True, count=0, own_records=[])))
        self.read_metadata = self.stack.enter_context(patch.object(m, '_read_metadata',
            side_effect=lambda *_, **__: dict(native_floor_ns=0)))
        self.restore = self.stack.enter_context(patch.object(m, 'restore_prepared', return_value=self.prepared))
        self.stack.enter_context(patch.object(m, 'export_prepared', return_value=dict(native_floor_ns=0)))
        self.stack.enter_context(patch.object(m, '_publish_metadata', side_effect=self.publish))

    def core(self, camera, **options):
        core = SimpleNamespace(control=self.control, idle_probe=Manager(self.events).idle,
            prepared_factory=self.factory, imu_options={}, state='NEW',
            start=Mock(side_effect=AssertionError('AV capture is forbidden in shutdown')),
            wake=Mock(side_effect=lambda **_: self.events.append('wake_once')),
            report={}, fast_sleep=True, _prepared=None)
        core.attach_sleep = Mock(side_effect=lambda backend, **_: setattr(core, 'state', 'SLEEP'))
        self.cores.append(core)
        return core

    def publish(self, control, metadata, **options):
        self.events.append('publish_resume_metadata')
        self.metadata.append(deepcopy(metadata))
        return dict(published=True)

    def submit(self, action, proof, **options):
        self.assertEqual(action, 'sleep')
        proof.validate(time.monotonic())
        self.events.append('one_sleep')
        self.current = snapshot('SLEEP', 1)
        return dict(command_id=self.current['command_id'])

    def test_existing_sleep_is_read_only_then_closes_local_usb_once(self):
        proof = self.camera.prepare_shutdown(expected_sleep=deepcopy(self.current), expected_boot_id=BOOT)
        self.assertTrue(proof['ready'])
        self.assertTrue(proof['native_usb_restored'])
        self.assertTrue(proof['resume_metadata_verified'])
        self.assertTrue(proof['guard_cancelled'])
        self.assertTrue(proof['local_control_closed'])
        self.control._submit.assert_not_called()
        self.factory.assert_not_called()
        self.restore.assert_called_once()
        self.assertEqual(self.events[-1], 'local_close')
        before = list(self.events)
        self.assertEqual(self.camera.prepare_shutdown(), proof)
        self.assertEqual(before, self.events)
        self.camera.close()
        self.assertEqual(self.control._submit.call_count, 0)

    def test_standalone_awake_stages_unarmed_resume_plan_then_one_sleep_and_no_av(self):
        self.current = snapshot('AWAKE', 0)
        proof = self.camera.prepare_shutdown()
        self.assertTrue(proof['ready'])
        self.assertEqual(self.control._submit.call_count, 1)
        self.control.launch.assert_not_called()
        self.assertLess(self.events.index('stage_unused_plan'), self.events.index('one_sleep'))
        self.assertLess(self.events.index('one_sleep'), self.events.index('publish_resume_metadata'))
        self.assertLess(self.events.index('publish_resume_metadata'), self.events.index('local_close'))
        self.assertTrue(all(core.start.call_count == 0 for core in self.cores))

    def test_standalone_without_owner_launches_pinned_owner_but_never_preview(self):
        self.current = snapshot('AWAKE', 0)
        self.discovery.return_value = None
        self.assertTrue(self.camera.prepare_shutdown()['ready'])
        self.control.launch.assert_called_once_with(recording_state=0, battery_present=True, activation_state=1)
        self.control._submit.assert_called_once()
        self.assertLess(self.events.index('owner_launch'), self.events.index('one_sleep'))

    def test_shutdown_then_new_camera_resumes_existing_standard_metadata_with_only_one_wake(self):
        self.current = snapshot('AWAKE', 0)
        self.camera.prepare_shutdown()
        self.assertEqual(self.control._submit.call_count, 1)
        next_camera = Camera(transport=SimpleNamespace(connected=True, _thread=object(), close=Mock()))
        next_camera.connect = Mock()
        with (patch.object(resume, 'build_stream_session', side_effect=self.core),
              patch.object(resume, '_discover', return_value=WORK),
              patch.object(resume, '_status_at', side_effect=lambda *_, **__: deepcopy(self.current)),
              patch.object(resume, '_read_metadata', return_value=self.metadata[-1]),
              patch.object(resume, 'restore_prepared', return_value=self.prepared),
              patch.object(resume, '_wait_native') as rollback):
            stream = resume.ResumableUSBStream(next_camera, decode=False, fast_sleep=True, retain_ecm=True)
            stream.start()
        self.cores[-1].wake.assert_called_once_with(timeout=60)
        self.cores[-1].start.assert_not_called()
        rollback.assert_not_called()
        self.assertEqual(self.control._submit.call_count, 1)  # No second Sleep.

    def test_unknown_pending_original_owner_blocks_new_controller_and_input(self):
        self.control.pending_task = 77
        self.camera._usb_stream = SimpleNamespace(cleanup_safe=True, _core=SimpleNamespace(control=self.control))
        with self.assertRaisesRegex(RuntimeError, 'pending'):
            self.camera.prepare_shutdown()
        self.camera.connect.assert_not_called()
        self.control._submit.assert_not_called()

    def test_expected_sleep_owner_change_never_attempts_input(self):
        expected = deepcopy(self.current)
        self.current['owner'] = dict(pid=999, start='123')
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            self.camera.prepare_shutdown(expected_sleep=expected)
        self.control._submit.assert_not_called()
        self.assertFalse(self.camera._shutdown_report['ready'])

    def test_boot_change_and_missing_metadata_are_terminal_without_replay(self):
        with self.assertRaisesRegex(RuntimeError, 'boot changed'):
            self.camera.prepare_shutdown(expected_boot_id='22222222-2222-2222-2222-222222222222')
        with self.assertRaisesRegex(RuntimeError, 'no input replay'):
            self.camera.prepare_shutdown()
        self.control._submit.assert_not_called()

    def test_lost_sleep_submission_is_not_replayed_and_keeps_controller(self):
        self.current = snapshot('AWAKE', 0)
        self.control._submit.side_effect = TimeoutError('Unknown Sleep delivery')
        with self.assertRaisesRegex(TimeoutError, 'Unknown Sleep'):
            self.camera.prepare_shutdown()
        with self.assertRaisesRegex(RuntimeError, 'no input replay'):
            self.camera.prepare_shutdown()
        self.control._submit.assert_called_once()
        self.assertIs(self.camera._shutdown_control, self.control)
        self.camera._transport.close.assert_not_called()

    def test_guard_or_local_close_failure_never_returns_ready(self):
        self.control._run.side_effect = RuntimeError('RTC guard cancellation unavailable')
        with self.assertRaisesRegex(RuntimeError, 'RTC'):
            self.camera.prepare_shutdown()
        self.assertFalse(self.camera._shutdown_report['ready'])
        self.camera._transport.close.assert_not_called()

    def test_missing_resume_metadata_and_failed_local_close_are_not_ready(self):
        self.read_metadata.side_effect = RuntimeError('No usable same-boot resume metadata')
        with self.assertRaisesRegex(RuntimeError, 'resume metadata'):
            self.camera.prepare_shutdown()
        self.camera._transport.close.assert_not_called()
        self.control._submit.assert_not_called()
        self.assertFalse(self.camera._shutdown_report['ready'])

    def test_local_control_close_failure_preserves_failed_receipt_without_replay(self):
        self.camera._transport.close.side_effect = RuntimeError('USB reader still alive')
        with self.assertRaisesRegex(RuntimeError, 'USB reader'):
            self.camera.prepare_shutdown()
        self.assertFalse(self.camera._shutdown_report['ready'])
        with self.assertRaisesRegex(RuntimeError, 'no input replay'):
            self.camera.prepare_shutdown()
        self.control._submit.assert_not_called()

    def test_bad_native_runtime_cannot_be_treated_as_closed_capture(self):
        self.control.bootstrap.query_runtime.side_effect = RuntimeError('Unexpected ECM f6 remains')
        with self.assertRaisesRegex(RuntimeError, 'ECM'):
            self.camera.prepare_shutdown()
        self.assertFalse(self.camera._shutdown_report['ready'])
        self.control._submit.assert_not_called()

    def test_real_guard_command_is_bounded_compiles_and_only_reads_pinned_source(self):
        self.camera.prepare_shutdown()
        command = self.control._run.call_args.args[0]
        self.assertLessEqual(len(command), 500)
        program = shlex.split(command)[-1]
        compile(program, 'shutdown-guard-proof', 'exec')
        self.assertIn('guard-finished.json', program)
        self.assertIn('closed_verified_ebadf', program)
        self.assertNotIn('m.main', program)
        self.assertNotIn('os.kill', program)


if __name__ == '__main__':
    unittest.main()
