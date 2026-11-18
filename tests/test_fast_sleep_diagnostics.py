"""Host-only failure snapshots never query, retry, clean up or hide an error."""

from contextlib import ExitStack
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _stream_session as m
from osmo360_camera.transport import CameraDisconnected
import test_stream_session_retained as retained_fixtures


class FastSleepDiagnosticsTests(unittest.TestCase):
    def setup_session(self):
        fixture = retained_fixtures.RetainedSessionTests()
        env, session, old, factory, prepared = fixture.setup_session()
        self.addCleanup(fixture.doCleanups)
        env.control.pending_command = None
        env.control.pending_task, env.control._unknown_task = 7, True
        env.control.bootstrap.session = SimpleNamespace(owned_task_id=7, quarantined=True,
            operations=[dict(command='python3 owned status', task_id=7, start_accepted=True,
                completed=False, result=None, error='LIBUSB_ERROR_IO')])
        env.control.last_status_error = dict(error='LIBUSB_ERROR_IO', task_id=7)
        old._lease.last_ack = dict(sequence=17, camera_monotonic=202.0, expires=212.0, state=1)
        old._lease.stats = dict(sent=17, acknowledged=17, timeouts=0)
        return env, session, old, factory

    def assert_original_failure(self, env, session, old, error, stage):
        before = list(env.events)
        with self.assertRaises(CameraDisconnected) as caught:
            session.sleep()
        self.assertIs(caught.exception, error)
        receipt = session.report['sleep_failure']
        self.assertEqual(receipt['stage'], stage)
        self.assertEqual(receipt['errors'][0], dict(type='CameraDisconnected', args=['LIBUSB_ERROR_IO']))
        self.assertEqual(receipt['control']['pending_task'], 7)
        self.assertTrue(receipt['control']['unknown_task'])
        self.assertTrue(receipt['control']['quarantined'])
        self.assertEqual(receipt['last_operation']['command'], 'python3 owned status')
        self.assertEqual(receipt['lease']['stats'], dict(sent=17, acknowledged=17, timeouts=0))
        self.assertEqual(receipt['lease']['last_ack']['expires'], 212.0)
        self.assertEqual(session.state, 'RECOVERY_REQUIRED')
        with self.assertRaises(m.StreamSessionError):
            session.close()
        self.assertNotIn('owner.finalize', env.events[len(before):])
        self.assertNotIn('usb.full_stop', env.events[len(before):])
        self.assertNotIn('owner.wake', env.events[len(before):])
        old._lease.stop.assert_not_called()
        self.assertLessEqual(len(json.dumps(receipt).encode()), 8192)
        return receipt

    def test_each_pre_input_failure_is_identified_without_recovery_commands(self):
        for stage in ('retained_assets', 'retained_owner_snapshot', 'retained_prepare',
                      'app_unregister', 'local_capture_stop', 'idle_readback', 'final_app_query',
                      'submit_local_sleep'):
            with self.subTest(stage=stage):
                env, session, old, factory = self.setup_session()
                error = CameraDisconnected('LIBUSB_ERROR_IO')
                with ExitStack() as stack:
                    if stage == 'retained_assets':
                        stack.enter_context(patch.object(factory, 'preload_assets', side_effect=error))
                    elif stage == 'retained_owner_snapshot':
                        stack.enter_context(patch.object(env.control, 'snapshot', side_effect=error))
                    elif stage == 'retained_prepare':
                        stack.enter_context(patch.object(retained_fixtures.Retained, 'prepare', side_effect=error))
                    elif stage == 'app_unregister':
                        stack.enter_context(patch.object(session, '_unregister_app', side_effect=error))
                    elif stage == 'local_capture_stop':
                        stack.enter_context(patch.object(session, '_stop_local_capture', side_effect=error))
                    elif stage == 'idle_readback':
                        stack.enter_context(patch.object(session, '_idle', side_effect=error))
                    elif stage == 'final_app_query':
                        stopped, stop, query = [], session._stop_local_capture, session._app_query
                        def local_stop():
                            result = stop()
                            stopped.append(True)
                            return result
                        def final_query():
                            if stopped: raise error
                            return query()
                        stack.enter_context(patch.object(session, '_stop_local_capture', side_effect=local_stop))
                        stack.enter_context(patch.object(session, '_app_query', side_effect=final_query))
                    else:
                        stack.enter_context(patch.object(m, 'submit_local_sleep', side_effect=error))
                    receipt = self.assert_original_failure(env, session, old, error, stage)
                self.assertFalse(receipt['submission_returned'])
                self.assertNotIn('owner.sleep', env.events)

    def test_post_submit_failure_keeps_pending_sleep_and_exact_original_exception(self):
        env, session, old, factory = self.setup_session()
        error, submit, wait = CameraDisconnected('LIBUSB_ERROR_IO'), env.control._submit, env.control.wait
        def submitted(action, proof):
            result = submit(action, proof)
            env.control.pending_command = result['command_id']
            return result
        def status(identifier, target, **options):
            if target == 'SLEEP': raise error
            return wait(identifier, target, **options)
        with patch.object(env.control, '_submit', side_effect=submitted), patch.object(env.control, 'wait', side_effect=status):
            receipt = self.assert_original_failure(env, session, old, error, 'wait_native_sleep')
        self.assertTrue(receipt['submission_returned'])
        self.assertEqual(receipt['sleep_id'], '0000000000000001')
        self.assertEqual(receipt['control']['pending_command'], '0000000000000001')
        self.assertEqual(env.events.count('owner.sleep'), 1)
        self.assertIs(session._retained_backend, factory.created[0])

    def test_snapshot_is_bounded_copied_and_never_reprs_arbitrary_objects(self):
        env, session, old, factory = self.setup_session()
        class NoRepr:
            def __repr__(self): raise AssertionError('unbounded repr')
        op = env.control.bootstrap.session.operations[-1]
        op.update(command='x' * 100000, error=NoRepr(), result=2**100000)
        error = CameraDisconnected('LIBUSB_ERROR_IO')
        with patch.object(session, '_idle', side_effect=error), self.assertRaises(CameraDisconnected) as caught:
            session.sleep()
        self.assertIs(caught.exception, error)
        receipt = session.report['sleep_failure']
        self.assertEqual(receipt['last_operation']['command'], 'x' * 256)
        self.assertIsNone(receipt['last_operation']['error'])
        self.assertIsNone(receipt['last_operation']['result'])
        old._lease.last_ack['expires'] = 999.0
        op['command'] = 'changed later'
        self.assertEqual(receipt['lease']['last_ack']['expires'], 212.0)
        self.assertEqual(receipt['last_operation']['command'], 'x' * 256)
        self.assertLessEqual(len(json.dumps(receipt).encode()), 8192)

    def test_lost_input_receipt_keeps_possible_sleep_and_never_retries_submission(self):
        env, session, old, factory = self.setup_session()
        error = m.StandbyUncertain('Sleep receipt lost', command_id='0000000000000001')
        error.__cause__ = CameraDisconnected('LIBUSB_ERROR_IO')
        def lost(control, proof):
            control.pending_command = '0000000000000001'
            raise error
        with patch.object(m, 'submit_local_sleep', side_effect=lost) as submit:
            with self.assertRaises(m.StandbyUncertain) as caught:
                session.sleep()
        self.assertIs(caught.exception, error)
        submit.assert_called_once()
        receipt = session.report['sleep_failure']
        self.assertEqual(receipt['stage'], 'submit_local_sleep')
        self.assertFalse(receipt['submission_returned'])
        self.assertEqual(receipt['sleep_id'], '0000000000000001')
        self.assertEqual(receipt['control']['pending_command'], '0000000000000001')
        self.assertEqual([row['type'] for row in receipt['errors']], ['StandbyUncertain', 'CameraDisconnected'])
        self.assertIs(session._retained_backend, factory.created[0])
        with self.assertRaises(m.StreamSessionError): session.close()
        old._lease.stop.assert_not_called()
        self.assertNotIn('owner.finalize', env.events)

    def test_diagnostic_failure_cannot_replace_primary_error_or_relax_guard(self):
        env, session, old, factory = self.setup_session()
        error = CameraDisconnected('LIBUSB_ERROR_IO')
        with (patch.object(session, '_idle', side_effect=error),
              patch.object(session, '_sleep_failure_diagnostics', side_effect=MemoryError('no diagnostic'))):
            with self.assertRaises(CameraDisconnected) as caught:
                session.sleep()
        self.assertIs(caught.exception, error)
        self.assertEqual(session.report['sleep_failure'], dict(stage='idle_readback', diagnostic_error_type='MemoryError'))
        self.assertEqual(session.state, 'RECOVERY_REQUIRED')
        with self.assertRaises(m.StreamSessionError): session.close()
        old._lease.stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()
