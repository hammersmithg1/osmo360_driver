"""Failure snapshots read existing Python evidence only, without new I/O."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import _prepared_stream as prepared
import test_prepared_stream as fixtures
import test_stream_session as sessions


class PreparedFailureDiagnosticsTests(unittest.TestCase):
    def backend(self):
        fixture = fixtures.PreparedTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        return fixture, fixture.armed()

    def test_network_failure_preserves_mutable_guard_report_and_original_exception(self):
        fixture, value = self.backend()
        error = TimeoutError('No new unmanaged ECM interface')
        fixture.network.report = dict(events=[], ready_observation=dict(interface=None, count=7))
        fixture.network.wait_ready = Mock(side_effect=error)
        commands = list(fixture.control.commands)
        with self.assertRaises(prepared.UnknownFutureApply) as caught:
            value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.assertIs(caught.exception.__cause__, error)
        failure = value.report['attach_failure']
        self.assertEqual(failure['phase'], 'host_network')
        fixture.network.report['ready_observation']['count'] = 999
        self.assertEqual(failure['network']['ready_observation']['count'], 7)
        self.assertEqual(fixture.control.commands, commands)
        self.assertNotIn('lease_start', fixture.events)
        self.assertNotIn('video_start', fixture.events)
        self.assertNotIn('imu_start', fixture.events)
        self.assertTrue(value.possible_future_apply)

    def test_missing_or_uncopyable_network_report_never_changes_the_failure(self):
        class Uncopyable:
            def __deepcopy__(self, memo):
                raise RuntimeError('Cannot copy diagnostic report')
        for report in (None, Uncopyable()):
            with self.subTest(report=type(report).__name__):
                fixture, value = self.backend()
                if report is not None:
                    fixture.network.report = report
                error = OSError('Original network observation failed')
                fixture.network.wait_ready = Mock(side_effect=error)
                with self.assertRaises(prepared.UnknownFutureApply) as caught:
                    value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
                self.assertIs(caught.exception.__cause__, error)
                if report is None:
                    self.assertIsNone(value.report['attach_failure']['network'])
                else:
                    self.assertEqual(value.report['attach_diagnostics_error'], 'RuntimeError')

    def test_lease_failure_is_distinguished_from_network_discovery(self):
        fixture, value = self.backend()
        lease = fixtures.Lease(fixture.events, {})
        lease.start = Mock(side_effect=TimeoutError('Lease ACK missing'))
        value.lease_factory = lambda config: lease
        with self.assertRaises(prepared.UnknownFutureApply):
            value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.assertEqual(value.report['attach_failure']['phase'], 'lease_start')
        self.assertFalse(value.lease_proven)


class WakeControlDiagnosticsTests(unittest.TestCase):
    def failure(self, *, submission_error=False, diagnostic_error=False):
        env = sessions.Environment()
        session = env.session().start()
        session.sleep()
        native = SimpleNamespace(quarantined=True, owned_task_id=77, operations=[
            dict(command='fixed status', task_id=77, start_accepted=True,
                 start_attempts=1, completed=False, exchanges=[{'not': 'copied'}])])
        env.control.bootstrap = SimpleNamespace(session=native)
        submit = env.control.submit_wake
        def queued(**options):
            env.control.pending_command, env.control.pending_task = '0000000000000002', 77
            env.control._unknown_task = submission_error
            result = submit(**options)
            if submission_error:
                raise TimeoutError('Wake submission acknowledgement missing')
            return result
        env.control.submit_wake = queued
        error = TimeoutError('Host network discovery deadline')
        def failed_attach(**options):
            session._wake_thread.join(1)  # Test only: make sampled evidence deterministic.
            self.assertFalse(session._wake_thread.is_alive())
            raise error
        env.prepared.attach_data_first = failed_attach
        if diagnostic_error:
            session._wake_control_diagnostics = Mock(side_effect=ValueError('Injected diagnostic failure'))
        with self.assertRaises(TimeoutError) as caught:
            session.wake(timeout=5)
        self.assertIs(caught.exception, error)
        return env, session, native

    def test_wake_failure_freezes_task_id_quarantine_and_acknowledged_submission(self):
        env, session, native = self.failure()
        evidence = session.stats['report']['wake_failure']['control']
        self.assertEqual(evidence['pending_command'], '0000000000000002')
        self.assertEqual(evidence['pending_task'], 77)
        self.assertEqual(evidence['owned_task_id'], 77)
        self.assertTrue(evidence['quarantined'])
        self.assertFalse(evidence['wake_thread_alive'])
        self.assertEqual(evidence['wake_submission']['command_id'], '0000000000000002')
        self.assertNotIn('exchanges', evidence['last_operation'])
        native.operations[-1]['task_id'] = 123
        self.assertEqual(session.stats['report']['wake_failure']['control']['last_operation']['task_id'], 77)
        self.assertEqual(env.events.count('owner.wake'), 1)
        self.assertNotIn('owner.detach', env.events)
        self.assertNotIn('owner.finalize', env.events)

    def test_lost_wake_ack_keeps_error_and_unknown_task_without_an_input_retry(self):
        env, session, _ = self.failure(submission_error=True)
        evidence = session.stats['report']['wake_failure']['control']
        self.assertTrue(evidence['unknown_task'])
        self.assertIsNone(evidence['wake_submission'])
        self.assertEqual(evidence['wake_error']['type'], 'TimeoutError')
        self.assertEqual(env.events.count('owner.wake'), 1)

    def test_diagnostic_exception_cannot_mask_primary_network_failure(self):
        _, session, _ = self.failure(diagnostic_error=True)
        self.assertEqual(session.stats['report']['wake_failure']['control_diagnostics_error'], 'ValueError')


if __name__ == '__main__':
    unittest.main()
