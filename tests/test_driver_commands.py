"""Client deadlines/cancellation never create another camera operation."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_driver.commands import (
    CameraCommandError, CameraCommandCancelled, run_command, wait_idle, prepare, start,
    prepare_shutdown,
)


class CommandTests(unittest.TestCase):
    def owner(self, states, requested=None):
        self.now = 0
        self.cancelled = False
        def sleep(seconds):
            self.now += seconds
            self.cancelled = True
        self.options = dict(clock=lambda: self.now, sleep=sleep)
        return SimpleNamespace(command=Mock(), status=Mock(side_effect=states), usb_charging=requested)

    def test_preparation_validates_actual_requested_policy(self):
        state = dict(busy='', error='', power=dict(requested=False, ready=True))
        owner = self.owner([dict(busy='prepare_power'), state], requested=False)
        self.assertIs(prepare(owner, **self.options), state)
        owner.command.assert_called_once_with('prepare_power', None)
        for power in ({'ready': True}, {'ready': True, 'requested': None},
                      {'ready': False, 'requested': False},
                      {'ready': True, 'requested': False, 'error': 'readback failed'}):
            owner = self.owner([dict(busy='', error='', power=power)], requested=False)
            with self.subTest(power=power), self.assertRaises(CameraCommandError):
                prepare(owner, **self.options)

    def test_command_failure_keeps_exact_status_copy(self):
        state = dict(busy='', error='unknown task', stats=dict(pending_task=12))
        owner = self.owner([state])
        with self.assertRaises(CameraCommandError) as caught:
            run_command(owner, 'start', **self.options)
        state['stats']['pending_task'] = 99
        self.assertEqual(caught.exception.status['stats']['pending_task'], 12)
        owner.command.assert_called_once_with('start', None)

    def test_cancellation_before_delivery_does_not_queue(self):
        owner = self.owner([dict(busy='')])
        with self.assertRaises(CameraCommandCancelled):
            run_command(owner, 'start', cancelled=lambda: True, **self.options)
        owner.command.assert_not_called()

    def test_cancellation_after_delivery_observes_completion_before_raising(self):
        owner = self.owner([dict(busy='start'), dict(busy='start'), dict(busy='', enabled=True)])
        with self.assertRaises(CameraCommandCancelled):
            run_command(owner, 'start', cancelled=lambda: self.cancelled, **self.options)
        self.assertEqual(owner.status.call_count, 3)
        owner.command.assert_called_once_with('start', None)

    def test_timeout_does_not_replay_or_close_pending_operation(self):
        owner = self.owner([])
        owner.status.side_effect = None
        owner.status.return_value = dict(busy='start', stats=dict(pending_task=7))
        with self.assertRaisesRegex(CameraCommandError, 'not replayed') as caught:
            run_command(owner, 'start', timeout=.1, **self.options)
        self.assertEqual(caught.exception.status['stats']['pending_task'], 7)
        owner.command.assert_called_once_with('start', None)

    def test_start_requires_both_fresh_channels(self):
        valid = dict(busy='', error='', state='streaming', enabled=True, frame_age_s=.1, imu_age_s=.01)
        owner = self.owner([valid])
        self.assertIs(start(owner, **self.options), valid)
        for field, value in (('enabled', False), ('state', 'error'),
                             ('frame_age_s', None), ('frame_age_s', 2),
                             ('imu_age_s', None), ('imu_age_s', float('nan'))):
            owner = self.owner([dict(valid, **{field: value})])
            owner.status.return_value = dict(valid, **{field: value})
            owner.status.side_effect = None
            with self.subTest(field=field, value=value), self.assertRaises(CameraCommandError):
                start(owner, timeout=.1, **self.options)

    def test_delayed_first_reader_callbacks_are_observed_without_second_start(self):
        pending = dict(busy='', error='', state='streaming', enabled=True,
                       frame_age_s=None, imu_age_s=None)
        ready = dict(pending, frame_age_s=.01, imu_age_s=.005)
        owner = self.owner([dict(busy='start'), pending, dict(pending, imu_age_s=.01), ready])
        self.assertIs(start(owner, timeout=.3, **self.options), ready)
        self.assertGreater(self.now, .1)
        owner.command.assert_called_once_with('start', None)

    def test_first_sample_wait_shares_original_command_budget(self):
        pending = dict(busy='', error='', state='streaming', enabled=True,
                       frame_age_s=None, imu_age_s=None)
        owner = self.owner([dict(busy='start'), dict(busy='start'), pending, pending, pending])
        with self.assertRaisesRegex(CameraCommandError, 'Fresh video/IMU deadline'):
            start(owner, timeout=.15, **self.options)
        self.assertAlmostEqual(self.now, .15)
        owner.command.assert_called_once_with('start', None)

    def test_wait_idle_observes_prior_error_without_issuing_another_command(self):
        final = dict(busy='', error='prior configure failed')
        owner = self.owner([dict(busy='configure'), final])
        self.assertIs(wait_idle(owner, **self.options), final)
        owner.command.assert_not_called()

    def test_shutdown_finishes_existing_transaction_then_issues_one_preparation(self):
        pending = dict(busy='configure')
        idle = dict(busy='', error='')
        ready = dict(idle, shutdown=dict(ready=True, local_control_closed=True))
        owner = self.owner([pending, idle, dict(busy='prepare_shutdown'), ready])
        def command(*args):
            self.assertEqual(owner.status.call_count, 2)
        owner.command.side_effect = command
        self.assertIs(prepare_shutdown(owner, timeout=.3, **self.options), ready)
        owner.command.assert_called_once_with('prepare_shutdown', None)

    def test_shutdown_timeout_of_existing_transaction_never_sends_prepare(self):
        owner = self.owner([])
        owner.status.side_effect = None
        owner.status.return_value = dict(busy='configure')
        with self.assertRaises(CameraCommandError):
            prepare_shutdown(owner, timeout=.1, **self.options)
        owner.command.assert_not_called()

    def test_shutdown_wait_and_prepare_share_one_deadline(self):
        owner = self.owner([dict(busy='configure'), dict(busy='configure'),
                            dict(busy=''), dict(busy='prepare_shutdown'),
                            dict(busy='prepare_shutdown')])
        with self.assertRaises(CameraCommandError):
            prepare_shutdown(owner, timeout=.15, **self.options)
        self.assertAlmostEqual(self.now, .15)
        owner.command.assert_called_once_with('prepare_shutdown', None)

    def test_shutdown_rejects_missing_proof_or_camera_error_without_replay(self):
        for state in (dict(busy='', error='unknown task', shutdown=dict(ready=True)),
                      dict(busy='', error='', shutdown=dict(ready=False)),
                      dict(busy='', error='', shutdown=None)):
            owner = self.owner([dict(busy=''), state])
            with self.subTest(state=state), self.assertRaises(CameraCommandError):
                prepare_shutdown(owner, **self.options)
            owner.command.assert_called_once_with('prepare_shutdown', None)


if __name__ == '__main__':
    unittest.main()
