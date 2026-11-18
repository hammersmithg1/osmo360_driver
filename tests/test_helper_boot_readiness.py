"""Installed helper hydration is observed without a second power command."""

from collections import deque
from copy import deepcopy
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera import _helper_install as m, _helper_lifecycle as lifecycle
from unittest.mock import Mock
from test_helper_install import Control, BOOT1, BOOT2

BOOT3 = '33333333-3333-3333-3333-333333333333'


class SequenceControl(Control):
    def __init__(self, package, observations):
        super().__init__(package, installed=True)
        self.observations = deque(observations)

    def response(self, operation, nonce):
        spec = self.observations[0]
        if operation == 'status' and len(self.observations) > 1:
            spec = self.observations.popleft()
        self.boot_id, self.hydrated = spec['boot'], spec['cache'] == 'ready'
        value = super().response(operation, nonce)
        if self.installed and not self.hydrated:
            value['boot_cache'].pop('mode')
            if spec['cache'] == 'pending':
                value['boot_cache']['phase'] = 'pending'
            else:
                value['boot_cache'].pop('work')
        if operation == 'status' and spec.get('change'):
            spec['change'](value)
        return value


class BootReadinessTests(unittest.TestCase):
    def setUp(self):
        self.package = m.package_profile()
        self.profile = self.package['manifest']['profile']
        self.camera = SimpleNamespace(_control_lock=threading.RLock())

    def verify(self, control, **options):
        return m.verify_boot_cache(self.camera, self.profile, control=control, **options)

    def assert_readonly(self, control):
        self.assertTrue(all(operation == 'status' for operation in control.operations))
        self.assertFalse(any('base64 -d' in command or ' reboot' in command or ' install ' in command
                             for command in control.commands))

    def test_pending_cache_finishes_in_the_same_boot_without_installation(self):
        control = SequenceControl(self.package, [dict(boot=BOOT2, cache='pending'), dict(boot=BOOT2, cache='ready')])
        value = self.verify(control, current_boot_id=BOOT2, timeout=1)
        self.assertEqual(value['boot_id'], BOOT2)
        self.assertTrue(value['autostart_hydration_verified'])
        self.assertEqual(value['hydration_wait']['attempts'], 2)
        self.assertEqual(value['hydration_wait']['mode'], 'same-boot-pending')
        self.assert_readonly(control)

    def test_known_previous_boot_allows_missing_then_pending_then_ready_on_one_new_boot(self):
        control = SequenceControl(self.package, [dict(boot=BOOT2, cache=kind) for kind in ('absent', 'pending', 'ready')])
        value = self.verify(control, previous_boot_id=BOOT1, timeout=1)
        self.assertEqual(value['hydration_wait']['boot_id_before'], BOOT1)
        self.assertEqual(value['hydration_wait']['boot_id_after'], BOOT2)
        self.assertEqual(value['hydration_wait']['attempts'], 3)
        self.assert_readonly(control)

    def test_observed_new_boot_cannot_change_again_or_revert_to_previous_boot(self):
        for changed in (BOOT1, BOOT3):
            with self.subTest(changed=changed):
                control = SequenceControl(self.package, [dict(boot=BOOT2, cache='pending'),
                                                        dict(boot=changed, cache='ready')])
                with self.assertRaisesRegex(m.HelperInstallError, 'boot (changed|did not change)'):
                    self.verify(control, previous_boot_id=BOOT1, timeout=1)
                self.assert_readonly(control)

    def test_same_previous_boot_is_not_accepted_as_a_completed_public_reboot(self):
        control = SequenceControl(self.package, [dict(boot=BOOT1, cache='ready')])
        with self.assertRaisesRegex(m.HelperInstallError, 'did not change'):
            self.verify(control, previous_boot_id=BOOT1, timeout=1)
        self.assert_readonly(control)

    def test_missing_or_persistent_pending_cache_times_out_without_repair(self):
        for kind in ('pending', 'absent'):
            with self.subTest(kind=kind):
                control = SequenceControl(self.package, [dict(boot=BOOT2, cache=kind)])
                with self.assertRaises(TimeoutError):
                    self.verify(control, previous_boot_id=BOOT1, timeout=.15)
                self.assertLessEqual(control.clock(), .15)
                self.assert_readonly(control)

    def test_disappearing_permanent_root_or_pending_cache_is_never_reinstalled(self):
        control = SequenceControl(self.package, [dict(boot=BOOT2, cache='absent')])
        with self.assertRaisesRegex(m.HelperInstallError, 'pending.*disappeared'):
            self.verify(control, current_boot_id=BOOT2, timeout=1)
        self.assert_readonly(control)
        control = SequenceControl(self.package, [dict(boot=BOOT2, cache='ready')])
        control.installed = False
        with self.assertRaises(m.HelperInstallError):
            self.verify(control, previous_boot_id=BOOT1, timeout=1)
        self.assert_readonly(control)

    def test_foreign_or_inconsistent_cache_and_hook_never_become_verified(self):
        def corrupt(value, field):
            if field == 'hook': value['autostart']['enabled'] = False
            elif field == 'profile': value['boot_cache']['profile'] = '00' * 32
            elif field == 'boot': value['boot_cache']['boot_id'] = BOOT3
            elif field == 'work': value['boot_cache']['work'] = '/tmp/foreign'
            elif field == 'phase': value['boot_cache']['phase'] = 'pending'
            else: value['boot_cache']['mode'] = 'execute-code'
        for field in ('hook', 'profile', 'boot', 'work', 'phase', 'mode'):
            control = SequenceControl(self.package, [dict(boot=BOOT2, cache='ready',
                change=lambda value, field=field: corrupt(value, field))])
            with self.subTest(field=field), self.assertRaises(m.HelperInstallError):
                self.verify(control, previous_boot_id=BOOT1, timeout=1)
            self.assert_readonly(control)

    def test_late_ready_status_never_authorizes_success_or_any_later_start(self):
        control = SequenceControl(self.package, [dict(boot=BOOT2, cache='ready')])
        control.after_send = lambda command: setattr(control.clock, 'now', 2)
        with self.assertRaises(TimeoutError):
            self.verify(control, previous_boot_id=BOOT1, timeout=1)
        self.assertEqual(len(control.commands), 1)
        self.assert_readonly(control)

    def test_public_lifecycle_uses_actual_readonly_protocol_for_pending_and_known_reboot(self):
        for known_reboot in (False, True):
            with self.subTest(known_reboot=known_reboot):
                control = SequenceControl(self.package, [dict(boot=BOOT2, cache='pending'),
                                                         dict(boot=BOOT2, cache='ready')])
                camera = SimpleNamespace(_control_lock=threading.RLock(), _usb_stream=None,
                    _usb_preview=None, _imu=None, connect=Mock(),
                    state=Mock(return_value=SimpleNamespace(recording_state=0)), reboot=Mock())
                receipt = dict(installed=True, already_installed=True, boot_id=BOOT2,
                    previous_boot_id=BOOT2, profile=self.profile, _control=control,
                    boot_cache=dict(hydrated=False, boot_id=BOOT2, profile=self.profile, phase='pending',
                                    work='/dev/shm/osmo360-sdk-cache-' + self.profile[:12]))
                with (patch.object(m, 'install', return_value=receipt) as install,
                      patch.object(m, '_controller', return_value=control)):
                    value = lifecycle.install(camera, timeout=1, clock=control.clock,
                        **({'previous_boot_id': BOOT1} if known_reboot else {}))
                self.assertEqual(install.call_count, 0 if known_reboot else 1)
                camera.reboot.assert_not_called()
                self.assertFalse(value['reboot_attempted'])
                self.assertEqual(value['boot_id'], BOOT2)
                self.assertEqual(value['hydration_wait']['attempts'], 2)
                self.assert_readonly(control)


if __name__ == '__main__':
    unittest.main()
