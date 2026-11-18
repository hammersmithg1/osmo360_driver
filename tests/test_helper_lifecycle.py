"""Persistent API lifecycle checks; no camera or filesystem writes."""

import threading
import types
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import Camera
from osmo360_camera import _helper_lifecycle as m
from test_helper_install import BOOT1, BOOT2


class HelperLifecycleTests(unittest.TestCase):
    def camera(self):
        return types.SimpleNamespace(
            _control_lock=threading.RLock(),
            _usb_stream=None,
            _usb_preview=None,
            _imu=None,
            _transport=Mock(),
            connect=Mock(),
            state=Mock(return_value=types.SimpleNamespace(recording_state=0)),
            reboot=Mock(return_value=types.SimpleNamespace(reconnect_elapsed=2)),
            close=Mock(),
        )

    def receipt(self, **extra):
        result = dict(
            installed=True,
            profile="a" * 64,
            boot_id="old-boot",
            previous_boot_id="old-boot",
            owner_receipt=None,
        )
        result.update(extra)
        return result

    def test_public_camera_methods_forward_without_adding_outer_control_lock(self):
        camera = Camera(transport=Mock())
        with patch.object(m, "install", return_value="installed") as install:
            self.assertEqual(camera.install_helpers(timeout=40), "installed")
            install.assert_called_once_with(camera, timeout=40)
            install.reset_mock()
            self.assertEqual(camera.install_helpers(timeout=40, previous_boot_id=BOOT1), 'installed')
            install.assert_called_once_with(camera, timeout=40, previous_boot_id=BOOT1)
        with patch.object(m, "remove", return_value="removed") as remove:
            self.assertEqual(camera.remove_helpers(timeout=50), "removed")
            remove.assert_called_once_with(camera, timeout=50)

    def test_install_only_succeeds_after_new_boot_hydration(self):
        camera = self.camera()
        events = []
        with (
            patch.object(
                m.backend,
                "install",
                side_effect=lambda *a, **k: events.append("publish") or self.receipt(),
            ),
            patch.object(
                m.backend,
                "verify_installed",
                side_effect=lambda *a, **k: (
                    events.append("verify") or dict(installed=True, boot_id="new-boot")
                ),
            ),
        ):
            camera.reboot.side_effect = lambda **kw: events.append("reboot")
            result = m.install(camera)
        self.assertEqual(events, ["publish", "reboot", "verify"])
        self.assertTrue(result["installed"])
        self.assertTrue(result["autostart_hydration_verified"])

    def test_verified_same_boot_repeat_install_does_not_reboot(self):
        camera = self.camera()
        receipt = self.receipt(
            already_installed=True,
            boot_cache=dict(
                hydrated=True, boot_id="old-boot", profile="a" * 64, mode="bytes-only",
                work='/dev/shm/osmo360-sdk-cache-' + 'a' * 12
            ),
        )
        with (
            patch.object(m.backend, "install", return_value=receipt),
            patch.object(m.backend, "verify_installed") as verify,
        ):
            result = m.install(camera)
        self.assertTrue(result["already_installed"])
        camera.reboot.assert_not_called()
        verify.assert_not_called()

    def test_public_known_previous_boot_mode_is_readonly_even_when_verification_fails(self):
        for fails in (False, True):
            with self.subTest(fails=fails):
                camera = self.camera()
                proof = dict(boot_id=BOOT2, hydration_wait=dict(attempts=2))
                with (patch.object(m.backend, 'package_profile', return_value=dict(manifest=dict(profile='a' * 64))),
                      patch.object(m.backend, 'install') as install,
                      patch.object(m.backend, 'verify_boot_cache', return_value=proof) as verify):
                    if fails:
                        verify.side_effect = TimeoutError('cache absent on known new boot')
                        with self.assertRaises(m.HelperLifecycleError):
                            m.install(camera, previous_boot_id=BOOT1)
                    else:
                        value = m.install(camera, previous_boot_id=BOOT1)
                        self.assertTrue(value['verification_only'])
                        self.assertFalse(value['reboot_attempted'])
                        self.assertEqual((value['boot_id_before'], value['boot_id']), (BOOT1, BOOT2))
                    install.assert_not_called()
                    camera.reboot.assert_not_called()
                    self.assertEqual(verify.call_args.kwargs['previous_boot_id'], BOOT1)

    def test_pending_existing_cache_waits_without_reboot_but_ambiguous_absence_preserves_initial_recovery(self):
        for pending in (True, False):
            with self.subTest(pending=pending):
                camera = self.camera()
                cache = dict(hydrated=False, boot_id=BOOT1, profile='a' * 64)
                if pending: cache.update(phase='pending', work='/dev/shm/osmo360-sdk-cache-' + 'a' * 12)
                receipt = self.receipt(already_installed=True, boot_id=BOOT1, previous_boot_id=BOOT1, boot_cache=cache)
                with (patch.object(m.backend, 'install', return_value=receipt),
                      patch.object(m.backend, 'verify_boot_cache', return_value=dict(boot_id=BOOT1, hydration_wait=dict(attempts=2))) as wait,
                      patch.object(m.backend, 'verify_installed', return_value=dict(boot_id=BOOT2)) as verify):
                    result = m.install(camera)
                self.assertTrue(result['already_installed'])
                self.assertEqual(camera.reboot.call_count, 0 if pending else 1)
                self.assertEqual(wait.call_count, 1 if pending else 0)
                self.assertEqual(verify.call_count, 0 if pending else 1)

    def test_invalid_installed_cache_is_not_rebooted_into_a_success_claim(self):
        for cache in (dict(hydrated=True, boot_id=BOOT1, profile='a' * 64, mode='bytes-only', work='/tmp/foreign'),
                      dict(hydrated=False, boot_id=BOOT1, profile='a' * 64, mode='bytes-only'),
                      dict(hydrated=False, boot_id=BOOT1, profile='a' * 64, phase='pending')):
            camera = self.camera()
            with patch.object(m.backend, 'install', return_value=self.receipt(
                    already_installed=True, boot_id=BOOT1, boot_cache=cache)):
                with self.assertRaises(m.HelperLifecycleError):
                    m.install(camera)
            camera.reboot.assert_not_called()

    def test_install_rejects_capture_before_persistent_writes(self):
        camera = self.camera()
        camera._imu = object()
        with patch.object(m.backend, "install") as install:
            with self.assertRaises(RuntimeError):
                m.install(camera)
        install.assert_not_called()
        camera.reboot.assert_not_called()

    def test_remove_closes_producers_outside_lock_before_removing_hook(self):
        camera = self.camera()
        camera._imu = object()
        events = []

        def close():
            self.assertFalse(camera._control_lock._is_owned())
            events.append("close")
            camera._imu = None

        camera.close.side_effect = close
        receipt = dict(
            removed=True,
            installed=False,
            requires_reboot=True,
            previous_boot_id="old-boot",
            owner_receipt=None,
        )
        with (
            patch.object(
                m.backend,
                "remove",
                side_effect=lambda *a, **kw: events.append("remove-hook") or receipt,
            ),
            patch.object(
                m.backend,
                "verify_removed",
                side_effect=lambda *a, **kw: (
                    events.append("verify-absent")
                    or dict(removed=True, ram_absent=True)
                ),
            ),
        ):
            camera.reboot.side_effect = lambda **kw: events.append("reboot")
            result = m.remove(camera)
        self.assertEqual(events, ["close", "remove-hook", "reboot", "verify-absent"])
        self.assertTrue(result["removed"])

    def test_unverified_removal_never_reboots_or_claims_success(self):
        camera = self.camera()
        with patch.object(m.backend, "remove", return_value={"removed": False}):
            with self.assertRaises(m.HelperLifecycleError) as error:
                m.remove(camera)
        self.assertFalse(error.exception.report["removed"])
        camera.reboot.assert_not_called()

    def test_known_owner_finalizes_once_using_the_existing_task_session(self):
        camera = self.camera()
        control = Mock(lock=camera._control_lock)
        control.bootstrap.transport = camera._transport
        receipt = self.receipt(
            _control=control,
            owner_receipt={
                "work": "owned",
                "nonce": "owned",
                "owner": {},
                "service": {},
            },
        )
        with (
            patch.object(m.backend, "install", return_value=receipt),
            patch.object(
                m.backend, "verify_installed", return_value={"boot_id": "new"}
            ),
            patch.object(m, "StandbyControl") as cls,
        ):
            cls.return_value.finalize.side_effect = OSError("lost receipt")
            camera._transport.wait_disconnected.return_value = True
            result = m.install(camera)
        cls.return_value.finalize.assert_called_once()
        control.resolve_pending.assert_called_once()
        self.assertIs(cls.call_args.args[0], control.bootstrap)
        self.assertNotIn("_control", result["installation"])
        camera.reboot.assert_not_called()
        self.assertIn("lost receipt", result["reboot_receipt_error"])

    def test_failed_producer_cleanup_keeps_persistent_install_untouched(self):
        camera = self.camera()
        camera._usb_stream = Mock()
        camera.close.side_effect = RuntimeError("unknown IMU mask")
        with patch.object(m.backend, "remove") as remove:
            with self.assertRaises(RuntimeError):
                m.remove(camera)
        remove.assert_not_called()
        camera.reboot.assert_not_called()

    def test_late_persistent_result_never_starts_a_reboot(self):
        camera = self.camera()
        now = [0.0]

        def installed(*a, **kw):
            now[0] = 11.0
            return self.receipt()

        with patch.object(m.backend, "install", side_effect=installed):
            with self.assertRaises(m.HelperLifecycleError):
                m.install(camera, timeout=10, clock=lambda: now[0])
        camera.reboot.assert_not_called()

    def test_late_idempotent_receipt_is_not_reported_as_success(self):
        camera = self.camera()
        now = [0.0]

        def installed(*a, **kw):
            now[0] = 11.0
            return self.receipt(
                already_installed=True,
                boot_cache=dict(
                    hydrated=True,
                    boot_id="old-boot",
                    profile="a" * 64,
                    mode="bytes-only",
                ),
            )

        with patch.object(m.backend, "install", side_effect=installed):
            with self.assertRaises(m.HelperLifecycleError):
                m.install(camera, timeout=10, clock=lambda: now[0])
        camera.reboot.assert_not_called()

    def test_new_producer_after_close_prevents_persistent_removal(self):
        for field in ("_imu", "_usb_preview"):
            with self.subTest(field=field):
                camera = self.camera()
                camera._usb_stream = Mock()

                def close():
                    camera._usb_stream = None
                    setattr(camera, field, object())

                camera.close.side_effect = close
                with patch.object(m.backend, "remove") as remove:
                    with self.assertRaises(RuntimeError):
                        m.remove(camera)
                remove.assert_not_called()
                camera.reboot.assert_not_called()

    def test_owned_reboot_rechecks_record0_after_upload(self):
        camera = self.camera()
        control = Mock(lock=camera._control_lock)
        control.bootstrap.transport = camera._transport
        receipt = self.receipt(_control=control, owner_receipt={"same": True})
        camera.state.side_effect = [
            types.SimpleNamespace(recording_state=0),
            types.SimpleNamespace(recording_state=2),
        ]
        with (
            patch.object(m.backend, "install", return_value=receipt),
            patch.object(m, "StandbyControl") as cls,
        ):
            with self.assertRaises(m.HelperLifecycleError):
                m.install(camera)
        cls.assert_not_called()
        camera.reboot.assert_not_called()

    def test_budget_expiry_while_resolving_old_task_does_not_finalize(self):
        camera = self.camera()
        now = [0.0]
        control = Mock(lock=camera._control_lock)
        control.bootstrap.transport = camera._transport
        control.resolve_pending.side_effect = lambda **kw: now.__setitem__(0, 11.0)
        receipt = self.receipt(_control=control, owner_receipt={"same": True})
        with (
            patch.object(m.backend, "install", return_value=receipt),
            patch.object(m, "StandbyControl") as cls,
        ):
            with self.assertRaises(m.HelperLifecycleError):
                m.install(camera, timeout=10, clock=lambda: now[0])
        cls.assert_not_called()
        camera.reboot.assert_not_called()


if __name__ == "__main__":
    unittest.main()
