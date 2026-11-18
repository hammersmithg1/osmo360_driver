"""Charging host lease tests with fake USB/D1; no native helper is executed."""

import ast
import ctypes as C
import json
import shlex
import subprocess
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import Camera
from osmo360_camera import _usb_charging as m


BOOT = '11111111-1111-1111-1111-111111111111'
NEW_BOOT = '22222222-2222-2222-2222-222222222222'
ORIGINAL = dict(charge_disable=0, restrict_chg=0, restrict_cur=500000)


class Environment:
    def __init__(self):
        self.boot, self.commands, self.native = BOOT, [], []
        self.output = None
        self.ignored = self.bad_nonce = self.timeout = False
        transport = SimpleNamespace(connected=True, _thread=object(),
            _device_identity=dict(bus=2, address=7), request=Mock(
                return_value=SimpleNamespace(payload=bytes.fromhex('1200550000'))))
        self.camera = SimpleNamespace(_transport=transport, connected=True,
            _control_lock=threading.RLock(), _imu=None, _usb_preview=None,
            _usb_stream=None, _record_owned=False,
            state=Mock(return_value=SimpleNamespace(recording_state=0)), connect=Mock())
        self.control = SimpleNamespace(pending_command=None, pending_task=None,
            _unknown_task=False, detached=False)
        self.control.bootstrap = SimpleNamespace(transport=transport,
            session=SimpleNamespace(quarantined=False), pull_bytes=self.pull)
        self.control._run = self.run

    def run(self, command, *, timeout_ms=5000, pre_send=None):
        if pre_send:
            pre_send()
        self.commands.append(command)
        if command.startswith('python3 -B -c '):
            tree = ast.parse(shlex.split(command)[-1])
            call = tree.body[-1].value if isinstance(tree.body[-1], ast.Expr) else None
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Subscript):
                operation, enabled, nonce = [ast.literal_eval(arg) for arg in call.args[1:4]]
                if len(nonce) != 16:
                    raise AssertionError('Native charging protocol requires a16-hex operation nonce')
                self.native.append(operation)
                if self.timeout:
                    self.control.bootstrap.session.quarantined = True
                    self.control.pending_task = 42
                    raise TimeoutError('Uncertain diagnostic START')
                receipt = dict(schema=1, identity=dict(boot_id=self.boot), original_policy=ORIGINAL)
                self.output = dict(schema=1, operation=operation,
                    operation_nonce='wrong' if self.bad_nonce else nonce,
                    ok=not self.ignored, ready=not self.ignored, boot_id=self.boot,
                    receipt=receipt, state='restored' if operation == 'restore' else 'prepared',
                    policy=ORIGINAL if operation == 'restore' else dict(
                        charge_disable=int(not enabled), restrict_chg=int(not enabled), restrict_cur=0),
                    telemetry={}, errors=['SET ignored'] if self.ignored else [],
                    physical_proof='unknown', policy_readback_is_cache=True, i=[1, 2])
        return dict(completed=True, success=True, result=0)

    def pull(self, path, **_):
        value = dict(boot=self.boot, i=[1, 2]) if 'osmo_charge_boot_' in path else self.output
        return json.dumps(value).encode()


class ChargingHostTests(unittest.TestCase):
    def setUp(self):
        self.env = Environment()
        self.controller = patch.object(m, '_controller', return_value=self.env.control)
        self.controller.start()
        self.addCleanup(self.controller.stop)
        self.lease = m.USBChargingLease(self.env.camera, False)

    def inventory_usb(self, descriptors, failure_index=None):
        usb = Mock()
        devices = (C.c_void_p * len(descriptors))(*range(1, len(descriptors)+1))

        def initialize(pointer):
            pointer._obj.value = 1
            return 0

        def listing(context, pointer):
            C.cast(pointer, C.POINTER(C.POINTER(C.c_void_p)))[0] = C.cast(devices, C.POINTER(C.c_void_p))
            return len(descriptors)

        def descriptor(device, output):
            index = int(device)-1
            if index == failure_index:
                return -1
            vendor, product = descriptors[index]
            raw = bytearray(18)
            raw[8:10], raw[10:12] = vendor.to_bytes(2, 'little'), product.to_bytes(2, 'little')
            C.memmove(output, bytes(raw), len(raw))
            return 0

        usb.libusb_init.side_effect = initialize
        usb.libusb_get_device_list.side_effect = listing
        usb.libusb_get_device_descriptor.side_effect = descriptor
        usb.libusb_error_name.return_value = b'LIBUSB_ERROR_IO'
        return usb

    def test_complete_inventory_distinguishes_absence_one_and_multiple_without_open(self):
        for devices, expected in (([], 0), ([(0x1234, 1)], 0), ([(0x2CA3, 0x25)], 1),
                                  ([(0x2CA3, 0x25), (0x2CA3, 0x25)], 2)):
            with self.subTest(devices=devices):
                usb = self.inventory_usb(devices)
                with patch.object(m._usb, 'load_usb', return_value=usb):
                    self.assertEqual(m.attached_camera_count(), expected)
                usb.libusb_free_device_list.assert_called_once()
                usb.libusb_exit.assert_called_once()
                usb.libusb_open.assert_not_called()
                usb.libusb_claim_interface.assert_not_called()

    def test_partial_inventory_and_unknown_dji_mode_never_prove_absence(self):
        for devices, failure in (([(0x2CA3, 0x25), (0x1234, 1)], 1),
                                 ([(0x2CA3, 0x99)], None)):
            usb = self.inventory_usb(devices, failure)
            with patch.object(m._usb, 'load_usb', return_value=usb):
                with self.assertRaises(RuntimeError):
                    m.attached_camera_count()
            usb.libusb_free_device_list.assert_called_once()
            usb.libusb_exit.assert_called_once()
            usb.libusb_open.assert_not_called()

    def test_public_camera_factory_is_pure_and_optional(self):
        camera = Camera(transport=Mock())
        lease = camera.usb_charging(False)
        self.assertFalse(lease.status['ready'])
        camera._transport.open.assert_not_called()
        self.assertEqual(lease.status['state'], 'unprepared')

    def test_prepare_check_restore_keep_one_original_receipt_and_bound_commands(self):
        first = self.lease.prepare()
        self.assertTrue(first['ready'])
        self.assertEqual(first['receipt']['original_policy'], ORIGINAL)
        self.assertEqual(first['physical_proof'], 'unknown')
        receipt = self.lease.receipt
        self.lease.recheck()
        self.lease.prepare()
        restored = self.lease.restore()
        self.assertEqual(self.env.native, ['prepare', 'check', 'prepare', 'restore'])
        self.assertEqual(restored['policy'], ORIGINAL)
        self.assertEqual(self.lease.receipt, receipt)
        self.assertTrue(all(len(command) <= 500 for command in self.env.commands))

    def test_host_protocol_drives_actual_native_engine_with_fake_policy_io(self):
        from test_usb_charging_native import FakeIO, FakeStore
        from osmo360_camera._device import usb_charging as native
        io, store = FakeIO((0, 0, 500000)), FakeStore()
        self.env.boot = io.receipt['boot_id']
        run = self.env.run

        def execute(command, **options):
            before = len(self.env.native)
            result = run(command, **options)
            if len(self.env.native) > before:
                call = ast.parse(shlex.split(command)[-1]).body[-1].value
                operation, enabled, nonce = [ast.literal_eval(arg) for arg in call.args[1:4]]
                self.env.output = native.execute(io, store, operation, enabled, nonce,
                    sleep=lambda _: io.values.update(battery_current_now=0))
                self.env.output['i'] = [1, 2]
            return result

        self.env.control._run = execute
        first = self.lease.prepare()
        self.assertTrue(first['ready'])
        self.assertEqual(first['state'], 'cache_verified')
        self.assertEqual(first['receipt']['original_policy'], ORIGINAL)
        self.assertEqual(first['evidence']['native']['settled_telemetry']['battery_current_now']['value'], 0)
        self.assertTrue(self.lease.recheck()['ready'])
        self.assertEqual(self.lease.restore()['policy'], ORIGINAL)
        self.assertEqual(io.writes, [('charge_disable', 1), ('charge_disable', 0), ('restrict_cur', 500000)])
        self.assertEqual(self.env.native, ['prepare', 'check', 'restore'])

    def test_live_capture_prevents_any_diagnostic_start_for_policy_write(self):
        for name, value in (('_imu', object()), ('_usb_preview', object()),
                            ('_usb_stream', SimpleNamespace(owns_camera_resources=True)),
                            ('_record_owned', True)):
            with self.subTest(owner=name):
                setattr(self.env.camera, name, value)
                with self.assertRaisesRegex(m.USBChargingError, 'closed capture'):
                    self.lease.prepare()
                setattr(self.env.camera, name, None if name != '_record_owned' else False)
                self.assertEqual(self.env.commands, [])
                self.assertEqual(self.env.native, [])

    def test_recording_or_app_registration_blocks_before_native_policy_change(self):
        self.env.camera.state.return_value.recording_state = 2
        with self.assertRaisesRegex(m.USBChargingError, 'Record0'):
            self.lease.prepare()
        self.assertEqual(self.env.native, [])
        self.env.camera.state.return_value.recording_state = 0
        self.env.camera._transport.request.return_value.payload = b'bad APP reply'
        with self.assertRaises(m.USBChargingError):
            self.lease.prepare()
        self.assertEqual(self.env.native, [])

    def test_ignored_policy_is_a_failure_and_not_reported_as_ready(self):
        self.env.ignored = True
        with self.assertRaisesRegex(m.USBChargingError, 'SET ignored'):
            self.lease.prepare()
        self.assertFalse(self.lease.status['ready'])
        self.assertEqual(self.env.native, ['prepare'])
        self.assertIsNotNone(self.lease.receipt)

    def test_uncertain_task_blocks_restore_and_replacement_controller(self):
        self.env.timeout = True
        with self.assertRaises(m.USBChargingError):
            self.lease.prepare()
        before = len(self.env.commands)
        with self.assertRaisesRegex(m.USBChargingError, 'unknown'):
            self.lease.restore(verified_rebind=True)
        with self.assertRaisesRegex(m.USBChargingError, 'unknown'):
            self.lease.prepare(verified_rebind=True)
        self.assertEqual(len(self.env.commands), before)
        self.assertEqual(self.lease.status['state'], 'unknown')

    def test_generation_change_requires_verified_transition_even_for_restore(self):
        self.lease.prepare()
        self.env.camera._transport._thread = object()
        before = list(self.env.native)
        with self.assertRaisesRegex(m.USBChargingError, 'verified USB transition'):
            self.lease.restore()
        self.assertEqual(self.env.native, before)
        self.lease.restore(verified_rebind=True)
        self.assertEqual(self.env.native[-1], 'restore')

    def test_public_context_restores_after_actual_stream_wrapper_verified_close(self):
        from osmo360_camera._stream_resume import ResumableUSBStream
        stream = ResumableUSBStream.__new__(ResumableUSBStream)
        stream.camera = self.env.camera
        stream._lock = threading.RLock()
        stream._unknown_shadow = False

        def close_core():
            self.env.camera._transport._thread = object()
            stream._core.cleanup_safe = True
            stream._core.owns_camera_resources = False
            return {'state': 'DETACHED'}

        stream._core = SimpleNamespace(close=close_core, cleanup_safe=False, owns_camera_resources=True)
        with self.lease:
            self.env.camera._usb_stream = stream
            stream.close()
        self.assertEqual(self.env.native, ['prepare', 'restore'])
        self.assertEqual(self.lease.status['state'], 'restored')

    def test_closed_stream_proof_cannot_authorize_later_external_generation(self):
        self.lease.prepare()
        stream = SimpleNamespace(camera=self.env.camera, cleanup_safe=True,
                                 owns_camera_resources=False)
        self.env.camera._usb_stream = stream
        transport = self.env.camera._transport
        transport._thread = object()
        self.env.camera._usb_charging_closed_transition = (
            stream, transport, transport._thread, 2, 7)
        transport._thread = object()  # External change AFTER the verified close.
        with self.assertRaisesRegex(m.USBChargingError, 'verified USB transition'):
            self.lease.restore()
        self.assertEqual(self.env.native, ['prepare'])

    def test_new_boot_retires_original_without_writing_old_boot_policy(self):
        self.lease.prepare()
        self.env.camera._transport._thread = object()
        self.env.boot = NEW_BOOT
        old_native = list(self.env.native)
        report = self.lease.restore(verified_rebind=True)
        self.assertEqual(report['state'], 'retired_boot')
        self.assertFalse(report['original_policy_restored'])
        self.assertEqual(self.env.native, old_native)
        self.assertIsNone(self.lease.receipt)
        self.lease.prepare()
        self.assertEqual(self.lease.receipt['identity']['boot_id'], NEW_BOOT)

    def test_readonly_check_can_observe_active_capture_without_policy_write(self):
        self.lease.prepare()
        self.env.camera._imu = object()
        self.env.camera.state.reset_mock()
        self.env.camera._transport.request.reset_mock()
        self.lease.recheck()
        self.assertEqual(self.env.native[-1], 'check')
        self.env.camera.state.assert_not_called()
        self.env.camera._transport.request.assert_not_called()

    def test_restore_after_verified_closed_capture_needs_no_c1_get_or_video_wake(self):
        self.lease.prepare()
        self.env.camera.state.side_effect = AssertionError('C1 unavailable in native Sleep')
        self.env.camera._transport.request.side_effect = AssertionError('No APP/C1 for scoped restore')
        report = self.lease.restore()
        self.assertEqual(report['state'], 'restored')
        self.assertEqual(self.env.native[-1], 'restore')

    def test_stale_operation_export_is_rejected(self):
        self.env.bad_nonce = True
        with self.assertRaisesRegex(m.USBChargingError, 'Stale/malformed'):
            self.lease.prepare()
        self.assertFalse(self.lease.status['ready'])
        self.assertTrue(self.lease.uncertain)
        before = list(self.env.native)
        with self.assertRaisesRegex(m.USBChargingError, 'unknown'):
            self.lease.prepare(verified_rebind=True)
        self.assertEqual(self.env.native, before)

    def test_terminal_restore_with_lost_export_never_repeats_policy_mutation(self):
        self.lease.prepare()
        self.env.control.bootstrap.pull_bytes = Mock(side_effect=RuntimeError('Export unavailable'))
        with self.assertRaisesRegex(m.USBChargingError, 'Export unavailable'):
            self.lease.restore()
        self.assertTrue(self.lease.uncertain)
        self.assertEqual(self.env.native, ['prepare', 'restore'])
        with self.assertRaisesRegex(m.USBChargingError, 'unknown'):
            self.lease.restore(verified_rebind=True)
        self.assertEqual(self.env.native, ['prepare', 'restore'])

    def test_generated_cleanup_checks_device_inode_in_the_same_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'owned.json'
            path.write_text('{}')
            stat = path.stat()
            command = m._unlink_command(str(path), [stat.st_dev, stat.st_ino])
            result = subprocess.run(shlex.split(command), capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(path.exists())
            path.write_text('{}')
            command = m._unlink_command(str(path), [stat.st_dev + 1, stat.st_ino])
            result = subprocess.run(shlex.split(command), capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertTrue(path.exists())

    def test_new_process_can_prepare_from_source_pinned_sleep_without_waking_capture(self):
        self.env.camera.state.side_effect = m.RequestTimeout('C1 sleeps')
        snapshot = dict(state='SLEEP', owner_alive=True,
            requested_readiness=dict(system_power_level=3), shadow=dict(value='none'),
            owner=dict(pid=10, start='20'), service=dict(pid=4, start='5'), command_id='0000000000000001')
        with patch('osmo360_camera._stream_resume._discover', return_value='/dev/shm/osmo-usb-standby-123456789abc'), \
                patch('osmo360_camera._stream_resume._status_at', return_value=snapshot):
            result = self.lease.prepare()
        self.assertTrue(result['ready'])
        self.assertEqual(self.env.native, ['prepare'])
        self.env.camera._transport.request.assert_not_called()

    def test_sleep_idle_fallback_never_accepts_missing_or_awake_owner(self):
        self.env.camera.state.side_effect = m.RequestTimeout('C1 sleeps')
        with patch('osmo360_camera._stream_resume._discover', return_value=None):
            with self.assertRaisesRegex(m.USBChargingError, 'without a verified'):
                self.lease.prepare()
        self.assertEqual(self.env.native, [])

    def test_context_cleanup_preserves_original_exception_and_restore_failure(self):
        self.lease.prepare = Mock()
        self.lease.restore = Mock(side_effect=m.USBChargingError('restore unknown'))
        original = ValueError('original failure')
        with self.assertRaises(ValueError) as caught:
            with self.lease:
                raise original
        self.assertIs(caught.exception, original)
        self.lease.restore.assert_called_once()
        if hasattr(original, '__notes__'):
            self.assertTrue(any('restore unknown' in note for note in original.__notes__))


if __name__ == '__main__':
    unittest.main()
