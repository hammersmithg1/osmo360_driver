"""Offline charge-lease semantics; no camera, native task or SDK execution."""
from copy import deepcopy
import json
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera._device import usb_charging as m


BOOT = '78a5fd15-1183-4582-8606-e603630f837c'
BOOT2 = 'cc80550e-6c22-48a6-bd9b-d71308cb7ac1'
NONCE = '0123456789abcdef'


class FakeIO:
    def __init__(self, original=(0, 0, 1000000)):
        self.values = dict(zip(m.POLICY, original))
        self.values.update(usb_online=1, usb_current_now=1394000, battery_current_now=402842,
                           battery_status='Charging', battery_present=1, battery_temp=400,
                           battery_capacity=90)
        self.receipt = dict(boot_id=BOOT, kernel=m.KERNEL, module_sha256=m.MODULE_SHA,
                            module_pin=[1, 2, 104728, 5], class_pin=[6, 7], device_pin=[8, 9],
                            driver_pin=[10, 11], driver_target=m.DRIVER,
                            sysfs_pins={key: [12, 20 + i] for i, key in enumerate(m.PATHS)})
        self.writes, self.reads, self.actions = [], [], []
        self.read_errors = set()

    def check(self, expected=None):
        if expected is not None:
            m.require(self.receipt == expected, 'Lease generation/pins changed')

    def read(self, key):
        self.reads.append(key)
        if key in self.read_errors:
            raise OSError('unavailable ' + key)
        return self.values[key]

    def write(self, key, value):
        self.writes.append((key, value))
        action = self.actions.pop(0) if self.actions else None
        if action == 'before':
            raise OSError('before delivery')
        if action != 'ignore':
            if key == 'charge_disable':
                self.values['restrict_chg'] = value
                if value:
                    self.values['restrict_cur'] = 0
            else:
                self.values[key] = value
            self.values['charge_disable'] = int(bool(self.values['restrict_chg'] and
                                                      self.values['restrict_cur'] == 0))
        if action == 'foreign':
            self.values.update(charge_disable=0, restrict_chg=0, restrict_cur=777000)
        elif action == 'boot':
            self.receipt['boot_id'] = BOOT2
            raise OSError('boot changed after delivery')
        elif action == 'after':
            raise OSError('after delivery')
        elif action == 'interrupt':
            raise KeyboardInterrupt('injected interrupt after delivery')


class FakeStore:
    def __init__(self):
        self.files, self.writes = {}, []

    def read(self, name):
        return deepcopy(self.files.get(name))

    def write(self, name, value, nonce):
        if name == 'original.json' and name in self.files:
            raise FileExistsError(name)
        self.files[name] = deepcopy(value)
        self.writes.append((name, deepcopy(value), nonce))


class NativeChargingTests(unittest.TestCase):
    def setUp(self):
        self.io, self.store = FakeIO(), FakeStore()
        self.sleep = Mock(side_effect=lambda seconds: self.io.values.update(battery_current_now=0))

    def run_op(self, operation='prepare', enabled=False):
        return m.execute(self.io, self.store, operation, enabled, NONCE, sleep=self.sleep)

    def assert_policy(self, expected):
        self.assertEqual(m.policy(self.io), dict(zip(m.POLICY, expected)))

    def test_disable_and_exact_default_restore_without_current_limit_write(self):
        off = self.run_op()
        self.assertTrue(off['ok'])
        self.assertTrue(off['ready'])
        self.assertEqual(off['state'], 'cache_verified')
        self.assertEqual(self.io.writes, [('charge_disable', 1)])
        self.assert_policy((1, 1, 0))
        on = self.run_op('restore')
        self.assertTrue(on['ok'])
        self.assertEqual(self.io.writes, [('charge_disable', 1), ('charge_disable', 0),
                                          ('restrict_cur', 1000000)])
        self.assert_policy((0, 0, 1000000))
        self.assertFalse(self.store.files['state.json']['dirty'])
        self.assertEqual(on['restore']['state'], 'cache_restored')

    def test_nondefault_original_current_and_restriction_are_preserved_exactly(self):
        for original in ((0, 0, 500000), (0, 1, 750000), (0, 0, 0)):
            with self.subTest(original=original):
                self.io, self.store = FakeIO(original), FakeStore()
                self.assertTrue(self.run_op()['ok'])
                self.assertTrue(self.run_op('restore')['ok'])
                self.assert_policy(original)
                if original[1]:
                    self.assertEqual(self.io.writes[-1], ('restrict_chg', 1))

    def test_already_disabled_never_briefly_enables_and_original_is_immutable(self):
        self.io = FakeIO((1, 1, 0))
        self.assertTrue(self.run_op()['ok'])
        original = deepcopy(self.store.files['original.json'])
        self.assertTrue(self.run_op()['ok'])
        self.assertTrue(self.run_op('check')['ok'])
        self.assertTrue(self.run_op('restore')['ok'])
        self.assertEqual(self.io.writes, [])
        self.assertEqual(self.store.files['original.json'], original)
        self.assertEqual(sum(name == 'original.json' for name, _, _ in self.store.writes), 1)

    def test_repeated_prepare_does_not_resnapshot_disabled_policy(self):
        self.assertTrue(self.run_op()['ok'])
        original = deepcopy(self.store.files['original.json'])
        self.assertTrue(self.run_op()['ok'])
        self.assertEqual(self.io.writes, [('charge_disable', 1)])
        self.assertEqual(self.store.files['original.json'], original)
        self.assertTrue(self.run_op('restore')['ok'])
        self.assert_policy((0, 0, 1000000))

    def test_optional_enabled_true_is_noop_when_enabled_and_restores_disabled_original(self):
        self.assertTrue(self.run_op(enabled=True)['ok'])
        self.assertEqual(self.io.writes, [])
        self.io, self.store = FakeIO((1, 1, 0)), FakeStore()
        self.assertTrue(self.run_op(enabled=True)['ok'])
        self.assertEqual(self.io.writes, [('charge_disable', 0)])
        self.assert_policy((0, 0, 0))
        self.assertTrue(self.run_op('restore', True)['ok'])
        self.assertEqual(self.io.writes[-1], ('charge_disable', 1))

    def test_check_is_readonly_and_fails_if_requested_policy_does_not_match(self):
        self.assertTrue(self.run_op()['ok'])
        writes, state_writes = list(self.io.writes), len(self.store.writes)
        self.assertTrue(self.run_op('check')['ok'])
        self.assertFalse(self.run_op('check', True)['ok'])
        self.assertEqual(self.io.writes, writes)
        self.assertEqual(len(self.store.writes), state_writes)

    def test_unprepared_check_restore_never_create_receipt_or_write_policy(self):
        for operation in ('check', 'restore'):
            self.assertFalse(self.run_op(operation)['ok'])
        self.assertEqual(self.store.files, {})
        self.assertEqual(self.io.writes, [])

    def test_boot_class_driver_module_or_inode_change_blocks_old_lease(self):
        for field in ('boot_id', 'class_pin', 'driver_pin', 'module_pin', 'sysfs_pins'):
            with self.subTest(field=field):
                self.setUp()
                self.assertTrue(self.run_op()['ok'])
                if field == 'boot_id':
                    self.io.receipt[field] = BOOT2
                elif field == 'sysfs_pins':
                    self.io.receipt[field]['charge_disable'][1] += 1
                else:
                    self.io.receipt[field][1] += 1
                prior = list(self.io.writes)
                for operation in ('prepare', 'check', 'restore'):
                    result = self.run_op(operation)
                    self.assertFalse(result['ok'])
                    self.assertFalse(result['ready'])
                self.assertEqual(self.io.writes, prior)

    def test_foreign_policy_is_not_overwritten_even_at_close(self):
        self.assertTrue(self.run_op()['ok'])
        self.io.values.update(charge_disable=0, restrict_chg=0, restrict_cur=777000)
        prior = list(self.io.writes)
        for operation in ('prepare', 'check', 'restore'):
            result = self.run_op(operation)
            self.assertFalse(result['ok'])
            self.assertFalse(result['ready'])
        self.assertEqual(self.io.writes, prior)
        self.assert_policy((0, 0, 777000))

    def test_pending_mutation_missing_state_and_corrupt_original_fail_closed(self):
        for change in ('pending', 'state', 'policy'):
            with self.subTest(change=change):
                self.setUp()
                self.assertTrue(self.run_op()['ok'])
                if change == 'pending':
                    self.store.files['state.json']['pending'] = dict(key='charge_disable', value=1)
                elif change == 'state':
                    del self.store.files['state.json']
                else:
                    self.store.files['original.json']['original_policy']['charge_disable'] = 5
                prior = list(self.io.writes)
                self.assertFalse(self.run_op('restore')['ok'])
                self.assertEqual(self.io.writes, prior)

    def test_inconsistent_dirty_or_malformed_schema_never_restores_policy(self):
        for change in ('dirty', 'schema_bool', 'missing_pending', 'extra_original'):
            with self.subTest(change=change):
                self.setUp()
                self.assertTrue(self.run_op()['ok'])
                if change == 'dirty':
                    self.store.files['state.json']['dirty'] = False
                elif change == 'schema_bool':
                    self.store.files['original.json']['schema'] = True
                elif change == 'missing_pending':
                    del self.store.files['state.json']['pending']
                else:
                    self.store.files['original.json']['other'] = 'unknown schema'
                writes = list(self.io.writes)
                self.assertFalse(self.run_op('restore')['ok'])
                self.assertEqual(self.io.writes, writes)

    def test_failed_or_ignored_apply_rolls_back_only_known_owned_states(self):
        for action in ('before', 'after', 'ignore', 'interrupt'):
            with self.subTest(action=action):
                self.setUp()
                self.io.actions = [action]
                result = self.run_op()
                self.assertFalse(result['ok'])
                self.assertFalse(result['ready'])
                self.assertEqual(result['restore']['state'], 'cache_restored_after_error')
                self.assert_policy((0, 0, 1000000))
                self.assertFalse(self.store.files['state.json']['dirty'])
                self.assertEqual(self.store.files['state.json']['pending'], None)
                self.assertIn(action if action in ('before', 'after') else
                              'readback' if action == 'ignore' else 'interrupt', result['errors'][0])

    def test_failed_apply_does_not_overwrite_foreign_policy_or_new_boot(self):
        for action in ('foreign', 'boot'):
            with self.subTest(action=action):
                self.setUp()
                self.io.actions = [action]
                result = self.run_op()
                self.assertFalse(result['ok'])
                self.assertEqual(result['restore']['state'], 'unknown')
                self.assertTrue(result['restore']['errors'])
                self.assertEqual(self.io.writes, [('charge_disable', 1)])

    def test_restore_error_never_reports_not_needed_or_retries_uncertain_write(self):
        self.assertTrue(self.run_op()['ok'])
        self.io.actions = ['after']
        result = self.run_op('restore')
        self.assertFalse(result['ok'])
        self.assertEqual(result['restore']['state'], 'unknown')
        self.assertTrue(result['restore']['errors'])
        self.assertEqual(self.io.writes, [('charge_disable', 1), ('charge_disable', 0)])

    def test_write_preconditions_fail_without_creating_original_or_mutating(self):
        for key, value in [('battery_present', 0), ('usb_online', 0), ('battery_temp', 451),
                           ('battery_temp', -201)]:
            with self.subTest(key=key, value=value):
                self.setUp()
                self.io.values[key] = value
                self.assertFalse(self.run_op()['ok'])
                self.assertEqual(self.store.files, {})
                self.assertEqual(self.io.writes, [])

    def test_cached_policy_and_independent_live_telemetry_do_not_claim_physical_off(self):
        result = self.run_op()
        self.assertTrue(result['ok'])
        self.assertEqual(result['physical_proof'], 'unknown')
        self.assertTrue(result['policy_readback_is_cache'])
        self.assertEqual(result['telemetry']['battery_status']['value'], 'Charging')
        self.assertEqual(result['telemetry']['battery_current_now']['value'], 0)
        # IBUS is still nonzero: this feature does not cap total USB input.
        self.assertEqual(result['telemetry']['usb_current_now']['value'], 1394000)
        self.assertEqual(result['policy'], m.OFF)
        self.assertLessEqual(len(m.encoded(result, 16384)), 16384)

    def test_prepare_settles_before_fresh_current_reads_and_check_never_waits(self):
        def settled(seconds):
            self.io.reads.append('settled-' + str(seconds))
            self.io.values['battery_current_now'] = 0
        self.sleep.side_effect = settled
        result = self.run_op()
        self.assertTrue(result['ready'])
        self.sleep.assert_called_once_with(2)
        marker = self.io.reads.index('settled-2')
        self.assertGreater(self.io.reads.index('usb_current_now'), marker)
        self.assertGreater(self.io.reads.index('battery_current_now'), marker)
        self.assertEqual(result['evidence']['settle']['target_seconds'], 2)
        self.assertIn('settled_telemetry', result['evidence'])
        self.sleep.reset_mock()
        self.assertTrue(self.run_op('check')['ok'])
        self.sleep.assert_not_called()

    def test_missing_fresh_current_or_interrupted_settle_fails_ready_and_restores(self):
        for failure in ('usb_current_now', 'battery_current_now', 'interrupt'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure == 'interrupt':
                    self.sleep.side_effect = KeyboardInterrupt('settle interrupted')
                else:
                    self.io.read_errors.add(failure)
                result = self.run_op()
                self.assertFalse(result['ok'])
                self.assertFalse(result['ready'])
                self.assertEqual(result['restore']['state'], 'cache_restored_after_error')
                self.assert_policy((0, 0, 1000000))

    def test_any_positive_ibat_after_settle_fails_without_tolerance_zero_negative_pass(self):
        for value in (1, 50000, 402842, 0, -1, -250000):
            with self.subTest(ibat=value):
                self.setUp()
                self.sleep.side_effect = lambda seconds: self.io.values.update(battery_current_now=value)
                result = self.run_op()
                self.assertEqual(result['ok'], value <= 0)
                self.assertEqual(result['ready'], value <= 0)
                self.assertEqual(result['physical_proof'], 'unknown')
                if value > 0:
                    self.assertIn('remains positive', result['errors'][0])
                    self.assert_policy((0, 0, 1000000))

    def test_readonly_check_requires_fresh_nonpositive_ibat_without_sleep_or_restore(self):
        for value in (1, 0, -10000, None):
            with self.subTest(ibat=value):
                self.setUp()
                self.assertTrue(self.run_op()['ok'])
                self.sleep.reset_mock()
                if value is None:
                    self.io.read_errors.add('battery_current_now')
                else:
                    self.io.values['battery_current_now'] = value
                writes = list(self.io.writes)
                result = self.run_op('check')
                self.assertEqual(result['ok'], value is not None and value <= 0)
                self.assertEqual(result['ready'], value is not None and value <= 0)
                self.assertEqual(self.io.writes, writes)
                self.assertEqual(result['restore']['state'], 'not_needed')
                self.sleep.assert_not_called()

    def test_invalid_policy_and_api_arguments_cannot_write(self):
        for original in ((0, 1, 0), (1, 0, 0), (1, 1, 123), (0, 0, -1)):
            self.io, self.store = FakeIO(original), FakeStore()
            self.assertFalse(self.run_op()['ok'])
            self.assertEqual(self.io.writes, [])
        self.setUp()
        for operation, enabled, nonce in [('probe', False, NONCE), ('prepare', 0, NONCE),
                                           ('prepare', False, '../bad')]:
            self.assertFalse(m.execute(self.io, self.store, operation, enabled, nonce)['ok'])
        self.assertEqual(self.io.writes, [])


class NativeBoundaryTests(unittest.TestCase):
    def test_wrong_native_kernel_or_uid_rejected_before_any_device_open(self):
        for uid, kernel in ((501, m.KERNEL), (0, 'other-kernel')):
            with self.subTest(uid=uid, kernel=kernel), patch.object(m.os, 'geteuid', return_value=uid), \
                    patch.object(m.os, 'uname', return_value=SimpleNamespace(release=kernel)), \
                    patch.object(m.os, 'open') as opened:
                with self.assertRaises(RuntimeError):
                    m.NativeIO()
                opened.assert_not_called()

    def test_wrong_module_hash_short_read_or_changing_module_is_rejected(self):
        before = SimpleNamespace(st_mode=stat.S_IFREG | 0o600, st_size=104728,
                                 st_dev=1, st_ino=2, st_mtime_ns=3)
        changed = SimpleNamespace(**dict(vars(before), st_mtime_ns=4))
        for data, after in ((b'a' * 104728, before), (b'a' * 104727, before),
                            (b'a' * 104729, before), (b'a' * 104728, changed)):
            with self.subTest(bytes=len(data), changed=after is changed), \
                    patch.object(m.os, 'geteuid', return_value=0), \
                    patch.object(m.os, 'uname', return_value=SimpleNamespace(release=m.KERNEL)), \
                    patch.object(m.Path, 'resolve', lambda path, strict=True:
                                 Path('/sys/devices/fake/power_supply') / path.name), \
                    patch.object(m.os.path, 'realpath', side_effect=lambda path:
                                 m.DRIVER if str(path).endswith('/driver') else str(path)), \
                    patch.object(m.os, 'open', return_value=17), \
                    patch.object(m.os, 'fstat', side_effect=[before, after]), \
                    patch.object(m.os, 'read', return_value=data) as read, \
                    patch.object(m.os, 'close') as close:
                with self.assertRaises(RuntimeError):
                    m.NativeIO()
                read.assert_called_once_with(17, 104729)
                close.assert_called_once_with(17)

    def test_changed_sysfs_inode_is_rejected_before_read_or_write(self):
        io = m.NativeIO.__new__(m.NativeIO)
        io.check = Mock()
        io.receipt = {'sysfs_pins': {'charge_disable': [1, 2]}}
        info = SimpleNamespace(st_mode=stat.S_IFREG | 0o600, st_dev=1, st_ino=99)
        with patch.object(m.os, 'open', return_value=17), patch.object(m.os, 'fstat', return_value=info), \
                patch.object(m.os, 'close') as close, patch.object(m.os, 'read') as read, \
                patch.object(m.os, 'write') as write:
            with self.assertRaisesRegex(RuntimeError, 'node changed'):
                io.open_node('charge_disable', m.os.O_WRONLY)
            close.assert_called_once_with(17)
            read.assert_not_called()
            write.assert_not_called()

    def test_scalar_reads_are_bounded_and_invalid_reply_is_not_accepted(self):
        io = m.NativeIO.__new__(m.NativeIO)
        io.open_node = Mock(return_value=17)
        for data in (b'1' * 129, b'1 trailing', b'\xff', b''):
            with self.subTest(data=data[:15]), patch.object(m.os, 'read', return_value=data) as read, \
                    patch.object(m.os, 'close') as close:
                with self.assertRaises((RuntimeError, UnicodeError)):
                    io.read('charge_disable')
                read.assert_called_once_with(17, 129)
                close.assert_called_once_with(17)

    def test_native_policy_write_whitelist_excludes_input_current_and_thermal_controls(self):
        io = m.NativeIO.__new__(m.NativeIO)
        io.open_node = Mock(return_value=17)
        for key, value in [('input_current_limit', 500000), ('usb_current_now', 0),
                           ('battery_temp', 250), ('charge_disable', True), ('restrict_chg', 2),
                           ('restrict_cur', -1)]:
            with self.subTest(key=key):
                with self.assertRaises(RuntimeError):
                    io.write(key, value)
        io.open_node.assert_not_called()

    def test_immutable_original_exclusive_creation_and_atomic_state_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            store = m.NativeStore.__new__(m.NativeStore)
            store.work, store.check = Path(directory), Mock()
            store.write('original.json', {'schema': 1}, NONCE)
            with self.assertRaises(FileExistsError):
                store.write('original.json', {'schema': 2}, NONCE)
            self.assertEqual(json.loads((store.work / 'original.json').read_text()), {'schema': 1})
            for value in (1, 2):
                store.write('state.json', {'value': value}, NONCE)
            self.assertEqual(json.loads((store.work / 'state.json').read_text()), {'value': 2})
            self.assertEqual(sorted(p.name for p in store.work.iterdir()), ['original.json', 'state.json'])

    def test_failed_atomic_state_write_keeps_previous_state_and_removes_own_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            store = m.NativeStore.__new__(m.NativeStore)
            store.work, store.check = Path(directory), Mock()
            store.write('state.json', {'value': 1}, NONCE)
            with patch.object(m.os, 'write', side_effect=OSError('injected short write')):
                with self.assertRaises(OSError):
                    store.write('state.json', {'value': 2}, NONCE)
            self.assertEqual(json.loads((store.work / 'state.json').read_text()), {'value': 1})
            self.assertEqual([p.name for p in store.work.iterdir()], ['state.json'])

    def test_lease_file_reads_bound_size_and_refuse_unsafe_file_identity(self):
        store = m.NativeStore.__new__(m.NativeStore)
        store.work, store.check = Path('/not-an-actual-lease'), Mock()
        base = dict(st_mode=stat.S_IFREG | 0o600, st_uid=0, st_nlink=1, st_size=20)
        for changes in ({'st_size': 8193}, {'st_nlink': 2}, {'st_uid': 501},
                        {'st_mode': stat.S_IFREG | 0o644}):
            with self.subTest(changes=changes), patch.object(m.os, 'open', return_value=17), \
                    patch.object(m.os, 'fstat', return_value=SimpleNamespace(**dict(base, **changes))), \
                    patch.object(m.os, 'read') as read, patch.object(m.os, 'close') as close:
                with self.assertRaises(RuntimeError):
                    store.read('original.json')
                read.assert_not_called()
                close.assert_called_once_with(17)
        with patch.object(m.os, 'open', return_value=17), \
                patch.object(m.os, 'fstat', return_value=SimpleNamespace(**base)), \
                patch.object(m.os, 'read', return_value=b'0' * 8193) as read, patch.object(m.os, 'close'):
            with self.assertRaises(RuntimeError):
                store.read('state.json')
            read.assert_called_once_with(17, 8193)

    def test_work_output_paths_are_fixed_and_cannot_escape_volatile_lease(self):
        for work in ('/tmp/charge', '/dev/shm/osmo-charge-' + 'a' * 11,
                     '/dev/shm/osmo-charge-' + 'a' * 12 + '/..'):
            with self.assertRaises(RuntimeError):
                m.main(work, 'prepare', False, NONCE, '/tmp/result')
        with self.assertRaises(RuntimeError):
            m.main('/dev/shm/osmo-charge-0123456789ab', 'prepare', False, NONCE,
                   '/blackbox/export_tmp/osmo_charge_0123456789ac.json')

    def test_helper_source_is_ascii_and_import_does_not_run_native_io(self):
        source = Path(m.__file__).read_bytes()
        source.decode('ascii')
        namespace = {'__name__': 'usb_charging_library'}
        with patch.object(m.os, 'open', side_effect=AssertionError('No device opens during import')):
            exec(compile(source, '<native charging helper>', 'exec'), namespace)
        self.assertTrue(callable(namespace['main']))


if __name__ == '__main__':
    unittest.main()
