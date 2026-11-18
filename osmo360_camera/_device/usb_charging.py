"""Volatile, exact-generation USB battery-charge policy lease.

No current-limit, PMIC, GPIO, thermal, reboot or background-worker controls.
Sysfs policy readback is cached driver state, never physical proof. Reads and
reports are bounded; a blocked kernel syscall still has no hard deadline.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import time

KERNEL = '5.15.148-ab639'
MODULE = '/lib/modules/' + KERNEL + '/qti_battery_charger.ko'
MODULE_SHA = '20325ac30bb9da94f402822f8b45b803aceb066fd28f6d23a722304090e753bc'
ROOT = '/sys/class/qcom-battery'
DRIVER = '/sys/bus/platform/drivers/qti_battery_charger'
POLICY = ('charge_disable', 'restrict_chg', 'restrict_cur')
PATHS = {key: ROOT + '/' + key for key in POLICY}
PATHS.update({
    'usb_current_now': '/sys/class/power_supply/usb/current_now',
    'usb_online': '/sys/class/power_supply/usb/online',
    'battery_current_now': '/sys/class/power_supply/battery/current_now',
    'battery_status': '/sys/class/power_supply/battery/status',
    'battery_present': '/sys/class/power_supply/battery/present',
    'battery_capacity': '/sys/class/power_supply/battery/capacity',
    'battery_temp': '/sys/class/power_supply/battery/temp',
})
TELEMETRY = tuple(key for key in PATHS if key not in POLICY)
OFF = dict(charge_disable=1, restrict_chg=1, restrict_cur=0)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def error_text(error):
    return type(error).__name__ + ': ' + str(error)[:240]


def encoded(value, limit=8192):
    data = json.dumps(value, separators=(',', ':'), allow_nan=False).encode('ascii')
    require(len(data) <= limit, 'Oversize JSON')
    return data


def valid_policy(value):
    require(type(value) is dict and set(value) == set(POLICY), 'Invalid policy fields')
    require(all(type(value[k]) is int for k in POLICY), 'Non-integer policy')
    require(value['charge_disable'] in (0, 1) and value['restrict_chg'] in (0, 1)
            and 0 <= value['restrict_cur'] <= 2147483647, 'Invalid policy values')
    require(value['charge_disable'] == int(bool(value['restrict_chg'] and
                                               value['restrict_cur'] == 0)),
            'Inconsistent driver policy')
    return dict(value)


class NativeIO:
    def __init__(self):
        require(os.geteuid() == 0 and os.uname().release == KERNEL, 'Unsupported native kernel/uid')
        require(os.path.realpath(ROOT) == ROOT, 'Unexpected charger class target')
        device = Path('/sys/class/power_supply/usb').resolve(strict=True).parent.parent
        require(Path('/sys/class/power_supply/battery').resolve(strict=True).parent.parent == device,
                'Power supplies belong to different devices')
        self.device = str(device)
        self.driver_link = str(device / 'driver')
        require(os.path.realpath(self.driver_link) == DRIVER, 'Unexpected charger driver')
        descriptor = os.open(MODULE, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            before = os.fstat(descriptor)
            data = os.read(descriptor, 104729)
            after = os.fstat(descriptor)
            require(stat.S_ISREG(before.st_mode) and before.st_size == 104728
                    and self.module_identity(before) == self.module_identity(after)
                    and len(data) == 104728 and hashlib.sha256(data).hexdigest() == MODULE_SHA,
                    'Unsupported native charger module')
            module_pin = self.module_identity(before)
        finally:
            os.close(descriptor)
        self.receipt = dict(boot_id=self.boot_id(), kernel=KERNEL, module_sha256=MODULE_SHA,
            module_pin=module_pin, class_pin=self.identity(ROOT), device_pin=self.identity(self.device),
            driver_pin=self.identity(self.driver_link), driver_target=DRIVER,
            sysfs_pins={key: self.identity(path, regular=True) for key, path in PATHS.items()})

    @staticmethod
    def identity(path, regular=False):
        value = os.stat(path)
        require(not regular or stat.S_ISREG(value.st_mode), 'Non-file sysfs node')
        return [value.st_dev, value.st_ino]

    @staticmethod
    def module_identity(value):
        return [value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns]

    @staticmethod
    def boot_id():
        descriptor = os.open('/proc/sys/kernel/random/boot_id', os.O_RDONLY | os.O_NOFOLLOW)
        try:
            data = os.read(descriptor, 65)
            require(len(data) <= 64, 'Oversize boot ID')
            boot = data.decode('ascii').strip()
            require(re.fullmatch('[0-9a-f]{8}(-[0-9a-f]{4}){3}-[0-9a-f]{12}', boot), 'Invalid boot ID')
            return boot
        finally:
            os.close(descriptor)

    def check(self, expected=None):
        r = self.receipt
        require(self.boot_id() == r['boot_id'], 'Camera boot changed')
        require(os.path.realpath(ROOT) == ROOT and self.identity(ROOT) == r['class_pin']
                and self.identity(self.device) == r['device_pin'], 'Charger class/device changed')
        require(os.path.realpath(self.driver_link) == DRIVER
                and self.identity(self.driver_link) == r['driver_pin'], 'Driver binding changed')
        require(self.module_identity(os.stat(MODULE)) == r['module_pin'], 'Charger module changed')
        if expected is not None:
            require(r == expected, 'Lease generation/pins changed')

    def open_node(self, key, flags):
        self.check()
        descriptor = os.open(PATHS[key], flags | os.O_NOFOLLOW)
        value = os.fstat(descriptor)
        if not stat.S_ISREG(value.st_mode) or [value.st_dev, value.st_ino] != self.receipt['sysfs_pins'][key]:
            os.close(descriptor)
            raise RuntimeError('Sysfs node changed: ' + key)
        return descriptor

    def read(self, key):
        descriptor = self.open_node(key, os.O_RDONLY)
        try:
            data = os.read(descriptor, 129)
            require(len(data) <= 128, 'Oversize scalar sysfs reply')
            value = data.decode('ascii').strip()
            if key == 'battery_status':
                return value
            require(re.fullmatch('-?[0-9]{1,12}', value), 'Invalid scalar sysfs reply')
            return int(value)
        finally:
            os.close(descriptor)

    def write(self, key, value):
        require(type(value) is int and ((key in ('charge_disable', 'restrict_chg') and value in (0, 1))
                or (key == 'restrict_cur' and 0 <= value <= 2147483647)), 'Unreviewed policy write')
        descriptor = self.open_node(key, os.O_WRONLY)
        try:
            data = (str(value) + '\n').encode('ascii')
            require(os.write(descriptor, data) == len(data), 'Short sysfs write')
        finally:
            os.close(descriptor)


class NativeStore:
    def __init__(self, work):
        require(type(work) is str and re.fullmatch('/dev/shm/osmo-charge-[0-9a-f]{12}', work),
                'Invalid charging work directory')
        self.work = Path(work)
        info = self.work.lstat()
        require(stat.S_ISDIR(info.st_mode) and info.st_uid == 0 and stat.S_IMODE(info.st_mode) == 0o700
                and os.path.realpath(work) == work, 'Unsafe charging work directory')
        self.pin = [info.st_dev, info.st_ino]

    def check(self):
        info = self.work.lstat()
        require(stat.S_ISDIR(info.st_mode) and info.st_uid == 0 and stat.S_IMODE(info.st_mode) == 0o700
                and [info.st_dev, info.st_ino] == self.pin and os.path.realpath(self.work) == str(self.work),
                'Charging work directory changed')

    def read(self, name):
        self.check()
        require(name in ('original.json', 'state.json'), 'Invalid lease filename')
        try:
            descriptor = os.open(str(self.work / name), os.O_RDONLY | os.O_NOFOLLOW)
        except FileNotFoundError:
            return None
        try:
            info = os.fstat(descriptor)
            require(stat.S_ISREG(info.st_mode) and info.st_uid == 0 and info.st_nlink == 1
                    and stat.S_IMODE(info.st_mode) == 0o600 and info.st_size <= 8192, 'Unsafe lease file')
            data = os.read(descriptor, 8193)
            require(len(data) <= 8192, 'Oversize lease file')
            return json.loads(data.decode('ascii'))
        finally:
            os.close(descriptor)

    def write(self, name, value, nonce):
        self.check()
        require(name in ('original.json', 'state.json'), 'Invalid lease filename')
        path = self.work / (name if name == 'original.json' else '.state-' + nonce + '.tmp')
        data = encoded(value)
        descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        info = os.fstat(descriptor)
        complete = False
        try:
            require(os.write(descriptor, data) == len(data), 'Short lease write')
            os.fsync(descriptor)
            complete = True
        finally:
            os.close(descriptor)
            if name == 'state.json' and not complete:
                try:
                    now = path.lstat()
                    if [now.st_dev, now.st_ino] == [info.st_dev, info.st_ino]:
                        path.unlink()
                except FileNotFoundError:
                    pass
        if name == 'state.json':
            self.check()
            os.replace(str(path), str(self.work / name))


def policy(io):
    return valid_policy({key: io.read(key) for key in POLICY})


def telemetry(io):
    result = {}
    for key in TELEMETRY:
        before = time.monotonic_ns()
        try:
            result[key] = dict(value=io.read(key), start_ns=before, end_ns=time.monotonic_ns())
        except Exception as error:
            result[key] = dict(error=error_text(error), start_ns=before, end_ns=time.monotonic_ns())
    return result


def charge_observation(measured):
    require(all(type(measured[key].get('value')) is int for key in
                ('usb_current_now', 'battery_current_now')), 'Fresh IBUS/IBAT observation unavailable')
    require(measured['battery_current_now']['value'] <= 0,
            'Battery charge current remains positive')
    return measured


def write_guard(io, expected):
    io.check(expected)
    require(io.read('usb_online') == 1 and io.read('battery_present') == 1, 'USB/battery absent')
    temperature = io.read('battery_temp')
    require(type(temperature) is int and -200 <= temperature <= 450, 'Battery temperature outside write range')


def restore_policy(io, original, owned, state, store, nonce, events):
    io.check(original['identity'])
    current = policy(io)
    require(current in owned, 'Foreign policy changed; refusing restoration')
    target = original['original_policy']
    # Never briefly enable an originally disabled battery charger.
    writes = [('charge_disable', 1)] if target['charge_disable'] else [
        ('charge_disable', 0), ('restrict_cur', target['restrict_cur']),
        ('restrict_chg', target['restrict_chg'])]
    for key, value in writes:
        if current[key] == value:
            continue
        write_guard(io, original['identity'])
        require(policy(io) == current, 'Foreign policy changed before restoration')
        state['pending'] = dict(key=key, value=value)
        store.write('state.json', state, nonce)
        events.append(dict(operation='restore', key=key, value=value))
        io.write(key, value)
        expected = (dict(charge_disable=value, restrict_chg=value, restrict_cur=0)
                    if key == 'charge_disable' else dict(current, **{key: value}))
        current = policy(io)
        require(current == expected, 'Restoration SET readback mismatch')
        state.update(last_applied=current, dirty=current != target, pending=None)
        store.write('state.json', state, nonce)
    require(current == target, 'Original policy not exactly restored')
    state.update(last_applied=current, dirty=False, pending=None)
    store.write('state.json', state, nonce)
    return current


def execute(io, store, operation, enabled, nonce, sleep=time.sleep):
    result = dict(schema=1, operation=operation, operation_nonce=nonce, ok=False, ready=False,
                  state='error', boot_id=io.receipt['boot_id'], receipt=None, policy=None,
                  telemetry={}, errors=[], restore=dict(state='not_needed', errors=[]), changed=False,
                  evidence=dict(policy_readback_is_cache=True), physical_proof='unknown',
                  policy_readback_is_cache=True, hard_timeout_guaranteed=False, events=[])
    try:
        require(operation in ('prepare', 'check', 'restore') and type(enabled) is bool
                and type(nonce) is str and re.fullmatch('[0-9a-f]{16,64}', nonce), 'Invalid operation arguments')
        io.check()
        current = policy(io)
        original, state = store.read('original.json'), store.read('state.json')
        if original is None:
            require(operation == 'prepare' and state is None, 'Charging lease is not prepared')
            write_guard(io, io.receipt)
            original = dict(schema=1, identity=json.loads(encoded(io.receipt)), original_policy=current)
            state = dict(schema=1, boot_id=io.receipt['boot_id'], last_applied=current, dirty=False, pending=None)
            store.write('original.json', original, nonce)
            store.write('state.json', state, nonce)
        require(type(original) is dict and set(original) == {'schema', 'identity', 'original_policy'}
                and type(original.get('schema')) is int and original['schema'] == 1
                and type(state) is dict and set(state) == {'schema', 'boot_id', 'last_applied', 'dirty', 'pending'}
                and type(state.get('schema')) is int and state['schema'] == 1, 'Invalid charging receipt/state')
        io.check(original['identity'])
        valid_policy(original['original_policy'])
        require(state.get('boot_id') == io.receipt['boot_id'] and state.get('pending') is None,
                'Lease generation or pending mutation is unknown')
        valid_policy(state['last_applied'])
        require(type(state.get('dirty')) is bool
                and state['dirty'] == (state['last_applied'] != original['original_policy']),
                'Inconsistent charging lease ownership')
        require(current == state['last_applied'],
                'Foreign policy changed; refusing operation')
        result['receipt'] = original
        result['policy'] = current
        if operation == 'restore':
            result['changed'] = current != original['original_policy']
            current = restore_policy(io, original, [current], state, store, nonce, result['events'])
            result['restore']['state'] = 'cache_restored'
            result['state'], result['ready'] = 'restored', True
        else:
            target = OFF if not enabled else (current if not current['charge_disable'] else
                                               dict(charge_disable=0, restrict_chg=0, restrict_cur=0))
            if operation == 'check':
                require(current == target, 'Requested policy is not applied')
            elif current != target:
                write_guard(io, original['identity'])
                require(policy(io) == current, 'Foreign policy changed before apply')
                state.update(dirty=True, pending=dict(key='charge_disable', value=int(not enabled)))
                store.write('state.json', state, nonce)
                result['changed'] = True
                try:
                    result['events'].append(dict(operation='apply', key='charge_disable', value=int(not enabled)))
                    io.write('charge_disable', int(not enabled))
                    observed = policy(io)
                    require(observed == target, 'Charging SET readback mismatch')
                    current = observed
                    state.update(last_applied=current, dirty=current != original['original_policy'], pending=None)
                    store.write('state.json', state, nonce)
                except BaseException as error:
                    result['errors'].append(error_text(error))
                    try:
                        current = restore_policy(io, original, [current, target], state, store, nonce, result['events'])
                        result['restore']['state'] = 'cache_restored_after_error'
                    except BaseException as restore_error:
                        result['restore']['state'] = 'unknown'
                        result['restore']['errors'].append(error_text(restore_error))
                    raise RuntimeError('Charging apply failed; inspect original and restoration errors')
            result['state'], result['ready'] = 'cache_verified', True
        result['policy'] = current
        result['ok'] = True
    except BaseException as error:
        result['errors'].append(error_text(error))
        if operation == 'restore':
            result['restore']['state'] = 'unknown'
            result['restore']['errors'].append(error_text(error))
        try:
            result['policy'] = policy(io)
        except Exception:
            result['policy'] = None
    if result['ok'] and operation == 'prepare' and not enabled:
        # The reviewed A/B sampled the current reduction after two seconds.
        # Cache readback alone must not release the webcam immediately after SET.
        result['ready'] = False
        result['evidence']['settle'] = dict(target_seconds=2, start_ns=time.monotonic_ns())
        try:
            sleep(2)
            result['evidence']['settle']['end_ns'] = time.monotonic_ns()
            result['evidence']['settled_telemetry'] = telemetry(io)
            measured = charge_observation(result['evidence']['settled_telemetry'])
            io.check(original['identity'])
            require(policy(io) == current, 'Policy changed during settle')
            result['ready'] = True
            result['telemetry'] = measured
        except BaseException as error:
            result.update(ok=False, state='error')
            result['errors'].append(error_text(error))
            try:
                current = restore_policy(io, original, [current], state, store, nonce, result['events'])
                result['policy'] = current
                result['restore']['state'] = 'cache_restored_after_error'
            except BaseException as restore_error:
                result['restore']['state'] = 'unknown'
                result['restore']['errors'].append(error_text(restore_error))
    elif result['ok'] and operation == 'check' and not enabled:
        # Read-only post-transition gate: never roll back a policy while the
        # caller may already be draining native preview and IMU capture.
        result['ready'] = False
        try:
            result['evidence']['checked_telemetry'] = telemetry(io)
            measured = charge_observation(result['evidence']['checked_telemetry'])
            io.check(original['identity'])
            require(policy(io) == current, 'Policy changed during check')
            result['telemetry'] = measured
            result['ready'] = True
        except BaseException as error:
            result.update(ok=False, state='error')
            result['errors'].append(error_text(error))
    if not result['telemetry']:
        result['telemetry'] = telemetry(io)
    return result


def main(work, operation, enabled, operation_nonce, output):
    require(type(work) is str and re.fullmatch('/dev/shm/osmo-charge-[0-9a-f]{12}', work), 'Invalid work path')
    require(output == '/blackbox/export_tmp/osmo_charge_' + work[-12:] + '.json', 'Invalid output path')
    try:
        result = execute(NativeIO(), NativeStore(work), operation, enabled, operation_nonce)
    except BaseException as error:
        result = dict(schema=1, operation=operation, operation_nonce=operation_nonce, ok=False,
                      ready=False, state='error', errors=[error_text(error)], physical_proof='unknown')
    descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(descriptor)
        result['i'] = [info.st_dev, info.st_ino]
        data = encoded(result, 16384)
        require(os.write(descriptor, data) == len(data), 'Short charging report write')
    finally:
        os.close(descriptor)


if __name__ == 'usb_charging_control':
    main(WORK, OPERATION, ENABLED, OP_NONCE, OUTPUT)
