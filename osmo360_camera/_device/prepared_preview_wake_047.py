"""Finite prepared-plan adapter for the stock multi-cycle USB standby owner.

Stage this file AND byte-identical prepared_preview_wake_045.py in fresh
/dev/shm/osmo-prepared-preview047-<12hex> with manifest.json. Launch ONLY while
the exact previous Sleep is stable, near the one expected Wake (also for long
Sleep). Reuses045's single-intent apply, independent120s BOOTTIME supervisor,
hash/native-plan checks and strict cancel unchanged. No key, power, recording,
RTC-cancel, new owner, upload or retry is performed here. Import has no action.
"""
import hashlib
from pathlib import Path
import re
import sys
import types

BASE_SHA = '33c8a2a1af90fe94dd7c93b58d4f15234b5c5dc0f35e088420c16c1118a654fc'
OWNER_SHA = '5a2199c2120a35f1b1b20bb22315dbd5b54b5e4ba89a2a9cd9f35afb77ef088e'
WORK = r'/dev/shm/osmo-prepared-preview047-([0-9a-f]{12})'
STANDBY_WORK = r'/dev/shm/osmo-usb-standby-([0-9a-f]{12})'


def load_base():
    path = Path(__file__).with_name('prepared_preview_wake_045.py')
    source = path.read_bytes()
    if hashlib.sha256(source).hexdigest() != BASE_SHA:
        raise RuntimeError('Byte-identical frozen045 observer dependency required')
    module = types.ModuleType('prepared047_frozen045')
    module.__file__ = str(path)
    exec(compile(source, str(path), 'exec'), module.__dict__)
    return module


base = load_base()
_validate045, _wait045, _launch045 = base.validate_manifest, base.wait_and_launch, base.launch


def validate_manifest(value, work):
    renamed = {'standby_work', 'standby_nonce', 'standby_owner', 'owner_sha256',
               'sleep_command_id', 'wake_command_id'}
    rest = {'schema', 'nonce', 'sys_service', 'plan_work', 'scripts',
            'bundle_plain_sha256', 'native_baseline'}
    if (type(value) is not dict or set(value) != renamed | rest
            or not re.fullmatch(WORK, str(work)) or value.get('nonce') != work.name[-12:]):
        raise ValueError('Exact047 manifest/work nonce required')
    match = re.fullmatch(STANDBY_WORK, value['standby_work'])
    if (not match or match[1] != value['standby_nonce']
            or not re.fullmatch('[0-9a-f]{64}', value['owner_sha256'])
            or value['owner_sha256'] != OWNER_SHA):
        raise ValueError('Pinned standby owner path/nonce/hash required')
    for key in ('sleep_command_id', 'wake_command_id'):
        if not isinstance(value[key], str) or not re.fullmatch('[0-9a-f]{16}', value[key]):
            raise ValueError('Exact sixteen-digit command ID required')
    sleep_id, wake_id = int(value['sleep_command_id'], 16), int(value['wake_command_id'], 16)
    if sleep_id < 1 or wake_id != sleep_id + 1:
        raise ValueError('Exact completed Sleep then next Wake sequence required')
    # Keep045's strict seven-script, native5/SSP, MAC/IPv4 and token codec
    # byte-identical. Only its source-owner field names/path are adapted.
    canonical = {key: value[key] for key in rest}
    canonical.update(s3_work='/dev/shm/osmo-gui-qs-exit-' + value['standby_nonce'],
                     s3_nonce=value['standby_nonce'], s3_owner=value['standby_owner'])
    _validate045(canonical, Path('/dev/shm/osmo-prepared-preview-' + value['nonce']))
    return value


def environment():
    script = Path(__file__)
    work = script.parent
    if (sys.platform != 'linux' or base.os.geteuid() != 0 or not re.fullmatch(WORK, str(work))
            or script.name != 'prepared_preview_wake_047.py'):
        raise ValueError('Exact047 camera root RAM script path required')
    base.private(work, directory=True)
    base.private(script)
    dependency = script.with_name('prepared_preview_wake_045.py')
    base.private(dependency)
    if hashlib.sha256(base.read_bytes(dependency, 100000)).hexdigest() != BASE_SHA:
        raise ValueError('Frozen045 dependency changed')
    base.tmpfs(work)
    return work


def source_guard(cfg):
    work = Path(cfg['standby_work'])
    base.private(work, directory=True)
    base.tmpfs(work)
    if hashlib.sha256(base.read_bytes(work / 'usb_standby_owner.py', 100000)).hexdigest() != cfg['owner_sha256']:
        raise ValueError('Reviewed standby owner source changed')
    configuration = base.read_json(work / 'config.json')
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if (configuration.get('nonce') != cfg['standby_nonce'] or configuration.get('owner') != cfg['standby_owner']
            or configuration.get('sys_service') != cfg['sys_service'] or configuration.get('boot_id') != boot_id
            or base.read_json(work / 'owner.json') != cfg['standby_owner']
            or not base.alive(cfg['standby_owner']) or not base.alive(cfg['sys_service'])):
        raise ValueError('Pinned same-boot standby owner/system process changed')
    if any((work / name).exists() for name in ('reboot.json', 'finalize.json', 'recovery-intent.json')):
        raise ValueError('Standby recovery/finalize requested; no prepared apply')
    status = base.read_json(work / 'status.json')
    if (status.get('nonce') != cfg['standby_nonce'] or status.get('owner') != cfg['standby_owner']
            or status.get('service') != cfg['sys_service'] or status.get('owner_alive') is not True
            or status.get('retained_shadow') is not True or status.get('error')):
        raise ValueError('Matching live standby status required')
    allowed = {'SLEEP': cfg['sleep_command_id'], 'WAKING': cfg['wake_command_id'], 'AWAKE': cfg['wake_command_id']}
    identifier = allowed.get(status.get('state'))
    if (identifier is None or status.get('command_id') != identifier
            or type(status.get('last_sequence')) is not int or status['last_sequence'] != int(identifier, 16)):
        raise ValueError('Owner is not in the exact expected Sleep/Wake generation')
    return work


def correlated(value, cfg, identifier, action):
    return (value.get('nonce'), value.get('owner'), value.get('command_id'), value.get('action')) == (
        cfg['standby_nonce'], cfg['standby_owner'], identifier, action)


def moment(value):
    at = value.get('at', {}).get('boottime')
    if type(at) not in (int, float) or not base.math.isfinite(at) or at < 0:
        raise ValueError('Finite native command evidence time required')
    return at


def command_proof(cfg, work, identifier, action):
    folder = work / ('cmd-' + identifier)
    request = base.read_json(folder / 'request.json')
    result = base.read_json(folder / 'result.json')
    finished = base.read_json(folder / 'guard-finished.json')
    intent = base.read_json(folder / 'key-intent.json')
    if (any(not correlated(value, cfg, identifier, action) for value in (request, result, finished, intent))
            or result.get('complete') is not True or result.get('lpc_set_calls') != 0
            or result.get('service') != cfg['sys_service'] or finished.get('cancelled') is not True
            or any(finished.get('closed', {}).get(name, {}).get('closed_verified_ebadf') is not True
                   for name in ('alarm90', 'alarm240'))):
        raise ValueError('Correlated native result and both closed transition RTCs required')
    pulse = result.get('pulse', {})
    if (pulse.get('key'), pulse.get('hold_seconds'), pulse.get('press_release_complete')) != (
            357, 1.1 if action == 'sleep' else .1, True):
        raise ValueError('Exact one stock QS transition pulse required')
    if not moment(intent) <= moment(result) <= moment(finished):
        raise ValueError('Native command/guard evidence order')
    if action == 'sleep':
        if (any(request.get(key) is not True for key in ('record0', 'battery_present1',
                'activation_state1', 'streams_closed', 'own_app_absent'))
                or result.get('get_level') not in (2, 3) or result.get('shadow_value') != 'none'
                or result.get('gui', {}).get('processing3_after_intent') is not True
                or result.get('gui', {}).get('done3_after_intent') is not True
                or not correlated(result.get('gui', {}), cfg, identifier, action)):
            raise ValueError('Completed exact Sleep with fresh GUI processing/done required')
    else:
        exited = result.get('real_exit', {})
        if (result.get('get_level') != 0 or result.get('shadow_value') != 'exit'
                or exited.get('completed') is not True or exited.get('bytes_written') != 5
                or exited.get('literal') != 'exit' or exited.get('real_role_after') != 'peripheral'
                or exited.get('error')):
            raise ValueError('Completed exact Wake GET0 and real EXIT/peripheral required')
        if not moment(result) <= moment(exited) <= moment(finished):
            raise ValueError('Completed EXIT must precede transition guard closure')
    return dict(request=request, result=result, finished=finished, intent=intent)


def ready_proof(cfg, since):
    work = source_guard(cfg)
    sleep = command_proof(cfg, work, cfg['sleep_command_id'], 'sleep')
    status = base.read_json(work / 'status.json')
    if status.get('state') != 'AWAKE':
        return None
    wake = command_proof(cfg, work, cfg['wake_command_id'], 'wake')
    result = wake['result']
    if status.get('last_result') != result:
        raise ValueError('Stable AWAKE must publish this exact completed Wake result')
    wake_at = moment(wake['intent'])
    if wake_at < since or moment(sleep['finished']) > wake_at:
        raise ValueError('Expected Wake must follow this finite observer launch')
    return dict(sleep_command_id=cfg['sleep_command_id'], wake_command_id=cfg['wake_command_id'],
        wake_at=wake['intent']['at'], exit_at=result['real_exit'].get('at'),
        standby_owner=cfg['standby_owner'], sys_service=cfg['sys_service'],
        both_transition_rtcs_closed=True, stable_state='AWAKE', real_exit_completed=True,
        real_role_after='peripheral')


def wait_and_launch(*args, **kwargs):
    return _wait045(*args, proof=ready_proof, guard=source_guard, **kwargs)


def launch(work):
    cfg = validate_manifest(base.read_json(work / 'manifest.json'), work)
    source = source_guard(cfg)
    status = base.read_json(source / 'status.json')
    sleep = command_proof(cfg, source, cfg['sleep_command_id'], 'sleep')
    if (status.get('state') != 'SLEEP' or status.get('last_result') != sleep['result']
            or (source / ('cmd-' + cfg['wake_command_id'])).exists()):
        raise ValueError('Arm047 only in exact stable Sleep before its one future Wake')
    return _launch045(work)


# Bind only the adapter's private imported045 module. Its source remains
# byte-identical; SDK and separately running045 instances are untouched.
base.__file__ = __file__
base.environment = environment
base.validate_manifest = validate_manifest
base.source_guard = source_guard
base.ready_proof = ready_proof
base.wait_and_launch = wait_and_launch
base.launch = launch


if __name__ == '__main__':
    base.main()
