"""Finite camera-only, single-use prepared-preview wake watcher.

Upload to fresh /dev/shm/osmo-prepared-preview-<12hex>/ with manifest.json.
``launch`` detaches a 120s CLOCK_BOOTTIME-bounded observer. It never presses a
key, changes power/mode, cancels RTC recovery, or uploads/constructs a command.
After the pinned 9db helper proves its one native QS wake and real EXIT, this
observer launches ONLY the seven preverified native USB-plan scripts' apply.sh.
The unchanged plan owns its 90s setup/10s renewable lease and rollback. An
exclusive intent precedes Popen; uncertain launch is never retried. RAM reports
prove scheduling, never video/IMU readiness. No action occurs on import.
"""
import argparse
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time

WORK = r'/dev/shm/osmo-prepared-preview-([0-9a-f]{12})'
S3_WORK = r'/dev/shm/osmo-gui-qs-exit-([0-9a-f]{12})'
PLAN_WORK = r'/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}'
S3_SHA = '9db827623c2e7f9767fc8a52ba5dfc601a768fafdbc1687666b1cbe7cf26e959'
SYS_SHA = 'f2fc2a40b8c10b0d1a8289089bb0ab6700372fa062404f44c3dcac00ad513029'
NAMES = frozenset(('apply.sh', 'rollback.sh', 'watchdog.sh', 'lifecycle.py',
                   'netdev.py', 'lease.py', 'ecm_dhcp.py'))
FUNCTIONS = ('ffs.diag', 'ffs.bulk', 'ffs.bulk1', 'ffs.bulk5', 'ffs.bulk4')
GADGET = Path('/sys/kernel/config/usb_gadget/g1')
MAX_JSON, MAX_SCRIPT, TIMEOUT = 16384, 65536, 120.0


def boot():
    return time.clock_gettime(7)  # Linux CLOCK_BOOTTIME, includes real suspend.


def stamp():
    return dict(boottime=boot(), monotonic=time.monotonic())


def fail(exc):
    return (type(exc).__name__ + ': ' + str(exc)).encode('ascii', 'backslashreplace')[:240].decode()


def unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('Duplicate JSON key')
        value[key] = item
    return value


def encoded(value):
    data = (json.dumps(value, sort_keys=True, ensure_ascii=True, allow_nan=False) + '\n').encode('ascii')
    if len(data) > MAX_JSON:
        raise ValueError('RAM report exceeds16KiB')
    return data


def private(path, *, directory=False):
    info = path.lstat()
    if (info.st_uid != 0 or info.st_mode & 0o022 or path.is_symlink()
            or path.resolve() != path
            or not (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode))):
        raise ValueError('Exact private root-owned path required')
    return info


def read_bytes(path, limit=MAX_JSON):
    private(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022 or info.st_size > limit:
            raise ValueError('Unexpected owned regular file/bound')
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError('Owned file exceeds bound')
    return data


def read_json(path):
    return json.loads(read_bytes(path), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def write_new(path, value):
    data = encoded(value)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)


def publish(work, value):
    path = work / 'report.json'
    if path.exists():
        prior = read_json(path)
        if (prior.get('nonce'), prior.get('owner')) != (value['nonce'], value['owner']):
            raise ValueError('Foreign report ownership')
    temporary = work / ('report.next-' + os.urandom(4).hex())
    write_new(temporary, value)
    os.replace(temporary, path)


def token(pid=None):
    pid = os.getpid() if pid is None else pid
    fields = (Path('/proc') / str(pid) / 'stat').read_text().split(') ', 1)[1].split()
    return dict(pid=pid, start=fields[19])


def valid_token(value):
    return (type(value) is dict and set(value) == {'pid', 'start'}
            and type(value['pid']) is int and value['pid'] > 1
            and isinstance(value['start'], str) and bool(re.fullmatch(r'[0-9]{1,24}', value['start'])))


def alive(value):
    try:
        return valid_token(value) and token(value['pid']) == value
    except (OSError, ValueError, IndexError):
        return False


def tmpfs(path):
    rows = []
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        left, separator, right = line.partition(' - ')
        fields, tail = left.split(), right.split()
        if separator and len(fields) >= 6 and len(tail) >= 3:
            point = re.sub(r'\\([0-7]{3})', lambda match: chr(int(match[1], 8)), fields[4])
            if str(path) == point or str(path).startswith(point.rstrip('/') + '/'):
                rows.append((len(point), tail[0]))
    if not rows or max(rows)[1] != 'tmpfs':
        raise ValueError('Canonical RAM tmpfs required')


def environment():
    script = Path(__file__)
    work = script.parent
    if (sys.platform != 'linux' or os.geteuid() != 0 or not re.fullmatch(WORK, str(work))
            or script.name != 'prepared_preview_wake_045.py'):
        raise ValueError('Exact camera root RAM script path required')
    private(work, directory=True)
    private(script)
    tmpfs(work)
    return work


def validate_manifest(value, work):
    required = {'schema', 'nonce', 's3_work', 's3_nonce', 's3_owner', 'sys_service',
                'plan_work', 'scripts', 'bundle_plain_sha256', 'native_baseline'}
    if (type(value) is not dict or set(value) != required or type(value['schema']) is not int
            or value['schema'] != 1 or value['nonce'] != work.name[-12:]
            or not re.fullmatch(WORK, str(work))):
        raise ValueError('Manifest schema/owned nonce')
    match = re.fullmatch(S3_WORK, value['s3_work'])
    if (not match or match[1] != value['s3_nonce']
            or not re.fullmatch(PLAN_WORK, value['plan_work'])
            or not valid_token(value['s3_owner']) or not valid_token(value['sys_service'])
            or not re.fullmatch('[0-9a-f]{64}', value['bundle_plain_sha256'])):
        raise ValueError('Pinned S3/plan paths, owners or bundle hash')
    if type(value['scripts']) is not dict or set(value['scripts']) != NAMES:
        raise ValueError('Exact seven-script allowlist required')
    for item in value['scripts'].values():
        if (type(item) is not dict or set(item) != {'sha256', 'bytes'}
                or type(item['bytes']) is not int or not 0 < item['bytes'] <= MAX_SCRIPT
                or not re.fullmatch('[0-9a-f]{64}', item['sha256'])):
            raise ValueError('Exact script size/hash required')
    native = value['native_baseline']
    fields = {'udc', 'vendor', 'product', 'configuration', 'original_links', 'original_max_speed',
              'dev_addr', 'host_addr', 'net_if', 'original_flags', 'original_netdev_absent',
              'initial_ifname', 'camera_ipv4', 'ram_dir', 'function_name'}
    if (type(native) is not dict or set(native) != fields
            or (native['vendor'], native['product'], native['configuration'], native['original_max_speed'],
                native['ram_dir'], native['function_name']) !=
               ('0x2ca3', '0x0025', 'BULK', 'super-speed-plus', '/dev/shm', 'ecm.ecm')
            or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', native['udc'])
            or type(native['original_links']) not in (list, tuple) or len(native['original_links']) != 5
            or type(native['camera_ipv4']) not in (list, tuple)
            or type(native['original_netdev_absent']) is not bool):
        raise ValueError('Exact native shooting baseline required')
    for target, name in zip(native['original_links'], FUNCTIONS):
        if (not isinstance(target, str) or '\0' in target or '\n' in target
                or os.path.normpath(str(GADGET / 'configs/c.1' / target)) != str(GADGET / 'functions' / name)):
            raise ValueError('Native five function links/order')
    for key in ('dev_addr', 'host_addr'):
        mac = native[key]
        if (not isinstance(mac, str) or not re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', mac)
                or int(mac[:2], 16) & 1 or mac == '00:00:00:00:00:00'):
            raise ValueError('Exact unicast runtime MAC required')
    if native['dev_addr'] == native['host_addr']:
        raise ValueError('Distinct ECM MACs required')
    if native['original_netdev_absent']:
        if (native['net_if'], native['original_flags'], native['initial_ifname']) != (None, None, '(unnamed net_device)'):
            raise ValueError('Exact absent-netdev baseline')
    elif (not isinstance(native['net_if'], str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', native['net_if'])
          or native['original_flags'] != '0x1002' or native['initial_ifname'] != native['net_if']):
        raise ValueError('Exact retained down-netdev baseline')
    # Preserve the bootstrap's foreign IPv4 evidence; never silently adopt a
    # camera that already owns an address in the temporary preview subnet.
    for entry in native['camera_ipv4']:
        if (type(entry) not in (list, tuple) or len(entry) != 2
                or not isinstance(entry[0], str)):
            raise ValueError('IPv4 baseline entry')
        address = ipaddress.ip_interface(entry[1])
        if address.version != 4 or address.network.overlaps(ipaddress.ip_network('192.168.253.0/30')):
            raise ValueError('Temporary preview subnet already in use')
        if entry[0] == native['net_if']:
            raise ValueError('ECM interface already has IPv4')
    return value


def plan_proof(cfg):
    work = Path(cfg['plan_work'])
    private(work, directory=True)
    tmpfs(work)
    if any(os.path.lexists(work / name) for name in ('used', 'armed', 'usb.touched', 'f6.owned', 'apply.owner')):
        raise ValueError('Prepared plan has already been used; never retry')
    if read_bytes(work / 'bundle.ready', 65) != (cfg['bundle_plain_sha256'] + '\n').encode('ascii'):
        raise ValueError('Exact verified bundle-ready marker required')
    for name, item in cfg['scripts'].items():
        data = read_bytes(work / name, MAX_SCRIPT)
        if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
            raise ValueError('Prepared script size/hash mismatch: ' + name)
    return dict(seven_scripts_verified=True, bundle_plain_sha256=cfg['bundle_plain_sha256'])


def attribute(path):
    # This retail ECM driver includes a trailing NUL in its sysfs strings.
    # Match the already-reviewed netdev helper; never trim an internal NUL.
    data = path.read_bytes().rstrip(b'\0\r\n')
    if not data or b'\0' in data or len(data) > 256:
        raise ValueError('Unexpected native sysfs attribute')
    return data.decode('ascii')


def native_proof(cfg, gadget=GADGET):
    native = cfg['native_baseline']
    config = gadget / 'configs/c.1'
    for path, expected in ((gadget / 'idVendor', native['vendor']), (gadget / 'idProduct', native['product']),
        (config / 'strings/0x409/configuration', native['configuration']),
        (gadget / 'UDC', native['udc']), (gadget / 'max_speed', native['original_max_speed'])):
        if attribute(path) != expected:
            raise ValueError('Native gadget baseline changed: ' + path.name)
    links = {path.name: os.readlink(path) for path in config.iterdir() if path.is_symlink()}
    if links != dict(zip(('f1', 'f2', 'f3', 'f4', 'f5'), native['original_links'])):
        raise ValueError('Exactly the five native links must remain')
    if os.path.lexists(config / 'f6'):
        raise ValueError('Foreign/old sixth USB function')
    function = gadget / 'functions/ecm.ecm'
    for name, expected in (('dev_addr', native['dev_addr']), ('host_addr', native['host_addr'])):
        if attribute(function / name).lower() != expected:
            raise ValueError('ECM runtime MAC changed')
    if attribute(function / 'ifname') != native['initial_ifname']:
        raise ValueError('ECM baseline ifname changed')
    if native['net_if'] is not None:
        if attribute(Path('/sys/class/net') / native['net_if'] / 'flags') != native['original_flags']:
            raise ValueError('Retained ECM interface flags changed')
    return dict(native_five_links=True, udc=native['udc'], max_speed=native['original_max_speed'])


def source_guard(cfg):
    work = Path(cfg['s3_work'])
    private(work, directory=True)
    tmpfs(work)
    if hashlib.sha256(read_bytes(work / 'gui_qs_exit_forward_probe.py', 100000)).hexdigest() != S3_SHA:
        raise ValueError('Frozen9db S3 helper digest changed')
    source = read_json(work / 'config.json')
    if (source.get('nonce') != cfg['s3_nonce'] or source.get('forward_native_exit') is not True
            or source.get('host_record0') is not True or source.get('host_battery_present1') is not True
            or source.get('host_activation_state1') is not True
            or read_json(work / 'owner.json') != cfg['s3_owner'] or not alive(cfg['s3_owner'])
            or not alive(cfg['sys_service'])):
        raise ValueError('Pinned S3/root service ownership changed')
    if any((work / name).exists() for name in ('reboot.json', 'worker-error.json', 'wake-error.json', 'finalize.json')):
        raise ValueError('S3 recovery/failure was requested; no prepared apply')
    return work


def ready_proof(cfg, since):
    work = source_guard(cfg)
    status = read_json(work / 'status.json')
    if status.get('error') or status.get('reason'):
        raise ValueError('S3 owner reported failure/recovery')
    if not (work / 'wake.json').exists() or status.get('backup90_cancelled') is not True:
        return None
    wake = read_json(work / 'wake.json')
    intent = read_json(work / 'entry-intent.json')
    receipt = read_json(work / 'entry-result.json')
    observed = read_json(work / 'entry-observed.json')
    common = (intent, receipt, observed, wake, status)
    if any(value.get('nonce') != cfg['s3_nonce'] or value.get('owner') != cfg['s3_owner'] for value in common):
        raise ValueError('S3 entry/wake/status correlation mismatch')
    pulse, entry = wake.get('pulse', {}), receipt.get('pulse', {})
    if (any(value.get('lpc_set_calls') != 0 for value in (intent, receipt, observed, wake))
            or intent.get('activation_raw_state') != 1
            or (entry.get('key'), entry.get('hold_seconds'), entry.get('press_release_complete')) != (357, 1.1, True)
            or observed.get('get_after', {}).get('returncode') != 0
            or observed.get('get_after', {}).get('level') not in (2, 3)
            or wake.get('confirmation') != 'stock_qs_wake_get0'
            or (pulse.get('key'), pulse.get('hold_seconds'), pulse.get('press_release_complete')) != (357, .1, True)
            or wake.get('get_after', {}).get('returncode') != 0
            or wake.get('get_after', {}).get('level') != 0):
        raise ValueError('Exact stock QS entry and wake GET0 required')
    for ready in (intent.get('guards', {}), wake.get('readiness_before', {}), wake.get('readiness_after', {})):
        if (ready.get('service') != cfg['sys_service'] or ready.get('verified_executable') != SYS_SHA
                or ready.get('clients_ever_ready') != 1 or ready.get('reboot_disabled') != 0):
            raise ValueError('Same exact native ready system service required')
    if wake['readiness_after'].get('system_power_level') != 0:
        raise ValueError('Native wake manager GET0 required')
    times = [value.get('at', {}).get('boottime') for value in (intent, receipt, observed, wake)]
    if (any(type(value) not in (int, float) or not math.isfinite(value) for value in times)
            or times != sorted(times) or times[-1] < since):
        raise ValueError('Fresh correlated native wake required')
    if (status.get('wake') != wake or status.get('alarm90_cancel', {}).get('closed_verified_ebadf') is not True
            or not status.get('alarm240')):
        raise ValueError('Resident owner must confirm90s cancellation and retain240s recovery')
    if not (work / 'real-exit-result.json').exists() or 'real_exit_forward' not in status:
        return None
    exited = read_json(work / 'real-exit-result.json')
    if (exited != status['real_exit_forward'] or exited.get('nonce') != cfg['s3_nonce']
            or exited.get('owner') != cfg['s3_owner'] or exited.get('completed') is not True
            or exited.get('literal') != 'exit' or exited.get('bytes_written') != 5
            or exited.get('real_role_after') != 'peripheral' or exited.get('error')
            or exited.get('at', {}).get('boottime', -1) < times[-1]):
        raise ValueError('Completed real native EXIT/peripheral proof required')
    return dict(wake_at=wake['at'], exit_at=exited['at'], s3_owner=cfg['s3_owner'],
                sys_service=cfg['sys_service'], backup90_cancelled=True,
                real_exit_completed=True, real_role_after='peripheral')


def wait_and_launch(work, cfg, *, clock=boot, sleep=time.sleep, proof=ready_proof,
                    plan=plan_proof, native=native_proof, guard=source_guard,
                    popen=subprocess.Popen, publisher=publish, started=None, owner_token=None):
    started = clock() if started is None else started
    result = dict(nonce=cfg['nonce'], owner=token() if owner_token is None else owner_token, phase='waiting_native_wake',
                  started_boottime=started, deadline_boottime=started + TIMEOUT,
                  apply_attempts=0, media_ready_proven=False, recovery_owner_unchanged=True)
    publisher(work, result)
    def current():
        if owner_token is not None and not alive(owner_token):
            raise ValueError('Independent observer parent exited; no apply')
        if (work / 'cancel-request.json').exists():
            request = read_json(work / 'cancel-request.json')
            if request.get('nonce') != cfg['nonce'] or request.get('owner') != result['owner']:
                raise ValueError('Cancellation correlation mismatch')
            raise InterruptedError('Owned prepared observer cancellation; no launch retry')
        guard(cfg)
        if clock() >= started + TIMEOUT:
            raise TimeoutError('120s prepared-wake observer deadline; no apply/retry')
    while True:
        current()
        evidence = proof(cfg, started)
        if evidence is None:
            sleep(.05)
            continue
        plan(cfg)
        native(cfg)
        current()
        intent = dict(nonce=cfg['nonce'], owner=result['owner'], plan_work=cfg['plan_work'],
                      apply_sha256=cfg['scripts']['apply.sh']['sha256'], wake=evidence, at=stamp())
        # Intent is irrevocable, including failed/uncertain Popen. No call may
        # retry this single-use plan through this watcher afterward.
        write_new(work / 'apply-intent.json', intent)
        plan(cfg)
        native(cfg)
        current()
        log = Path(cfg['plan_work']) / 'apply.log'
        fd = os.open(log, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        with os.fdopen(fd, 'wb') as stream:
            result['apply_attempts'] = 1
            result['phase'] = 'apply_launch_intent'
            publisher(work, result)
            current()
            # Same argv/session/stdin/output as the reviewed generated
            # `setsid /bin/sh .../apply.sh </dev/null >.../apply.log 2>&1`.
            child = popen(['/bin/sh', cfg['plan_work'] + '/apply.sh'], stdin=subprocess.DEVNULL,
                          stdout=stream, stderr=stream, start_new_session=True, close_fds=True)
        result.update(phase='apply_scheduled', apply_pid=child.pid, apply_at=stamp(), wake=evidence)
        publisher(work, result)
        return result


def watch(work):
    cfg = validate_manifest(read_json(work / 'manifest.json'), work)
    deadline = boot() + 1
    while not (work / 'owner-ready.json').exists() and boot() < deadline:
        time.sleep(.01)
    ready = read_json(work / 'owner-ready.json')
    owner_token = read_json(work / 'owner.json')
    if ready.get('owner') != owner_token or ready.get('nonce') != cfg['nonce'] or not alive(owner_token):
        raise ValueError('Independent observer parent missing')
    started = ready['started_boottime']
    try:
        return wait_and_launch(work, cfg, started=started, owner_token=owner_token)
    except BaseException as exc:
        prior = read_json(work / 'report.json') if (work / 'report.json').exists() else {}
        prior.update(nonce=cfg['nonce'], owner=owner_token,
                     phase='cancelled_no_retry' if isinstance(exc, InterruptedError) else 'failed_no_retry',
                     error=fail(exc), at=stamp())
        publish(work, prior)
        raise


def reap_owned(child):
    """Only the Popen watcher child; never signal the separate apply session."""
    if child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=.2)
        except subprocess.TimeoutExpired:
            child.kill()
            try:
                child.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                pass  # No false child-death proof after a kernel-blocked read.


def supervise(work, cfg, owned, started, child, *, clock=boot, sleep=time.sleep):
    """Parent does no sysfs/native calls; its BOOTTIME limit is independent."""
    while child.poll() is None:
        if clock() >= started + TIMEOUT or (work / 'cancel-request.json').exists():
            reap_owned(child)
            result = dict(nonce=cfg['nonce'], owner=owned, phase='observer_stopped_no_retry',
                          error='observer deadline/cancel', watcher_returncode=child.poll(),
                          apply_intent_present=(work / 'apply-intent.json').exists(),
                          media_ready_proven=False, recovery_owner_unchanged=True)
            publish(work, result)
            return result
        sleep(.05)
    return read_json(work / 'report.json')


def owner(work):
    cfg = validate_manifest(read_json(work / 'manifest.json'), work)
    owned, started = token(), boot()
    write_new(work / 'owner.json', owned)
    # A separate worker can block on a driver sysfs read without trapping this
    # parent's finite timer. It receives no real-role FD or power client.
    child = subprocess.Popen([sys.executable, __file__, '_watch'], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True, close_fds=True)
    write_new(work / 'watcher.json', dict(nonce=cfg['nonce'], owner=owned, process=token(child.pid)))
    write_new(work / 'owner-ready.json', dict(nonce=cfg['nonce'], owner=owned,
                                           started_boottime=started, deadline_boottime=started + TIMEOUT))
    return supervise(work, cfg, owned, started, child)


def launch(work):
    cfg = validate_manifest(read_json(work / 'manifest.json'), work)
    source = source_guard(cfg)
    if (source / 'wake.json').exists():
        raise ValueError('Must arm before the one native wake')
    plan_proof(cfg)
    native_proof(cfg)
    write_new(work / 'launch-intent.json', dict(nonce=cfg['nonce'], at=stamp()))
    fd = os.open(work / 'owner.log', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        child = subprocess.Popen([sys.executable, __file__, '_owner'], stdin=subprocess.DEVNULL,
                                 stdout=stream, stderr=stream, start_new_session=True, close_fds=True)
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        if (work / 'owner-ready.json').exists():
            ready = read_json(work / 'owner-ready.json')
            if ready.get('nonce') != cfg['nonce'] or not alive(ready.get('owner')) or ready['owner']['pid'] != child.pid:
                raise ValueError('Prepared observer ownership missing')
            return dict(launched=True, nonce=cfg['nonce'], owner=ready['owner'], armed=ready,
                        report=str(work / 'report.json'), no_media_success_claim=True)
        if child.poll() is not None:
            raise ValueError('Prepared observer exited before ready; never retry')
        time.sleep(.02)
    raise TimeoutError('Prepared observer launch uncertain; never retry')


def status(work):
    cfg = validate_manifest(read_json(work / 'manifest.json'), work)
    value = read_json(work / 'report.json')
    if value.get('nonce') != cfg['nonce'] or value.get('owner') != read_json(work / 'owner.json'):
        raise ValueError('Prepared report correlation mismatch')
    return dict(value, observer_alive=alive(value['owner']))


def cancel(work):
    """Stop only this observer, never an already scheduled native USB plan.

    Success requires the observer stopped AND no apply intent. Otherwise this
    command fails: the host retains/finalizes S3 recovery before any cleanup
    of an uncertain plan. A proven lease ACK separately permits normal owned
    plan rollback. No generic PID kill or late USB cleanup is performed here.
    """
    cfg = validate_manifest(read_json(work / 'manifest.json'), work)
    owned = read_json(work / 'owner.json')
    if not valid_token(owned):
        raise ValueError('Owned observer token required')
    request = dict(nonce=cfg['nonce'], owner=owned)
    path = work / 'cancel-request.json'
    if path.exists():
        if read_json(path) != request:
            raise ValueError('Foreign cancellation request')
    else:
        write_new(path, request)
    deadline = time.monotonic() + 2
    while alive(owned) and time.monotonic() < deadline:
        time.sleep(.02)
    watcher = read_json(work / 'watcher.json')
    if watcher.get('nonce') != cfg['nonce'] or watcher.get('owner') != owned or not valid_token(watcher.get('process')):
        raise ValueError('Owned watcher child token required')
    stopped = not alive(owned) and not alive(watcher['process'])
    intent = (work / 'apply-intent.json').exists()
    if not stopped or intent:
        # D1 exposes a command status, not stdout. Never return success while
        # an apply child could still be between preflight and lifecycle.claim.
        # The operator must finalize the existing S3/reboot recovery first.
        raise RuntimeError('Cancellation uncertain or apply intent exists; retain S3 recovery before cleanup')
    return dict(nonce=cfg['nonce'], owner=owned, observer_stopped=True,
                no_future_apply=True, apply_intent_present=False,
                no_usb_or_power_mutation=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('launch', 'status', 'cancel', '_owner', '_watch'))
    args = parser.parse_args()
    work = environment()
    value = {'launch': launch, 'status': status, 'cancel': cancel, '_owner': owner, '_watch': watch}[args.mode](work)
    sys.stdout.buffer.write(encoded(value))


if __name__ == '__main__':
    main()
