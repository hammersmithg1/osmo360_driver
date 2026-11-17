"""Finite camera-only retained-six UDC experiment; never an apply/USB-mode API.

Stage with frozen047/045 and manifest.json in fresh private tmpfs work. Arm
only in exact stable Sleep just before Wake. One isolated worker observes receipts, then
writes UDC empty and the same saved UDC exactly once. The independent RTC
supervisor remains runnable if a kernel attribute write blocks. Any failure
requests the literal stock reboot; no key, LPC setter, network setter or retry.
"""
import ast
import ctypes
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time
import types

WORK = r'/dev/shm/osmo-retained-sleep060-([0-9a-f]{12})'
OWNER_SHA = '9529f3ee74274b5458a4dcda670a1efc94e6c607b7e65971629d03de7058aeb8'
SOURCES = {'prepared_preview_wake_047.py': '246c4e253c07c675f4992b6b4ec494fda3c2d973ad7ad91801134203e569d3a5',
           'prepared_preview_wake_045.py': '33c8a2a1af90fe94dd7c93b58d4f15234b5c5dc0f35e088420c16c1118a654fc'}
NAMES = {'apply.sh', 'rollback.sh', 'watchdog.sh', 'lifecycle.py', 'netdev.py', 'lease.py', 'ecm_dhcp.py'}
FUNCTIONS = ('ffs.diag', 'ffs.bulk', 'ffs.bulk1', 'ffs.bulk5', 'ffs.bulk4', 'ecm.ecm')
GADGET = Path('/sys/kernel/config/usb_gadget/g1')
MAX_JSON = 16384
MAX_FAILURE = 8192
COMPLETION_QUERY = struct.Struct('!8s16sQ')
COMPLETION_MAGIC = b'OSMO060Q'
MAX_REPLY = 1024


def boot(): return time.clock_gettime(7)


def finite(value): return type(value) in (int, float) and math.isfinite(value)


def token(value):
    return type(value) is dict and set(value) == {'pid', 'start'} and type(value['pid']) is int and value['pid'] > 1 and isinstance(value['start'], str) and value['start'].isdigit()


def encoded(value):
    raw = (json.dumps(value, sort_keys=True, allow_nan=False) + '\n').encode('ascii')
    if len(raw) > MAX_JSON: raise ValueError('Bounded16KiB diagnostic required')
    return raw


def diagnostic_text(value, limit=600):
    return str(value).encode('ascii', 'backslashreplace')[:limit].decode('ascii')


def ram_failure_snapshot(b, path, kind):
    """Only fixed private RAM diagnostics; bounded prefix, never log/media reads."""
    try:
        b.private(path)
        fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
                raise ValueError('Private root diagnostic file required')
            raw = stream.read(8193)
        value = json.loads(raw) if len(raw) <= 8192 else None
        if type(value) is dict:
            keys = {'native_status': ('state', 'command_id', 'action', 'last_sequence', 'owner', 'service', 'error'),
                'result': ('terminal', 'reset_attempted', 'completed_boottime', 'worker', 'lease_sequence', 'lease_expires')}.get(kind)
            if keys is not None:
                value = {key: value[key] for key in keys if key in value}
            if kind == 'native_status' and type(json.loads(raw).get('last_result')) is dict:
                terminal = json.loads(raw)['last_result']
                value['last_result'] = {key: terminal[key] for key in
                    ('complete', 'get_level', 'shadow_value', 'lpc_set_calls', 'error') if key in terminal}
            text = json.dumps(value, sort_keys=True, ensure_ascii=True)
        else:
            text = raw[:1024].decode('ascii', 'backslashreplace')
        return dict(bytes_total=info.st_size, text=diagnostic_text(text, 1024),
                    truncated=info.st_size > len(raw) or len(raw) > 8192 or len(text) > 1024)
    except FileNotFoundError:
        return dict(absent=True)
    except BaseException as error:
        return dict(read_error=diagnostic_text(error, 200))


def failure_payload(work, cfg, b, where, error, stage):
    paths = {'checkpoint': work / 'checkpoint.json', 'worker_error': work / 'worker-error.json',
        'result': work / 'result.json', 'reset_intent': work / 'reset-intent.json',
        'native_status': Path(cfg['standby_work']) / 'status.json',
        'lease_state': Path(cfg['lease_work']) / 'lease.state'}
    value = dict(schema=1, nonce=cfg['nonce'], where=where, stage=diagnostic_text(stage, 160),
        error_type=type(error).__name__, error=diagnostic_text(error),
        monotonic=time.monotonic(),
        sleep_command_id=cfg['sleep_command_id'], wake_command_id=cfg['wake_command_id'],
        snapshots={kind: ram_failure_snapshot(b, path, kind) for kind, path in paths.items()})
    if type(getattr(error, 'pin_difference', None)) is dict:
        value['pin_difference'] = error.pin_difference
    # JSON escaping can expand a bounded text. Keep exact error/stage and mark
    # any lower-priority snapshot omission instead of exceeding the8KiB cap.
    for key in ('result', 'native_status', 'worker_error', 'reset_intent', 'lease_state', 'checkpoint'):
        if len(json.dumps(value, sort_keys=True, allow_nan=False).encode('ascii')) + 1 <= MAX_FAILURE: break
        value['snapshots'][key] = dict(omitted='failure diagnostic byte cap')
    if len(encoded(value)) > MAX_FAILURE: raise ValueError('Failure diagnostic byte cap')
    return value


def export_failure(work, cfg, b, where, error, stage):
    output = Path('/blackbox/export_tmp/osmo_rebind_failure_' + cfg['nonce'] + '.json')
    b.private(output.parent, directory=True)
    b.write_new(output, failure_payload(work, cfg, b, where, error, stage))


def recover_with_diagnostic(owner, base, work, cfg, b, error, hard):
    """The literal stock reboot is spawned BEFORE the first durable byte."""
    original, attempted = base.stock_reboot, False
    def stock_first():
        nonlocal attempted
        request = original()
        if not attempted:
            attempted = True  # Even failed/blocked export can never be replayed.
            def diagnostic_only():
                try: export_failure(work, cfg, b, 'supervisor', error, 'supervisor-recovery')
                except BaseException: pass
            # Durable I/O cannot delay owner.recovery's independent reserve.
            try: threading.Thread(target=diagnostic_only, daemon=True).start()
            except BaseException: pass
        return request
    base.stock_reboot = stock_first
    try:
        owner.recovery(base, work, dict(owner=base.proc_token(), nonce=cfg['nonce']), None, str(error)[:200], hard)
    finally:
        base.stock_reboot = original


def load_verified(path, digest, name):
    source = path.read_bytes()
    if hashlib.sha256(source).hexdigest() != digest: raise ValueError('Frozen source changed: ' + path.name)
    module = types.ModuleType(name); module.__file__ = str(path)
    exec(compile(source, str(path), 'exec'), module.__dict__)
    return module


def modules(work, cfg):
    for name, digest in SOURCES.items():
        if hashlib.sha256((work / name).read_bytes()).hexdigest() != digest: raise ValueError('Frozen observer changed')
    observer = load_verified(work / 'prepared_preview_wake_047.py', SOURCES['prepared_preview_wake_047.py'], 'retained060_observer')
    observer.base.private(work, directory=True); observer.base.tmpfs(work)
    owner_path = Path(cfg['standby_work']) / 'usb_standby_owner.py'
    observer.base.private(owner_path)
    owner = load_verified(owner_path, OWNER_SHA, 'retained060_stock_owner')
    return observer, owner, owner.base_module()  # Frozen9db definitions only.


def manifest(work):
    raw = (work / 'manifest.json').read_bytes()
    if not 0 < len(raw) <= MAX_JSON: raise ValueError('Manifest size')
    cfg = json.loads(raw)
    keys = {'schema', 'nonce', 'standby_work', 'standby_nonce', 'standby_owner', 'owner_sha256', 'sys_service',
        'sleep_command_id', 'wake_command_id', 'lease_work', 'lease_nonce', 'lease_port', 'plan_sha256',
        'udc', 'dev_mac', 'host_mac', 'camera_ip', 'host_ip', 'host_sequence', 'completion_port'}
    match = re.fullmatch(WORK, str(work))
    if (type(cfg) is not dict or set(cfg) != keys or type(cfg['schema']) is not int or cfg['schema'] != 1
            or match is None or cfg['nonce'] != match[1] or cfg['owner_sha256'] != OWNER_SHA
            or not isinstance(cfg['standby_nonce'], str) or not re.fullmatch('[0-9a-f]{12}', cfg['standby_nonce'])
            or cfg['standby_work'] != '/dev/shm/osmo-usb-standby-' + cfg['standby_nonce']
            or not token(cfg['standby_owner']) or not token(cfg['sys_service'])
            or not isinstance(cfg['lease_work'], str) or not re.fullmatch('/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}', cfg['lease_work'])
            or not isinstance(cfg['lease_nonce'], str) or not re.fullmatch('[0-9a-f]{32}', cfg['lease_nonce'])
            or type(cfg['lease_port']) is not int or not 49152 <= cfg['lease_port'] <= 65535
            or (cfg['completion_port'] is not None and (type(cfg['completion_port']) is not int
                or not 49152 <= cfg['completion_port'] <= 65535 or cfg['completion_port'] == cfg['lease_port']))
            or type(cfg['host_sequence']) is not int or not 1 <= cfg['host_sequence'] < 2**64
            or type(cfg['plan_sha256']) is not dict or set(cfg['plan_sha256']) != NAMES
            or any(not isinstance(v, str) or not re.fullmatch('[0-9a-f]{64}', v) for v in cfg['plan_sha256'].values())
            or not isinstance(cfg['udc'], str) or not re.fullmatch('[A-Za-z0-9_.:-]{1,100}', cfg['udc'])
            or (cfg['camera_ip'], cfg['host_ip']) != ('192.168.253.1', '192.168.253.2')):
        raise ValueError('Exact retained-six manifest required')
    for key in ('dev_mac', 'host_mac'):
        value = cfg[key]
        if not isinstance(value, str) or not re.fullmatch('(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', value) or int(value[:2], 16) & 1:
            raise ValueError('Exact unicast MAC required')
    if cfg['dev_mac'] == cfg['host_mac']: raise ValueError('Distinct MACs required')
    for key in ('sleep_command_id', 'wake_command_id'):
        if not isinstance(cfg[key], str) or not re.fullmatch('[0-9a-f]{16}', cfg[key]): raise ValueError('Exact command ID')
    if int(cfg['sleep_command_id'], 16) < 1 or int(cfg['wake_command_id'], 16) != int(cfg['sleep_command_id'], 16) + 1:
        raise ValueError('Consecutive Sleep/Wake IDs')
    return cfg


def attribute(path):
    raw = path.read_bytes().strip(b'\0 \t\r\n')
    if len(raw) > 256 or b'\0' in raw: raise ValueError('Unexpected sysfs text')
    return raw.decode('ascii')


def ipv4(interface):
    """Read-only libc ioctl: embedded Python need not provide fcntl."""
    lib = ctypes.CDLL(None, use_errno=True)
    values = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for operation in (0x8915, 0x891b):
            buf = ctypes.create_string_buffer(struct.pack('16s24x', interface.encode()))
            if lib.ioctl(sock.fileno(), operation, buf): raise OSError(ctypes.get_errno(), 'Read USB IPv4')
            values.append(socket.inet_ntoa(buf.raw[20:24]))
    return str(ipaddress.IPv4Interface('/'.join(values)))


def process(base, pid, path, mode):
    before = base.proc_token(pid)
    args = (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0')
    args = [x for x in args if x]
    if len(args) != 3 or args[1:] != [str(path).encode(), mode.encode()] or not base.alive(before):
        raise ValueError('Exact lease process argv/start required')
    return before


def source_state(observer, cfg):
    base, work = observer.base, Path(cfg['standby_work'])
    base.private(work, directory=True); base.tmpfs(work)
    if hashlib.sha256(base.read_bytes(work / 'usb_standby_owner.py', 100000)).hexdigest() != OWNER_SHA:
        raise ValueError('Native owner changed')
    conf, state = base.read_json(work / 'config.json'), base.read_json(work / 'status.json')
    if (conf.get('boot_id') != Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            or conf.get('nonce') != cfg['standby_nonce'] or conf.get('owner') != cfg['standby_owner']
            or conf.get('sys_service') != cfg['sys_service'] or base.read_json(work / 'owner.json') != cfg['standby_owner']
            or not base.alive(cfg['standby_owner']) or not base.alive(cfg['sys_service'])
            or state.get('nonce') != cfg['standby_nonce'] or state.get('owner') != cfg['standby_owner']
            or state.get('service') != cfg['sys_service'] or state.get('retained_shadow') is not True
            or state.get('error') or any((work / name).exists() for name in ('reboot.json', 'finalize.json', 'recovery-intent.json'))):
        raise ValueError('Same live native owner/system/boot required')
    return state


def snapshot(observer, base, cfg, *, gadget=GADGET):
    p, b = Path(cfg['lease_work']), observer.base
    b.private(p, directory=True); b.tmpfs(p)
    for name, digest in cfg['plan_sha256'].items():
        if hashlib.sha256(b.read_bytes(p / name, 65536)).hexdigest() != digest: raise ValueError('Active plan changed')
    if (b.read_bytes(p / 'status') != b'APPLIED\n' or not (p / 'armed').is_file()
            or not (p / 'used').is_file() or not (p / 'f6.owned').is_file()
            or any(os.path.lexists(p / name) for name in ('lease.closed', 'lease.end', 'rollback.lock'))):
        raise ValueError('Only the still-armed owned active plan is eligible')
    nodes = ast.parse(b.read_bytes(p / 'lease.py', 65536))
    configs = [ast.literal_eval(n.value) for n in nodes.body if isinstance(n, ast.Assign)
               and any(isinstance(x, ast.Name) and x.id == 'CONFIG' for x in n.targets)]
    expected = dict(work=cfg['lease_work'], nonce=cfg['lease_nonce'], port=cfg['lease_port'],
        camera_ip=cfg['camera_ip'], host_ip=cfg['host_ip'], dev_mac=cfg['dev_mac'], lease_seconds=10, setup_seconds=90)
    if configs != [expected]: raise ValueError('Exact existing lease config required')
    state = b.read_json(p / 'lease.state'); now = time.monotonic()
    if (state.get('nonce') != cfg['lease_nonce'] or state.get('state') != 'active'
            or type(state.get('seq')) is not int or state['seq'] < cfg['host_sequence']
            or not finite(state.get('expires')) or not finite(state.get('updated'))
            or not now < state['expires'] <= state['updated'] + 10.000001 or state['updated'] > now + .5):
        raise ValueError('An unclosed fresh active lease is required')
    receiver = b.read_json(p / 'lease.receiver.owner')
    if not token(receiver) or process(base, receiver['pid'], p / 'lease.py', 'receiver') != receiver:
        raise ValueError('Original receiver changed')
    watchdog = int(b.read_bytes(p / 'watchdog.pid', 32))
    if watchdog != int(b.read_bytes(p / 'watchdog.ready', 32)): raise ValueError('Watchdog PID mismatch')
    watchdog = process(base, watchdog, p / 'lease.py', 'watchdog')
    c = gadget / 'configs/c.1'
    links = {x.name: os.readlink(x) for x in c.iterdir() if x.is_symlink()}
    if set(links) != {'f' + str(i) for i in range(1, 7)}: raise ValueError('Exactly six links required')
    for i, name in enumerate(FUNCTIONS, 1):
        if (c / links['f' + str(i)]).resolve() != gadget.resolve() / 'functions' / name:
            raise ValueError('Owned six-function targets changed')
    f = gadget / 'functions/ecm.ecm'
    interface = attribute(f / 'ifname')
    if not re.fullmatch('[A-Za-z0-9_.-]{1,15}', interface) or attribute(p / 'ifname') != interface:
        raise ValueError('Exact active USB ifname required')
    net = Path('/sys/class/net') / interface
    if (attribute(gadget / 'UDC') != cfg['udc'] or attribute(gadget / 'max_speed') != 'high-speed'
            or attribute(f / 'dev_addr').lower() != cfg['dev_mac'] or attribute(f / 'host_addr').lower() != cfg['host_mac']
            or attribute(net / 'address').lower() != cfg['dev_mac'] or not int(attribute(net / 'flags'), 0) & 1
            or ipv4(interface) != cfg['camera_ip'] + '/30'):
        raise ValueError('UDC/HS/MAC/IPv4 changed')
    info = (gadget / 'UDC').stat()
    return dict(udc=cfg['udc'], max_speed='high-speed', links=links, ifname=interface,
        ifindex=int(attribute(net / 'ifindex')), camera_mac=cfg['dev_mac'], host_mac=cfg['host_mac'],
        camera_ipv4=cfg['camera_ip'] + '/30', receiver=receiver, watchdog=watchdog,
        udc_identity=[info.st_dev, info.st_ino], lease_sequence=state['seq'], lease_expires=state['expires'])


def same_active(pre, post):
    return all(pre.get(key) == post.get(key) for key in pre if key not in ('lease_sequence', 'lease_expires')) and post['lease_sequence'] >= pre['lease_sequence']


def pinned_change(message, pre, current):
    """Evidence only: preserve the exact changed-key list, never accept drift."""
    values = {key: dict(before=pre.get(key), current=current.get(key)) for key in pre
        if key not in ('lease_sequence', 'lease_expires') and pre.get(key) != current.get(key)}
    if current['lease_sequence'] < pre['lease_sequence']:
        values['lease_sequence'] = dict(before=pre['lease_sequence'], current=current['lease_sequence'])
    detail = dict(changed_keys=sorted(values), values=values)
    if len(json.dumps(detail, sort_keys=True, allow_nan=False).encode('ascii')) + 1 > 2048:
        prefix = diagnostic_text(json.dumps(values, sort_keys=True, separators=(',', ':')), 1400)
        detail = dict(changed_keys=sorted(values), values_prefix=prefix, truncated=True)
        while len(encoded(detail)) > 2048:
            detail['values_prefix'] = detail['values_prefix'][:len(detail['values_prefix']) // 2]
    error = ValueError(message + '; ' + json.dumps(detail, sort_keys=True, separators=(',', ':')))
    error.pin_difference = detail
    return error


def check_cancel(work):
    if (work / 'cancel.json').exists(): raise RuntimeError('Owned unattempted cancellation requested')


def sleep_arm_proof(observer, cfg, armed_at):
    """Completed Sleep has no active transition RTC or future Wake input."""
    state = source_state(observer, cfg)
    root = Path(cfg['standby_work'])
    observer.source_guard(cfg)
    sleep = observer.command_proof(cfg, root, cfg['sleep_command_id'], 'sleep')
    if (state.get('state') != 'SLEEP' or state.get('command_id') != cfg['sleep_command_id']
            or state.get('last_sequence') != int(cfg['sleep_command_id'], 16)
            or state.get('last_result') != sleep['result'] or observer.moment(sleep['finished']) > armed_at
            or (root / ('cmd-' + cfg['wake_command_id'])).exists()):
        raise ValueError('Arm only after exact stable completed Sleep before future Wake')
    return sleep


def wake_proof(observer, cfg, armed_at):
    state = source_state(observer, cfg)
    observer.source_guard(cfg)
    root = Path(cfg['standby_work'])
    sleep = observer.command_proof(cfg, root, cfg['sleep_command_id'], 'sleep')
    if observer.moment(sleep['finished']) > armed_at: raise ValueError('Sleep must complete before this arm')
    if state.get('state') != 'AWAKE': return None
    wake = observer.command_proof(cfg, root, cfg['wake_command_id'], 'wake')
    if (state.get('last_result') != wake['result'] or observer.moment(wake['intent']) < armed_at
            or observer.moment(sleep['finished']) > observer.moment(wake['intent'])):
        raise ValueError('Exact stable Wake/EXIT/closed RTC proof required')
    return dict(sleep=sleep['result'], wake=wake['result'])


def pin_udc_fd(fd, identity):
    """Pin the current path and the held open-file description to one node."""
    opened = os.fstat(fd)
    current = os.stat(GADGET / 'UDC', follow_symlinks=False)
    if ([opened.st_dev, opened.st_ino] != identity
            or [current.st_dev, current.st_ino] != identity):
        raise ValueError('Active UDC path/open FD identity changed')


def rebind_once(work, cfg, observer, base, pre, guard, checkpoint=lambda name: None,
                *, armed_at=None, verified_wake=None):
    check_cancel(work); guard()
    if (not finite(armed_at) or type(verified_wake) is not dict
            or not verified_wake or wake_proof(observer, cfg, armed_at) != verified_wake):
        raise ValueError('Exact fresh native Wake proof required before opening UDC')
    guard()
    checkpoint('udc-open')
    fd = os.open(GADGET / 'UDC', os.O_WRONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        # Configfs may allocate a fresh inode for every unheld lookup. Opening
        # the live node pins it: only comparisons during this FD lifetime count.
        info = os.fstat(fd)
        if info.st_dev != pre['udc_identity'][0]:
            raise ValueError('UDC configfs filesystem changed before reset')
        active = dict(pre, udc_identity=[info.st_dev, info.st_ino])
        pin_udc_fd(fd, active['udc_identity'])
        check_cancel(work); guard()
        checkpoint('snapshot-before-reset')
        current = snapshot(observer, base, cfg)
        if not same_active(active, current): raise pinned_change('Pinned applied plan changed before reset', active, current)
        pin_udc_fd(fd, active['udc_identity'])
        check_cancel(work); guard()
        observer.base.write_new(work / 'reset-intent.json', dict(nonce=cfg['nonce'], reset_attempted=1, at=boot()))
        check_cancel(work); guard()
        checkpoint('udc-unbind-write')
        guard()  # A diagnostic checkpoint may have stalled past the deadline.
        pin_udc_fd(fd, active['udc_identity'])
        if os.write(fd, b'\n') != 1: raise OSError('Partial UDC unbind; never retry')
        checkpoint('udc-unbind-returned')
        if attribute(GADGET / 'UDC') != '': raise ValueError('UDC did not unbind')
        check_cancel(work); guard()
        os.lseek(fd, 0, os.SEEK_SET)
        checkpoint('udc-bind-write')
        guard()  # No late bind after a stalled diagnostic checkpoint.
        pin_udc_fd(fd, active['udc_identity'])
        if os.write(fd, cfg['udc'].encode()) != len(cfg['udc']): raise OSError('Partial UDC bind; never retry')
        checkpoint('udc-bind-returned')
        guard(); checkpoint('snapshot-after-reset'); post = snapshot(observer, base, cfg)
        if not same_active(active, post): raise pinned_change('Six functions/HS/IP/receiver/watchdog were not preserved', active, post)
        pin_udc_fd(fd, active['udc_identity'])
        guard()
        return dict(schema=1, nonce=cfg['nonce'], standby_owner=cfg['standby_owner'], sys_service=cfg['sys_service'],
            wake_command_id=cfg['wake_command_id'], terminal=True, reset_attempted=1,
            unbind_completed=True, bind_completed=True, same_six_links=True, lease_fresh=True,
            original_udc_identity=pre['udc_identity'], active_udc_identity=active['udc_identity'],
            lease_sequence=post['lease_sequence'], lease_expires=post['lease_expires'], pre=pre, post=post)
    finally: os.close(fd)


def worker(work, cfg):
    observer, owner, base = modules(work, cfg); b = observer.base
    stage = 'worker-start'
    def checkpoint(name):
        nonlocal stage
        stage = name
        try: owner.replace(work / 'checkpoint.json', dict(nonce=cfg['nonce'], stage=stage, at=boot()))
        except BaseException: pass  # Diagnostics must not alter reset behavior.
    try:
        checkpoint('guard-read')
        guard = b.read_json(work / 'guard.json')
        def fresh():
            check_cancel(work)
            if not base.alive(guard['owner']) or boot() >= guard['deadline_boottime']:
                raise TimeoutError('Independent guard/deadline unavailable')
            source_state(observer, cfg)
        fresh(); checkpoint('source-sleep'); armed_at = boot()
        sleep_arm_proof(observer, cfg, armed_at)
        checkpoint('snapshot-at-arm'); pre = snapshot(observer, base, cfg)
        b.write_new(work / 'armed.json', dict(nonce=cfg['nonce'], worker=base.proc_token(), pre=pre, at=armed_at))
        proof = None
        checkpoint('waiting-exact-sleep-wake')
        while proof is None:
            fresh(); proof = wake_proof(observer, cfg, b.read_json(work / 'armed.json')['at'])
            if proof is None: time.sleep(.05)
        fresh()
        checkpoint('wake-proof-complete')
        done = rebind_once(work, cfg, observer, base, pre, fresh, checkpoint,
            armed_at=b.read_json(work / 'armed.json')['at'], verified_wake=proof)
        done.update(wake_proof=proof, completed_boottime=boot(), worker=base.proc_token())
        checkpoint('result-write'); b.write_new(work / 'result.json', done)
    except BaseException as error:
        try: export_failure(work, cfg, b, 'worker', error, stage)
        except BaseException: pass  # Independent guard remains armed throughout.
        b.write_new(work / 'worker-error.json', dict(nonce=cfg['nonce'], stage=stage, error=str(error)[:300]))


def spawn(work, mode):
    return subprocess.Popen([sys.executable, str(work / 'retained_sleep_060.py'), mode],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True, close_fds=True)


def publish_done(work, cfg, b, done):
    if (done.get('nonce') != cfg['nonce'] or done.get('terminal') is not True
            or done.get('reset_attempted') != 1 or time.monotonic() >= done['lease_expires']
            or (Path(cfg['lease_work']) / 'lease.closed').exists()
            or any(done.get('closed', {}).get(name, {}).get('closed_verified_ebadf') is not True
                   for name in ('alarm90', 'alarm240'))):
        raise ValueError('Final fresh DONE and both closed RTCs required')
    output = Path('/blackbox/export_tmp/osmo_rebind_' + cfg['nonce'] + '.json')
    b.private(output.parent, directory=True)
    b.write_new(output, done)  # Fresh exclusive diagnostic; never update/replay.


def optional_done_export(work, cfg, b, done):
    def diagnostic_only():
        try: publish_done(work, cfg, b, done)
        except BaseException as error:
            try: b.write_new(work / 'file-export-error.json', dict(error=diagnostic_text(error, 300)))
            except BaseException: pass
    thread = threading.Thread(target=diagnostic_only, daemon=True)
    thread.start(); thread.join(timeout=.05)  # Durable diagnostics never hold the successful owner alive.


def completion_ticket(cfg):
    return hashlib.sha256(bytes.fromhex(cfg['lease_nonce']) + bytes.fromhex(cfg['nonce'])
        + bytes.fromhex(cfg['wake_command_id'])).digest()[:16]


def completion_terminal(cfg, done, now, *, closed=True):
    if (done.get('nonce') != cfg['nonce'] or done.get('terminal') is not True
            or done.get('reset_attempted') != 1 or done.get('standby_owner') != cfg['standby_owner']
            or done.get('sys_service') != cfg['sys_service'] or done.get('wake_command_id') != cfg['wake_command_id']
            or any(done.get(key) is not True for key in ('unbind_completed', 'bind_completed', 'same_six_links', 'lease_fresh'))
            or not finite(done.get('lease_expires')) or now >= done['lease_expires']
            or done['original_udc_identity'] != done['pre']['udc_identity']
            or done['active_udc_identity'][0] != done['original_udc_identity'][0]
            or done['active_udc_identity'] != done['post']['udc_identity']
            or not same_active(dict(done['pre'], udc_identity=done['active_udc_identity']), done['post'])):
        raise ValueError('Strict terminal held-FD/reset postproof required')
    if closed and any(done.get('closed', {}).get(key, {}).get('closed_verified_ebadf') is not True
                       for key in ('alarm90', 'alarm240')):
        raise ValueError('Both final RTCs must be closed before any completion reply')


def completion_reply(cfg, done, raw, peer, state, now):
    """Authenticated data-only receipt. A query never renews the lease."""
    if len(raw) != COMPLETION_QUERY.size or peer[0] != cfg['host_ip']: return None
    magic, ticket, challenge = COMPLETION_QUERY.unpack(raw)
    if magic != COMPLETION_MAGIC or ticket != completion_ticket(cfg): return None
    completion_terminal(cfg, done, now)
    completion_lease(cfg, done, state, now)
    value = dict(schema=1, nonce=cfg['nonce'], standby_owner=cfg['standby_owner'], sys_service=cfg['sys_service'],
        wake_command_id=cfg['wake_command_id'], ticket=ticket.hex(), challenge=challenge,
        terminal=True, reset_attempted=1, udc_rebind_completed=True,
        original_plan_work=cfg['lease_work'], lease_expires=state['expires'], now=now)
    reply = encoded(value)
    if len(reply) > MAX_REPLY: raise ValueError('Bounded completion reply required')
    return reply


def completion_lease(cfg, done, state, now):
    if (state.get('nonce') != cfg['lease_nonce'] or state.get('state') != 'active'
            or type(state.get('seq')) is not int or state['seq'] < done['lease_sequence']
            or not finite(state.get('expires')) or not finite(state.get('updated'))
            or not now < state['expires'] <= state['updated'] + 10.000001 or state['updated'] > now + .5):
        raise ValueError('Current owned unclosed lease and final DONE required')


def completion_server(work, cfg):
    """Finite UDP reader only: no UDC, key, power or lease-writing path."""
    def expired(*args): raise TimeoutError('Finite completion responder alarm')
    previous = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, 10)
    try: return _completion_server(work, cfg)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def _completion_server(work, cfg):
    observer, owner, base = modules(work, cfg); b = observer.base
    done = b.read_json(work / 'result.json')
    if cfg['completion_port'] is None: raise ValueError('Completion responder was not requested')
    started = boot()
    lease = Path(cfg['lease_work'])
    def current():
        now = time.monotonic()
        if boot() >= started + 10 or now >= done['lease_expires'] or (lease / 'lease.closed').exists():
            raise TimeoutError('Finite completion responder expired')
        if not base.alive(cfg['standby_owner']) or not base.alive(cfg['sys_service']):
            raise ValueError('Original owner/system exited')
        post = done['post']; net = Path('/sys/class/net') / post['ifname']
        if (attribute(net / 'address').lower() != cfg['dev_mac']
                or int(attribute(net / 'ifindex')) != post['ifindex']
                or ipv4(post['ifname']) != cfg['camera_ip'] + '/30'):
            raise ValueError('Completion responder USB identity changed')
        state = b.read_json(lease / 'lease.state')
        now = time.monotonic()
        if boot() >= started + 10 or now >= done['lease_expires']:
            raise TimeoutError('Finite completion responder expired during read')
        completion_lease(cfg, done, state, now)
        return state, now
    state, now = current()
    completion_terminal(cfg, done, now, closed=False)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_BINDTODEVICE', 25), done['post']['ifname'].encode() + b'\0')
        sock.bind((cfg['camera_ip'], cfg['completion_port']))
        b.write_new(work / 'serve-ready.json', dict(nonce=cfg['nonce'], process=base.proc_token(),
            port=cfg['completion_port'], ifname=done['post']['ifname']))
        while not (work / 'done.json').exists():
            current(); time.sleep(.01)  # Socket is bound but cannot reply yet.
        final = b.read_json(work / 'done.json')
        if {key: value for key, value in final.items() if key != 'closed'} != done:
            raise ValueError('Final DONE must be this exact immutable worker result')
        done = final
        state, now = current()
        completion_terminal(cfg, done, now)
        for _ in range(200):
            state, now = current()
            sock.settimeout(max(.001, min(.1, started + 10 - boot(), done['lease_expires'] - now)))
            try: raw, peer = sock.recvfrom(65)
            except socket.timeout: continue
            state, now = current()
            reply = completion_reply(cfg, done, raw, peer, state, now)
            if reply is not None:
                current()  # No stale reply after an intervening close/expiry.
                sock.sendto(reply, peer)


def supervise(work, cfg):
    observer, owner, base = modules(work, cfg); b = observer.base
    short = hard = child = responder = None
    responder_deadline = None
    try:
        short, hard = base.Alarm(90), base.Alarm(240)
        sampled = boot(); proof = short.verify(90); hard_proof = hard.verify(240)
        deadline = sampled + proof['remaining']
        b.write_new(work / 'guard.json', dict(nonce=cfg['nonce'], owner=base.proc_token(), deadline_boottime=deadline,
            alarm90=proof, alarm240=hard_proof))
        child = spawn(work, '_worker')
        while True:
            readable, _, _ = select.select([short.fd, hard.fd], [], [], .05)
            if readable or boot() >= deadline: raise TimeoutError('Independent retained-six deadline')
            if not base.alive(cfg['standby_owner']) or not base.alive(cfg['sys_service']):
                raise ValueError('Original native owner/system process exited')
            if (work / 'armed.json').exists() and not (work / 'owner-ready.json').exists():
                armed = b.read_json(work / 'armed.json')
                if armed.get('nonce') != cfg['nonce'] or armed['worker']['pid'] != child.pid or not base.alive(armed['worker']):
                    raise ValueError('Owned worker readiness mismatch')
                b.write_new(work / 'owner-ready.json', dict(nonce=cfg['nonce'], guard=base.proc_token(), worker=armed['worker']))
            if (work / 'result.json').exists():
                result = b.read_json(work / 'result.json')
                if (result.get('nonce') != cfg['nonce'] or result.get('terminal') is not True or result.get('reset_attempted') != 1
                        or result.get('worker', {}).get('pid') != child.pid or not finite(result.get('completed_boottime'))
                        or result['completed_boottime'] >= deadline or boot() >= deadline
                        or not finite(result.get('lease_expires')) or time.monotonic() >= result['lease_expires']):
                    raise ValueError('Late/unowned terminal result')
                if cfg['completion_port'] is not None:
                    if responder is None:
                        responder_deadline = boot() + 1
                        responder = spawn(work, '_serve')
                        continue
                    if responder.poll() is not None or boot() >= responder_deadline:
                        raise RuntimeError('Completion responder failed before readiness')
                    if not (work / 'serve-ready.json').exists(): continue
                    ready = b.read_json(work / 'serve-ready.json')
                    if (ready.get('nonce') != cfg['nonce'] or ready.get('process', {}).get('pid') != responder.pid
                            or not token(ready.get('process')) or not base.alive(ready['process'])
                            or ready.get('port') != cfg['completion_port'] or ready.get('ifname') != result['post']['ifname']):
                        raise ValueError('Owned completion responder readiness required')
                if select.select([short.fd, hard.fd], [], [], 0)[0]: raise TimeoutError('RTC already expired')
                closed = dict(alarm90=short.cancel())
                if boot() >= deadline: raise TimeoutError('Late guard cancellation')
                closed['alarm240'] = hard.cancel()
                result['closed'] = closed
                b.write_new(work / 'done.json', result)
                optional_done_export(work, cfg, b, result)
                return
            if (work / 'cancel.json').exists() and not (work / 'reset-intent.json').exists():
                try: child.terminate()
                except ProcessLookupError: pass
                try: child.wait(timeout=1)
                except subprocess.TimeoutExpired: raise RuntimeError('Cancellation worker still alive')
                if (work / 'reset-intent.json').exists(): raise ValueError('Reset raced cancellation; stock recovery required')
                closed = dict(alarm90=short.cancel(), alarm240=hard.cancel())
                b.write_new(work / 'cancelled.json', dict(nonce=cfg['nonce'], no_future_reset=True, closed=closed))
                return
            if (work / 'worker-error.json').exists() or child.poll() is not None:
                raise RuntimeError('Isolated worker failed; no reset retry')
    except BaseException as error:
        recover_with_diagnostic(owner, base, work, cfg, b, error, hard)
    finally:
        for alarm in (short, hard):
            if alarm is not None: alarm.close()


def frontend(work, cfg, mode):
    observer, owner, base = modules(work, cfg); b = observer.base
    output = Path('/blackbox/export_tmp/osmo_rebind_' + cfg['nonce'] + '.json')
    if mode == 'launch':
        if output.exists(): raise ValueError('Fresh exclusive DONE export required')
        sleep_arm_proof(observer, cfg, boot())  # No helper RTC exists before this gate.
        b.write_new(work / 'launch-intent.json', dict(nonce=cfg['nonce']))
        child = spawn(work, '_owner'); deadline = time.monotonic() + 2
        while not (work / 'owner-ready.json').exists():
            if child.poll() is not None or time.monotonic() >= deadline: raise TimeoutError('Guard arm receipt missing; never relaunch')
            time.sleep(.02)
        return b.read_json(work / 'owner-ready.json')
    if mode == 'cancel':
        if (work / 'reset-intent.json').exists(): raise ValueError('Reset already attempted; no cancellation claim')
        b.write_new(work / 'cancel.json', dict(nonce=cfg['nonce']))
        deadline = time.monotonic() + 2
        while not (work / 'cancelled.json').exists():
            if time.monotonic() >= deadline: raise TimeoutError('No cancellation proof')
            time.sleep(.02)
        if (work / 'reset-intent.json').exists(): raise ValueError('Reset raced cancellation')
        return b.read_json(work / 'cancelled.json')
    if (work / 'done.json').exists():
        done = b.read_json(work / 'done.json')
        if time.monotonic() >= done['lease_expires'] or (Path(cfg['lease_work']) / 'lease.closed').exists():
            raise ValueError('DONE lease proof expired before publication')
        if not output.exists() or b.read_json(output) != done: raise ValueError('Final DONE export unavailable/foreign')
        return done
    return dict(nonce=cfg['nonce'], terminal=False)


def main():
    work = Path(__file__).resolve().parent
    if (sys.platform != 'linux' or os.geteuid() != 0 or Path(__file__).name != 'retained_sleep_060.py'
            or not re.fullmatch(WORK, str(work))): raise ValueError('Exact camera root tmpfs work required')
    cfg = manifest(work)
    if len(sys.argv) != 2 or sys.argv[1] not in ('launch', 'status', 'cancel', '_owner', '_worker', '_serve'):
        raise ValueError('Fixed helper command required')
    if sys.argv[1] == '_owner': supervise(work, cfg)
    elif sys.argv[1] == '_worker': worker(work, cfg)
    elif sys.argv[1] == '_serve': completion_server(work, cfg)
    else: print(json.dumps(frontend(work, cfg, sys.argv[1])), flush=True)


if __name__ == '__main__': main()
