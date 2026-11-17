"""Camera tmpfs lease receiver and independent watchdog; no commands in packets."""
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import sys
import time

CONFIG = None
MAGIC = b'O360LSE1'
REQUEST = struct.Struct('!8s16sQB')
REPLY = struct.Struct('!8s16sQddB')


def atomic(name, value):
    path = Path(CONFIG['work']) / name
    temporary = path.with_name(path.name + '.' + str(os.getpid()) + '.next')
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(temporary, flags, 0o600)
    try:
        with os.fdopen(fd, 'w') as stream: json.dump(value, stream)
        os.replace(temporary, path)
    finally:
        try: temporary.unlink()
        except FileNotFoundError: pass


def load(name):
    path = Path(CONFIG['work']) / name
    if path.is_symlink(): raise ValueError('Lease state is a symlink')
    try: return json.loads(path.read_text())
    except FileNotFoundError: return None


def identity(pid):
    root = Path('/proc') / str(pid)
    try:
        fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        args = (root / 'cmdline').read_bytes().split(b'\0')
    except FileNotFoundError: return None
    if fields[0] == 'Z' or not any(args): return None
    return {'start': fields[19], 'args': args}


def receiver_alive():
    owner = load('lease.receiver.owner')
    if owner is None: return False
    pid = owner['pid']
    if type(pid) is not int or pid <= 1: raise ValueError('Invalid receiver PID')
    current = identity(pid)
    if not current or current['start'] != owner['start']: return False
    if str(Path(CONFIG['work']) / 'lease.py').encode() not in current['args'] or b'receiver' not in current['args']:
        raise ValueError('Receiver PID ownership mismatch')
    return True


class Lease:
    def __init__(self, nonce, setup_deadline, seconds=10):
        self.nonce = bytes.fromhex(nonce)
        if len(self.nonce) != 16: raise ValueError('Expected 16-byte session nonce')
        self.deadline = setup_deadline
        self.seconds = seconds
        self.seq = 0
        self.state = 'waiting'
        self.updated = 0.0
    def accept(self, data, source_ip, expected_ip, now):
        if self.state == 'closed' or source_ip != expected_ip or len(data) != REQUEST.size:
            return None
        magic, nonce, seq, opcode = REQUEST.unpack(data)
        if magic != MAGIC or nonce != self.nonce or seq <= self.seq or opcode not in (0, 1):
            return None
        # An expired session cannot be revived before the watchdog next polls.
        if now >= self.deadline: return None
        self.seq, self.updated = seq, now
        self.state = 'closed' if opcode == 1 else 'active'
        self.deadline = now if opcode == 1 else now + self.seconds
        return REPLY.pack(MAGIC, nonce, seq, now, self.deadline, 2 if opcode else 1)
    def snapshot(self):
        return {'nonce': self.nonce.hex(), 'state': self.state, 'seq': self.seq,
                'updated': self.updated, 'expires': self.deadline}


def expired(now, bootstrap, state, alive, closed=False):
    """Pure decision: None means keep lease; any text means restore USB now."""
    if closed: return 'explicit-close'
    if not bootstrap or bootstrap.get('nonce') != CONFIG['nonce']:
        return 'invalid-bootstrap'
    deadline = bootstrap.get('deadline')
    if type(deadline) not in (int, float) or not math.isfinite(deadline): return 'invalid-bootstrap'
    if state is None: return 'setup-expired' if now >= deadline else None
    if state.get('nonce') != CONFIG['nonce'] or state.get('state') not in ('waiting', 'active', 'closed'):
        return 'invalid-state'
    if state['state'] == 'closed': return 'explicit-close'
    if not alive: return 'receiver-dead'
    end, updated = state.get('expires'), state.get('updated')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (end, updated)):
        return 'invalid-state'
    if state['state'] == 'waiting':
        if end != deadline: return 'invalid-state'
    elif (type(state.get('seq')) is not int or state['seq'] < 1
          or end > updated + CONFIG['lease_seconds'] or updated > now + .5):
        return 'invalid-state'
    return 'lease-expired' if now >= end else None


def stop_receiver():
    # A separate close marker makes close irreversible even if the receiver
    # happened to be atomically publishing its last accepted heartbeat.
    atomic('lease.closed', {'nonce': CONFIG['nonce'], 'monotonic': time.monotonic()})
    owner = load('lease.receiver.owner')
    if not owner: return
    if not receiver_alive(): return
    pid = owner['pid']
    try: os.kill(pid, signal.SIGTERM)
    except ProcessLookupError: return
    deadline = time.monotonic() + 2
    while receiver_alive() and time.monotonic() < deadline: time.sleep(.05)
    if receiver_alive():
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass


def receiver():
    work = Path(CONFIG['work'])
    bootstrap = load('lease.bootstrap')
    if not bootstrap or bootstrap['nonce'] != CONFIG['nonce']: raise ValueError('Missing matching watchdog bootstrap')
    interface = (work / 'ifname').read_text().strip()
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface): raise ValueError('Unsafe interface')
    if (Path('/sys/class/net') / interface / 'address').read_text().strip().lower() != CONFIG['dev_mac']:
        raise ValueError('USB interface MAC changed')
    lease = Lease(CONFIG['nonce'], bootstrap['deadline'], CONFIG['lease_seconds'])
    alive = True
    def stop(*_):
        nonlocal alive
        alive = False
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, 25, interface.encode() + b'\0')
        sock.bind((CONFIG['camera_ip'], CONFIG['port']))
        sock.settimeout(.25)
        info = identity(os.getpid())
        with (work / 'lease.receiver.owner').open('x') as stream:
            json.dump({'pid': os.getpid(), 'start': info['start']}, stream)
        atomic('lease.state', lease.snapshot())
        (work / 'lease.receiver.ready').write_text('READY\n')
        while alive and not (work / 'lease.closed').exists():
            try: data, source = sock.recvfrom(1024)
            except socket.timeout: continue
            now = time.monotonic()
            if (work / 'lease.closed').exists(): break
            reply = lease.accept(data, source[0], CONFIG['host_ip'], now)
            if reply:
                atomic('lease.state', lease.snapshot())
                sock.sendto(reply, source)
                if lease.state == 'closed': break


def watchdog():
    work = Path(CONFIG['work'])
    start = time.monotonic()
    bootstrap = {'nonce': CONFIG['nonce'], 'started': start,
                 'deadline': start + CONFIG['setup_seconds']}
    with (work / 'lease.bootstrap').open('x') as stream: json.dump(bootstrap, stream)
    (work / 'watchdog.ready').write_text(str(os.getpid()) + '\n')
    while True:
        try:
            state = load('lease.state')
            reason = expired(time.monotonic(), bootstrap, state,
                             receiver_alive() if state is not None else False,
                             (work / 'lease.closed').exists())
        except Exception as error:
            reason = 'invalid-state:' + type(error).__name__
        if reason in ('lease-expired', 'setup-expired'):
            # Re-read once at the deadline to avoid acting on a stale snapshot
            # when a receiver atomically renewed between the preceding reads.
            # A last-instant race remains fail-closed: it may restore early,
            # but can never extend a dead host's lease.
            try:
                state = load('lease.state')
                reason = expired(time.monotonic(), bootstrap, state,
                                 receiver_alive() if state is not None else False,
                                 (work / 'lease.closed').exists())
            except Exception as error:
                reason = 'invalid-state:' + type(error).__name__
        if reason:
            try: atomic('lease.closed', {'nonce': CONFIG['nonce'], 'monotonic': time.monotonic()})
            except Exception: pass  # RAM failure must not skip USB restoration.
            try: atomic('lease.end', {'reason': reason, 'monotonic': time.monotonic()})
            except Exception: pass  # Diagnostic write failure must not skip rollback.
            os.execv('/bin/sh', ['/bin/sh', str(work / 'rollback.sh')])
        time.sleep(.25)


if __name__ == '__main__':
    if CONFIG is None: raise ValueError('Unconfigured lease helper')
    if sys.argv[1] == 'receiver': receiver()
    elif sys.argv[1] == 'watchdog': watchdog()
    elif sys.argv[1] == 'close': stop_receiver()
    else: raise ValueError('Unknown lease action')
