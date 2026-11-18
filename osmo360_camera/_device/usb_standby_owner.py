"""EXPERIMENTAL camera-only multi-cycle QS owner; no action on import.

Stage this file and BYTE-IDENTICAL frozen9db gui_qs_exit_forward_probe.py in a
fresh /dev/shm/osmo-usb-standby-<12hex> directory. Nothing here uploads, opens
USB, configures ECM, records media, changes a preference, requests S5, or uses
direct LPC SET. launch binds one RAM RoleShadow; it remains until an explicit
stock reboot. Stable AWAKE/SLEEP has NO90/240s recurring deadline. Each INIT,
sleep or wake has a separate detached90/240s RTC recovery process. The native
GET/QS worker cannot block that process. Host disappearance in a stable state
does not reboot the camera. Lost command replies NEVER justify a key replay.

This is source/fake-only research, NOT an SDK sleep API. Logical state does not
prove power saving or Linux suspension. Streaming health is a separate owner.
Only explicit status snapshots replace one owned <=16KiB blackbox diagnostic
export; executable code, IPC, role shadow and transition records stay in RAM.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import select
import socket
import stat
import subprocess
import sys
import time

BASE_NAME = 'gui_qs_exit_forward_probe.py'
BASE_SHA = '9db827623c2e7f9767fc8a52ba5dfc601a768fafdbc1687666b1cbe7cf26e959'
WORK_PATTERN = r'/dev/shm/osmo-usb-standby-([0-9a-f]{12})'
GUI_LOG = Path('/blackbox/gui/log/gui.log')
PROCESSING = 'SHUTDOWN: on power level callback processing, stop force shutdown timer'
DONE = 'SHUTDOWN: on power level callback done: quit shutdown page'
SHORT, RESERVE, ENTRY_SECONDS = 90.0, 240.0, 6.0
MAX_JSON, MAX_LOG = 16384, 1048576


def boot():
    return time.clock_gettime(7)


def stamp():
    return dict(boottime=boot(), monotonic=time.monotonic(), realtime=time.time())


def encoded(value):
    data = (json.dumps(value, sort_keys=True, allow_nan=False) + '\n').encode('ascii')
    if len(data) > MAX_JSON:
        raise ValueError('Bounded16KiB JSON required')
    return data


def private(path, directory=False):
    info = path.lstat()
    if (path.is_symlink() or path.resolve() != path or info.st_uid != 0
            or info.st_mode & 0o022 or not
            (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode))):
        raise ValueError('Private root-owned canonical path required')
    return info


def read(path):
    private(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    with os.fdopen(fd, 'rb') as stream:
        if os.fstat(stream.fileno()).st_size > MAX_JSON:
            raise ValueError('JSON bound')
        return json.loads(stream.read(MAX_JSON + 1))


def new(path, value):
    data = encoded(value)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)


def replace(path, value):
    if os.path.lexists(path):
        private(path)
    temporary = path.with_name(path.name + '.next-' + os.urandom(4).hex())
    new(temporary, value)
    os.replace(temporary, path)


def base_module():
    """Import definitions only, after digest verification; never call9db main."""
    path = Path(__file__).with_name(BASE_NAME)
    private(path)
    if hashlib.sha256(path.read_bytes()).hexdigest() != BASE_SHA:
        raise ValueError('Byte-identical frozen9db dependency required')
    spec = importlib.util.spec_from_file_location('frozen_qs_9db', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def environment(base):
    script = Path(__file__)
    work = script.parent
    if (sys.platform != 'linux' or os.geteuid() != 0 or script.name != 'usb_standby_owner.py'
            or not re.fullmatch(WORK_PATTERN, str(work))):
        raise ValueError('Exact camera root RAM path required')
    private(work, True)
    private(script)
    rows = [r for r in base.mount_rows() if str(work) == r['point']
            or str(work).startswith(r['point'].rstrip('/') + '/')]
    if not rows or max(rows, key=lambda row: len(row['point']))['fs'] != 'tmpfs':
        raise ValueError('Canonical tmpfs required')
    return work


def configuration(work):
    cfg = read(work / 'config.json')
    if (cfg.get('nonce') != work.name[-12:] or cfg.get('boot_id') !=
            Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
        raise ValueError('Same boot and owned nonce required')
    return cfg


def command_number(value):
    if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{16}', value):
        raise ValueError('Command ID is the canonical16-digit increasing hexadecimal sequence')
    number = int(value, 16)
    if number == 0:
        raise ValueError('Command IDs start at1')
    return number


class Machine:
    """Pure decision state: a retired command number can never press a key again."""
    def __init__(self, owner, nonce):
        self.value = dict(owner=owner, nonce=nonce, state='INITIALIZING', last_sequence=0,
            retained_shadow=True, native_restored=False, periodic_reboot=False,
            physical_sleep_proven=False, power_saving_proven=False)

    def accept(self, request):
        number = command_number(request['command_id'])
        if (request.get('owner'), request.get('nonce')) != (self.value['owner'], self.value['nonce']):
            raise ValueError('Command owner/nonce mismatch')
        if number <= self.value['last_sequence']:
            return False
        if number != self.value['last_sequence'] + 1:
            raise ValueError('No skipped command sequence')
        action = request.get('action')
        expected = {'sleep': 'AWAKE', 'wake': 'SLEEP'}.get(action)
        if expected is None or self.value['state'] != expected:
            raise ValueError('Command does not match stable state')
        if action == 'sleep' and any(request.get(key) is not True for key in
                ('record0', 'battery_present1', 'activation_state1', 'streams_closed', 'own_app_absent')):
            raise ValueError('Fresh host idle/activation/producer teardown guards required')
        self.value.update(last_sequence=number, command_id=request['command_id'],
                          action=action, state='SLEEPING' if action == 'sleep' else 'WAKING')
        return True

    def complete(self, result):
        if (result.get('nonce'), result.get('owner'), result.get('command_id'), result.get('action')) != (
                self.value['nonce'], self.value['owner'], self.value['command_id'], self.value['action']):
            raise ValueError('Terminal receipt correlation mismatch')
        if result.get('complete') is not True:
            raise ValueError('Terminal result not proven')
        self.value.update(state='SLEEP' if result['action'] == 'sleep' else 'AWAKE',
                          last_result=result)


class GuiCursor:
    """Read only bytes appended after pinning this existing stock log inode."""
    def __init__(self, path=GUI_LOG):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        info = os.fstat(self.fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0:
            self.close()
            raise ValueError('Existing root-owned GUI log required')
        self.inode, self.offset = (info.st_dev, info.st_ino), info.st_size
        self.bytes, self.partial, self.found = 0, b'', []

    @staticmethod
    def event_time(line, year):
        match = re.match(r'(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d\d\d)\s', line)
        if not match:
            return None
        month, day, hour, minute, second, millis = map(int, match.groups())
        return time.mktime((year, month, day, hour, minute, second, 0, 0, -1)) + millis / 1000

    def poll(self, after, now=None):
        info, opened = self.path.stat(), os.fstat(self.fd)
        if ((info.st_dev, info.st_ino) != self.inode or (opened.st_dev, opened.st_ino) != self.inode
                or opened.st_size < self.offset):
            raise ValueError('GUI log rotated/replaced/truncated during transition')
        count = opened.st_size - self.offset
        if self.bytes + count > MAX_LOG:
            raise ValueError('GUI appended read exceeds1MiB')
        data = os.pread(self.fd, count, self.offset)
        # A writer may append between this poll's metadata reads and pread.
        # Timestamp the completed read, so valid newly read events are not
        # permanently discarded against an upper bound sampled too early.
        now = time.time() if now is None else now
        self.offset += len(data)
        self.bytes += len(data)
        lines = (self.partial + data).split(b'\n')
        self.partial = lines.pop()
        if len(self.partial) > 8192:
            raise ValueError('Unbounded GUI log line')
        for raw in lines:
            line = raw.decode('utf8', 'replace')
            kind = 'processing3' if PROCESSING in line else 'done3' if DONE in line else None
            if kind is None or len(self.found) == 2:
                continue
            at = self.event_time(line, time.localtime(after).tm_year)
            # Logcat has millisecond precision. Reject old/stale events even
            # when a writer flushed them after our initial EOF snapshot.
            if at is None or at + .001 < after or at > now + .001:
                continue
            expected = 'processing3' if not self.found else 'done3'
            if kind == expected:
                self.found.append(dict(kind=kind, realtime=at, line=line[:512]))
        return len(self.found) == 2

    def proof(self):
        return dict(inode=list(self.inode), appended_bytes=self.bytes,
                    processing3_after_intent=len(self.found) == 2,
                    done3_after_intent=len(self.found) == 2, events=list(self.found))

    def close(self):
        if getattr(self, 'fd', None) is not None:
            os.close(self.fd)
            self.fd = None


def ready(base, cfg, *, awake):
    value = base.readiness()
    base.require_ready(value, awake=awake)
    if value['service'] != cfg['sys_service']:
        raise ValueError('Original dji_sys process changed')
    return value


def correlated(value, cfg, cmd):
    return (value.get('nonce'), value.get('owner'), value.get('command_id'), value.get('action')) == (
        cfg['nonce'], cfg['owner'], cmd['command_id'], cmd['action'])


def validate_result(value, cfg, cmd):
    if not correlated(value, cfg, cmd) or value.get('complete') is not True or value.get('lpc_set_calls') != 0:
        raise ValueError('Correlated completed GET/QS-only result required')
    if value.get('service') != cfg['sys_service']:
        raise ValueError('Terminal system service changed')
    if cmd['action'] == 'init':
        if value.get('shadow_verified') is not True or value.get('get_level') != 0:
            raise ValueError('Bound/readiness initialization proof required')
        return
    pulse = value.get('pulse', {})
    hold = 1.1 if cmd['action'] == 'sleep' else .1
    if (pulse.get('key'), pulse.get('hold_seconds'), pulse.get('press_release_complete')) != (357, hold, True):
        raise ValueError('Exact complete QS pulse required')
    if cmd['action'] == 'sleep':
        gui = value.get('gui', {})
        if (value.get('get_level') not in (2, 3) or value.get('shadow_value') != 'none'
                or gui.get('processing3_after_intent') is not True or gui.get('done3_after_intent') is not True):
            raise ValueError('Logical S3 plus fresh GUI timer-cancellation proof required')
    else:
        exited = value.get('real_exit', {})
        if (value.get('get_level') != 0 or exited.get('completed') is not True
                or exited.get('bytes_written') != 5 or exited.get('literal') != 'exit'
                or exited.get('real_role_after') != 'peripheral'):
            raise ValueError('Matching GET0 and real EXIT/peripheral proof required')


def fresh(base, work, cfg, folder, cmd, margin=0):
    armed = read(folder / 'armed.json')
    if (not correlated(armed, cfg, cmd) or not base.alive(cfg['owner'])
            or not base.alive(armed['guard']) or (work / 'reboot.json').exists()
            or boot() + margin >= armed['deadline_boottime']):
        raise ValueError('Live independent transition guard required before input')
    return armed


def transition(base, work, cfg, folder, cmd, client, key, real_fd):
    """Blocking native operations run only here, never in the RTC guardian."""
    action = cmd['action']
    before_ready = ready(base, cfg, awake=action == 'sleep')
    base.battery()
    shadow = base.RoleShadow(work).proof()
    before = client.get()
    required = (0,) if action == 'sleep' else (2, 3)
    if before.get('returncode') != 0 or before.get('level') not in required:
        raise ValueError('Native level does not match requested transition')
    fresh(base, work, cfg, folder, cmd, 2)
    try:
        intent_at = stamp()
        new(folder / 'key-intent.json', dict(cmd, at=intent_at, lpc_set_calls=0))
        pulse = key.pulse(1.1 if action == 'sleep' else .1,
            lambda: fresh(base, work, cfg, folder, cmd, 2))
        deadline = time.monotonic() + (ENTRY_SECONDS if action == 'sleep' else 3)
        gui_done = False
        while True:
            fresh(base, work, cfg, folder, cmd)
            after = client.get()
            fresh(base, work, cfg, folder, cmd)
            if action == 'sleep' and (folder / 'gui-independent.json').exists():
                gui = read(folder / 'gui-independent.json')
                if not correlated(gui, cfg, cmd):
                    raise ValueError('Independent GUI proof correlation')
                gui_done = gui.get('processing3_after_intent') is True and gui.get('done3_after_intent') is True
            level_ok = after.get('returncode') == 0 and after.get('level') in ((2, 3) if action == 'sleep' else (0,))
            shadow = base.RoleShadow(work).proof()
            if level_ok and (gui_done and shadow['value'] == 'none' if action == 'sleep' else shadow['value'] == 'exit'):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Native transition/log completion deadline')
            time.sleep(.05)
        result = dict(cmd, complete=True, at=stamp(), pulse=pulse, get_level=after['level'],
            service=ready(base, cfg, awake=action == 'wake')['service'],
            shadow_value=shadow['value'], lpc_set_calls=0)
        if action == 'sleep':
            result['gui'] = gui
        else:
            # Keep9db's RPC-release grace in this prototype. The independent
            # prepared-plan experiment separately measures removal of grace.
            until = time.monotonic() + base.FORWARD_DELAY
            while time.monotonic() < until:
                fresh(base, work, cfg, folder, cmd)
                time.sleep(.02)
            copied = os.dup(real_fd)
            os.set_inheritable(copied, False)
            role = base.RealRole.adopt(copied, read(work / 'real-role.json')['proof'])
            try:
                new(folder / 'real-exit-intent.json', dict(cmd, at=stamp(), literal='exit'))
                result['real_exit'] = role.forward_exit_once(
                    lambda: fresh(base, work, cfg, folder, cmd, .1))
            finally:
                role.close()
        validate_result(result, cfg, cmd)
        return result
    finally:
        pass  # Native GET/QS handles stay in the long-lived isolated worker.


def entry_log_observer(base, work, identifier):
    """Independent read-only log child; a blocked native GET cannot hide GUI12s."""
    cfg = configuration(work)
    folder = work / ('cmd-' + identifier)
    cmd = read(folder / 'request.json')
    cursor = None
    try:
        cursor = GuiCursor()  # Pin EOF/inode BEFORE armed.json permits any key.
        new(folder / 'gui-log-ready.json', dict(cmd, process=base.proc_token(), inode=list(cursor.inode)))
        while not (folder / 'key-intent.json').exists():
            if not base.alive(cfg['owner']) or (work / 'reboot.json').exists():
                return
            time.sleep(.05)
        intent = read(folder / 'key-intent.json')
        if not correlated(intent, cfg, cmd):
            raise ValueError('GUI key-intent correlation')
        while True:
            if boot() - intent['at']['boottime'] >= ENTRY_SECONDS:
                raise TimeoutError('Fresh GUI Processing3/Done3 deadline before12s fallback')
            if cursor.poll(intent['at']['realtime']):
                new(folder / 'gui-independent.json', dict(cmd, **cursor.proof(), at=stamp()))
                return
            time.sleep(.05)
    except BaseException as error:
        new(folder / 'gui-log-error.json', dict(cmd, error=str(error)[:240], at=stamp()))
        raise
    finally:
        if cursor is not None:
            cursor.close()


def guard_shadow(base, work):
    """Only owned RAM reads/stat; never read through a possibly lost bind."""
    source = work / 'role.txt'
    private(source)
    if base.identity(base.TARGET) != base.identity(source):
        raise ValueError('Role shadow inode no longer visible to native writer')
    data = source.read_bytes()
    if len(data) > 64 or data not in (b'peripheral\n', b'none\n', b'exit\n'):
        raise ValueError('Unexpected owned RAM role contents')
    return data.decode('ascii').strip()


def listener(path):
    if os.path.lexists(path):
        raise ValueError('Fresh owned IPC socket required')
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    os.chmod(path, 0o600)
    server.listen(1)
    return server


def notify(path, data=b'!'):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(.5)
        sock.connect(str(path))
        sock.sendall(data)


def receive(server):
    with server.accept()[0] as connection:
        connection.settimeout(.5)
        return connection.recv(17)


def worker(base, work, real_fd):
    cfg = configuration(work)
    client = key = None
    try:
        client, key = base.PowerClient(), base.QSKey()
        base.require_get0(client.get())
        server = listener(work / 'worker.sock')
        new(work / 'worker-ready.json', dict(owner=cfg['owner'], nonce=cfg['nonce'], process=base.proc_token()))
        seen = 0
        while True:
            # No polling or RTC wakeup in stable standby. The coordinator
            # supplies an ID only after a separate guardian is armed.
            identifier = receive(server).decode('ascii')
            if not re.fullmatch('[0-9a-f]{16}', identifier):
                raise ValueError('Fixed IPC command ID required')
            if not base.alive(cfg['owner']) or (work / 'reboot.json').exists():
                return
            path = work / 'job.json'
            if path.exists():
                cmd = read(path)
                if cmd['command_id'] != identifier:
                    raise ValueError('IPC/file command mismatch')
                sequence = command_number(cmd['command_id'])
                if sequence > seen:
                    # Retire the number BEFORE any native read/input. No IPC
                    # duplicate or exception can cause this worker to retry it.
                    seen = sequence
                    folder = work / ('cmd-' + cmd['command_id'])
                    try:
                        value = transition(base, work, cfg, folder, cmd, client, key, real_fd)
                    except BaseException as error:
                        value = dict(cmd, complete=False, error=str(error)[:240], at=stamp())
                    new(folder / 'result.json', value)
    finally:
        if key is not None:
            key.close()
        if client is not None:
            client.close()


def recovery(base, work, cfg, folder, reason, hard, *, hard_fired=False):
    """Reboot is spawned before any potentially blocking join/native destroy."""
    request = base.stock_reboot()
    try:
        replace(work / 'reboot.json', dict(nonce=cfg['nonce'], owner=cfg['owner'], reason=reason,
                at=stamp(), request=request, retained_shadow=True))
    except BaseException:
        pass
    if hard is not None and hard.fd is not None and not hard_fired:
        while True:
            readable, _, _ = select.select([hard.fd], [], [], .2)
            if readable:
                hard.fired()
                break
    base.stock_reboot()
    # Actual reboot kills us. A failed reserve request must not pretend cleanup
    # succeeded, unmount, press a key, or start recurring reboot timers.
    while True:
        time.sleep(60)


def guardian(base, work, identifier):
    cfg = configuration(work)
    folder = work / ('cmd-' + identifier)
    cmd = read(folder / 'request.json')
    short = hard = log_child = None
    hard_fired = False
    try:
        short, hard = base.Alarm(SHORT), base.Alarm(RESERVE)
        sampled_at = boot()
        short_proof = short.verify(SHORT)
        hard_proof = hard.verify(RESERVE)
        # Sample BEFORE gettime: this deadline is conservatively no later than
        # actual expiry, even when log-reader preparation takes several seconds.
        short_deadline = sampled_at + short_proof['remaining']

        def check_alarms():
            nonlocal hard_fired
            readable, _, _ = select.select([alarm.fd for alarm in (short, hard)
                                           if alarm.fd is not None], [], [], 0)
            if readable:
                for alarm in (short, hard):
                    if alarm.fd in readable:
                        alarm.fired()
                hard_fired = hard.fd in readable
                raise TimeoutError('Independent transition RTC already readable')
        if cmd['action'] == 'sleep':
            log_child = spawn(work, '_entrylog', '--command-id', identifier)
            deadline = time.monotonic() + 4
            while not (folder / 'gui-log-ready.json').exists():
                if log_child.poll() is not None or time.monotonic() >= deadline:
                    raise ValueError('Independent GUI reader not pinned before key')
                readable, _, _ = select.select([short.fd, hard.fd], [], [], .05)
                if readable:
                    raise TimeoutError('RTC expired before GUI observer ready')
            pinned = read(folder / 'gui-log-ready.json')
            if not correlated(pinned, cfg, cmd) or not base.alive(pinned.get('process')):
                raise ValueError('Independent GUI observer owner mismatch')
        check_alarms()
        if boot() >= short_deadline:
            raise TimeoutError('RTC deadline expired during observer preparation')
        armed = dict(cmd, guard=base.proc_token(), deadline_boottime=short_deadline,
            alarm90=short_proof, alarm240=hard_proof)
        new(folder / 'armed.json', armed)
        while True:
            check_alarms()
            if boot() >= armed['deadline_boottime']:
                raise TimeoutError('Transition deadline passed before cancellation')
            if cmd['action'] == 'sleep' and (folder / 'key-intent.json').exists():
                intent = read(folder / 'key-intent.json')
                if not correlated(intent, cfg, cmd):
                    raise ValueError('Key-intent owner mismatch')
                if (folder / 'gui-log-error.json').exists():
                    raise ValueError('Independent GUI reader failed')
                if boot() - intent['at']['boottime'] >= ENTRY_SECONDS and not (folder / 'gui-independent.json').exists():
                    raise TimeoutError('Independent6s GUI deadline before stock12s full-off fallback')
            if (folder / 'cancel.json').exists():
                cancel = read(folder / 'cancel.json')
                result = read(folder / 'result.json')
                if not correlated(cancel, cfg, cmd) or not base.alive(cfg['owner']):
                    raise ValueError('Guard cancellation owner correlation')
                validate_result(result, cfg, cmd)
                role_value = guard_shadow(base, work)
                if cmd['action'] != 'init' and role_value != result['shadow_value']:
                    raise ValueError('Role shadow changed before guard cancellation')
                if not base.alive(cfg['sys_service']):
                    raise ValueError('Original dji_sys process exited')
                if log_child is not None and log_child.poll() is None:
                    # Do not block a safety process joining a reader. Its
                    # ordinary return follows the already-published proof.
                    time.sleep(.02)
                    continue
                if boot() >= armed['deadline_boottime']:
                    raise TimeoutError('Late cancellation after terminal validation')
                check_alarms()
                closed = dict(alarm90=short.cancel())
                if boot() >= armed['deadline_boottime']:
                    raise TimeoutError('Late cancellation before reserve close')
                closed['alarm240'] = hard.cancel()
                new(folder / 'guard-finished.json', dict(cmd, cancelled=True, closed=closed, at=stamp()))
                return
            if not base.alive(cfg['owner']) or (folder / 'error.json').exists() or (work / 'finalize.json').exists():
                raise ValueError('Owner death/error/finalize with transition guard')
            readable, _, _ = select.select([short.fd, hard.fd], [], [], .1)
            if readable:
                for alarm in (short, hard):
                    if alarm.fd in readable:
                        alarm.fired()
                hard_fired = hard.fd in readable
                raise TimeoutError('Independent transition RTC expired')
    except BaseException as error:
        recovery(base, work, cfg, folder, str(error)[:240], hard, hard_fired=hard_fired)
    finally:
        for alarm in (short, hard):
            if alarm is not None:
                alarm.close()


def spawn(work, mode, *args, pass_fds=()):
    return subprocess.Popen([sys.executable, __file__, mode, *args], stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        close_fds=True, pass_fds=pass_fds)


def wait(base, path, seconds, process=None):
    deadline = time.monotonic() + seconds
    while not path.exists():
        if process is not None and process.poll() is not None:
            raise ValueError('Owned child exited before ready')
        if time.monotonic() >= deadline:
            raise TimeoutError('Waiting for ' + path.name)
        time.sleep(.02)
    return read(path)


def arm(base, work, cfg, cmd):
    folder = work / ('cmd-' + cmd['command_id'])
    folder.mkdir(mode=0o700)
    new(folder / 'request.json', cmd)
    child = spawn(work, '_guard', '--command-id', cmd['command_id'])
    value = wait(base, folder / 'armed.json', 4, child)
    if not correlated(value, cfg, cmd) or not base.alive(value['guard']):
        raise ValueError('Detached RTC guard not armed')
    return folder, child


def cancel_guard(base, work, cfg, folder, cmd):
    value = read(folder / 'result.json')
    validate_result(value, cfg, cmd)
    new(folder / 'cancel.json', cmd)
    done = wait(base, folder / 'guard-finished.json', 2)
    if not correlated(done, cfg, cmd) or done.get('cancelled') is not True:
        raise ValueError('Independent guard did not confirm cancellation')
    return done


def publish(work, machine, base, cfg):
    value = dict(machine.value, at=stamp(), owner_alive=True,
                 shadow=base.RoleShadow(work).proof(), service=cfg['sys_service'])
    replace(work / 'status.json', value)
    return value


def export_status(base, work, cfg, value):
    """One explicit diagnostic file, bounded size and tracked current inode.

    Reuse9db's exclusive create/ownership checks, but not its16-update finite
    experiment cap. This writer is serialized by the frontend's status.lock.
    The coordinator never writes the export, so it cannot race this marker.
    """
    output = base.Export(work, cfg['nonce'])
    if not output.marker.exists():
        output.create(value)
    else:
        previous = output.owned()
        temporary = output.path.with_name(output.path.name + '.next-' + cfg['nonce'])
        inode = base.write_new(temporary, value)
        try:
            base.write_ram(output.marker, dict(nonce=cfg['nonce'],
                inodes=previous['inodes'] + [inode], updates=previous['updates'] + 1))
            output.owned()
            os.replace(temporary, output.path)
            base.write_ram(output.marker, dict(nonce=cfg['nonce'], inodes=[inode], updates=previous['updates'] + 1))
        finally:
            if temporary.exists():
                temporary.unlink()
    return str(output.path)


def request_recovery(work, cfg):
    path = work / 'recovery-intent.json'
    if path.exists():
        value = read(path)
        if value.get('nonce') != cfg['nonce']:
            raise ValueError('Foreign recovery intent')
        return dict(recovery_already_requested=True)
    new(path, dict(nonce=cfg['nonce'], at=stamp()))
    process = spawn(work, '_recovery')
    return dict(recovery_requested=True, process=process.pid)


def recovery_owner(base, work):
    """Fresh independent recovery for a failure/finalize in a stable state."""
    cfg = configuration(work)
    hard = base.Alarm(RESERVE)
    new(work / 'recovery-ready.json', dict(owner=base.proc_token(), alarm240=hard.verify(RESERVE)))
    recovery(base, work, cfg, None, 'Explicit finalize or stable owner failure', hard)


def prune(work, last):
    """Keep only two terminal command directories; sequence state prevents replay."""
    for path in work.glob('cmd-*'):
        name = path.name[4:]
        if not re.fullmatch('[0-9a-f]{16}', name) or int(name, 16) >= max(0, last - 1):
            continue
        private(path, True)
        if not (path / 'guard-finished.json').exists():
            raise ValueError('Cannot prune a nonterminal guard')
        for child in path.iterdir():
            private(child)
            if child.name not in {'request.json', 'armed.json', 'key-intent.json', 'result.json',
                    'cancel.json', 'guard-finished.json', 'real-exit-intent.json',
                    'gui-log-ready.json', 'gui-independent.json'}:
                raise ValueError('Unknown command file; refuse cleanup')
        for child in path.iterdir():
            child.unlink()
        path.rmdir()


def owner(base, work):
    cfg = configuration(work)
    cfg['owner'] = base.proc_token()
    replace(work / 'config.json', cfg)
    new(work / 'owner.json', cfg['owner'])
    machine = Machine(cfg['owner'], cfg['nonce'])
    cmd = dict(owner=cfg['owner'], nonce=cfg['nonce'], command_id='0000000000000000', action='init')
    folder = None
    real = None
    try:
        server = listener(work / 'control.sock')
        folder, guard = arm(base, work, cfg, cmd)
        real = base.RealRole()
        new(work / 'real-role.json', dict(owner=cfg['owner'], nonce=cfg['nonce'], proof=real.check()))
        child = spawn(work, '_worker', '--real-fd', str(real.fd), pass_fds=(real.fd,))
        worker_ready = wait(base, work / 'worker-ready.json', 4, child)
        if worker_ready.get('owner') != cfg['owner'] or not base.alive(worker_ready.get('process')):
            raise ValueError('Native worker not ready')
        new(work / 'bind-intent.json', dict(cmd, at=stamp()))
        shadow = base.RoleShadow(work).bind()
        value = ready(base, cfg, awake=True)
        new(folder / 'result.json', dict(cmd, complete=True, shadow_verified=True,
            get_level=value['system_power_level'], service=value['service'], lpc_set_calls=0))
        cancel_guard(base, work, cfg, folder, cmd)
        machine.value['state'] = 'AWAKE'
        publish(work, machine, base, cfg)
        new(work / 'owner-ready.json', dict(nonce=cfg['nonce'], owner=cfg['owner'], state='AWAKE', retained_shadow=True))
        while True:
            # Stable states have no active RTC and no20Hz proc/log reads.
            # A host request wakes this local IPC; the30s timeout is not a
            # camera-power deadline and cannot cause an ordinary reboot.
            readable, _, _ = select.select([server], [], [], 30)
            if readable:
                receive(server)
            if child.poll() is not None or (work / 'finalize.json').exists():
                raise ValueError('Worker exited or explicit finalize: reboot-only cleanup')
            if (work / 'status-request.json').exists():
                request = read(work / 'status-request.json')
                if request.get('owner') != cfg['owner'] or request.get('nonce') != cfg['nonce']:
                    raise ValueError('Status request owner mismatch')
                machine.value['requested_readiness'] = ready(base, cfg, awake=False)
                value = publish(work, machine, base, cfg)
                replace(work / 'status-ack.json', dict(owner=cfg['owner'], nonce=cfg['nonce'],
                    request=request['request'], status=value))
                (work / 'status-request.json').unlink()
            inbox = work / 'request.json'
            if inbox.exists():
                request = read(inbox)
                if not machine.accept(request):
                    inbox.unlink()
                    continue
                prune(work, machine.value['last_sequence'])
                publish(work, machine, base, cfg)
                folder, guard = arm(base, work, cfg, request)
                inbox.unlink()
                replace(work / 'job.json', request)
                notify(work / 'worker.sock', request['command_id'].encode('ascii'))
                while not (folder / 'result.json').exists():
                    if guard.poll() is not None or child.poll() is not None or (work / 'reboot.json').exists():
                        raise ValueError('Worker/guard failure during transition')
                    if (work / 'finalize.json').exists():
                        raise ValueError('Explicit finalize during transition')
                    time.sleep(.02)
                terminal = read(folder / 'result.json')
                validate_result(terminal, cfg, request)
                cancel_guard(base, work, cfg, folder, request)
                machine.complete(terminal)
                publish(work, machine, base, cfg)
    except BaseException as error:
        machine.value.update(state='RECOVERY_PENDING', error=str(error)[:240], retained_shadow=True)
        try:
            replace(work / 'status.json', dict(machine.value, at=stamp()))
            if folder is not None:
                replace(folder / 'error.json', dict(error=str(error)[:240]))
        finally:
            # Also covers a stable-state failure after the last guard closed.
            # Never remove the bind or run a native destructor first.
            # A stable state has no old timer to protect this request. Spawn a
            # NEW detached RTC recovery owner before requesting ordinary reboot.
            # Active transition guardians independently see error.json too.
            request_recovery(work, cfg)
        while True:
            time.sleep(60)


def launch(base, work, args):
    if not all((args.record0, args.battery_present1, args.activation_state1)):
        raise ValueError('Fresh explicit host Record0/battery1/activation1 required')
    base.battery()
    value = base.readiness()
    base.require_ready(value)
    if any(row['point'] == str(base.TARGET) for row in base.mount_rows()):
        raise ValueError('No existing role shadow may be adopted by launch')
    cfg = dict(nonce=work.name[-12:], boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        sys_service=value['service'], at=stamp())
    new(work / 'config.json', cfg)
    proc = spawn(work, '_owner')
    result = wait(base, work / 'owner-ready.json', 8, proc)
    if result.get('nonce') != cfg['nonce'] or not base.alive(result.get('owner')) or result['owner']['pid'] != proc.pid:
        raise ValueError('Detached owner launch uncertain; never retry')
    return result


def submit(base, work, args):
    cfg = configuration(work)
    status = read(work / 'status.json')
    number = command_number(args.command_id)
    if status.get('owner') != cfg['owner'] or not base.alive(cfg['owner']):
        raise ValueError('No live same-boot owner; explicit reboot recovery required')
    if number <= status['last_sequence']:
        return dict(command_id=args.command_id, already_known=True, input_replayed=False,
                    receipt=status.get('last_result') if status.get('command_id') == args.command_id else None)
    request = dict(owner=cfg['owner'], nonce=cfg['nonce'], command_id=args.command_id, action=args.mode,
        record0=args.record0, battery_present1=args.battery_present1, activation_state1=args.activation_state1,
        streams_closed=args.streams_closed, own_app_absent=args.own_app_absent)
    # Validate without changing real owner state. A single inbox prevents
    # concurrent different commands from becoming another native input.
    check = Machine(cfg['owner'], cfg['nonce'])
    check.value.update(status)
    check.accept(request)
    new(work / 'request.json', request)
    notify(work / 'control.sock')
    return dict(command_id=args.command_id, submitted=True, input_completion_proven=False,
                retained_shadow=True, query_same_command_after_uncertain_reply=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('launch', 'sleep', 'wake', 'status', 'finalize', '_owner', '_worker', '_guard', '_recovery', '_entrylog'))
    parser.add_argument('--command-id')
    parser.add_argument('--real-fd', type=int)
    for option in ('record0', 'battery-present1', 'activation-state1', 'streams-closed', 'own-app-absent'):
        parser.add_argument('--' + option, action='store_true')
    args = parser.parse_args()
    base = base_module()
    work = environment(base)
    if args.mode == '_owner':
        return owner(base, work)
    if args.mode == '_worker':
        return worker(base, work, args.real_fd)
    if args.mode == '_guard':
        return guardian(base, work, args.command_id)
    if args.mode == '_entrylog':
        return entry_log_observer(base, work, args.command_id)
    if args.mode == '_recovery':
        return recovery_owner(base, work)
    if args.mode == 'launch':
        value = launch(base, work, args)
    elif args.mode in ('sleep', 'wake'):
        value = submit(base, work, args)
    elif args.mode == 'finalize':
        cfg = configuration(work)
        path = work / 'finalize.json'
        if not path.exists():
            new(path, dict(owner=cfg['owner'], nonce=cfg['nonce'], at=stamp()))
        if base.alive(cfg['owner']):
            notify(work / 'control.sock')
        else:
            base.RoleShadow(work).proof()
            request_recovery(work, cfg)
        value = dict(requested='stock reboot', retained_shadow=True, native_restored=False)
    else:
        cfg = configuration(work)
        lock = work / 'status.lock'
        lock.mkdir(mode=0o700)
        try:
            if base.alive(cfg['owner']):
                pending = work / 'status-request.json'
                if pending.exists():
                    prior = read(pending)
                    if (prior.get('nonce'), prior.get('owner')) != (cfg['nonce'], cfg['owner']):
                        raise ValueError('Foreign pending status request')
                    request = prior['request']
                else:
                    request = os.urandom(6).hex()
                    new(pending, dict(owner=cfg['owner'], nonce=cfg['nonce'], request=request))
                notify(work / 'control.sock')
                deadline = time.monotonic() + 2
                while True:
                    if (work / 'status-ack.json').exists():
                        ack = read(work / 'status-ack.json')
                        if ack.get('request') == request and ack.get('owner') == cfg['owner']:
                            value = ack['status']
                            break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Fresh status acknowledgement missing; no input retry')
                    time.sleep(.02)
            else:
                value = read(work / 'status.json')
            value.update(owner_alive=base.alive(cfg['owner']), retained_shadow=True, native_restored=False)
            path = export_status(base, work, cfg, value)
            value = dict(snapshot_exported=True, nonce=cfg['nonce'], export=path,
                         retained_shadow=True, native_restored=False, owner_alive=value['owner_alive'])
        finally:
            lock.rmdir()
    sys.stdout.buffer.write(encoded(value))


if __name__ == '__main__':
    main()
