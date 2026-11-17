"""Finite CAMERA-ONLY stock QS standby experiment; no action on import.

Fresh /dev/shm/osmo-gui-qs-exit-<12hex>/gui_qs_exit_forward_probe.py only.
launch --record0 --battery-present1 --activation-state1 arms the preserved
90s/240s RTC/reboot owner and shared-namespace RAM role shadow. After returning,
a worker delays2s and sends exactly one gpio-keys357 hold1.1s with release in
finally. That follows stock GUI entry, never direct LPC SET3/0 or shutter355.
Root must have just verified activation GET00/32 raw1, Record0 and battery1.
The caller owns/preserves any active ECM preview lease; this helper never
reconfigures its configfs/functions, network or preview lifecycle.

wake sends one gpio-keys357 short0.1s then reads native GET0 and manager readiness.
It cancels90s only with matching entry/GET0 evidence;240s backup remains. Logical
levels/key delivery do NOT establish physical AP sleep, power saving, healthy
video or IMU. GUI stock error/force-shutdown fallbacks still exist; entry proof
failure requests stock reboot promptly. No preference or sleep timer is set.
NO UNMOUNT after shadow bind: finalize and error recovery use stock reboot.
Separate candidate: default diagnostics do NOT forward any kernel role write.
Only launch --forward-native-exit permits the owner, after one verified stock
QS wake/GET0 and an observed exact shadow exit, to write exit\n ONCE through
a real sysfs FD pinned before bind. No none/peripheral/host forwarding or retry.
The original90/240s recovery and no-unmount policy are retained.
Modes: launch, wake, status, finalize, cleanup (awake prelaunch only).
"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import stat
import struct
import subprocess
import sys
import time
from collections import deque

TARGET = Path('/sys/devices/platform/soc/a600000.ssusb/mode')
SYS_SHA256 = 'f2fc2a40b8c10b0d1a8289089bb0ab6700372fa062404f44c3dcac00ad513029'
WORK_PATTERN = r'/dev/shm/osmo-gui-qs-exit-([0-9a-f]{12})'
MAX_JSON = 16384
DIAGNOSTIC_HISTORY = 12
DELAY = 2.0
BACKUP = 90.0
HARD_LIMIT = 240.0
FORWARD_DELAY = 2.0
FORWARD_RESULT_TIMEOUT = 2.0
API_TIMEOUT_MS = 3000
KERNEL_MODULE = Path('/lib/modules/5.15.148-ab639/dwc3-msm.ko')
KERNEL_MODULE_SHA256 = 'e337497c0adcc5986f1c64c24aedb1700f5dbdcaf2ecde756484b7d23dfc9a2c'
DRIVER = Path('/sys/bus/platform/drivers/msm-dwc3')
SYSFS_MAGIC = 0x62656572


def stamp():
    return dict(boottime=time.clock_gettime(7), monotonic=time.monotonic())


def failure(exc):
    return type(exc).__name__ + ': ' + str(exc)


def encoded(value):
    data = (json.dumps(value, sort_keys=True) + '\n').encode('ascii')
    if len(data) > MAX_JSON:
        raise ValueError('JSON exceeds16KiB')
    return data


def diagnostic_report(value):
    """Bound status/export duplication; never change the separate RAM proofs."""
    result = dict(value)
    if isinstance(value.get('samples'), list):
        samples = value['samples']
        result['samples'] = samples[-DIAGNOSTIC_HISTORY:]
        omitted = len(samples) - len(result['samples'])
        if omitted:
            result['sample_history_omitted'] = value.get('sample_history_omitted', 0) + omitted
    for name in ('observed', 'last_requested_snapshot'):
        snapshot_value = value.get(name)
        if not isinstance(snapshot_value, dict):
            continue
        intent = snapshot_value.get('real-exit-intent')
        if (isinstance(intent, dict) and isinstance(value.get('real_role'), dict)
                and intent.get('proof') == value['real_role']):
            projected = dict(snapshot_value)
            projected['real-exit-intent'] = {key: item for key, item in intent.items() if key != 'proof'}
            projected['real-exit-intent']['proof_ref'] = 'real_role'
            result[name] = projected
    return result


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if info.st_uid != 0 or not stat.S_ISREG(info.st_mode) or info.st_size > MAX_JSON:
            raise ValueError('unexpected owned JSON file')
        return json.load(stream)


def write_new(path, value):
    data = encoded(value)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)
        stream.flush()
    return list(identity(path))


def write_ram(path, value):
    """Private RAM files only; refuse a foreign/nonregular existing target."""
    if os.path.lexists(path):
        read_json(path)
    temporary = path.with_name(path.name + '.next-' + os.urandom(4).hex())
    write_new(temporary, value)
    try:
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def identity(path):
    info = Path(path).stat()
    return info.st_dev, info.st_ino


def proc_token(pid=None):
    pid = os.getpid() if pid is None else int(pid)
    data = (Path('/proc') / str(pid) / 'stat').read_text().split(') ', 1)[1].split()
    return dict(pid=pid, start=data[19])


def alive(token):
    try:
        return proc_token(token['pid']) == token
    except (OSError, ValueError, KeyError):
        return False


def mount_rows():
    rows = []
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        left, sep, right = line.partition(' - ')
        fields, tail = left.split(), right.split()
        if sep and len(fields) >= 6 and len(tail) >= 3:
            decode = lambda x: re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), x)
            rows.append(dict(root=decode(fields[3]), point=decode(fields[4]), fs=tail[0]))
    return rows


def environment(work_text=None):
    script = Path(__file__)
    work = script.parent if work_text is None else Path(work_text)
    match = re.fullmatch(WORK_PATTERN, str(work))
    if not match or script.name != 'gui_qs_exit_forward_probe.py' or script.parent != work:
        raise ValueError('exact fresh RAM script path required')
    if os.getuid() != 0 or sys.platform != 'linux' or ctypes.sizeof(ctypes.c_long) != 8:
        raise ValueError('camera Linux root /64-bit time ABI required')
    for path in (work, script):
        info = path.lstat()
        if (path.is_symlink() or info.st_uid != 0 or info.st_mode & 0o022
                or str(path.resolve()) != str(path)):
            raise ValueError('private root-owned RAM paths required')
    if not work.is_dir() or not script.is_file():
        raise ValueError('regular script and work directory required')
    rows = [r for r in mount_rows() if str(work) == r['point'] or str(work).startswith(r['point'].rstrip('/') + '/')]
    if not rows or max(rows, key=lambda r: len(r['point']))['fs'] != 'tmpfs':
        raise ValueError('work must be on tmpfs')
    return work, match[1]


def readiness():
    """Same exact-ELF, bounded read-only manager check as awake preflight45."""
    pids = []
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                if (entry / 'comm').read_text().strip() == 'dji_sys':
                    pids.append(entry)
            except OSError:
                pass
    if len(pids) != 1:
        raise ValueError('unique system service required')
    proc = pids[0]
    token = proc_token(proc.name)
    if hashlib.sha256((proc / 'exe').read_bytes()).hexdigest() != SYS_SHA256:
        raise ValueError('system executable digest mismatch')
    namespace = identity('/proc/self/ns/mnt')
    if identity(proc / 'ns/mnt') != namespace:
        raise ValueError('helper and dji_sys mount namespaces differ')
    rows = [line.split() for line in (proc / 'maps').read_text().splitlines()]
    bases = [int(r[0].split('-')[0], 16) for r in rows if len(r) > 5
             and r[5] == '/usr/bin/dji_sys' and int(r[2], 16) == 0]
    if len(bases) != 1:
        raise ValueError('PIE mapping')
    ranges = [(int(r[0].split('-')[0], 16), int(r[0].split('-')[1], 16)) for r in rows if 'r' in r[1]]
    fd = os.open(proc / 'mem', os.O_RDONLY | os.O_CLOEXEC)
    try:
        def read(address, size):
            if not any(lo <= address and address + size <= hi for lo, hi in ranges):
                raise ValueError('read outside readable mapping')
            data = os.pread(fd, size, address)
            if len(data) != size:
                raise ValueError('short manager read')
            return int.from_bytes(data, 'little')
        info = read(bases[0] + 0x158ef8, 8)
        manager = read(info + 0x90640, 8)
        if read(manager + 0x8e0 + 0x128, 8) != manager:
            raise ValueError('manager owner cross-check')
        result = dict(clients_ever_ready=read(manager + 8, 1), reboot_disabled=read(manager + 9, 1),
                      is_suspend_resume=read(manager + 7, 1), system_power_level=read(manager + 0x8f0, 4),
                      verified_executable=SYS_SHA256, service=token, mount_namespace=list(namespace))
    finally:
        os.close(fd)
    if proc_token(proc.name) != token or identity(proc / 'ns/mnt') != namespace:
        raise ValueError('service or mount namespace changed')
    return result


def require_ready(value, *, awake=True):
    if value['clients_ever_ready'] != 1 or value['reboot_disabled'] != 0:
        raise ValueError('M8 readiness/reboot guard')
    if awake and value['system_power_level'] != 0:
        raise ValueError('system manager must be logical0 before S3')


def battery():
    value = Path('/sys/class/power_supply/battery/present').read_text().strip()
    if value != '1':
        raise ValueError('battery must be present1')
    return value


class RealRole:
    """Pin the original kernfs attribute before the shadow; never retarget it."""
    def __init__(self):
        self.fd = None
        self.attempted = False
        if any(row['point'] == str(TARGET) for row in mount_rows()):
            raise ValueError('real role must be opened before any bind mount')
        self.namespace = identity('/proc/self/ns/mnt')
        self.parent_inode = identity(TARGET.parent)
        self.driver_inode = identity(DRIVER)
        self.module_inode = identity(KERNEL_MODULE)
        self.module_sysfs = (TARGET.parent / 'driver' / 'module').resolve()
        self.module_sysfs_inode = identity(self.module_sysfs)
        self.check_driver()
        expected = identity(TARGET)
        try:
            self.fd = os.open(TARGET, os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC)
            self.fd_inode = self.opened_identity()
            if self.fd_inode != expected:
                raise ValueError('role inode changed during open')
            if self.read_role() != 'peripheral':
                raise ValueError('real pre-bind role must be peripheral')
            self.check()
        except BaseException:
            self.close()
            raise

    def opened_identity(self):
        info = os.fstat(self.fd)
        if info.st_uid != 0 or not stat.S_ISREG(info.st_mode):
            raise ValueError('original real role must be root-owned sysfs attribute')
        libc = ctypes.CDLL(None, use_errno=True)
        libc.fstatfs.argtypes = [ctypes.c_int, ctypes.c_void_p]
        libc.fstatfs.restype = ctypes.c_int
        data = ctypes.create_string_buffer(256)  # Linux/aarch64 statfs is120B.
        if libc.fstatfs(self.fd, data):
            raise OSError(ctypes.get_errno(), 'fstatfs real role')
        if int.from_bytes(data.raw[:8], sys.byteorder) != SYSFS_MAGIC:
            raise ValueError('pinned original role FD is not sysfs')
        return info.st_dev, info.st_ino

    def check_driver(self):
        if (identity('/proc/self/ns/mnt') != self.namespace
                or identity(TARGET.parent) != self.parent_inode
                or (TARGET.parent / 'driver').resolve() != DRIVER
                or identity(DRIVER) != self.driver_inode
                or self.module_sysfs != Path('/sys/module/dwc3_msm')
                or (TARGET.parent / 'driver' / 'module').resolve() != self.module_sysfs
                or identity(self.module_sysfs) != self.module_sysfs_inode
                or KERNEL_MODULE.is_symlink()
                or identity(KERNEL_MODULE) != self.module_inode
                or hashlib.sha256(KERNEL_MODULE.read_bytes()).hexdigest() != KERNEL_MODULE_SHA256):
            raise ValueError('original USB platform/driver/module/namespace changed')

    def check(self):
        if self.fd is None or self.opened_identity() != self.fd_inode:
            raise ValueError('original real role FD identity changed')
        self.check_driver()
        return dict(inode=list(self.fd_inode), parent_inode=list(self.parent_inode),
                    driver=str(DRIVER), driver_inode=list(self.driver_inode),
                    module_sha256=KERNEL_MODULE_SHA256, module_inode=list(self.module_inode),
                    module_sysfs_inode=list(self.module_sysfs_inode), namespace=list(self.namespace))

    @classmethod
    def adopt(cls, fd, proof):
        """An inherited FD only: never open the now-shadowed target path."""
        if type(fd) is not int or fd < 3:
            raise ValueError('explicit inherited real role FD required')
        result = object.__new__(cls)
        result.fd, result.attempted = fd, False
        try:
            result.namespace = tuple(proof['namespace'])
            result.parent_inode = tuple(proof['parent_inode'])
            result.driver_inode = tuple(proof['driver_inode'])
            result.module_inode = tuple(proof['module_inode'])
            result.module_sysfs = Path('/sys/module/dwc3_msm')
            result.module_sysfs_inode = tuple(proof['module_sysfs_inode'])
            result.fd_inode = tuple(proof['inode'])
            if proof.get('driver') != str(DRIVER) or proof.get('module_sha256') != KERNEL_MODULE_SHA256:
                raise ValueError('original role driver proof mismatch')
            result.check()
        except BaseException:
            result.close()
            raise
        return result

    def read_role(self):
        data = os.pread(self.fd, 64, 0)
        if data not in (b'peripheral\n', b'none\n', b'host\n'):
            raise ValueError('unexpected original real role readback')
        return data.decode('ascii').strip()

    def forward_exit_once(self, guard):
        if self.attempted:
            raise ValueError('real EXIT already attempted; no retry')
        self.check()
        guard()
        # Set before the syscall: partial/failed writes are uncertain, never retried.
        self.attempted = True
        os.lseek(self.fd, 0, os.SEEK_SET)
        count = os.write(self.fd, b'exit\n')
        if count != 5:
            raise OSError('partial real EXIT write; no retry')
        return dict(completed=True, bytes_written=count, literal='exit', at=stamp(),
                    real_role_after=self.read_role(), no_media_success_claim=True)

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)


class RoleShadow:
    def __init__(self, work):
        self.source = work / 'role.txt'

    def proof(self):
        if self.source.is_symlink() or TARGET.is_symlink():
            raise ValueError('unexpected role symlink')
        info = self.source.stat()
        if info.st_uid != 0 or not stat.S_ISREG(info.st_mode) or info.st_mode & 0o022:
            raise ValueError('private RAM role regular file required')
        if identity(TARGET) != identity(self.source):
            raise ValueError('role bind inode mismatch')
        rows = [r for r in mount_rows() if r['point'] == str(TARGET)]
        expected_root = '/' + self.source.parent.name + '/role.txt'
        if len(rows) != 1 or rows[0]['fs'] != 'tmpfs' or rows[0]['root'] != expected_root:
            raise ValueError('exact owned role mountinfo missing')
        return dict(source=str(self.source), target=str(TARGET), inode=list(identity(self.source)),
                    mount=rows[0], value=TARGET.read_text().strip())

    def bind(self):
        if any(r['point'] == str(TARGET) for r in mount_rows()):
            raise ValueError('target already has a mount')
        if TARGET.read_text().strip() != 'peripheral':
            raise ValueError('native role must be peripheral')
        fd = os.open(self.source, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write('peripheral\n')
        libc = ctypes.CDLL(None, use_errno=True)
        libc.mount.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                               ctypes.c_ulong, ctypes.c_void_p]
        libc.mount.restype = ctypes.c_int
        if libc.mount(os.fsencode(self.source), os.fsencode(TARGET), None, 4096, None):
            raise OSError(ctypes.get_errno(), 'bind owned role RAM shadow')
        return self.proof()


class Timespec(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]


class Itimerspec(ctypes.Structure):
    _fields_ = [('it_interval', Timespec), ('it_value', Timespec)]


class Alarm:
    def __init__(self, seconds):
        self.lib = ctypes.CDLL(None, use_errno=True)
        self.lib.timerfd_create.argtypes = [ctypes.c_int, ctypes.c_int]
        self.lib.timerfd_create.restype = ctypes.c_int
        self.lib.timerfd_settime.argtypes = [ctypes.c_int, ctypes.c_int,
                                            ctypes.POINTER(Itimerspec), ctypes.POINTER(Itimerspec)]
        self.lib.timerfd_settime.restype = ctypes.c_int
        self.lib.timerfd_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(Itimerspec)]
        self.lib.timerfd_gettime.restype = ctypes.c_int
        self.fd = self.lib.timerfd_create(9, os.O_CLOEXEC | os.O_NONBLOCK)
        if self.fd < 0:
            raise OSError(ctypes.get_errno(), 'create CLOCK_BOOTTIME_ALARM')
        timer = Itimerspec(Timespec(0, 0), Timespec(int(seconds), 0))
        try:
            if self.lib.timerfd_settime(self.fd, 0, ctypes.byref(timer), None):
                raise OSError(ctypes.get_errno(), 'arm reboot alarm')
            self.verify(seconds)
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise

    def verify(self, seconds):
        left = Itimerspec()
        if self.lib.timerfd_gettime(self.fd, ctypes.byref(left)):
            raise OSError(ctypes.get_errno(), 'verify reboot alarm')
        remaining = left.it_value.tv_sec + left.it_value.tv_nsec / 1e9
        if not max(0.01, seconds - 2) <= remaining <= seconds:
            raise ValueError('alarm not independently armed')
        return dict(clock_id=9, remaining=remaining, armed_at=stamp())

    def cancel(self):
        # This owner is single-threaded, never duplicates timer FDs, and creates
        # them only after its LPC worker has been spawned. CLOEXEC also keeps
        # them out of any later stock-reboot child. Closing our last reference
        # disarms the object without depending on vendor ALARM gettime rounding.
        if self.fd is None:
            raise ValueError('90s alarm already closed')
        fd, self.fd = self.fd, None
        os.close(fd)
        try:
            os.fstat(fd)
        except OSError as exc:
            if exc.errno != 9:  # Linux EBADF: descriptor no longer exists.
                raise
        else:
            raise ValueError('90s descriptor still open or unexpectedly reused')
        return dict(method='close_last_owner_fd', closed_fd=fd,
                    closed_verified_ebadf=True, at=stamp())

    def fired(self):
        raw = os.read(self.fd, 8)
        if len(raw) != 8 or struct.unpack('=Q', raw)[0] != 1:
            raise ValueError('unexpected alarm expiration')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class PowerClient:
    def __init__(self):
        self.lib = ctypes.CDLL('/usr/lib/libsysmode_util.so', use_errno=True)
        self.lib.sysmode_create_lpc_client_handle.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_ushort]
        self.lib.sysmode_create_lpc_client_handle.restype = ctypes.c_int
        self.lib.sysmode_get_system_power_level.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        self.lib.sysmode_get_system_power_level.restype = ctypes.c_int
        self.lib.sysmode_destroy_lpc_client_handle.argtypes = [ctypes.c_void_p]
        self.lib.sysmode_destroy_lpc_client_handle.restype = None
        self.handle = ctypes.c_void_p()
        self.host_id = int.from_bytes(os.urandom(2), 'little') or 1
        result = self.lib.sysmode_create_lpc_client_handle(ctypes.byref(self.handle), self.host_id)
        if result != 0 or not self.handle.value:
            if self.handle.value:
                self.lib.sysmode_destroy_lpc_client_handle(self.handle)
            raise ValueError('LPC create failed: ' + str(result))

    def get(self):
        value = ctypes.c_int(-1)
        result = self.lib.sysmode_get_system_power_level(self.handle, ctypes.byref(value))
        return dict(returncode=result, level=value.value if result == 0 else None, at=stamp())

    def close(self):
        if self.handle.value:
            self.lib.sysmode_destroy_lpc_client_handle(self.handle)
            self.handle = ctypes.c_void_p()


def require_get0(result):
    if result.get('returncode') != 0 or result.get('level') != 0:
        raise ValueError('native GET0 required')


class Export:
    """One exclusive export; later phases replace only our tracked inode."""
    def __init__(self, work, nonce):
        self.work, self.nonce = work, nonce
        self.path = Path('/blackbox/export_tmp/osmo_probe_' + nonce + '.json')
        self.marker = work / 'export-owner.json'

    def create(self, value):
        value = diagnostic_report(value)
        if str(Path('/blackbox').resolve()) != '/blackbox' or not Path('/blackbox').is_dir():
            raise ValueError('blackbox path')
        self.path.parent.mkdir(mode=0o700, exist_ok=True)
        if self.path.parent.is_symlink() or self.path.parent.resolve() != self.path.parent:
            raise ValueError('export directory')
        inode = write_new(self.path, value)
        write_new(self.marker, dict(nonce=self.nonce, inodes=[inode], updates=1))

    def owned(self):
        owner = read_json(self.marker)
        info = self.path.lstat()
        if (owner['nonce'] != self.nonce or not stat.S_ISREG(info.st_mode) or info.st_uid != 0
                or [info.st_dev, info.st_ino] not in owner['inodes']):
            raise ValueError('export inode ownership mismatch')
        return owner

    def update(self, value):
        value = diagnostic_report(value)
        owner = self.owned()
        if owner['updates'] >= 16:
            raise ValueError('finite export phase limit')
        temporary = self.path.with_name(self.path.name + '.next-' + self.nonce)
        inode = write_new(temporary, value)
        try:
            # Track both possible owned inodes before atomic replacement.
            write_ram(self.marker, dict(nonce=self.nonce, inodes=owner['inodes'] + [inode],
                                        updates=owner['updates'] + 1))
            self.owned()
            os.replace(temporary, self.path)
            write_ram(self.marker, dict(nonce=self.nonce, inodes=[inode], updates=owner['updates'] + 1))
        finally:
            if temporary.exists():
                temporary.unlink()


def config(work):
    value = read_json(work / 'config.json')
    if value['nonce'] != work.name[-12:] or value['host_record0'] is not True or value['host_battery_present1'] is not True or value.get('host_activation_state1') is not True:
        raise ValueError('host Record0/battery/activated raw-state1 guard required')
    if type(value.get('forward_native_exit', False)) is not bool:
        raise ValueError('explicit boolean forwarding policy required')
    return value


def snapshot(work, *, fresh=False):
    result = dict(at=stamp())
    if fresh:
        try:
            result['readiness'] = readiness()
        except BaseException as exc:
            result['readiness_error'] = failure(exc)
    try:
        result['role'] = RoleShadow(work).proof()['value']
    except Exception as exc:
        result['role_error'] = failure(exc)
    for key in ('entry-intent', 'entry-result', 'entry-observed', 'worker-error', 'wake', 'wake-error',
                'wake-returning', 'real-exit-intent', 'real-exit-result'):
        path = work / (key + '.json')
        if path.exists():
            result[key] = read_json(path)
    return result


def stock_reboot():
    """Spawn literal stock command, without blocking the independent owner."""
    try:
        proc = subprocess.Popen(['systemctl', 'reboot'], stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)
        return dict(command=['systemctl', 'reboot'], child_pid=proc.pid,
                    at=stamp(), pending_reboot=True)
    except BaseException as exc:
        return dict(command=['systemctl', 'reboot'], error=failure(exc), at=stamp(), pending_reboot=True)


def wait_file(path, timeout, process=None):
    deadline = time.monotonic() + timeout
    while not path.exists():
        if process is not None and process.poll() is not None:
            raise ValueError('prepared subprocess exited')
        if time.monotonic() >= deadline:
            raise TimeoutError('waiting for ' + path.name)
        time.sleep(.02)
    return read_json(path)


def entry_fresh(work, cfg, owner_token, armed, margin=0.0):
    now = stamp()
    deadline = armed.get('backup_deadline_boottime')
    if (armed.get('nonce') != cfg['nonce'] or armed.get('owner') != owner_token
            or type(deadline) not in (int, float) or not math.isfinite(deadline)
            or now['boottime'] + margin >= deadline
            or not alive(owner_token) or (work / 'reboot.json').exists()):
        raise ValueError('fresh armed recovery owner required before key input')
    return now


class QSKey:
    """One pinned gpio-keys FD; emits only EV_KEY357 and SYN_REPORT."""
    def __init__(self):
        entries = [path for path in Path('/sys/class/input').glob('event*')
                   if path.joinpath('device/name').read_text().strip() == 'gpio-keys']
        if len(entries) != 1:
            raise ValueError('exactly one gpio-keys input required')
        self.sysfs = entries[0].resolve(strict=True)
        self.sysfs_inode = identity(self.sysfs)
        words = self.sysfs.joinpath('device/capabilities/key').read_text().split()
        if not words or any(not re.fullmatch('[0-9a-fA-F]{1,16}', word) for word in words):
            raise ValueError('invalid gpio-keys bitmap')
        bits = int(''.join(word.zfill(16) for word in words), 16)
        if not bits & (1 << 357):
            raise ValueError('gpio-keys must advertise QS357')
        self.node = Path('/dev/input') / entries[0].name
        self.fd = os.open(self.node, os.O_WRONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            info = os.fstat(self.fd)
            self.fd_identity = (info.st_dev, info.st_ino, info.st_rdev)
            if not stat.S_ISCHR(info.st_mode):
                raise ValueError('input node is not a character device')
            self.check()
        except BaseException:
            self.close()
            raise

    def check(self):
        info = os.stat(self.node)
        opened = os.fstat(self.fd)
        if (identity(self.sysfs) != self.sysfs_inode
                or self.fd_identity != (info.st_dev, info.st_ino, info.st_rdev)
                or self.fd_identity != (opened.st_dev, opened.st_ino, opened.st_rdev)):
            raise ValueError('original gpio-keys identity changed')

    def _write(self, pressed):
        # The last release uses the original FD even if an entry was renamed;
        # it never opens or writes a replacement event node.
        info = os.fstat(self.fd)
        if self.fd_identity != (info.st_dev, info.st_ino, info.st_rdev):
            raise ValueError('opened gpio-keys FD identity changed')
        payload = struct.pack('=qqHHi', 0, 0, 1, 357, int(pressed))
        payload += struct.pack('=qqHHi', 0, 0, 0, 0, 0)
        if os.write(self.fd, payload) != len(payload):
            raise ValueError('partial QS/SYN event write')

    def pulse(self, seconds, guard, sleep=time.sleep):
        if seconds not in (0.1, 1.1):
            raise ValueError('only fixed QS entry1.1s/wake0.1s pulses allowed')
        self.check()
        guard()  # Must follow every potentially slow input lookup/read.
        attempted = False
        error = None
        began = stamp()
        try:
            guard()
            attempted = True
            self._write(True)
            sleep(seconds)
        except BaseException as exc:
            error = exc
        finally:
            if attempted:
                try:
                    self._write(False)
                except BaseException as exc:
                    if error is None:
                        error = exc
                    else:
                        error.add_note('QS release failed: ' + failure(exc))
        if error is not None:
            raise error
        return dict(key=357, hold_seconds=seconds, press_release_complete=True,
                    input_node=str(self.node), started=began, finished=stamp(), lpc_set_calls=0)

    def close(self):
        if getattr(self, 'fd', None) is not None:
            os.close(self.fd)
            self.fd = None


def worker(work):
    """Only stock QS entry; blocking getters cannot stop the RTC/reboot owner."""
    client = key = None
    try:
        cfg = config(work)
        client, key = PowerClient(), QSKey()
        before = client.get()
        require_get0(before)
        write_new(work / 'worker-ready.json', dict(nonce=cfg['nonce'], process=proc_token(),
                    get0=before, input_node=str(key.node), lpc_set_calls=0, at=stamp()))
        go = wait_file(work / 'go.json', 8)
        owner_token = go['owner']
        armed = read_json(work / 'armed.json')
        while stamp()['boottime'] < go['not_before']:
            entry_fresh(work, cfg, owner_token, armed, margin=2.0)
            time.sleep(.02)
        guards = readiness()
        require_ready(guards)
        battery()
        proof = RoleShadow(work).proof()
        before = client.get()
        require_get0(before)
        fresh = entry_fresh(work, cfg, owner_token, armed, margin=2.0)
        write_new(work / 'entry-intent.json', dict(nonce=cfg['nonce'], owner=owner_token,
                    at=fresh, guards=guards, get0=before, shadow=proof,
                    activation_raw_state=1, entry_key=357, hold_seconds=1.1, lpc_set_calls=0))
        pulse = key.pulse(1.1, lambda: entry_fresh(work, cfg, owner_token, armed, margin=2.0))
        write_new(work / 'entry-result.json', dict(nonce=cfg['nonce'], owner=owner_token,
                                                   at=stamp(), pulse=pulse, lpc_set_calls=0))
        # Stock GUI schedules its request after the one-second key timer and
        # animation500ms timer. A bare key receipt is not logical entry proof.
        deadline = time.monotonic() + 4
        observed = client.get()
        while observed.get('returncode') == 0 and observed.get('level') not in (2, 3):
            if time.monotonic() >= deadline:
                raise ValueError('stock QS entry did not reach logical2/3')
            entry_fresh(work, cfg, owner_token, armed)
            time.sleep(.05)
            observed = client.get()
        if observed.get('returncode') != 0 or observed.get('level') not in (2, 3):
            raise ValueError('stock QS logical entry getter failed')
        write_new(work / 'entry-observed.json', dict(nonce=cfg['nonce'], owner=owner_token,
                    at=stamp(), get_after=observed, gui_processing3_observed=None,
                    gui_processing_note='manager GET is not the GUI observer callback', lpc_set_calls=0))
        while alive(owner_token) and not (work / 'reboot.json').exists():
            time.sleep(.1)
    except BaseException as exc:
        write_ram(work / 'worker-error.json', dict(error=failure(exc), at=stamp()))
    finally:
        if key is not None:
            key.close()
        if client is not None:
            client.close()


def valid_entry(cfg, intent, receipt, observed):
    pulse = receipt.get('pulse', {})
    return (all(value.get('nonce') == cfg['nonce'] and value.get('owner') == cfg['owner']
                for value in (intent, receipt, observed))
            and intent.get('activation_raw_state') == 1
            and intent.get('lpc_set_calls') == receipt.get('lpc_set_calls') == observed.get('lpc_set_calls') == 0
            and pulse.get('key') == 357 and pulse.get('hold_seconds') == 1.1
            and pulse.get('press_release_complete') is True
            and observed.get('get_after', {}).get('returncode') == 0
            and observed.get('get_after', {}).get('level') in (2, 3)
            and receipt.get('at', {}).get('boottime', -1) >= intent['at']['boottime']
            and observed.get('at', {}).get('boottime', -1) >= receipt['at']['boottime'])


def valid_wake(value, cfg, intent, receipt, observed):
    before, after = value.get('readiness_before', {}), value.get('readiness_after', {})
    expected_service = intent.get('guards', {}).get('service')
    pulse = value.get('pulse', {})
    return (valid_entry(cfg, intent, receipt, observed)
            and value.get('confirmation') == 'stock_qs_wake_get0'
            and value.get('nonce') == cfg['nonce'] and value.get('owner') == cfg['owner']
            and value.get('lpc_set_calls') == 0
            and pulse.get('key') == 357 and pulse.get('hold_seconds') == .1
            and pulse.get('press_release_complete') is True
            and value.get('get_after', {}).get('returncode') == 0
            and value.get('get_after', {}).get('level') == 0
            and expected_service is not None
            and before.get('service') == after.get('service') == expected_service
            and all(item.get('clients_ever_ready') == 1 and item.get('reboot_disabled') == 0
                    for item in (before, after))
            and after.get('system_power_level') == 0
            and value.get('at', {}).get('boottime', -1) >= observed['at']['boottime'])


def forward_native_exit(work, inherited_fd):
    """Separate finite child: a blocking kernfs store cannot trap the RTC owner."""
    role = None
    cfg = config(work)
    owner_token = read_json(work / 'owner.json')
    result = dict(nonce=cfg['nonce'], owner=owner_token, completed=False,
                  literal='exit', no_media_success_claim=True)
    try:
        if cfg.get('forward_native_exit') is not True:
            raise ValueError('diagnostics only; EXIT forwarding was not authorized')
        original = read_json(work / 'real-role.json')
        if original.get('nonce') != cfg['nonce'] or original.get('owner') != owner_token:
            raise ValueError('original real FD owner mismatch')
        role = RealRole.adopt(inherited_fd, original['proof'])
        armed = read_json(work / 'armed.json')
        intent = read_json(work / 'entry-intent.json')
        receipt = read_json(work / 'entry-result.json')
        observed = read_json(work / 'entry-observed.json')
        wake = read_json(work / 'wake.json')
        if not valid_wake(wake, dict(cfg, owner=owner_token), intent, receipt, observed):
            raise ValueError('one verified stock QS wake GET0 required')

        def guard():
            ready = readiness()
            require_ready(ready)
            if ready['service'] != intent.get('guards', {}).get('service'):
                raise ValueError('original dji_sys service changed before EXIT')
            if RoleShadow(work).proof()['value'] != 'exit':
                raise ValueError('exact stock EXIT must already be observed in owned shadow')
            entry_fresh(work, cfg, owner_token, armed, margin=.1)

        if (work / 'real-exit-intent.json').exists() or (work / 'real-exit-result.json').exists():
            raise ValueError('real EXIT intent already exists; no retry')
        guard()
        write_new(work / 'real-exit-intent.json', dict(result, at=stamp(), proof=role.check()))
        # The final guard follows the intent write: slow publication cannot
        # create a late store after the independent owner requested recovery.
        result.update(role.forward_exit_once(guard))
    except BaseException as exc:
        result['error'] = failure(exc)
        raise
    finally:
        try:
            write_new(work / 'real-exit-result.json', result)
        finally:
            if role is not None:
                role.close()
    return result


def owner(work):
    """Never calls blocking LPC. Keeps shadow through all late ACT callbacks."""
    cfg = config(work)
    cfg['owner'] = proc_token()
    write_new(work / 'owner.json', cfg['owner'])
    exported = Export(work, cfg['nonce'])
    result = dict(nonce=cfg['nonce'], owner=cfg['owner'], phase='preparing',
                  physical_wake_proven=False, power_saving_proven=False, cleanup='stock reboot only')
    short = hard = real = None
    hard_expired = False
    bind_attempted = False
    process = None
    forwarder = None
    forward_deadline = None
    try:
        result['readiness_before'] = readiness()
        require_ready(result['readiness_before'])
        result['battery_present'] = battery()
        process = subprocess.Popen([sys.executable, __file__, '_worker'], stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        ready = wait_file(work / 'worker-ready.json', 4, process)
        require_get0(ready['get0'])
        if ready['nonce'] != cfg['nonce'] or not alive(ready['process']):
            raise ValueError('prepared LPC worker not live')
        short, hard = Alarm(BACKUP), Alarm(HARD_LIMIT)
        result['alarm90'] = short.verify(BACKUP)
        result['alarm240'] = hard.verify(HARD_LIMIT)
        real = RealRole()
        result['real_role'] = real.check()
        write_new(work / 'real-role.json', dict(nonce=cfg['nonce'], owner=cfg['owner'],
                                               proof=result['real_role']))
        bind_attempted = True
        result['shadow'] = RoleShadow(work).bind()
        # Verify namespace/readiness again after the shared bind exists.
        result['readiness_mounted'] = readiness()
        require_ready(result['readiness_mounted'])
        armed = dict(nonce=cfg['nonce'], owner=cfg['owner'], at=stamp(),
                     backup_deadline_boottime=result['alarm90']['armed_at']['boottime'] + BACKUP)
        write_new(work / 'armed.json', armed)
        result.update(phase='armed', armed=armed)
        exported.update(result)
        write_new(work / 'owner-ready.json', armed)
        wait_file(work / 'launch-returning.json', 3)
        write_new(work / 'go.json', dict(owner=cfg['owner'], not_before=stamp()['boottime'] + DELAY))
        samples = deque(maxlen=48)
        awake = False
        published_s3 = False
        last_sample = 0
        while True:
            now = stamp()
            if now['monotonic'] - last_sample >= 1:
                samples.append(dict(at=now, role=RoleShadow(work).proof()['value']))
                last_sample = now['monotonic']
                result['samples'] = list(samples)
                result['observed'] = snapshot(work)
                write_ram(work / 'status.json', diagnostic_report(result))
            if (work / 'snapshot-request.json').exists():
                request = read_json(work / 'snapshot-request.json')
                if request.get('nonce') != cfg['nonce'] or not re.fullmatch('[0-9a-f]{12}', request.get('request', '')):
                    raise ValueError('snapshot request ownership mismatch')
                # Only this owner replaces the export/inode marker. Host status
                # never races Export.update against the alarm owner.
                result['last_requested_snapshot'] = dict(request=request['request'],
                                                          **snapshot(work, fresh=True))
                exported.update(result)
                write_ram(work / 'snapshot-ack.json', dict(nonce=cfg['nonce'], request=request['request'], at=stamp()))
                (work / 'snapshot-request.json').unlink()
            if (work / 'finalize.json').exists():
                request = read_json(work / 'finalize.json')
                if request['nonce'] != cfg['nonce']:
                    raise ValueError('finalize nonce mismatch')
                result['reason'] = 'host finalize: reboot required to remove shadow'
                break
            if (work / 'entry-intent.json').exists() and not (work / 'entry-observed.json').exists():
                intent = read_json(work / 'entry-intent.json')
                # Do not wait90s for a hung getter while the stock GUI's12s
                # fallback is running. Monotonic excludes real AP suspend time.
                if now['monotonic'] - intent['at']['monotonic'] > 6:
                    raise ValueError('QS entry proof deadline; stock reboot required')
            if (work / 'worker-error.json').exists() or (work / 'wake-error.json').exists():
                raise ValueError('QS worker/wake failure; stock reboot required')
            if (work / 'entry-result.json').exists() and not published_s3:
                result.update(phase='Stock QS entry receipt; no physical claim', observed=snapshot(work))
                exported.update(result)
                published_s3 = True
            if (work / 'wake.json').exists() and not awake:
                wake = read_json(work / 'wake.json')
                intent = read_json(work / 'entry-intent.json')
                receipt = read_json(work / 'entry-result.json')
                observed = read_json(work / 'entry-observed.json')
                if not valid_wake(wake, cfg, intent, receipt, observed):
                    raise ValueError('invalid stock QS entry/wake GET evidence')
                if stamp()['boottime'] - result['alarm90']['armed_at']['boottime'] >= BACKUP:
                    raise ValueError('GUI wake confirmation exceeded90s backup deadline')
                result['alarm90_cancel'] = short.cancel()
                awake = True
                phase = 'Stock QS wake GET0 observed; shadow retained until reboot'
                result.update(phase=phase,
                              backup90_cancelled=True, wake=wake, observed=snapshot(work))
                write_ram(work / 'status.json', diagnostic_report(result))
                exported.update(result)
            # Do not gate the wake D1 reply on a syscall that may reset its USB
            # path. The wake process first publishes its normal return marker;
            # then this owner waits an explicit2s before launching the writer.
            if awake and cfg.get('forward_native_exit') is True and 'real_exit_forward' not in result:
                if (work / 'wake-returning.json').exists():
                    returning = read_json(work / 'wake-returning.json')
                    if returning.get('nonce') != cfg['nonce'] or returning.get('owner') != cfg['owner']:
                        raise ValueError('wake return marker ownership mismatch')
                    if forwarder is None and now['monotonic'] >= returning['at']['monotonic'] + FORWARD_DELAY:
                        forward_started = stamp()
                        forward_deadline = forward_started['monotonic'] + FORWARD_RESULT_TIMEOUT
                        forwarder = subprocess.Popen([sys.executable, __file__, '_forward_exit',
                            '--real-role-fd', str(real.fd)], pass_fds=(real.fd,),
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
                        result['real_exit_child'] = dict(pid=forwarder.pid, at=forward_started)
                    if forwarder is not None:
                        if (work / 'real-exit-result.json').exists():
                            forwarded = read_json(work / 'real-exit-result.json')
                            completed_at = forwarded.get('at', {}).get('monotonic')
                            if (forwarded.get('nonce') != cfg['nonce'] or forwarded.get('owner') != cfg['owner']
                                    or forwarded.get('completed') is not True
                                    or forwarded.get('literal') != 'exit' or forwarded.get('bytes_written') != 5
                                    or type(completed_at) not in (int, float) or not math.isfinite(completed_at)
                                    or completed_at >= forward_deadline or time.monotonic() >= forward_deadline):
                                raise ValueError('real EXIT child did not conclusively complete')
                            result['real_exit_forward'] = forwarded
                            write_ram(work / 'status.json', diagnostic_report(result))
                            exported.update(result)
                        elif forwarder.poll() is not None or now['monotonic'] >= forward_deadline:
                            raise ValueError('real EXIT child deadline/error; no retry; recovery retained')
                elif now['monotonic'] - result['wake']['at']['monotonic'] > 3:
                    raise ValueError('wake return marker missing; never forward without it')
            active = [hard.fd] + ([] if awake else [short.fd])
            readable, _, _ = select.select(active, [], [], .1)
            if readable:
                for alarm in (short, hard):
                    if alarm.fd in readable:
                        alarm.fired()
                hard_expired = hard.fd in readable
                result['reason'] = '240s guard limit' if hard.fd in readable else '90s backup; no verified stock QS wake'
                break
            if process.poll() is not None:
                raise ValueError('QS worker died while role shadow retained')
    except BaseException as exc:
        result.update(error=failure(exc), reason='preflight/test error')
    finally:
        # Binding may have succeeded before proof/assignment. Once attempted,
        # even failed diagnostics cannot bypass the ordinary reboot request.
        if bind_attempted or (work / 'entry-intent.json').exists():
            result.update(phase='pending reboot; no success claim', pending_reboot=True)
            try:
                result['observed'] = snapshot(work)
                write_ram(work / 'reboot.json', dict(nonce=cfg['nonce'], reason=result.get('reason'), at=stamp()))
                write_ram(work / 'status.json', diagnostic_report(result))
                exported.update(result)
            except BaseException as exc:
                result['export_error'] = failure(exc)
            finally:
                # No LPC destroy/join occurs in this process before reboot.
                result['stock_reboot'] = stock_reboot()
            try:
                write_ram(work / 'status.json', diagnostic_report(result))
                exported.update(result)
            except BaseException:
                pass
            # A successful spawn, returned client, or failed systemctl request
            # cannot cancel recovery. Keep the independent hard alarm alive;
            # actual reboot kills this owner naturally. If still running at
            # 240s, issue the reserve stock request before closing anything.
            if hard is not None and not hard_expired:
                try:
                    while True:
                        try:
                            ready, _, _ = select.select([hard.fd], [], [], .2)
                        except InterruptedError:
                            continue
                        if ready:
                            hard.fired()
                            break
                        if stamp()['boottime'] - result['alarm240']['armed_at']['boottime'] >= HARD_LIMIT:
                            break
                except BaseException as exc:
                    result['hard_alarm_error'] = failure(exc)
                finally:
                    result['reserve_reboot'] = stock_reboot()
                result['phase'] = '240s reserve reboot requested; no success claim'
                try:
                    write_ram(work / 'status.json', diagnostic_report(result))
                    exported.update(result)
                except BaseException:
                    pass
        else:
            result.update(phase='failed before bind/S3', pending_reboot=False)
            write_ram(work / 'status.json', diagnostic_report(result))
            try:
                exported.update(result)
            except BaseException:
                pass
        for alarm in (short, hard):
            if alarm is not None:
                alarm.close()
        if real is not None:
            real.close()
    return result


def launch(work, nonce, *, record0, battery_present1, activation_state1, forward_native_exit=False):
    if not record0 or not battery_present1 or not activation_state1:
        raise ValueError('explicit fresh host Record0 and battery1 guards required')
    battery()
    result = readiness()
    require_ready(result)
    if TARGET.read_text().strip() != 'peripheral':
        raise ValueError('native peripheral role required')
    if any(r['point'] == str(TARGET) for r in mount_rows()):
        raise ValueError('target must not already be bind mounted')
    if type(forward_native_exit) is not bool:
        raise ValueError('explicit boolean forwarding policy required')
    cfg = dict(nonce=nonce, host_record0=True, host_battery_present1=True, host_activation_state1=True,
               forward_native_exit=forward_native_exit, at=stamp())
    write_new(work / 'config.json', cfg)
    Export(work, nonce).create(dict(phase='launch prepared', nonce=nonce, guards=cfg,
                                     physical_wake_proven=False, power_saving_proven=False))
    log_fd = os.open(work / 'owner.log', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(log_fd, 'wb') as log:
        proc = subprocess.Popen([sys.executable, __file__, '_owner'], stdin=subprocess.DEVNULL,
                                stdout=log, stderr=log, start_new_session=True)
    armed = wait_file(work / 'owner-ready.json', 7, proc)
    if not alive(armed['owner']) or armed['nonce'] != nonce:
        raise ValueError('detached recovery owner missing')
    write_new(work / 'launch-returning.json', dict(nonce=nonce, at=stamp()))
    return dict(launched=True, owner=armed['owner'], armed=armed,
                delay_after_launch_returning=DELAY, export=str(Export(work, nonce).path),
                no_physical_success_claim=True)


def wake(work):
    cfg = config(work)
    owner_token = read_json(work / 'owner.json')
    client = key = None
    try:
        armed = read_json(work / 'armed.json')
        entry_fresh(work, cfg, owner_token, armed, margin=.5)
        if (work / 'wake-intent.json').exists() or (work / 'wake.json').exists():
            raise ValueError('one QS wake only; never replay input')
        RoleShadow(work).proof()
        intent = read_json(work / 'entry-intent.json')
        receipt = read_json(work / 'entry-result.json')
        observed = read_json(work / 'entry-observed.json')
        if not valid_entry(dict(cfg, owner=owner_token), intent, receipt, observed):
            raise ValueError('matching stock QS entry proof required')
        ready = readiness()
        require_ready(ready, awake=False)
        if ready.get('service') != intent.get('guards', {}).get('service'):
            raise ValueError('dji_sys restarted since entry')
        client, key = PowerClient(), QSKey()
        before = client.get()
        if before.get('returncode') != 0 or before.get('level') not in (2,3):
            raise ValueError('logical standby2/3 required before stock QS wake')
        entry_fresh(work, cfg, owner_token, armed, margin=.5)
        write_new(work / 'wake-intent.json', dict(nonce=cfg['nonce'], owner=owner_token,
                                                 at=stamp(), lpc_set_calls=0))
        pulse = key.pulse(.1, lambda: entry_fresh(work, cfg, owner_token, armed, margin=.5))
        deadline = time.monotonic() + 3
        after = client.get()
        while after.get('returncode') == 0 and after.get('level') != 0 and time.monotonic() < deadline:
            entry_fresh(work, cfg, owner_token, armed)
            time.sleep(.05)
            after = client.get()
        require_get0(after)
        after_ready = readiness()
        require_ready(after_ready)
        evidence = dict(nonce=cfg['nonce'], owner=owner_token, at=stamp(),
                        confirmation='stock_qs_wake_get0', lpc_set_calls=0,
                        pulse=pulse, get_before=before, get_after=after,
                        readiness_before=ready, readiness_after=after_ready,
                        shadow=RoleShadow(work).proof(), no_physical_success_claim=True)
        if not valid_wake(evidence, dict(cfg, owner=owner_token), intent, receipt, observed):
            raise ValueError('stock QS wake GET0/service evidence mismatch')
        entry_fresh(work, cfg, owner_token, armed)
        write_new(work / 'wake.json', evidence)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if (work / 'reboot.json').exists() or not alive(owner_token):
                raise ValueError('recovery already requested during wake confirmation')
            if (work / 'status.json').exists():
                owner_status = read_json(work / 'status.json')
                if (owner_status.get('nonce') == cfg['nonce'] and owner_status.get('owner') == owner_token
                        and owner_status.get('wake') == evidence
                        and owner_status.get('backup90_cancelled') is True
                        and owner_status.get('alarm90_cancel', {}).get('closed_verified_ebadf') is True):
                    write_new(work / 'wake-returning.json', dict(nonce=cfg['nonce'], owner=owner_token, at=stamp()))
                    return dict(evidence, backup90_cancelled=True, hard240_retained=True,
                                native_exit_forward_planned=cfg.get('forward_native_exit', False),
                                native_exit_forward_delay_seconds=FORWARD_DELAY)
            time.sleep(.02)
        raise TimeoutError('owner did not confirm90s cancellation; hard recovery retained')
    except BaseException as exc:
        write_ram(work / 'wake-error.json', dict(error=failure(exc), at=stamp()))
        raise
    finally:
        if key is not None:
            key.close()
        if client is not None:
            client.close()


def status(work):
    cfg = config(work)
    token = read_json(work / 'owner.json')
    if not alive(token) or (work / 'reboot.json').exists():
        return dict(owner_alive=alive(token), pending_reboot=(work / 'reboot.json').exists(),
                    observed=snapshot(work, fresh=True), export=str(Export(work, cfg['nonce']).path))
    request = os.urandom(6).hex()
    write_new(work / 'snapshot-request.json', dict(nonce=cfg['nonce'], request=request, at=stamp()))
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if (work / 'snapshot-ack.json').exists():
            ack = read_json(work / 'snapshot-ack.json')
            if ack.get('nonce') == cfg['nonce'] and ack.get('request') == request:
                return dict(snapshot_exported=True, request=request, at=ack['at'],
                            export=str(Export(work, cfg['nonce']).path), no_physical_success_claim=True)
        if not alive(token) or (work / 'reboot.json').exists():
            raise ValueError('owner stopped before fresh snapshot publication')
        time.sleep(.02)
    raise TimeoutError('fresh snapshot export was not confirmed')


def finalize(work):
    cfg = config(work)
    write_new(work / 'finalize.json', dict(nonce=cfg['nonce'], at=stamp()))
    token = read_json(work / 'owner.json')
    if alive(token) and not (work / 'reboot.json').exists():
        return dict(requested='stock reboot', owner=token, pending_reboot=True)
    # No unmount even if the owner died or its previous reboot request failed.
    return stock_reboot()


def cleanup(work, nonce):
    """Only awake PRELAUNCH cleanup; a mounted/attempted probe needs reboot."""
    if any(r['point'] == str(TARGET) for r in mount_rows()) or (work / 'entry-intent.json').exists():
        raise ValueError('mounted/S3 probe requires stock reboot, never cleanup-unmount')
    for name in ('owner.json', 'worker-ready.json'):
        if (work / name).exists():
            value = read_json(work / name)
            token = value['process'] if name == 'worker-ready.json' else value
            if alive(token):
                raise ValueError('owned process still live; no cleanup')
    value = readiness()
    require_ready(value)
    output = Export(work, nonce)
    if output.marker.exists():
        output.owned()
        output.path.unlink()
    allowed = {'config.json','export-owner.json','owner.log','owner.json','status.json','worker-error.json',
               'worker-ready.json','owner-ready.json','armed.json','launch-returning.json','role.txt','real-role.json'}
    for path in work.iterdir():
        if path.name == 'gui_qs_exit_forward_probe.py':
            continue
        if path.name not in allowed or path.is_symlink() or not path.is_file() or path.stat().st_uid != 0:
            raise ValueError('unknown work file; refusing cleanup')
    for path in work.iterdir():
        if path.name != 'gui_qs_exit_forward_probe.py':
            path.unlink()
    return dict(cleaned_prelaunch=True, no_unmount=True, power_set_calls=0)


def main():
    import signal
    def interrupted(number, frame):
        raise KeyboardInterrupt('Signal ' + str(number))
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['launch','wake','status','finalize','cleanup','_owner','_worker','_forward_exit'])
    parser.add_argument('--record0', action='store_true')
    parser.add_argument('--battery-present1', action='store_true')
    parser.add_argument('--activation-state1', action='store_true')
    parser.add_argument('--forward-native-exit', action='store_true', help='one delayed source-proven real EXIT; default diagnostics only')
    parser.add_argument('--real-role-fd', type=int)
    args = parser.parse_args()
    work, nonce = environment()
    if args.mode == 'launch':
        result = launch(work, nonce, record0=args.record0, battery_present1=args.battery_present1, activation_state1=args.activation_state1, forward_native_exit=args.forward_native_exit)
    elif args.mode == '_owner':
        result = owner(work)
    elif args.mode == '_worker':
        worker(work)
        return
    elif args.mode == '_forward_exit':
        result = forward_native_exit(work, args.real_role_fd)
    elif args.mode == 'wake':
        result = wake(work)
    elif args.mode == 'status':
        result = status(work)
    elif args.mode == 'finalize':
        result = finalize(work)
    else:
        result = cleanup(work, nonce)
    print('OSMO_ROLE_JSON=' + json.dumps(result, sort_keys=True), flush=True)


if __name__ == '__main__':
    main()
