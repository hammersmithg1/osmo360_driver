#!/usr/bin/env python3
"""Template copied into the confirmed camera tmpfs; CONFIG is set by generator."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

CONFIG = None


def read(path):
    raw = Path(path).read_bytes().rstrip(b'\0\r\n')
    if b'\0' in raw:
        raise ValueError('Unexpected internal NUL in sysfs value')
    return raw.decode('ascii')


def attrs():
    function = Path(CONFIG['function'])
    for key, expected in [('dev_addr', CONFIG['dev_mac']), ('host_addr', CONFIG['host_mac'])]:
        if read(function / key).lower() != expected:
            raise ValueError('USB network function MAC changed: ' + key)
    return read(function / 'ifname')


def matches():
    root = Path(CONFIG['sys_net'])
    return [p.name for p in root.iterdir() if read(p / 'address').lower() == CONFIG['dev_mac']]


def valid_name(name):
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', name):
        raise ValueError('Unsafe discovered interface name')
    return name


def saved_name():
    path = Path(CONFIG['work']) / 'ifname'
    if path.is_symlink():
        raise ValueError('ifname state is a symlink')
    return valid_name(read(path)) if path.exists() else None


def save_name(name):
    previous = saved_name()
    if previous is not None and previous != name:
        raise ValueError('Discovered interface changed')
    path = Path(CONFIG['work']) / 'ifname'
    temporary = path.with_name('ifname.new')
    if temporary.exists() or temporary.is_symlink():
        raise ValueError('Unexpected ifname temporary state')
    temporary.write_text(name + '\n')
    os.replace(temporary, path)


def ip(*args):
    result = subprocess.run(['ip', *args], capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise ValueError('ip operation failed: ' + ' '.join(args))
    return result.stdout


def preflight():
    reported, names = attrs(), matches()
    if CONFIG['original_netdev_absent']:
        if reported != CONFIG['initial_ifname']:
            raise ValueError('Original absent-netdev placeholder changed')
        if names:
            raise ValueError('USB network device MAC already exists before binding')
        print('ABSENT_NETDEV_CONFIRMED')
    else:
        name = valid_name(CONFIG['original_net_if'])
        if reported != name or names != [name]:
            raise ValueError('Retained USB network interface changed')
        if read(Path(CONFIG['sys_net']) / name / 'flags') != CONFIG['original_flags']:
            raise ValueError('Retained USB network flags changed')
        if ip('-o', '-4', 'addr', 'show', 'dev', name).strip():
            raise ValueError('Retained USB network device already has IPv4')
        print('RETAINED_NETDEV_CONFIRMED')


def discover():
    deadline = time.monotonic() + 8
    while True:
        reported = attrs()
        names = matches()
        if len(names) > 1:
            raise ValueError('Ambiguous USB network device MAC')
        if names:
            name = valid_name(names[0])
            if reported not in (CONFIG['initial_ifname'], name):
                raise ValueError('Function ifname disagrees with discovered MAC')
            if ip('-o', '-4', 'addr', 'show', 'dev', name).strip():
                raise ValueError('New USB network device already has IPv4')
            save_name(name)  # Only a validated, MAC-matched name reaches rollback state.
            print(name)
            return
        if time.monotonic() >= deadline:
            raise ValueError('USB network device did not appear after binding')
        time.sleep(.1)


def cleanup():
    names = matches()
    if not names:
        return
    if len(names) != 1:
        raise ValueError('Ambiguous cleanup MAC; refuse to touch interfaces')
    name = valid_name(names[0])
    previous = saved_name()
    if previous is not None and previous != name:
        raise ValueError('Cleanup interface differs from owned state')
    save_name(name)
    failed = False
    try:
        # The only IPv4 address this overlay may delete is its own exact alias.
        subprocess.run(['ip', 'addr', 'del', CONFIG['address'], 'dev', name],
                       capture_output=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        failed = True
    try:
        ip('link', 'set', name, 'down')
    except (OSError, ValueError, subprocess.SubprocessError):
        failed = True
    if failed:
        raise ValueError('Network cleanup failed; USB rollback must continue')


def verify():
    attrs()
    names = matches()
    if not names:
        print('NETDEV_REMOVED')
        return
    if len(names) != 1:
        raise ValueError('Multiple retained USB network devices')
    name = valid_name(names[0])
    previous = saved_name()
    if previous is not None and previous != name:
        raise ValueError('Retained interface differs from owned state')
    if int(read(Path(CONFIG['sys_net']) / name / 'flags'), 16) & 1:
        raise ValueError('Retained USB network device is still UP')
    if (not CONFIG['original_netdev_absent']
            and read(Path(CONFIG['sys_net']) / name / 'flags') != CONFIG['original_flags']):
        raise ValueError('Retained USB network device flags were not restored')
    if ip('-o', '-4', 'addr', 'show', 'dev', name).strip():
        raise ValueError('Retained USB network device still has IPv4')
    print('NETDEV_RETAINED_DOWN_EMPTY:' + name)


def stop_dhcp():
    work = Path(CONFIG['work'])
    path = work / 'dhcp.pid'
    if not path.exists():
        return
    if path.is_symlink() or not re.fullmatch(r'[1-9][0-9]*', read(path)):
        raise ValueError('Invalid DHCP PID file')
    pid = int(read(path))
    expected = str(work / 'ecm_dhcp.py').encode()
    def owned():
        try:
            raw = Path('/proc').joinpath(str(pid), 'cmdline').read_bytes()
        except FileNotFoundError:
            return False
        if not raw:
            return False
        args = raw.split(b'\0')
        if expected not in args or str(work).encode() not in args:
            raise ValueError('Refuse to signal a PID outside this DHCP helper')
        return True
    if not owned():
        return
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 3
    while owned() and time.monotonic() < deadline:
        time.sleep(.05)
    if owned():
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            return


if __name__ == '__main__':
    try:
        if CONFIG is None:
            raise ValueError('Unconfigured template')
        {'preflight': preflight, 'discover': discover, 'cleanup': cleanup,
         'verify': verify, 'stop-dhcp': stop_dhcp}[sys.argv[1]]()
    except Exception as error:
        print(type(error).__name__ + ': ' + str(error), file=sys.stderr)
        raise SystemExit(1)
