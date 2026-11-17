#!/usr/bin/env python3
"""Quiesce only this RAM overlay's apply process group before USB rollback."""
import json
import os
from pathlib import Path
import signal
import sys
import time

CONFIG = None


def process(pid):
    root = Path(CONFIG['proc']) / str(pid)
    try:
        tail = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        args = (root / 'cmdline').read_bytes().split(b'\0')
    except FileNotFoundError:
        return None
    if tail[0] == 'Z' or not any(args):
        return None
    return {'start': tail[19], 'parent': int(tail[1]), 'group': int(tail[2]),
            'session': int(tail[3]), 'args': args}


def claim(pid):
    info = process(pid)
    if not info or str(Path(CONFIG['work']) / 'apply.sh').encode() not in info['args']:
        raise ValueError('Apply PID does not name this RAM script')
    if os.getpgid(pid) != pid or os.getsid(pid) != pid:
        raise ValueError('Apply must own its setsid process group')
    path = Path(CONFIG['work']) / 'apply.owner'
    with path.open('x') as stream:
        json.dump({'pid': pid, 'start': info['start']}, stream)


def members(owner):
    result = []
    for path in Path(CONFIG['proc']).iterdir():
        if not path.name.isdecimal():
            continue
        info = process(int(path.name))
        if info and info['group'] == owner['pid'] and info['session'] == owner['pid']:
            if int(info['start']) < int(owner['start']):
                raise ValueError('Process predates owned apply session')
            result.append(int(path.name))
    return result


def quiesce(from_apply=None):
    path = Path(CONFIG['work']) / 'apply.owner'
    if not path.exists():
        return
    if path.is_symlink():
        raise ValueError('Apply ownership state is a symlink')
    owner = json.loads(path.read_text())
    pid = owner['pid']
    if type(pid) is not int or pid <= 1:
        raise ValueError('Invalid apply owner PID')
    info = process(pid)
    if info and info['start'] != owner['start']:
        return  # Original session disappeared; never signal a reused PID.
    if info:
        expected = str(Path(CONFIG['work']) / 'apply.sh').encode()
        if expected not in info['args']:
            raise ValueError('Owned apply process changed executable arguments')
        if info['group'] != pid or info['session'] != pid:
            raise ValueError('Owned apply changed its setsid process group')
    if from_apply is not None:
        # helper -> rollback shell -> apply shell (its EXIT trap).
        caller = process(os.getppid())
        if from_apply != pid or not info or not caller or caller['parent'] != pid:
            raise ValueError('Invalid apply-trap rollback ancestry')
        return
    # Linux keeps a live process-group/session ID reserved after its leader dies.
    # Match both IDs, not only the leader, so orphaned ip/netdev children cannot
    # resume after rollback. A changed live leader identity above is never killed.
    if os.getpgrp() == pid:
        raise ValueError('Refuse to stop the rollback process group')
    if not members(owner):
        return
    try:
        os.killpg(pid, signal.SIGSTOP)
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if not members(owner):
            return
        time.sleep(.02)
    raise ValueError('Owned apply group did not stop before rollback')


if __name__ == '__main__':
    try:
        if CONFIG is None:
            raise ValueError('Unconfigured template')
        if sys.argv[1] == 'claim':
            claim(int(sys.argv[2]))
        elif sys.argv[1] == 'quiesce':
            quiesce(int(sys.argv[2]) if len(sys.argv) == 3 else None)
        else:
            raise ValueError('Unknown lifecycle action')
    except Exception as error:
        print(type(error).__name__ + ': ' + str(error), file=sys.stderr)
        raise SystemExit(1)
