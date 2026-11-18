"""A completion port is one kernel reservation across reset/child handoff."""

from copy import deepcopy
import errno
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_retained_completion_lease as fixture

m = fixture.m


@unittest.skipUnless(sys.platform == 'linux', 'Linux SO_BINDTODEVICE/FD ownership')
class EndpointReservationTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture.CompletionLeaseTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.work, self.cfg = self.f.work, dict(self.f.cfg, camera_ip='127.0.0.1')
        self.owner = self.f.guard['owner']
        self.base = SimpleNamespace(proc_token=lambda: self.owner)
        self.pre = dict(ifname='lo', ifindex=socket.if_nametoindex('lo'))
        (self.work / 'endpoint.json').unlink()
        self.f.write(self.f.lease / 'lease.state', self.f.state)

    def reserve(self, work=None):
        sock, value = m.reserve_endpoint(work or self.work, self.cfg, self.f.base(), self.base, self.pre)
        self.addCleanup(sock.close)
        return sock, value

    def assert_port_held(self, value):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'lo\0')
            with self.assertRaises(OSError) as caught:
                probe.bind((self.cfg['camera_ip'], value['port']))
        self.assertEqual(caught.exception.errno, errno.EADDRINUSE)

    def test_many_live_previous_responders_cannot_collide_with_kernel_allocation(self):
        live, ports = [], set()
        for index in range(24):
            work = self.work / str(index)
            work.mkdir()
            sock, value = self.reserve(work)
            live.append(sock)
            self.assertNotIn(value['port'], ports)
            ports.add(value['port'])
            self.assert_port_held(value)
        self.assertEqual(len(ports), 24)

    def test_exact_open_socket_survives_parent_close_in_real_child(self):
        sock, value = self.reserve()
        self.assert_port_held(value)
        # Import only the pure FD verifier in the child. No native helper main,
        # command, RTC or USB path is executed by this real process handoff.
        program = '''import importlib.util,json,socket,sys
spec=importlib.util.spec_from_file_location('device',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
cfg,value=json.loads(sys.argv[2]);sock=socket.socket(fileno=value['fd'])
m.endpoint_socket(cfg,value,sock)
print('same-fd',flush=True)
sys.stdin.buffer.read(1)
sock.close()
'''
        process = subprocess.Popen([sys.executable, '-c', program, str(fixture.SOURCE),
            json.dumps([self.cfg, value])], pass_fds=(sock.fileno(),),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.addCleanup(lambda: process.poll() is None and process.kill())
        self.assertEqual(process.stdout.readline(), b'same-fd\n')
        sock.close()
        self.assert_port_held(value)  # The inherited FD is still the reservation.
        stdout, stderr = process.communicate(b'q', timeout=5)
        self.assertEqual((process.returncode, stdout, stderr), (0, b'', b''))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'lo\0')
            probe.bind((self.cfg['camera_ip'], value['port']))

    def test_foreign_socket_or_metadata_cannot_claim_the_endpoint(self):
        sock, value = self.reserve()
        for changes in (dict(nonce='00' * 6), dict(owner=dict(pid=99, start='1')),
                        dict(port=self.cfg['lease_port']), dict(fd=True)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                m.endpoint_metadata(self.cfg, dict(value, **changes), owner=self.owner, post=self.pre)
        with self.assertRaisesRegex(ValueError, 'USB identity'):
            m.endpoint_metadata(self.cfg, value, post=dict(self.pre, ifindex=self.pre['ifindex'] + 1))
        with self.assertRaisesRegex(ValueError, 'Same reserved UDP socket'):
            m.endpoint_socket(self.cfg, dict(value, socket_identity=[9, 99999]), sock)
        with self.assertRaisesRegex(ValueError, 'Same reserved UDP socket'):
            m.endpoint_socket(self.cfg, dict(value, ifname='foreign'), sock)

    def test_failed_private_receipt_write_releases_socket_without_starting_worker(self):
        real_socket, opened = socket.socket, []
        def create(*args, **kwargs):
            value = real_socket(*args, **kwargs)
            opened.append(value)
            return value
        b = self.f.base()
        b.write_new = Mock(side_effect=OSError(errno.ENOSPC, 'RAM full'))
        with patch.object(m.socket, 'socket', side_effect=create), patch.object(m, 'spawn') as spawn:
            with self.assertRaisesRegex(OSError, 'RAM full'):
                m.reserve_endpoint(self.work, self.cfg, b, self.base, self.pre)
        self.assertEqual(len(opened), 1)
        self.assertEqual(opened[0].fileno(), -1)
        spawn.assert_not_called()

    def test_spawn_passes_only_reserved_descriptor_with_close_fds(self):
        sock, value = self.reserve()
        with patch.object(m.subprocess, 'Popen') as popen:
            m.spawn(self.work, '_serve', pass_fds=(sock.fileno(),))
        self.assertEqual(popen.call_args.kwargs['pass_fds'], (value['fd'],))
        self.assertTrue(popen.call_args.kwargs['close_fds'])

    def test_worker_rejects_changed_reserved_native_identity_before_reset(self):
        sock, value = self.reserve()
        self.f.write(self.work / 'guard.json', self.f.guard)
        b = self.f.base()
        base = SimpleNamespace(alive=lambda token: True, proc_token=lambda: dict(pid=55, start='456'))
        owner = SimpleNamespace(replace=Mock())
        with (patch.object(m, 'modules', return_value=(SimpleNamespace(base=b), owner, base)),
              patch.object(m, 'source_state'), patch.object(m, 'sleep_arm_proof'),
              patch.object(m, 'snapshot', return_value=dict(self.pre, ifindex=self.pre['ifindex'] + 1)),
              patch.object(m, 'boot', return_value=150.0),
              patch.object(m, 'rebind_once') as reset,
              patch.object(m, 'export_failure') as exported):
            m.worker(self.work, self.cfg)
        reset.assert_not_called()
        self.assertFalse((self.work / 'armed.json').exists())
        self.assertIn('USB identity changed', json.loads((self.work / 'worker-error.json').read_text())['error'])
        self.assertEqual(exported.call_args.args[3], 'worker')
        self.assert_port_held(value)

    def test_pre_wake_endpoint_export_is_bounded_exclusive_and_identified(self):
        sock, value = self.reserve()
        ready = dict(nonce=self.cfg['nonce'], guard=self.owner,
            worker=dict(pid=55, start='456'), endpoint=value)
        target, real_open = self.work / 'export.json', os.open
        def open_export(path, flags, mode=0o777):
            self.assertEqual(path, Path('/blackbox/export_tmp/osmo_rebind_endpoint_' + self.cfg['nonce'] + '.json'))
            return real_open(target, flags, mode)
        with patch.object(m.os, 'open', side_effect=open_export):
            result = m.export_endpoint(self.work, self.cfg, self.f.base(), ready)
            with self.assertRaises(FileExistsError):
                m.export_endpoint(self.work, self.cfg, self.f.base(), ready)
        info = target.stat()
        self.assertEqual(result['export_identity'], dict(device=info.st_dev, inode=info.st_ino))
        self.assertEqual(json.loads(target.read_bytes()), result)
        self.assertLessEqual(info.st_size, 4096)
        target.unlink()
        unrelated = self.work / 'unrelated'
        unrelated.write_bytes(b'keep')
        target.symlink_to(unrelated)
        with patch.object(m.os, 'open', side_effect=open_export), self.assertRaises(FileExistsError):
            m.export_endpoint(self.work, self.cfg, self.f.base(), ready)
        self.assertEqual(unrelated.read_bytes(), b'keep')

    def test_supervisor_reserves_before_worker_and_holds_until_child_ready(self):
        b, events = self.f.base(), []
        original_write = b.write_new
        def write(path, value):
            events.append(path.name)
            original_write(path, value)
        b.write_new = write
        done = deepcopy(self.f.done)
        done.pop('closed')
        for name in ('pre', 'post'):
            done[name].update(self.pre)
        done['worker'] = dict(pid=55, start='456')
        self.f.write(self.work / 'armed.json', dict(nonce=self.cfg['nonce'], worker=done['worker']))
        self.f.write(self.work / 'result.json', done)
        f = self
        class Alarm:
            fd = 3
            def __init__(self, *args): pass
            def verify(self, *args): return dict(remaining=90)
            def cancel(self): return dict(closed_verified_ebadf=True)
            def close(self): pass
        base = SimpleNamespace(proc_token=lambda: self.owner, alive=lambda token: True, Alarm=Alarm)
        def spawn(work, mode, **options):
            endpoint = json.loads((work / 'endpoint.json').read_text())
            events.append(mode)
            f.assert_port_held(endpoint)
            f.assertEqual(options, dict(pass_fds=(endpoint['fd'],)))
            if mode == '_worker':
                return SimpleNamespace(pid=55, poll=lambda: None)
            b.write_new(work / 'serve-ready.json', dict(nonce=f.cfg['nonce'],
                process=dict(pid=66, start='567'), port=endpoint['port'],
                ifname='lo', socket_identity=endpoint['socket_identity']))
            return SimpleNamespace(pid=66, poll=lambda: None)
        with (patch.object(m, 'modules', return_value=(SimpleNamespace(base=b), None, base)),
              patch.object(m, 'sleep_arm_proof'), patch.object(m, 'snapshot', return_value=self.pre),
              patch.object(m, 'boot', return_value=150.0),
              patch.object(m.time, 'monotonic', return_value=105.0),
              patch.object(m.select, 'select', return_value=([], [], [])),
              patch.object(m, 'spawn', side_effect=spawn),
              patch.object(m, 'optional_done_export'),
              patch.object(m, 'recover_with_diagnostic') as recovery):
            m.supervise(self.work, self.cfg)
        recovery.assert_not_called()
        self.assertLess(events.index('endpoint.json'), events.index('_worker'))
        self.assertLess(events.index('_worker'), events.index('_serve'))
        self.assertTrue((self.work / 'done.json').exists())
        endpoint = json.loads((self.work / 'endpoint.json').read_text())
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'lo\0')
            probe.bind((self.cfg['camera_ip'], endpoint['port']))


if __name__ == '__main__':
    unittest.main()
