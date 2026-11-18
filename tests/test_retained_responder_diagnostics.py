"""Preserve the exact native responder failure without changing recovery gates."""

from contextlib import ExitStack
import errno
import json
import os
import socket
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_retained_completion_lease as fixture

m = fixture.m


class ResponderDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        # Reuse the existing valid immutable reset/lease fixture; its tests are
        # not rerun or weakened for these diagnostic-only paths.
        self.f = fixture.CompletionLeaseTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.work, self.cfg = self.f.work, dict(self.f.cfg)
        self.cfg.update(sleep_command_id='0000000000000001',
                        standby_work=str(self.work / 'standby'))
        self.f.write(self.work / 'guard.json', self.f.guard)
        self.f.write(self.work / 'result.json',
            {key: value for key, value in self.f.done.items() if key != 'closed'})
        self.f.write(self.f.lease / 'lease.state', self.f.state)

    def environment(self, *, address=None):
        base = self.f.base()
        native = SimpleNamespace(alive=lambda owner: True,
            proc_token=lambda: dict(pid=66, start='456'))
        stack = ExitStack()
        stack.enter_context(patch.object(m, 'modules',
            return_value=(SimpleNamespace(base=base), None, native)))
        stack.enter_context(patch.object(m, 'boot', return_value=150.0))
        stack.enter_context(patch.object(m.time, 'monotonic', return_value=105.0))
        stack.enter_context(patch.object(m.signal, 'signal', return_value='old-handler'))
        stack.enter_context(patch.object(m.signal, 'setitimer'))
        stack.enter_context(patch.object(m, 'attribute', side_effect=lambda path:
            (address or self.cfg['dev_mac']) if path.name == 'address' else '12'))
        stack.enter_context(patch.object(m, 'ipv4', return_value=self.cfg['camera_ip'] + '/30'))
        return stack

    def receipt(self):
        return json.loads((self.work / 'serve-error.json').read_text())

    def test_real_busy_udp_port_is_reported_and_exported_before_readiness(self):
        native_socket = socket.socket
        test = self

        class DeviceSocket:
            """Only the privileged device bind is mocked; UDP port bind is real."""
            def __init__(self, *args): self.sock = native_socket(*args)
            def __enter__(self): return self
            def __exit__(self, *args): self.sock.close()
            def setsockopt(self, level, option, value):
                test.assertEqual((level, option, value), (socket.SOL_SOCKET,
                    getattr(socket, 'SO_BINDTODEVICE', 25), b'usb0\0'))
            def bind(self, target): return self.sock.bind(target)

        with native_socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('127.0.0.1', 0))
            self.cfg.update(camera_ip='127.0.0.1', completion_port=occupied.getsockname()[1])
            with self.environment(), patch.object(m.socket, 'socket', DeviceSocket):
                with self.assertRaises(OSError) as caught:
                    m.completion_server(self.work, self.cfg)
        self.assertEqual(caught.exception.errno, errno.EADDRINUSE)
        value = self.receipt()
        self.assertEqual((value['phase'], value['error_type'], value['errno']),
                         ('bind_port', 'OSError', errno.EADDRINUSE))
        self.assertEqual(value['completion_port'], self.cfg['completion_port'])
        self.assertLessEqual((self.work / 'serve-error.json').stat().st_size, 2048)
        self.assertFalse((self.work / 'serve-ready.json').exists())

        # Native files are root-owned. Adapt only that fixture property on a
        # non-root test host; the actual private descriptor/bound checks run.
        actual_fstat = m.os.fstat
        def root_owned(fd):
            status = actual_fstat(fd)
            return SimpleNamespace(st_uid=0, st_mode=status.st_mode, st_size=status.st_size)
        error = RuntimeError('child exited before readiness')
        error.responder_returncode = 1
        with patch.object(m.os, 'fstat', side_effect=root_owned):
            exported = m.failure_payload(self.work, self.cfg, self.f.base(),
                'supervisor', error, 'supervisor-recovery')
        saved = json.loads(exported['snapshots']['responder_error']['text'])
        self.assertEqual((saved['phase'], saved['errno']), ('bind_port', errno.EADDRINUSE))
        self.assertEqual(exported['responder_returncode'], 1)
        self.assertLessEqual(len(m.encoded(exported)), m.MAX_FAILURE)

    def test_changed_identity_is_distinguished_from_a_bind_error(self):
        with self.environment(address='02:ff:ff:ff:ff:ff'), patch.object(m.socket, 'socket') as create:
            with self.assertRaisesRegex(ValueError, 'USB identity changed'):
                m.completion_server(self.work, self.cfg)
        create.assert_not_called()
        value = self.receipt()
        self.assertEqual((value['phase'], value['error_type'], value['errno']),
                         ('current', 'ValueError', None))
        self.assertFalse((self.work / 'serve-ready.json').exists())

    def test_diagnostic_failure_preserves_primary_exception_and_alarm_cleanup(self):
        original = OSError(errno.EADDRINUSE, 'existing socket')
        with (
            self.environment(),
            patch.object(m, '_completion_server', side_effect=original),
            patch.object(m, 'responder_error', side_effect=OSError(errno.ENOSPC, 'RAM full')),
            patch.object(m.signal, 'setitimer') as timers,
        ):
            with self.assertRaises(OSError) as caught:
                m.completion_server(self.work, self.cfg)
        self.assertIs(caught.exception, original)
        self.assertEqual(timers.call_args_list[-1].args, (m.signal.ITIMER_REAL, 0))

    def test_first_error_is_exclusive_bounded_and_never_follows_a_symlink(self):
        timer = patch.object(m, 'boot', return_value=150.0)
        timer.start()
        self.addCleanup(timer.stop)
        m.responder_error(self.work, self.cfg, 'current', ValueError('first'))
        before = (self.work / 'serve-error.json').read_bytes()
        with self.assertRaises(FileExistsError):
            m.responder_error(self.work, self.cfg, 'bind_port', OSError('second'))
        self.assertEqual((self.work / 'serve-error.json').read_bytes(), before)
        (self.work / 'serve-error.json').unlink()
        target = self.work / 'unrelated'
        target.write_bytes(b'keep')
        (self.work / 'serve-error.json').symlink_to(target)
        with self.assertRaises(FileExistsError):
            m.responder_error(self.work, self.cfg, 'bind_port', OSError('second'))
        self.assertEqual(target.read_bytes(), b'keep')

    def test_child_exit_and_original_readiness_deadline_remain_distinct(self):
        exited = SimpleNamespace(poll=lambda: 1)
        with patch.object(m, 'boot', return_value=150.0):
            with self.assertRaisesRegex(RuntimeError, 'returncode=1') as caught:
                m.responder_startup_check(exited, 151.0)
        self.assertEqual(caught.exception.responder_returncode, 1)
        live = SimpleNamespace(poll=lambda: None)
        with patch.object(m, 'boot', return_value=151.0):
            with self.assertRaisesRegex(TimeoutError, 'readiness deadline') as caught:
                m.responder_startup_check(live, 151.0)
        self.assertFalse(hasattr(caught.exception, 'responder_returncode'))


if __name__ == '__main__':
    unittest.main()
