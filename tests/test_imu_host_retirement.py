"""Owned collector retirement never executes an old native restore handler."""

from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera.imu import IMUStopError, IMUStream
from test_video_cleanup_classification import reboot_proof


class OwnedProcess:
    pid = 12345
    stdout = None

    def __init__(self, *, exits=True, code=None):
        self.exits, self.code = exits, code
        self.kill = Mock(side_effect=self._kill)
        self.send_signal = Mock(side_effect=AssertionError('No SIGINT/TERM restore handler'))

    def _kill(self):
        if self.exits:
            self.code = -signal.SIGKILL

    def poll(self):
        return self.code

    def wait(self, *, timeout):
        if self.code is None:
            raise subprocess.TimeoutExpired('owned collector', timeout)
        return self.code


def owner(process=None):
    source = IMUStream()
    source._process = source._owned_process = process
    source._started = process is not None
    return source


class IMUHostQuiescenceTests(unittest.TestCase):
    def test_kills_only_exact_owned_process_and_never_restores_native_mask(self):
        process = OwnedProcess()
        source = owner(process)
        source._socket = SimpleNamespace(close=Mock())
        worker = SimpleNamespace(join=Mock(), is_alive=lambda: False)
        source._threads = [worker]
        receipt = source.quiesce_local(timeout=.5)
        process.kill.assert_called_once_with()
        process.send_signal.assert_not_called()
        source._socket.close.assert_called_once_with()
        self.assertTrue(receipt['collector_exited'])
        self.assertTrue(receipt['workers_stopped'])
        self.assertTrue(receipt['kill_sent'])
        self.assertFalse(receipt['mask_restored'])
        self.assertFalse(receipt['native_cleanup_replayed'])
        self.assertEqual(receipt['collector_returncode'], -signal.SIGKILL)
        self.assertFalse(source._closed)
        self.assertTrue(source._receiver_done.is_set())
        self.assertEqual(source.stats['host_quiescence'], receipt)
        with self.assertRaisesRegex(IMUStopError, 'normal restoration'):
            source.stop()
        process.send_signal.assert_not_called()

    def test_already_exited_or_unstarted_collector_is_not_signaled(self):
        for process in (None, OwnedProcess(code=1), OwnedProcess(code=0)):
            with self.subTest(process=process):
                source = owner(process)
                report = source.quiesce_local(timeout=.5)
                self.assertTrue(report['collector_exited'])
                self.assertFalse(report['mask_restored'])
                self.assertFalse(report['kill_sent'])
                if process is not None:
                    process.kill.assert_not_called()
                    process.send_signal.assert_not_called()

    def test_process_ownership_change_blocks_every_signal_and_socket_close(self):
        source = owner(OwnedProcess())
        foreign = source._process = OwnedProcess()
        source._socket = SimpleNamespace(close=Mock())
        with self.assertRaisesRegex(IMUStopError, 'ownership changed'):
            source.quiesce_local(timeout=.5)
        foreign.kill.assert_not_called()
        source._owned_process.kill.assert_not_called()
        source._socket.close.assert_not_called()

    def test_collector_wait_timeout_never_claims_exit_or_closes_receiver(self):
        process = OwnedProcess(exits=False)
        source = owner(process)
        source._socket = SimpleNamespace(close=Mock())
        with self.assertRaisesRegex(IMUStopError, 'did not exit'):
            source.quiesce_local(timeout=.5)
        self.assertFalse(source.stats['host_quiescence']['collector_exited'])
        self.assertFalse(source.stats['host_quiescence']['mask_restored'])
        source._socket.close.assert_not_called()
        process.send_signal.assert_not_called()

    def test_live_host_thread_blocks_success_even_after_process_exit(self):
        source = owner(OwnedProcess(code=1))
        source._threads = [SimpleNamespace(join=Mock(), is_alive=lambda: True)]
        with self.assertRaisesRegex(IMUStopError, 'host worker did not stop'):
            source.quiesce_local(timeout=.5)
        report = source.stats['host_quiescence']
        self.assertTrue(report['collector_exited'])
        self.assertFalse(report['workers_stopped'])
        self.assertFalse(report['mask_restored'])

    def test_invalid_timeout_cannot_signal_owned_process(self):
        process = OwnedProcess()
        source = owner(process)
        for timeout in (0, -1, True, float('nan'), float('inf')):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                source.quiesce_local(timeout=timeout)
        process.kill.assert_not_called()
        process.send_signal.assert_not_called()

    def test_inflight_receiver_or_normal_stop_signal_is_serialized_before_host_kill(self):
        for mode in ('receiver', 'normal-stop'):
            with self.subTest(mode=mode):
                process = OwnedProcess()
                source = owner(process)
                poll_entered, release_poll, quiesce_waiting = (threading.Event() for _ in range(3))
                events, results, failures = [], [], []
                lock = threading.Lock()

                class SignalLock:
                    def acquire(self, *args, **kwargs):
                        if threading.current_thread().name == 'host-quiesce':
                            quiesce_waiting.set()
                        return lock.acquire(*args, **kwargs)

                    def release(self):
                        lock.release()

                    def __enter__(self):
                        self.acquire()
                        return self

                    def __exit__(self, *_):
                        self.release()

                source._signal_lock = SignalLock()
                original_poll = process.poll

                def gated_poll():
                    if threading.current_thread().name == 'inflight-normal' and not poll_entered.is_set():
                        old_value = original_poll()
                        poll_entered.set()
                        if not release_poll.wait(2):
                            raise AssertionError('Test did not release the in-flight normal signal')
                        return old_value
                    return original_poll()

                process.poll = gated_poll
                process.send_signal = Mock(side_effect=lambda number: events.append(('normal', number)))
                process.kill.side_effect = lambda: (events.append(('host', signal.SIGKILL)), process._kill())
                source._socket = SimpleNamespace(recv=Mock(side_effect=OSError('local UDP error')), close=Mock())

                def normal():
                    try:
                        source._receive() if mode == 'receiver' else source.stop(timeout=.5)
                    except IMUStopError:
                        pass  # A concurrent explicit host kill is not a normal mask-restoring stop.
                    except BaseException as error:
                        failures.append(error)

                def quiesce():
                    try:
                        results.append(source.quiesce_local(timeout=1))
                    except BaseException as error:
                        failures.append(error)

                normal_thread = threading.Thread(target=normal, name='inflight-normal')
                quiesce_thread = threading.Thread(target=quiesce, name='host-quiesce')
                if mode == 'receiver':
                    source._threads = [normal_thread]
                normal_thread.start()
                try:
                    self.assertTrue(poll_entered.wait(2))
                    quiesce_thread.start()
                    self.assertTrue(quiesce_waiting.wait(2))
                    self.assertFalse(source._host_quiescence_selected)
                    process.kill.assert_not_called()
                    release_poll.set()
                    normal_thread.join(2)
                    quiesce_thread.join(2)
                    self.assertFalse(normal_thread.is_alive())
                    self.assertFalse(quiesce_thread.is_alive())
                    self.assertEqual(failures, [])
                    self.assertEqual(events, [('normal', signal.SIGINT), ('host', signal.SIGKILL)])
                    self.assertTrue(results[0]['collector_exited'])
                    self.assertTrue(results[0]['workers_stopped'])
                    self.assertFalse(results[0]['mask_restored'])
                    self.assertTrue(source._host_quiescence_selected)
                finally:
                    release_poll.set()
                    normal_thread.join(2)
                    if quiesce_thread.ident is not None:
                        quiesce_thread.join(2)

    @unittest.skipUnless(sys.platform in ('linux', 'darwin'), 'requires POSIX SIGKILL')
    def test_real_private_child_cannot_run_signal_handler_or_finally(self):
        with tempfile.TemporaryDirectory() as folder:
            signal_marker = Path(folder) / 'restore-handler'
            finally_marker = Path(folder) / 'restore-finally'
            program = (
                'import signal,time\n'
                'from pathlib import Path\n'
                f'marker=Path({str(signal_marker)!r})\n'
                'def handler(*args):\n'
                ' marker.write_text("native restore would run")\n'
                ' raise SystemExit(1)\n'
                'signal.signal(signal.SIGINT,handler)\n'
                'signal.signal(signal.SIGTERM,handler)\n'
                'try:\n'
                ' print("ready",flush=True)\n'
                ' while True: time.sleep(1)\n'
                'finally:\n'
                f' Path({str(finally_marker)!r}).write_text("finally restore would run")\n'
            )
            process = subprocess.Popen([sys.executable, '-u', '-c', program],
                                       stdout=subprocess.PIPE, text=True, start_new_session=True)
            try:
                readable, _, _ = select.select([process.stdout], [], [], 2)
                self.assertTrue(readable, 'Test child did not reach its signal handlers')
                self.assertEqual(process.stdout.readline().strip(), 'ready')
                source = owner(process)
                receipt = source.quiesce_local(timeout=2)
                self.assertEqual(process.returncode, -signal.SIGKILL)
                self.assertTrue(receipt['collector_exited'])
                self.assertFalse(receipt['mask_restored'])
                self.assertFalse(signal_marker.exists())
                self.assertFalse(finally_marker.exists())
                self.assertTrue(process.stdout.closed)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=2)
                process.stdout.close()


class IMUVerifiedRebootRetirementTests(unittest.TestCase):
    def test_verified_other_boot_retires_only_local_state_and_keeps_owner_marker(self):
        process = OwnedProcess(code=-signal.SIGKILL)
        source = owner(process)
        proof = reboot_proof(imu=source)
        proof.camera._imu = source
        source._socket = SimpleNamespace(close=Mock())
        receipt = source.retire_after_reboot(proof, timeout=.5)
        self.assertTrue(source._closed)
        self.assertTrue(receipt['old_mask_obsolete_after_verified_reboot'])
        self.assertFalse(receipt['mask_restored'])
        self.assertEqual(receipt['boot_id_before'], proof.boot_id_before)
        self.assertEqual(receipt['boot_id_after'], proof.boot_id_after)
        self.assertIs(proof.camera._imu, source)
        process.kill.assert_not_called()
        process.send_signal.assert_not_called()
        receipt['mask_restored'] = True
        self.assertFalse(source.retire_after_reboot(proof, timeout=.5)['mask_restored'])
        self.assertEqual(source._socket.close.call_count, 1)

    def test_postboot_retirement_refuses_live_collector_without_killing_it(self):
        process = OwnedProcess()
        source = owner(process)
        with self.assertRaisesRegex(IMUStopError, 'live or ownership changed'):
            source.retire_after_reboot(reboot_proof(imu=source), timeout=.5)
        process.kill.assert_not_called()
        process.send_signal.assert_not_called()
        self.assertFalse(source._closed)

    def test_missing_foreign_or_changed_reader_proof_never_releases_resources(self):
        source = owner(OwnedProcess(code=1))
        source._socket = SimpleNamespace(close=Mock())
        for proof in (object(), reboot_proof()):
            with self.subTest(proof=type(proof).__name__), self.assertRaises(IMUStopError):
                source.retire_after_reboot(proof, timeout=.5)
        proof = reboot_proof(imu=source)
        proof.transport._thread = object()
        with self.assertRaises(RuntimeError):
            source.retire_after_reboot(proof, timeout=.5)
        source._socket.close.assert_not_called()
        source._owned_process.kill.assert_not_called()
        self.assertFalse(source._closed)


if __name__ == '__main__':
    unittest.main()
