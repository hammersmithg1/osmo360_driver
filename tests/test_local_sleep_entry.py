"""Pure entry-proof transactions; no USB, sockets, native input or worker."""

from dataclasses import replace
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

from osmo360_camera import _app
from osmo360_camera import _local_sleep_entry as m
from osmo360_camera._standby_control import StandbyControl, _EntryProof


PRESENT = bytes.fromhex("12004400012300415050000000000002")
ABSENT = bytes.fromhex("1200550000")
NONCE = "aabbccddeeff"


class Environment:
    def __init__(self):
        self.now = 100.0
        self.transport = SimpleNamespace()
        self.camera = SimpleNamespace(
            _control_lock=threading.RLock(), _transport=self.transport, _imu=None
        )
        self.lease = SimpleNamespace(
            nonce=bytes.fromhex("ab" * 16), healthy=True, last_ack_monotonic=99.8
        )
        self.thread = SimpleNamespace(is_alive=lambda: False)
        self.link = SimpleNamespace(
            stopped=SimpleNamespace(is_set=lambda: True),
            _sock=SimpleNamespace(fileno=lambda: -1),
        )
        self.stream = SimpleNamespace(_thread=self.thread, _link=self.link)
        self.preview = SimpleNamespace(
            _stream=self.stream,
            _lease=self.lease,
            _plan={
                "work_dir": "/dev/shm/osmo-ncm-owned",
                "lease": {"nonce": "ab" * 16},
            },
        )
        self.camera._usb_preview = self.preview
        receiver_done = threading.Event()
        receiver_done.set()
        self.imu = SimpleNamespace(
            pid=9876,
            _process=SimpleNamespace(pid=9876, poll=lambda: 0),
            stats={"collector_returncode": 0},
            report={"restored": True, "errors": []},
            _closed=True,
            _receiver_done=receiver_done,
            _threads=[SimpleNamespace(is_alive=lambda: False)],
            _socket=SimpleNamespace(fileno=lambda: -1),
        )
        self.before, self.after = map(_app.decode_query_reply, (PRESENT, ABSENT))
        self.proof = m.LocalSleepEntryProof(
            self.camera,
            self.preview,
            self.stream,
            self.lease,
            self.imu,
            99.5,
            0,
            1,
            True,
            True,
            0,
            True,
            True,
            True,
            True,
            99.5,
            "ab" * 16,
            "/dev/shm/osmo-ncm-owned",
            _app.query_payload(),
            self.before,
            self.after,
        )

    def control(self):
        session = SimpleNamespace(
            transport=self.transport,
            reuse_owned_id=True,
            owned_task_id=None,
            quarantined=False,
        )
        self.deliveries = []

        def deliver(command, **options):
            if options.get("pre_send"):
                options["pre_send"]()
            self.deliveries.append(command)

        self.bootstrap = SimpleNamespace(
            transport=self.transport,
            session=session,
            run_checked=Mock(side_effect=deliver),
        )
        control = StandbyControl(
            self.bootstrap,
            source_provider=lambda: {},
            control_lock=self.camera._control_lock,
            clock=lambda: self.now,
            nonce=NONCE,
        )
        control._snapshot = Mock(return_value={"state": "AWAKE", "last_sequence": 0})
        control._resolve_pending = Mock()
        return control


class LocalSleepProofTests(unittest.TestCase):
    def test_valid_local_only_proof_uses_fixed_native_sleep_once_under_same_lock(self):
        env = Environment()
        control = env.control()
        seen = []

        def run(command, **kwargs):
            if kwargs.get("pre_send"):
                kwargs["pre_send"]()
            seen.append((command, env.camera._control_lock._is_owned()))

        env.bootstrap.run_checked.side_effect = run
        result = m.submit_local_sleep(control, env.proof)
        self.assertEqual(
            result,
            {
                "command_id": "0000000000000001",
                "submitted": True,
                "input_completion_proven": False,
            },
        )
        self.assertEqual(len(seen), 1)
        self.assertTrue(seen[0][1])
        self.assertEqual(
            seen[0][0],
            "python3 /dev/shm/osmo-usb-standby-aabbccddeeff/usb_standby_owner.py "
            "sleep --command-id 0000000000000001 --record0 --battery-present1 --activation-state1 "
            "--streams-closed --own-app-absent",
        )
        self.assertNotIsInstance(env.proof, _EntryProof)
        self.assertFalse(hasattr(env.proof, "preview_restored"))
        self.assertEqual(control.pending_command, "0000000000000001")

    def test_legacy_restored_preview_proof_and_subclass_are_not_accepted_as_local_proof(
        self,
    ):
        env = Environment()
        control = env.control()
        legacy = _EntryProof(
            99.5, 0, True, 1, True, True, _app.query_payload(), env.before, env.after
        )

        class Subclass(m.LocalSleepEntryProof):
            pass

        for value in (
            legacy,
            SimpleNamespace(validate=lambda now: None),
            Subclass(**env.proof.__dict__),
        ):
            with self.subTest(value=type(value).__name__), self.assertRaises(TypeError):
                m.submit_local_sleep(control, value)
        control._snapshot.assert_not_called()
        env.bootstrap.run_checked.assert_not_called()

    def test_native_and_mask_exit_facts_must_be_exact_before_owner_status_or_input(
        self,
    ):
        env = Environment()
        control = env.control()
        for field, value in (
            ("recording_state", 1),
            ("recording_state", False),
            ("activation_state", 0),
            ("activation_state", True),
            ("battery_present", 1),
            ("imu_mask_restored", False),
            ("collector_returncode", 1),
            ("collector_returncode", False),
            ("video_thread_stopped", False),
            ("datalink_socket_closed", False),
            ("lease_owned", False),
            ("lease_healthy", False),
        ):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                m.submit_local_sleep(control, replace(env.proof, **{field: value}))
        control._snapshot.assert_not_called()
        env.bootstrap.run_checked.assert_not_called()

    def test_proof_and_lease_age_native_nonce_work_and_app_absence_are_correlated(self):
        env = Environment()
        control = env.control()
        for field, value in (
            ("verified_at", 94.99),
            ("verified_at", 100.01),
            ("lease_ack_at", 96.0),
            ("lease_ack_at", 100.1),
            ("lease_nonce", "00"),
            ("lease_work", "/tmp/not-owned"),
            ("query", b"other"),
            ("app_before", env.after),
            ("app_after", env.before),
        ):
            with self.subTest(field=field), self.assertRaises(ValueError):
                m.submit_local_sleep(control, replace(env.proof, **{field: value}))
        control._snapshot.assert_not_called()
        env.bootstrap.run_checked.assert_not_called()

    def test_replaced_camera_preview_stream_or_lease_identity_blocks_native_input(self):
        mutations = (
            lambda env: setattr(env.camera, "_usb_preview", object()),
            lambda env: setattr(env.camera, "_imu", object()),
            lambda env: setattr(env.preview, "_stream", object()),
            lambda env: setattr(env.preview, "_lease", object()),
            lambda env: env.preview._plan.update(work_dir="/dev/shm/osmo-ncm-other"),
            lambda env: env.preview._plan["lease"].update(nonce="cd" * 16),
            lambda env: setattr(env.lease, "nonce", bytes.fromhex("cd" * 16)),
            lambda env: setattr(env.lease, "healthy", False),
        )
        for mutate in mutations:
            env = Environment()
            control = env.control()
            mutate(env)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                m.submit_local_sleep(control, env.proof)
            env.bootstrap.run_checked.assert_not_called()

    def test_current_closed_imu_and_video_socket_are_rechecked_not_just_saved_booleans(
        self,
    ):
        mutations = (
            lambda env: setattr(env.imu._process, "poll", lambda: None),
            lambda env: setattr(env.imu._process, "poll", lambda: False),
            lambda env: setattr(env.imu._process, "poll", lambda: 1),
            lambda env: env.imu.stats.update(collector_returncode=1),
            lambda env: env.imu.stats.update(collector_returncode=False),
            lambda env: env.imu.report.update(restored=False),
            lambda env: env.imu.report.update(errors=["mask unknown"]),
            lambda env: setattr(env.imu, "_closed", False),
            lambda env: env.imu._receiver_done.clear(),
            lambda env: setattr(env.imu._threads[0], "is_alive", lambda: True),
            lambda env: setattr(env.imu._socket, "fileno", lambda: 10),
            lambda env: setattr(env.thread, "is_alive", lambda: True),
            lambda env: setattr(env.link.stopped, "is_set", lambda: False),
            lambda env: setattr(env.link._sock, "fileno", lambda: 9),
            lambda env: setattr(env.lease, "last_ack_monotonic", 95.0),
            lambda env: setattr(env.lease, "last_ack_monotonic", 100.01),
        )
        for mutate in mutations:
            env = Environment()
            control = env.control()
            mutate(env)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                m.submit_local_sleep(control, env.proof)
            env.bootstrap.run_checked.assert_not_called()

    def test_wrong_shared_lock_or_transport_is_rejected_without_status_or_input(self):
        for field in ("lock", "transport"):
            env = Environment()
            control = env.control()
            if field == "lock":
                control.lock = threading.RLock()
            else:
                control.bootstrap.transport = object()
            with self.subTest(field=field), self.assertRaises(ValueError):
                m.submit_local_sleep(control, env.proof)
            control._snapshot.assert_not_called()
            env.bootstrap.run_checked.assert_not_called()

    def test_owner_status_can_stale_proof_and_no_sleep_start_follows(self):
        env = Environment()
        control = env.control()

        def late_status():
            env.now = 106.0
            return {"state": "AWAKE", "last_sequence": 0}

        control._snapshot.side_effect = late_status
        with self.assertRaises(ValueError):
            m.submit_local_sleep(control, env.proof)
        control._snapshot.assert_called_once()
        env.bootstrap.run_checked.assert_not_called()
        self.assertIsNone(control.pending_command)

    def test_ownership_changed_during_old_task_resolution_fails_immediately_before_start(
        self,
    ):
        env = Environment()
        control = env.control()
        control._resolve_pending.side_effect = lambda: setattr(
            env.camera, "_imu", object()
        )
        with self.assertRaises(ValueError):
            m.submit_local_sleep(control, env.proof)
        control._snapshot.assert_called_once()
        control._resolve_pending.assert_called_once()
        env.bootstrap.run_checked.assert_called_once()
        self.assertEqual(env.deliveries, [])
        self.assertIsNone(control.pending_command)

    def test_newer_current_heartbeat_is_allowed_without_requiring_old_ack_equality(
        self,
    ):
        env = Environment()
        control = env.control()
        env.lease.last_ack_monotonic = 99.99
        m.submit_local_sleep(control, env.proof)
        env.bootstrap.run_checked.assert_called_once()

    def test_actual_imu_public_properties_keep_terminated_pid_but_closed_collector_is_accepted(
        self,
    ):
        from osmo360_camera.imu import IMUStream

        env = Environment()
        control = env.control()
        # Actual IMUStream properties deliberately retain the exited Popen.
        # No start(), subprocess, UDP socket, native mask or file is used.
        actual = IMUStream()
        actual._process = SimpleNamespace(pid=12345, poll=lambda: 0)
        actual._closed = actual._started = actual._stop_requested = True
        actual._report = {"restored": True, "errors": []}
        actual._receiver_done.set()
        actual._threads = [SimpleNamespace(is_alive=lambda: False)]
        actual._socket = SimpleNamespace(fileno=lambda: -1)
        self.assertEqual(actual.pid, 12345)
        self.assertEqual(actual.stats["collector_returncode"], 0)
        m.submit_local_sleep(control, replace(env.proof, closed_imu=actual))
        env.bootstrap.run_checked.assert_called_once()
        actual._process.poll = lambda: None
        with self.assertRaises(ValueError):
            m.submit_local_sleep(control, replace(env.proof, closed_imu=actual))
        env.bootstrap.run_checked.assert_called_once()  # No second native input.


if __name__ == "__main__":
    unittest.main()
