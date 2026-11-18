"""Prepared USB ownership/cache/freshness with fake control/network/data only."""

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _prepared_stream as m
from osmo360_camera._usb_bootstrap import USBRuntime, FUNCTIONS
from osmo360_camera._usb_plan import build_usb_plan
from osmo360_camera.video import VideoFrame, VideoStream
from osmo360_camera.vendor.preview_metadata import records_from_video

OWNER, SERVICE = dict(pid=4001, start="100"), dict(pid=801, start="10")
RECEIPT = dict(
    work="/dev/shm/osmo-usb-standby-abcdef012345",
    nonce="abcdef012345",
    owner=OWNER,
    service=SERVICE,
)
FILES = {name: ("# exact offline " + name + "\n").encode() for name in m.SOURCES}


def runtime():
    return USBRuntime(
        "a600000.ssusb",
        "0x2ca3",
        "0x0025",
        "BULK",
        tuple("../../functions/" + name for name in FUNCTIONS),
        "super-speed-plus",
        "96:03:ae:cc:f8:36",
        "96:03:ae:cc:f8:37",
        "usb0",
        "0x1002",
        False,
        "usb0",
        (("lo", "127.0.0.1/8"),),
    )


class Control:
    def __init__(self, camera, events):
        self.lock, self.events = camera._control_lock, events
        self.work, self.nonce = RECEIPT["work"], RECEIPT["nonce"]
        self.owner, self.service = deepcopy(OWNER), deepcopy(SERVICE)
        self.state, self.sequence, self.commands, self.failure = "AWAKE", 0, [], None
        self.pending_task, self._unknown_task = None, False
        self.bootstrap = types.SimpleNamespace(
            transport=camera._transport,
            session=types.SimpleNamespace(quarantined=False),
            query_runtime=Mock(return_value=runtime()),
        )

    def snapshot(self):
        self.events.append("control_snapshot")
        return dict(
            state=self.state,
            last_sequence=self.sequence,
            command_id=f"{self.sequence:016x}",
        )

    def _run(self, command, **options):
        self.events.append("control_start")
        if options.get("pre_send"):
            options["pre_send"]()
        self.commands.append(command)
        if self.failure:
            self.failure(command)
        return dict(success=True)

    def resolve_pending(self):
        self.events.append("resolve_pending")

    def _remember_quarantine(self):
        self.events.append("remember_quarantine")


class Network:
    def __init__(self, events):
        self.events, self.cleanups = events, 0

    def preflight(self):
        self.events.append("host_preflight")
        return {"original": "host"}

    def wait_ready(self, baseline, mac, **options):
        self.events.append("network_ready")
        return dict(ok=True, ncm_interface="usb9")

    def cleanup_owned(self):
        self.cleanups += 1


class IMU:
    def __init__(self, events, **options):
        self.events, self.options, self.pid, self.report = events, options, None, None
        self.stats, self.stop_error = dict(collector_returncode=None), None

    def start(self):
        self.events.append("imu_start")
        self.pid = 321
        return self

    def stop(self):
        self.events.append("imu_stop")
        if self.stop_error:
            raise self.stop_error
        if self.pid is not None:
            self.report = dict(restored=True, quiescent=True, errors=[])
            self.stats["collector_returncode"] = 0
        return self.report


class Preview:
    def __init__(self, camera, events, **options):
        self.camera, self.events = camera, events
        self._lease = self._stream = None
        self._started = self._scheduled = self._closed = False
        self.report = dict(restored=False, errors=[])

    def stop(self):
        self.events.append("preview_stop")
        if self._stream:
            self._stream.stop()
        if self.camera._imu:
            self.camera._imu.stop()
            self.camera._imu = None
        if self._lease:
            self._lease.stop()
        self.report["restored"] = True
        self._closed = True
        self.camera._usb_preview = None
        return self.report


class Lease:
    def __init__(self, events, config):
        self.events, self.config, self.last_ack, self.healthy = (
            events,
            config,
            None,
            False,
        )

    def start(self, **options):
        self.events.append("lease_ack")
        self.last_ack, self.healthy = dict(sequence=1, state=1), True
        return self

    def stop(self, *, close_camera=True):
        self.events.append("lease_close" if close_camera else "lease_local_close")
        self.healthy = False
        return dict(closed=True)


class Video:
    def __init__(self, events, **options):
        self.events, self.options = events, options

    def start(self):
        self.events.append("video_start")
        return self

    def stop(self):
        self.events.append("video_stop")


class PreparedTests(unittest.TestCase):
    def setUp(self):
        self.patch = patch.dict(
            m.SOURCES,
            {name: hashlib.sha256(data).hexdigest() for name, data in FILES.items()},
            clear=True,
        )
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.events = []
        self.camera = types.SimpleNamespace(
            _control_lock=threading.RLock(),
            _transport=object(),
            _imu=None,
            _usb_preview=None,
        )
        self.control = Control(self.camera, self.events)
        self.network = Network(self.events)
        self.assets = m.PreparedAssets(
            self.control, RECEIPT, lambda: dict(FILES), nonce="012345abcdef"
        )
        self.counter = 0

    def backend(self, **options):
        def build(value):
            self.counter += 1
            return build_usb_plan(
                value, tag="offline-" + str(self.counter), nonce="ab" * 16, port=51337
            )

        return m.PreparedStream(
            self.camera,
            self.control,
            owner_receipt=RECEIPT,
            sleep_id=options.pop("sleep_id", "0000000000000001"),
            wake_id=options.pop("wake_id", "0000000000000002"),
            source_provider=lambda: dict(FILES),
            plan_builder=options.pop("plan_builder", build),
            assets=self.assets,
            network_guard=self.network,
            preview_factory=lambda camera, **kw: Preview(camera, self.events, **kw),
            imu_factory=lambda **kw: IMU(self.events, **kw),
            lease_factory=options.pop(
                "lease_factory", lambda config: Lease(self.events, config)
            ),
            video_factory=lambda **kw: Video(self.events, **kw),
            **options,
        )

    def armed(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = "SLEEP", 1
        value.arm_watcher()
        return value

    def test_prepare_stages_all_assets_plan_and_manifest_without_data_or_camera_adoption(
        self,
    ):
        value = self.backend().prepare()
        self.assertTrue(value.prepared)
        self.assertFalse(value.possible_future_apply)
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)
        self.assertFalse(value.preview._scheduled)
        self.assertIsNone(value.imu.pid)
        self.assertFalse(
            any(
                event in self.events
                for event in ("video_start", "imu_start", "lease_ack")
            )
        )
        manifest = value.report["manifest"]
        self.assertEqual(manifest["standby_owner"], OWNER)
        self.assertEqual(manifest["wake_command_id"], "0000000000000002")
        self.assertEqual(set(manifest["scripts"]), set(m.NAMES))
        self.assertEqual(
            manifest["bundle_plain_sha256"], value.plan["bundle"]["plain_sha256"]
        )
        self.assertTrue(
            all(
                len(command.encode("ascii")) <= 500 for command in self.control.commands
            )
        )
        with self.assertRaises(m.PreparedStreamError):
            value.prepare()

    def test_immutable_pair_uploaded_once_then_fresh_cycles_only_exclusive_hardlinks(
        self,
    ):
        first = self.backend().prepare()
        first.stop()
        before = list(self.control.commands)
        self.control.sequence = 2
        second = self.backend(
            sleep_id="0000000000000003", wake_id="0000000000000004"
        ).prepare()
        later = self.control.commands[len(before) :]
        self.assertIs(first.assets, second.assets)
        self.assertNotEqual(first.work, second.work)
        self.assertNotEqual(first.plan["work_dir"], second.plan["work_dir"])
        self.assertFalse(
            any("base64 -d >> " + self.assets.work in command for command in later)
        )
        self.assertEqual(sum("os.link(" in command for command in later), 2)
        self.assertFalse(any("ln -s" in command for command in later))
        for command in later:
            if "os.link(" in command:
                self.assertIn("follow_symlinks=False", command)
                self.assertIn("assert not a.st_mode&146", command)
        guard_offset = next(
            index
            for index, command in enumerate(later)
            if "import prepared_preview_wake_047" in command
        )
        sha_offsets = [
            index
            for index, command in enumerate(later)
            if "sha256sum " + self.assets.work in command
        ]
        self.assertTrue(sha_offsets and max(sha_offsets) < guard_offset)

    def test_uncertain_asset_upload_is_not_retried_and_no_watcher_or_usb_cleanup_is_scheduled(
        self,
    ):
        def fail(command):
            if "base64 -d" in command:
                raise OSError("upload result unknown")

        self.control.failure = fail
        value = self.backend()
        with self.assertRaises(OSError):
            value.prepare()
        commands = list(self.control.commands)
        with self.assertRaises(m.PreparedStreamError):
            self.assets.preload_assets()
        self.assertEqual(commands, self.control.commands)
        self.assertFalse(value.possible_future_apply)
        self.assertFalse(
            any(
                command.endswith(" launch") or "rollback.sh" in command
                for command in commands
            )
        )

    def test_watcher_retired_before_uncertain_start_and_no_early_normal_rollback(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = "SLEEP", 1

        def fail(command):
            if command.endswith(" launch"):
                self.assertTrue(value.possible_future_apply)
                raise OSError("lost watcher reply")
            if command.endswith(" cancel"):
                raise RuntimeError("apply intent exists")

        self.control.failure = fail
        with self.assertRaises(m.UnknownFutureApply):
            value.arm_watcher()
        with self.assertRaises(m.PreparedStreamError):
            value.arm_watcher()
        with self.assertRaises(m.UnknownFutureApply):
            value.stop()
        self.assertTrue(value.possible_future_apply)
        self.assertNotIn("preview_stop", self.events)
        self.assertEqual(self.network.cleanups, 0)
        self.assertIsNone(self.camera._usb_preview)

    def test_proven_no_intent_cancel_closes_only_unstarted_local_placeholders(self):
        value = self.armed()
        self.events.clear()
        result = value.stop()
        self.assertFalse(value.possible_future_apply)
        self.assertTrue(result["overlay_never_applied"])
        self.assertEqual(self.events, ["control_start", "imu_stop"])
        self.assertTrue(self.control.commands[-1].endswith(" cancel"))
        self.assertNotIn("preview_stop", self.events)
        self.assertEqual(self.network.cleanups, 0)

    def test_data_first_uses_network_then_exact_lease_before_data_and_never_control_queries(
        self,
    ):
        value = self.armed()
        self.events.clear()
        self.control.commands.clear()
        self.control.snapshot = Mock(
            side_effect=AssertionError("control wait on data path")
        )
        self.control._run = Mock(side_effect=AssertionError("D1 on data path"))
        self.control.bootstrap.query_runtime = Mock(
            side_effect=AssertionError("runtime query on data path")
        )
        preview, imu = value.attach_data_first(native_floor_ns=100, wake_host_ns=1000)
        self.assertEqual(
            self.events, ["network_ready", "lease_ack", "video_start", "imu_start"]
        )
        self.assertIs(self.camera._usb_preview, preview)
        self.assertIs(self.camera._imu, imu)
        self.assertTrue(value.lease_proven)
        self.assertFalse(value.possible_future_apply)
        self.assertFalse(value.witness.joint_ready)
        self.assertFalse(value.report["media_ready_proven"])
        self.assertEqual(preview._stream.options["interface"], "usb9")
        self.assertEqual(preview._stream.options["local_address"], "192.168.253.2")

    def test_first_native_and_host_epochs_reject_stale_without_inventing_timestamps(
        self,
    ):
        value = m.FreshnessWitness(100, 1000)
        self.assertFalse(
            value.observe_video(
                types.SimpleNamespace(timestamp_ns=100, host_receive_monotonic_ns=1001)
            )
        )
        self.assertFalse(
            value.observe_imu(
                types.SimpleNamespace(timestamp_ns=101, host_receive_monotonic_ns=999)
            )
        )
        self.assertTrue(
            value.observe_video(
                types.SimpleNamespace(timestamp_ns=101, host_receive_monotonic_ns=1000)
            )
        )
        self.assertFalse(value.joint_ready)
        self.assertTrue(
            value.observe_imu(
                types.SimpleNamespace(timestamp_ns=102, host_receive_monotonic_ns=1002)
            )
        )
        self.assertTrue(value.joint_ready)
        self.assertEqual(value.report["first_video"]["timestamp_ns"], 101)
        self.assertEqual(
            (value.report["rejected_video"], value.report["rejected_imu"]), (1, 1)
        )
        copied = value.report
        copied["first_imu"]["timestamp_ns"] = 999
        self.assertEqual(value.report["first_imu"]["timestamp_ns"], 102)

    def test_real_post_resume_motion_diagnostics_do_not_become_an_imu_readiness_gate(self):
        fixtures = Path(__file__).parent / "fixtures"
        empty = json.loads((fixtures / "preview_sei_resume_empty.json").read_text())
        raw = b"\0\0\0\1" + bytes.fromhex(empty["nal_hex"])
        metadata = {}
        for record in records_from_video(raw):
            metadata.update(record)
        witness = m.FreshnessWitness(0, 1000)
        frame = VideoFrame(empty["expected"]["frame_clock_ns_raw"], 1001, None, raw, metadata)
        self.assertTrue(witness.observe_video(frame))
        self.assertFalse(witness.joint_ready)
        self.assertEqual(witness.report["first_video_motion"]["sample_count"], 0)
        # A producer snapshot can reveal later native motion even while the
        # joint gate still lacks any actually read/accepted DIAG sample.
        current = json.loads((fixtures / "preview_sei_native.json").read_text())["current"][0]
        stream = VideoStream(decode=False)
        stream._on_video(b"\0\0\0\1" + bytes.fromhex(current["nal_hex"]), 0)
        witness.snapshot_latest(video=stream.stats["latest_video"], imu={
            "last_timestamp_ticks": 19_200_001,
            "last_host_receive_monotonic_ns": 1002,
        })
        result = witness.report
        self.assertEqual(result["latest_video"]["motion"]["sample_count"], 33)
        self.assertEqual(result["latest_imu"]["timestamp_ns"], 1_000_000_052)
        self.assertEqual(result["latest_imu"]["host_receive_monotonic_ns"], 1002)
        self.assertEqual(result["first_video"]["timestamp_ns"], frame.timestamp_ns)
        self.assertEqual(result["first_video_motion"]["sample_count"], 0)
        self.assertIsNone(result["first_imu"])
        self.assertFalse(result["joint_ready"])
        self.assertEqual((result["rejected_video"], result["rejected_imu"]), (0, 0))
        result["latest_video"]["motion"]["sample_count"] = 999
        self.assertEqual(witness.report["latest_video"]["motion"]["sample_count"], 33)

    def test_failed_lease_does_not_expose_normal_preview_cleanup(self):
        lease = Lease(self.events, {})
        lease.start = Mock(side_effect=TimeoutError("no ACK"))
        value = self.backend(lease_factory=lambda config: lease).prepare()
        self.control.state, self.control.sequence = "SLEEP", 1
        value.arm_watcher()
        with self.assertRaises(m.UnknownFutureApply):
            value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.assertFalse(value.lease_proven)
        self.assertTrue(value.possible_future_apply)
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)
        self.assertNotIn("preview_stop", self.events)
        attempts = list(self.events)
        with self.assertRaises(m.PreparedStreamError):
            value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.assertEqual(self.events, attempts)

    def test_ack_proven_stop_delegates_existing_cleanup_but_unknown_control_task_blocks_it(
        self,
    ):
        value = self.armed()
        value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.control.bootstrap.session.quarantined = True
        with self.assertRaises(m.PreparedStreamError):
            value.stop()
        self.assertNotIn("preview_stop", self.events)
        self.control.bootstrap.session.quarantined = False
        result = value.stop()
        self.assertTrue(result["native_restored"])
        self.assertIn("preview_stop", self.events)
        self.assertEqual(
            self.events[-4:], ["preview_stop", "video_stop", "imu_stop", "lease_close"]
        )

    def test_abandon_local_requires_mask_proof_before_stopping_heartbeat_or_video(self):
        value = self.armed()
        value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.events.clear()
        self.control.commands.clear()
        value.imu.stop_error = RuntimeError("collector still restoring")
        with self.assertRaises(m.PreparedStreamError):
            value.abandon_local()
        self.assertEqual(self.events, ["imu_stop"])
        self.assertEqual(self.control.commands, [])
        self.assertTrue(value.preview._lease.healthy)
        value.imu.stop_error = None
        self.events.clear()
        result = value.abandon_local()
        self.assertEqual(self.events, ["imu_stop", "video_stop", "lease_local_close"])
        self.assertFalse(result["native_restored"])
        self.assertTrue(result["retained_shadow"])
        self.assertTrue(result["lease_watchdog_restoration_deferred"])
        self.assertEqual(self.network.cleanups, 0)
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)
        self.assertEqual(self.control.commands, [])

    def test_abandon_local_never_drops_heartbeat_with_missing_or_false_quiescence(self):
        for proof in (
            dict(restored=True, errors=[]),
            dict(restored=True, quiescent=False, errors=[]),
            dict(restored=True, quiescent=1, errors=[]),
        ):
            with self.subTest(proof=proof):
                self.setUp()
                value = self.armed()
                value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
                value.imu.stats['collector_returncode'] = 0
                value.imu.stop = Mock(return_value=proof)
                self.events.clear()
                commands = list(self.control.commands)
                with self.assertRaisesRegex(m.PreparedStreamError, 'quiescence'):
                    value.abandon_local()
                self.assertEqual(self.events, [])
                self.assertEqual(self.control.commands, commands)
                self.assertTrue(value.preview._lease.healthy)
                self.assertFalse(value.abandoned)
                self.assertIs(self.camera._imu, value.imu)
                self.assertIs(self.camera._usb_preview, value.preview)

    def test_failed_start_before_collector_spawn_can_abandon_unstarted_placeholder(self):
        value = self.armed()
        value.imu.start = Mock(side_effect=RuntimeError('no collector was spawned'))
        with self.assertRaisesRegex(RuntimeError, 'no collector'):
            value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        self.assertTrue(value.imu_started)
        self.assertIsNone(value.imu.pid)
        self.assertIsNone(value.imu.report)
        result = value.abandon_local()
        self.assertTrue(result['local_abandoned'])
        self.assertFalse(value.preview._lease.healthy)

    def test_camera_link_commands_create_real_hardlinks_exclusively_and_reject_symlinks_or_writable_sources(
        self,
    ):
        self.assets.preload_assets()
        self.assets.link_into("/dev/shm/osmo-prepared-preview047-012345abcdef")
        command = next(value for value in self.control.commands if "os.link(" in value)
        words = shlex.split(command)
        program = words[words.index("-c") + 1]
        digest = hashlib.sha256(FILES[m.WATCHER]).hexdigest()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            assets, destination = root / "assets", root / "watcher"
            assets.mkdir()
            destination.mkdir()
            source = assets / m.WATCHER
            source.write_bytes(FILES[m.WATCHER])
            source.chmod(0o400)

            def private(path, **options):
                if path.is_symlink() or path.resolve() != path:
                    raise ValueError("noncanonical source")
                return path.lstat()

            fake = types.SimpleNamespace(
                Path=Path,
                hashlib=hashlib,
                base=types.SimpleNamespace(
                    private=private, read_bytes=lambda path, limit: path.read_bytes()
                ),
            )
            old_cwd = Path.cwd()
            try:
                os.chdir(assets)
                target = destination / m.WATCHER
                with (
                    patch.dict(sys.modules, {"prepared_preview_wake_047": fake}),
                    patch.object(sys, "argv", ["-c", m.WATCHER, str(target), digest]),
                ):
                    exec(compile(program, "camera-hardlink", "exec"), {})
                    self.assertEqual(
                        (source.stat().st_dev, source.stat().st_ino),
                        (target.stat().st_dev, target.stat().st_ino),
                    )
                    with self.assertRaises(FileExistsError):
                        exec(compile(program, "camera-hardlink", "exec"), {})
                    target.unlink()
                    source.chmod(0o600)
                    with self.assertRaises(AssertionError):
                        exec(compile(program, "camera-hardlink", "exec"), {})
                    source.unlink()
                    source.symlink_to(root / "outside")
                    with self.assertRaises(ValueError):
                        exec(compile(program, "camera-hardlink", "exec"), {})
                self.assertFalse(target.exists())
            finally:
                os.chdir(old_cwd)

    def test_asset_guard_rejects_another_boot_or_owner_before_any_link(self):
        self.assets.preload_assets()
        words = shlex.split(self.assets._guard_command())
        program = words[words.index("-c") + 1]
        config = dict(RECEIPT, sys_service=SERVICE, boot_id="same-boot")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            owner = root / "owner"
            owner.mkdir()
            cached = dict(config)

            def read_json(path):
                return deepcopy(cached if path.name == "cache.json" else config)

            fake = types.SimpleNamespace(
                Path=Path,
                base=types.SimpleNamespace(
                    read_json=read_json, private=lambda *args, **kwargs: None
                ),
            )
            original_read = Path.read_text

            def text(path, *args, **kwargs):
                return (
                    "same-boot\n"
                    if str(path) == "/proc/sys/kernel/random/boot_id"
                    else original_read(path, *args, **kwargs)
                )

            old_cwd = Path.cwd()
            try:
                os.chdir(root)
                with (
                    patch.dict(sys.modules, {"prepared_preview_wake_047": fake}),
                    patch.object(sys, "argv", ["-c", str(owner)]),
                    patch.object(Path, "read_text", text),
                ):
                    exec(compile(program, "camera-asset-guard", "exec"), {})
                    cached["boot_id"] = "prior-boot"
                    with self.assertRaises(AssertionError):
                        exec(compile(program, "camera-asset-guard", "exec"), {})
                    cached["boot_id"] = "same-boot"
                    cached["owner"] = dict(pid=4999, start="100")
                    with self.assertRaises(AssertionError):
                        exec(compile(program, "camera-asset-guard", "exec"), {})
            finally:
                os.chdir(old_cwd)

    def test_late_but_authenticated_lease_ack_still_allows_cleanup_of_executed_plan(
        self,
    ):
        now = [0.0]
        lease = Lease(self.events, {})
        original = lease.start

        def late(**options):
            original(**options)
            now[0] = 2
            return lease

        lease.start = late
        value = self.backend(
            lease_factory=lambda config: lease, clock=lambda: now[0]
        ).prepare()
        self.control.state, self.control.sequence = "SLEEP", 1
        value.arm_watcher()
        with self.assertRaises(TimeoutError):
            value.attach_data_first(timeout=1, native_floor_ns=0, wake_host_ns=0)
        self.assertTrue(value.lease_proven)
        self.assertFalse(value.possible_future_apply)
        self.assertTrue(value.stop()["native_restored"])

    def test_stopped_video_fault_is_preserved_after_mask_proof_and_local_lease_close(
        self,
    ):
        value = self.armed()
        value.attach_data_first(native_floor_ns=0, wake_host_ns=0)
        video_error = RuntimeError("metadata failed")
        value.preview._stream.stop = Mock(side_effect=video_error)
        value.preview._stream.stats = dict(stopped=True)
        self.events.clear()
        with self.assertRaises(RuntimeError) as raised:
            value.abandon_local()
        self.assertIs(raised.exception, video_error)
        self.assertIn("lease_local_close", self.events)
        self.assertFalse(value.report["native_restored"])
        self.assertTrue(value.abandoned)


if __name__ == "__main__":
    unittest.main()
