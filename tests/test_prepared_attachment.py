"""Process-reattach metadata and camera-RO proof fakes; no camera/network I/O."""

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shlex
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from osmo360_camera import _prepared_attachment as m
from osmo360_camera import _prepared_stream as prepared
from osmo360_camera._usb_bundle import build_bundle
from osmo360_camera._usb_plan import build_usb_plan
from test_prepared_stream import (
    Control,
    Network,
    Preview,
    IMU,
    Lease,
    Video,
    FILES,
    RECEIPT,
)


class PreparedAttachmentTests(unittest.TestCase):
    def setUp(self):
        self.source_patch = patch.dict(
            prepared.SOURCES,
            {name: hashlib.sha256(data).hexdigest() for name, data in FILES.items()},
            clear=True,
        )
        self.source_patch.start()
        self.addCleanup(self.source_patch.stop)
        self.events = []
        self.camera = types.SimpleNamespace(
            _control_lock=__import__("threading").RLock(),
            _transport=object(),
            _imu=None,
            _usb_preview=None,
        )
        self.control = Control(self.camera, self.events)
        self.control.pending_command = None
        self.network = Network(self.events)
        self.options = dict(
            network_guard=self.network,
            preview_factory=lambda camera, **kw: Preview(camera, self.events, **kw),
            imu_factory=lambda **kw: IMU(self.events, **kw),
            lease_factory=lambda config: Lease(self.events, config),
            video_factory=lambda **kw: Video(self.events, **kw),
        )
        self.backend = prepared.PreparedStream(
            self.camera,
            self.control,
            owner_receipt=RECEIPT,
            sleep_id="0000000000000001",
            wake_id="0000000000000002",
            source_provider=lambda: dict(FILES),
            plan_builder=lambda runtime: self.builder(
                runtime, tag="attachment-test", nonce="ab" * 16, port=51337
            ),
            **self.options,
        ).prepare()
        self.control.state, self.control.sequence = "SLEEP", 1
        self.metadata = m.export_prepared(self.backend, native_floor_ns=123456789000)
        self.control.commands.clear()
        self.events.clear()

    @staticmethod
    def builder(runtime, **parameters):
        return build_usb_plan(runtime, **parameters)

    def restore(self, value=None, **overrides):
        options = dict(self.options)
        options.update(overrides)
        return m.restore_prepared(
            self.camera,
            self.control,
            self.metadata if value is None else value,
            source_provider=lambda: self.fail(
                "Existing asset bytes must not be reuploaded"
            ),
            plan_builder=options.pop("plan_builder", self.builder),
            **options,
        )

    def test_metadata_is_compact_json_and_contains_native_floor_without_old_host_clocks(
        self,
    ):
        self.assertLess(len(m._json(self.metadata)), 4096)
        self.assertEqual(self.metadata, json.loads(json.dumps(self.metadata)))
        self.assertEqual(self.metadata["native_floor_ns"], 123456789000)
        self.assertNotIn("monotonic", m._json(self.metadata).decode())
        self.assertEqual(self.metadata["profile_hashes"], m._profiles())
        self.assertEqual(
            self.metadata["manifest_sha256"],
            hashlib.sha256(
                m._json(self.backend.report["manifest"]) + b"\n"
            ).hexdigest(),
        )

    def test_export_refuses_any_armed_started_or_non_sleep_backend(self):
        self.backend.possible_future_apply = True
        with self.assertRaises(m.PreparedAttachmentError):
            m.export_prepared(self.backend, native_floor_ns=1)
        self.backend.possible_future_apply = False
        self.control.state = "AWAKE"
        with self.assertRaises(m.PreparedAttachmentError):
            m.export_prepared(self.backend, native_floor_ns=1)
        self.control.state = "SLEEP"
        self.control.pending_command = "0000000000000002"
        with self.assertRaises(m.PreparedAttachmentError):
            m.export_prepared(self.backend, native_floor_ns=1)
        self.assertEqual(self.control.commands, [])

    def test_restore_reuses_exact_plan_work_assets_and_creates_only_local_unstarted_objects(
        self,
    ):
        fresh_network = Network(self.events)
        result = self.restore(network_guard=fresh_network)
        self.assertTrue(result.prepared)
        self.assertTrue(result.prepare_attempted)
        self.assertFalse(result.arm_attempted)
        self.assertFalse(result.possible_future_apply)
        self.assertFalse(result.lease_proven)
        self.assertIsNone(result.imu.pid)
        self.assertIsNone(result.preview._stream)
        self.assertIsNone(result.preview._lease)
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)
        self.assertEqual(result.work, self.backend.work)
        self.assertEqual(result.plan["sha256"], self.backend.plan["sha256"])
        self.assertEqual(result.plan["work_dir"], self.backend.plan["work_dir"])
        self.assertEqual(result.assets.work, self.backend.assets.work)
        self.assertTrue(result.assets.staged)
        self.assertEqual(result.native_floor_ns, self.metadata["native_floor_ns"])
        self.assertIs(result.network, fresh_network)
        self.assertIsNot(result.baseline, self.backend.baseline)
        self.assertIn("host_preflight", self.events)
        self.assertNotIn("video_start", self.events)
        self.assertNotIn("imu_start", self.events)
        self.metadata["owner_receipt"]["owner"]["pid"] = 9999
        self.assertEqual(result.owner_receipt["owner"]["pid"], RECEIPT["owner"]["pid"])

    def test_all_camera_commands_are_bounded_read_only_and_hash_precede_python_import(
        self,
    ):
        self.restore()
        commands = self.control.commands
        self.assertEqual(len(commands), 10)
        self.assertTrue(
            all(len(command.encode("ascii")) <= 500 for command in commands)
        )
        self.assertTrue(all("sha256sum" in command for command in commands[:4]))
        for command in commands:
            self.assertNotIn("base64", command)
            self.assertNotIn("mkdir", command)
            self.assertFalse(command.endswith((" launch", " cancel", " finalize")))
            self.assertNotIn("os.link(", command)
            if "python3" in command:
                self.assertIn("python3 -B -c ", command)
                words = shlex.split(command)
                compile(words[words.index("-c") + 1], "camera-readonly-proof", "exec")

    def test_profile_paths_types_hashes_and_manifest_are_strict_before_camera_commands(
        self,
    ):
        mutations = [
            lambda x: x["profile_hashes"].update({"usb_standby_owner.py": "00" * 32}),
            lambda x: x.update(prepared_work="/tmp/unowned"),
            lambda x: x.update(native_floor_ns=True),
            lambda x: x["runtime"].update(original_flags="0x1003"),
            lambda x: x["plan"].update(port=True),
            lambda x: x["plan"]["sha256"].update({"apply.sh": "00" * 32}),
            lambda x: x.update(manifest_sha256="00" * 32),
            lambda x: x.update(wake_id="0000000000000003"),
        ]
        for change in mutations:
            with self.subTest(change=change):
                value = deepcopy(self.metadata)
                change(value)
                with self.assertRaises((ValueError, m.PreparedAttachmentError)):
                    self.restore(value)
        self.assertEqual(self.control.commands, [])
        raw = m._json(self.metadata)
        with self.assertRaises(ValueError):
            m.decode_metadata(raw[:-1] + b',"schema":1}')

    def test_different_local_script_builder_fails_before_camera_proof_or_adoption(self):
        def changed(runtime, **parameters):
            plan = self.builder(runtime, **parameters)
            plan["scripts"]["apply.sh"] += "\n# changed build\n"
            plan["sha256"]["apply.sh"] = hashlib.sha256(
                plan["scripts"]["apply.sh"].encode()
            ).hexdigest()
            plan["byte_lengths"]["apply.sh"] = len(plan["scripts"]["apply.sh"].encode())
            bundle = build_bundle(plan["scripts"], plan["work_dir"])
            plan["commands"] = bundle.pop("commands")
            bundle.pop("files")
            plan["bundle"] = bundle
            return plan

        with self.assertRaises(m.PreparedAttachmentError):
            self.restore(plan_builder=changed)
        self.assertEqual(self.control.commands, [])
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)

    def test_unknown_ro_camera_proof_never_uploads_recovers_or_marks_asset_cache_owned(
        self,
    ):
        def fail(command):
            if "m.base.plan_proof(c)" in command:
                raise RuntimeError("used/armed or manifest changed")

        self.control.failure = fail
        with self.assertRaises(RuntimeError):
            self.restore()
        self.assertIsNone(self.camera._imu)
        self.assertIsNone(self.camera._usb_preview)
        self.assertFalse(
            any(
                command.endswith((" launch", " finalize"))
                for command in self.control.commands
            )
        )
        self.assertFalse(any("host_preflight" == item for item in self.events))

    def test_stale_state_or_pending_future_input_is_rejected_without_camera_proof(self):
        for state, sequence, pending in (
            ("AWAKE", 2, None),
            ("SLEEP", 3, None),
            ("SLEEP", 1, "0000000000000002"),
        ):
            self.control.state, self.control.sequence, self.control.pending_command = (
                state,
                sequence,
                pending,
            )
            with self.assertRaises(m.PreparedAttachmentError):
                self.restore()
        self.assertEqual(self.control.commands, [])

    def test_ro_exact_file_set_rejects_every_observer_or_apply_intent(self):
        self.restore()
        command = next(
            command
            for command in self.control.commands
            if "assert {p.name for p in w.iterdir()}" in command
        )
        words = shlex.split(command)
        program = words[words.index("-c") + 1]
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            work, plan = root / "watcher", root / "plan"
            work.mkdir()
            plan.mkdir()
            for name in (
                "prepared_preview_wake_047.py",
                "prepared_preview_wake_045.py",
                "manifest.json",
            ):
                (work / name).touch()
            for name in (*m.NAMES, "unpack.py", "scripts.z", "bundle.ready"):
                (plan / name).touch()
            fake = types.SimpleNamespace(
                environment=lambda: work,
                Path=Path,
                base=types.SimpleNamespace(
                    read_json=lambda path: dict(
                        plan_work=str(plan), scripts={name: "" for name in m.NAMES}
                    ),
                    private=lambda *args, **kwargs: None,
                ),
            )
            with patch.dict(sys.modules, {"prepared_preview_wake_047": fake}):
                exec(compile(program, "exact-existing-work", "exec"), {})
                for directory, name in (
                    (work, "launch-intent.json"),
                    (work, "apply-intent.json"),
                    (work, "owner.json"),
                    (work, "cancel-request.json"),
                    (plan, "used"),
                    (plan, "armed"),
                    (plan, "apply.log"),
                    (plan, "apply.owner"),
                ):
                    (directory / name).touch()
                    with self.subTest(name=name), self.assertRaises(AssertionError):
                        exec(compile(program, "exact-existing-work", "exec"), {})
                    (directory / name).unlink()

    def test_ro_hardlink_guard_accepts_exact_inode_but_rejects_copied_or_symlinked_file(
        self,
    ):
        self.restore()
        command = next(
            command
            for command in self.control.commands
            if "import os,sys,stat;" in command
        )
        words = shlex.split(command)
        program = words[words.index("-c") + 1]
        original = os.lstat

        def root_stat(path):
            values = list(original(path))
            values[4] = 0
            return os.stat_result(values)

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, target = root / "source", root / "target"
            source.write_bytes(b"exact")
            source.chmod(0o400)
            os.link(source, target)
            with (
                patch.object(sys, "argv", ["-c", str(source), str(target)]),
                patch.object(os, "lstat", side_effect=root_stat),
            ):
                exec(compile(program, "exact-existing-link", "exec"), {})
                target.unlink()
                target.write_bytes(b"exact")
                target.chmod(0o400)
                with self.assertRaises(AssertionError):
                    exec(compile(program, "exact-existing-link", "exec"), {})
                target.unlink()
                target.symlink_to(source)
                with self.assertRaises(AssertionError):
                    exec(compile(program, "exact-existing-link", "exec"), {})


if __name__ == "__main__":
    unittest.main()
