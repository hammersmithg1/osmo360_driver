"""Lazy resume/discovery and receipt-publication failures, entirely fake USB."""

from copy import deepcopy
import json
from pathlib import Path
import shlex
import tempfile
import threading
import types
import sys
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _stream_resume as m
from test_standby_control import snapshot, NONCE

WORK = "/dev/shm/osmo-usb-standby-" + NONCE


class Control:
    def __init__(self):
        self.commands, self.pulls = [], []
        self.work, self.nonce = WORK, NONCE
        self.last = snapshot("SLEEP", 1)
        self.owner, self.service = (
            deepcopy(self.last["owner"]),
            deepcopy(self.last["service"]),
        )
        self.bootstrap = types.SimpleNamespace(pull_bytes=self.pull)

    def _run(self, command, **options):
        if len(command) > 500:
            raise AssertionError("Overlong command")
        self.commands.append(command)

    def pull(self, path, **options):
        self.pulls.append(path)
        return self.raw

    def snapshot(self):
        return deepcopy(self.last)


class Core:
    def __init__(self, events, control=None):
        self.events, self.control = events, control or Control()
        self.report, self.state = {}, "NEW"
        self.fast_sleep = False
        self.owns_camera_resources = self.cleanup_safe = False
        self.imu_options = dict(read_buffer_bytes=65536, queue_size=16384)
        self.prepared_factory = types.SimpleNamespace(
            source_provider=lambda: {}, assets=None
        )
        manager = types.SimpleNamespace(
            clock=lambda: 0,
            timeout=60,
            ensure=lambda **options: events.append("RO.ready"),
        )
        self.idle_probe = types.SimpleNamespace(__self__=manager)
        self._prepared, self._native_floor_ns = object(), 123

    def start(self):
        self.events.append("core.start")
        self.state = "AWAKE"
        return self

    def wake(self, **options):
        self.events.append("core.wake")
        self.state = "AWAKE"
        return self

    def attach_sleep(self, prepared, **options):
        self.events.append("attach.sleep")
        self.state = "SLEEP"
        self._prepared = prepared

    def sleep(self):
        self.events.append("core.sleep")
        self.state = "SLEEP"
        return dict(sleep=True)

    def close(self):
        self.events.append("core.close")
        self.cleanup_safe = True
        return {}


class ResumeTests(unittest.TestCase):
    def setUp(self):
        self.events, self.cores = [], []
        self.camera = types.SimpleNamespace(
            _control_lock=threading.RLock(),
            _imu=None,
            _usb_preview=None,
            connect=lambda: self.events.append("connect"),
        )

        def build(camera, **options):
            core = Core(self.events)
            core.fast_sleep = options.get("fast_sleep", False)
            if options.get("bootstrap") is not None:
                core.control.bootstrap = options["bootstrap"]
            self.cores.append(core)
            return core

        self.build = patch.object(m, "build_stream_session", side_effect=build).start()
        self.addCleanup(patch.stopall)

    def wrapper(self, **options):
        return m.build_resumable_stream(self.camera, decode=False, **options)

    def test_build_is_lazy_and_native_no_shadow_starts_normal_core_only_after_ro_discovery(
        self,
    ):
        stream = self.wrapper()
        self.assertEqual(self.events, [])
        with patch.object(
            m, "_discover", side_effect=lambda c: self.events.append("discover") or None
        ):
            self.assertIs(stream.start(), stream)
        self.assertEqual(self.events, ["connect", "RO.ready", "discover", "core.start"])
        self.assertFalse(stream.attachment["existing"])
        with self.assertRaises(m.StreamSessionError):
            stream.start()

    def test_unknown_shadow_never_launches_wakes_or_implicitly_cleans(self):
        stream = self.wrapper()
        with patch.object(m, "_discover", side_effect=ValueError("foreign bind")):
            with self.assertRaises(ValueError):
                stream.start()
        self.assertTrue(stream.owns_camera_resources)
        self.assertFalse(stream.cleanup_safe)
        with self.assertRaises(m.StreamSessionError):
            stream.close()
        self.assertNotIn("core.start", self.events)
        self.assertNotIn("core.close", self.events)
        self.assertNotIn("core.wake", self.events)

    def test_existing_awake_reuses_completed_adapter_and_verifies_sources_before_core_start(
        self,
    ):
        stream = self.wrapper()
        previous = stream._core.control.bootstrap
        with (
            patch.object(m, "_discover", return_value=WORK),
            patch.object(m, "_status_at", return_value=snapshot()),
        ):
            stream.start()
        options = self.build.call_args.kwargs
        self.assertIs(options["bootstrap"], previous)
        self.assertEqual(options["existing"]["work"], WORK)
        self.assertEqual(self.events[-1], "core.start")
        self.assertEqual(len(self.cores), 2)

    def test_existing_sleep_restores_unarmed_backend_and_wakes_without_initial_upload_start(
        self,
    ):
        stream = self.wrapper()
        backend = types.SimpleNamespace(assets=object())
        metadata = dict(native_floor_ns=123456)
        with (
            patch.object(m, "_discover", return_value=WORK),
            patch.object(m, "_status_at", return_value=snapshot("SLEEP", 1)),
            patch.object(m, "_read_metadata", return_value=metadata),
            patch.object(m, "restore_prepared", return_value=backend) as restore,
        ):
            stream.start()
        self.assertEqual(self.events[-2:], ["attach.sleep", "core.wake"])
        self.assertNotIn("core.start", self.events)
        self.assertIs(stream._core.prepared_factory.assets, backend.assets)
        builder = restore.call_args.kwargs["plan_builder"]
        self.assertEqual(builder.keywords, dict(fast=True))

    def test_retained_opt_in_survives_rebuilding_core_for_existing_sleep(self):
        stream = self.wrapper(fast_sleep=True, retain_ecm=True)
        backend = types.SimpleNamespace(assets=object())
        with (
            patch.object(m, "_discover", return_value=WORK),
            patch.object(m, "_status_at", return_value=snapshot("SLEEP", 1)),
            patch.object(m, "_read_metadata", return_value=dict(native_floor_ns=123)),
            patch.object(m, "restore_prepared", return_value=backend),
        ):
            stream.start()
        self.assertTrue(
            all(
                call.kwargs["retain_ecm"] and call.kwargs["fast_sleep"]
                for call in self.build.call_args_list
            )
        )
        self.assertEqual(self.events[-2:], ["attach.sleep", "core.wake"])

    def test_reattached_fast_sleep_publishes_only_from_worker_ready_callback_without_wrapper_lock(
        self,
    ):
        stream = self.wrapper(fast_sleep=True)
        backend = types.SimpleNamespace(assets=object())
        with (
            patch.object(m, "_discover", return_value=WORK),
            patch.object(m, "_status_at", return_value=snapshot("SLEEP", 1)),
            patch.object(m, "_read_metadata", return_value=dict(native_floor_ns=123)),
            patch.object(m, "restore_prepared", return_value=backend),
        ):
            stream.start()
        self.assertTrue(stream._core.fast_sleep)
        backend.control = stream._core.control
        completed, errors = threading.Event(), []
        with (
            patch.object(m, "export_prepared", return_value={"ready": "new"}) as export,
            patch.object(
                m, "_publish_metadata", return_value=dict(published=True)
            ) as publish,
        ):
            stream.sleep()
            export.assert_not_called()
            publish.assert_not_called()

            # Wake/close can hold the wrapper lock while joining preparation.
            # Its READY callback must need only the shared control lock.
            def worker():
                try:
                    with self.camera._control_lock:
                        stream._core._prepare_ready_callback(backend, 456)
                except BaseException as error:
                    errors.append(error)
                finally:
                    completed.set()

            with stream._lock:
                thread = threading.Thread(target=worker)
                thread.start()
                self.assertTrue(completed.wait(1))
            thread.join(1)
        self.assertEqual(errors, [])
        export.assert_called_once_with(backend, native_floor_ns=456)
        publish.assert_called_once_with(backend.control, {"ready": "new"}, guard=None)

    def test_stale_or_missing_sleep_metadata_never_wakes_and_keeps_unknown_ownership(
        self,
    ):
        stream = self.wrapper()
        with (
            patch.object(m, "_discover", return_value=WORK),
            patch.object(m, "_status_at", return_value=snapshot("SLEEP", 1)),
            patch.object(m, "_read_metadata", side_effect=ValueError("stale receipt")),
        ):
            with self.assertRaises(ValueError):
                stream.start()
        self.assertNotIn("core.wake", self.events)
        self.assertTrue(stream.owns_camera_resources)
        self.assertFalse(stream.cleanup_safe)

    def test_borrowed_producers_prevent_discovery_or_owner_mutation(self):
        stream = self.wrapper()
        self.camera._imu = object()
        with patch.object(m, "_discover") as discover:
            with self.assertRaises(m.StreamSessionError):
                stream.start()
        discover.assert_not_called()
        self.assertNotIn("RO.ready", self.events)

    def test_publication_failure_preserves_actual_stable_sleep_without_recovery(self):
        stream = self.wrapper()
        with (
            patch.object(m, "export_prepared", return_value={}),
            patch.object(
                m, "_publish_metadata", side_effect=RuntimeError("uncertain START")
            ),
        ):
            with self.assertRaises(RuntimeError):
                stream.sleep()
        self.assertEqual(stream.state, "SLEEP")
        self.assertIn("uncertain START", stream.attachment["publication_error"])
        stream.close()
        self.assertEqual(self.events, ["core.sleep", "core.close"])

    def test_wake_returns_same_public_wrapper_so_fluent_use_keeps_receipt_publication(
        self,
    ):
        stream = self.wrapper()
        self.assertIs(stream.wake(timeout=30), stream)
        self.assertEqual(self.events, ["core.wake"])

    def test_wake_and_close_wait_for_metadata_publication_and_share_control_lock(self):
        for action in ("wake", "close"):
            with self.subTest(action=action):
                stream = self.wrapper()
                publishing, release, attempted, finished = [
                    threading.Event() for _ in range(4)
                ]
                failures = []

                def publish(control, value):
                    self.assertTrue(self.camera._control_lock._is_owned())
                    publishing.set()
                    if not release.wait(2):
                        raise AssertionError("test release missing")
                    self.events.append("publish.finished")
                    return dict(published=True)

                def run_sleep():
                    try:
                        stream.sleep()
                    except BaseException as error:
                        failures.append(error)

                def other():
                    attempted.set()
                    try:
                        getattr(stream, action)()
                    except BaseException as error:
                        failures.append(error)
                    finally:
                        finished.set()

                with (
                    patch.object(m, "export_prepared", return_value={}),
                    patch.object(m, "_publish_metadata", side_effect=publish),
                ):
                    sleeper = threading.Thread(target=run_sleep)
                    sleeper.start()
                    self.assertTrue(publishing.wait(1))
                    worker = threading.Thread(target=other)
                    worker.start()
                    self.assertTrue(attempted.wait(1))
                    self.assertFalse(finished.wait(0.03))
                    self.assertNotIn("core." + action, self.events)
                    release.set()
                    sleeper.join(2)
                    worker.join(2)
                self.assertFalse(sleeper.is_alive())
                self.assertFalse(worker.is_alive())
                self.assertEqual(failures, [])
                self.assertLess(
                    self.events.index("publish.finished"),
                    self.events.index("core." + action),
                )
                self.events.clear()


class CodecCommandTests(unittest.TestCase):
    def test_exact_discovery_command_is_bounded_and_compiles_no_wake_or_configuration(
        self,
    ):
        output = "/blackbox/export_tmp/osmo_resume_0123456789ab.json"
        command = m._command(m._discovery_program(output))
        self.assertLessEqual(len(command), 500)
        self.assertIn("'x'", m._discovery_program(output))
        self.assertNotIn("os.system", command)
        self.assertNotIn("reboot", command)
        self.assertNotIn("umount", command)

    def discovery(self, mounts, works):
        control = Control()
        control.raw = json.dumps(dict(m=mounts, w=works, i=[7, 8])).encode()
        return control, m._discover(control)

    def test_discovery_requires_same_inode_candidate_and_exact_tmpfs_mount_root(self):
        control, result = self.discovery([], [])
        self.assertIsNone(result)
        self.assertEqual(len(control.commands), 3)
        row = [
            "10",
            "1",
            "0:20",
            "/" + WORK.rsplit("/", 1)[1] + "/role.txt",
            m.TARGET,
            "rw",
            "-",
            "tmpfs",
            "tmpfs",
            "rw",
        ]
        self.assertEqual(self.discovery([row], [WORK])[1], WORK)
        for rows, works in (
            ([], [WORK]),
            ([row], []),
            ([row, row], [WORK]),
            ([row], [WORK, WORK]),
            ([row[:7] + ["sysfs"] + row[8:]], [WORK]),
            ([row], ["/tmp/foreign"]),
        ):
            with self.subTest(rows=rows, works=works), self.assertRaises(ValueError):
                self.discovery(rows, works)

    def test_status_executes_only_after_both_current_hash_checks_and_strict_native_snapshot(
        self,
    ):
        control = Control()
        control.raw = json.dumps(snapshot()).encode()
        result = m._status_at(control, WORK)
        self.assertEqual(result["state"], "AWAKE")
        self.assertEqual(len(control.commands), 3)
        self.assertTrue(all("sha256sum" in text for text in control.commands[:2]))
        self.assertTrue(control.commands[-1].endswith(" status"))

        def reject(command):
            if "sha256sum" in command:
                raise RuntimeError("digest mismatch")
            self.fail("Unverified owner executed")

        control._run = reject
        with self.assertRaises(RuntimeError):
            m._status_at(control, WORK)

    def test_metadata_publish_has_exclusive_temp_hash_before_atomic_commit_and_no_replay(
        self,
    ):
        control = Control()
        value = dict(
            sleep_id="0000000000000001",
            owner_receipt=dict(
                work=control.work,
                nonce=control.nonce,
                owner=control.owner,
                service=control.service,
            ),
        )
        with patch.object(m, "decode_metadata", side_effect=lambda x: x):
            result = m._publish_metadata(control, value)
        self.assertTrue(result["published"])
        self.assertIn("set -C", control.commands[0])
        self.assertIn("sha256sum", control.commands[-2])
        self.assertIn("session.json", control.commands[-1])
        self.assertIn("m.replace", control.commands[-1])
        control = Control()
        commands = []

        def fail(command):
            commands.append(command)
            if "base64 -d" in command:
                raise RuntimeError("lost RESULT")

        control._run = fail
        with patch.object(m, "decode_metadata", side_effect=lambda x: x):
            with self.assertRaises(RuntimeError):
                m._publish_metadata(control, value)
        self.assertEqual(sum("base64 -d" in text for text in commands), 1)
        self.assertFalse(any("m.replace" in text for text in commands))

    def test_atomic_commit_rechecks_current_sleep_owner_and_does_not_publish_after_changed_state(
        self,
    ):
        control = Control()
        value = dict(
            sleep_id="0000000000000001",
            owner_receipt=dict(
                work=control.work,
                nonce=control.nonce,
                owner=control.owner,
                service=control.service,
            ),
        )
        with patch.object(m, "decode_metadata", side_effect=lambda x: x):
            m._publish_metadata(control, value)
        words = shlex.split(control.commands[-1])
        program = words[words.index("-c") + 1]
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            name = next(
                word for word in program.split("'") if word.startswith("session.next-")
            )
            (work / name).write_text(json.dumps(value))
            state = dict(
                state="SLEEP",
                command_id=value["sleep_id"],
                owner=value["owner_receipt"]["owner"],
            )
            (work / "status.json").write_text(json.dumps(state))
            replace = Mock()
            fake = types.SimpleNamespace(
                base_module=lambda: None,
                environment=lambda b: work,
                configuration=lambda w: dict(owner=control.owner),
                read=lambda p: json.loads(p.read_text()),
                replace=replace,
            )
            with patch.dict(sys.modules, {"usb_standby_owner": fake}):
                exec(compile(program, "commit", "exec"), {})
                replace.assert_called_once_with(work / "session.json", value)
                replace.reset_mock()
                (work / name).write_text(json.dumps(value))
                state["command_id"] = "0000000000000003"
                (work / "status.json").write_text(json.dumps(state))
                with self.assertRaises(AssertionError):
                    exec(compile(program, "commit", "exec"), {})
                replace.assert_not_called()
        wrong = deepcopy(value)
        wrong["owner_receipt"]["owner"]["pid"] += 1
        control.commands.clear()
        with patch.object(m, "decode_metadata", side_effect=lambda x: x):
            with self.assertRaises(ValueError):
                m._publish_metadata(control, wrong)
        self.assertEqual(control.commands, [])

    def test_awake_old_overlay_is_only_polled_and_running_unknown_task_is_not_replayed(
        self,
    ):
        control = Control()
        now = [0]
        control.bootstrap.session = types.SimpleNamespace(quarantined=False)

        def fail(command):
            control.commands.append(command)
            raise m.BootstrapError("f6 still owned by old lease")

        control._run = fail
        control._resolve_readonly_exit1 = Mock(return_value=True)
        with self.assertRaises(TimeoutError):
            m._wait_native(
                control,
                clock=lambda: now[0],
                sleep=lambda seconds: now.__setitem__(0, now[0] + seconds),
            )
        self.assertLessEqual(now[0], 12.01)
        self.assertTrue(
            all(
                "read_text" in text and "unlink" not in text
                for text in control.commands
            )
        )
        control.commands.clear()
        control.bootstrap.session.quarantined = True
        control._resolve_readonly_exit1.return_value = False
        with self.assertRaises(m.BootstrapError):
            m._wait_native(control)
        self.assertEqual(len(control.commands), 1)

    def test_native_link_proof_accepts_actual_long_and_short_relative_spelling_but_not_foreign_target(
        self,
    ):
        control = Control()
        m._wait_native(control)
        words = shlex.split(control.commands[0])
        program = words[words.index("-c") + 1]
        names = ("ffs.diag", "ffs.bulk", "ffs.bulk1", "ffs.bulk5", "ffs.bulk4")
        with tempfile.TemporaryDirectory() as directory:
            gadget = Path(directory) / "usb_gadget/g1"
            links = gadget / "configs/c.1"
            links.mkdir(parents=True)
            (gadget / "functions").mkdir()
            for name in names:
                (gadget / "functions" / name).mkdir()
            (gadget / "max_speed").write_text("super-speed-plus\n")
            code = program.replace(
                repr("/sys/kernel/config/usb_gadget/g1"), repr(str(gadget))
            )
            for prefix in ("../../../../usb_gadget/g1/functions/", "../../functions/"):
                for index, name in enumerate(names, 1):
                    path = links / ("f" + str(index))
                    path.unlink(missing_ok=True)
                    path.symlink_to(prefix + name)
                exec(compile(code, "native-link-proof", "exec"), {})
            (links / "f4").unlink()
            (links / "f4").symlink_to("../../functions/ffs.bulk")
            with self.assertRaises(AssertionError):
                exec(compile(code, "native-link-proof", "exec"), {})

    def test_metadata_read_uses_owned_export_and_restores_fresh_regular_status_after_pull(
        self,
    ):
        control = Control()
        control.raw = b'{"metadata":true}'
        control.snapshot = Mock(return_value=control.last)
        with patch.object(
            m, "decode_metadata", return_value={"decoded": True}
        ) as decode:
            self.assertEqual(m._read_metadata(control), {"decoded": True})
        decode.assert_called_once_with(control.raw)
        control.snapshot.assert_called_once_with()
        self.assertEqual(len(control.commands), 1)
        self.assertIn("status.lock", control.commands[0])
        self.assertIn("m.export_status", control.commands[0])

    def test_duplicate_or_nonfinite_discovery_json_is_rejected(self):
        for raw in (b'{"m":[],"m":[],"w":[],"i":[1,2]}', b'{"m":NaN}', b"{}" * 17000):
            with self.assertRaises(ValueError):
                m._json(raw)


if __name__ == "__main__":
    unittest.main()
