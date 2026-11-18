"""Persistent helper host protocol with fake D1 and file exports only."""

from copy import deepcopy
import importlib.util
import json
import shlex
import threading
import types
import unittest
from unittest.mock import patch

from osmo360_camera import _helper_install as m
from test_prepared_stream import runtime

BOOT1 = "11111111-1111-1111-1111-111111111111"
BOOT2 = "22222222-2222-2222-2222-222222222222"


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Control:
    def __init__(self, package, *, installed=False):
        self.package, self.installed = package, installed
        self.clock, self.commands, self.outputs, self.operations = Clock(), [], {}, []
        self.sleep = self.clock.sleep

        def query_runtime(*, budget):
            budget()
            return runtime()

        self.bootstrap = types.SimpleNamespace(
            pull_bytes=lambda path, **kw: self.outputs[path],
            query_runtime=query_runtime,
        )
        self.boot_id, self.hydrated = BOOT1, False
        self.failure, self.mutate, self.after_send = None, None, None

    def response(self, operation, nonce):
        value = dict(
            schema=1,
            operation=operation,
            nonce=nonce,
            installed=self.installed,
            profile=self.package["manifest"]["profile"] if self.installed else None,
            export_identity=dict(device=1, inode=2),
            boot_id=self.boot_id,
        )
        if self.installed:
            value.update(
                manifest=deepcopy(self.package["manifest"]),
                autostart=dict(
                    verified=True,
                    hook=m.BOOT_HOOK,
                    backup_sha256=m.STOCK_HOOK_SHA,
                    current_sha256="ab" * 32,
                    enabled=True,
                ),
                boot_cache=dict(
                    hydrated=self.hydrated,
                    boot_id=self.boot_id,
                    profile=self.package["manifest"]["profile"],
                    mode="bytes-only",
                    work="/dev/shm/osmo360-sdk-cache-"
                    + self.package["manifest"]["profile"][:12],
                ),
            )
        return value

    def _run(self, command, **options):
        if options.get("pre_send"):
            options["pre_send"]()
        if len(command) > 500:
            raise AssertionError("Native task command exceeds500 bytes")
        self.commands.append(command)
        if self.failure:
            self.failure(command)
        parts = shlex.split(command)
        program = parts[2] if parts[:2] == ["python3", "-c"] else ""
        if "root_exists" in program:
            output, nonce = parts[-2:]
            value = self.response("probe", nonce)
            value.update(installed=False, profile=None, root_exists=self.installed)
            value.pop("manifest", None)
            self.outputs[output] = json.dumps(value).encode()
        elif "glob.glob('/dev/shm/osmo*')" in program:
            self.outputs[parts[-1]] = json.dumps(
                [[self.boot_id, self.installed, [], m.STOCK_HOOK_SHA], 1, 2]
            ).encode()
        elif "-B" in parts and "helper_install.py" in parts:
            operation = parts[parts.index("helper_install.py", parts.index("-B")) + 1]
            nonce = parts[parts.index("--nonce") + 1]
            self.operations.append(operation)
            if operation == "install":
                work = parts[parts.index("--work") + 1]
                if not work.endswith(nonce):
                    raise AssertionError("Install work and nonce must match")
                self.installed = True
            if operation == "remove":
                self.installed = False
            value = self.response(operation, nonce)
            if operation == "stage":
                group = parts[parts.index("--group") + 1]
                value.update(
                    staged=True,
                    group=group,
                    dest=parts[parts.index("--dest") + 1],
                    files={
                        name: self.package["manifest"]["files"][name]
                        for name in m.GROUPS[group]
                    },
                )
            if operation == "remove":
                value.update(removed=True, reboot_required=True)
            if self.mutate:
                self.mutate(value)
            self.outputs["/blackbox/export_tmp/osmo_helpers_" + nonce + ".json"] = (
                json.dumps(value).encode()
            )
        if self.after_send:
            self.after_send(command)
        return dict(completed=True, success=True, result=0)


class HelperHostTests(unittest.TestCase):
    def setUp(self):
        self.package = m.package_profile()
        self.camera = types.SimpleNamespace(_control_lock=threading.RLock())

    def test_package_profile_and_bundle_match_device_decoder_exactly(self):
        path = m.DEVICE / m.INSTALLER
        spec = importlib.util.spec_from_file_location(
            "installed_device_decoder_test", path
        )
        device = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(device)
        manifest, files = device.decode_bundle(
            self.package["packed"], json.loads(self.package["request"])
        )
        self.assertEqual(manifest, self.package["manifest"])
        self.assertEqual(files, self.package["sources"])
        self.assertEqual(sum(len(raw) for raw in files.values()), 182282)
        self.assertLessEqual(len(self.package["packed"]), 65536)

    def test_absent_root_falls_back_without_running_unknown_installer_or_staging(self):
        control = Control(self.package)
        stage = m.InstalledSources(self.package)
        files = {name: self.package["sources"][name] for name in m.GROUPS["owner"]}
        self.assertFalse(
            stage(control, files, "/dev/shm/osmo-usb-standby-aaaaaaaaaaaa")
        )
        self.assertEqual(control.operations, [])
        self.assertTrue(any("os.unlink" in command for command in control.commands))
        count = len(control.commands)
        self.assertFalse(
            stage(control, files, "/dev/shm/osmo-usb-standby-bbbbbbbbbbbb")
        )
        self.assertEqual(len(control.commands), count)

    def test_installed_groups_use_verified_copy_without_payload_upload_or_old_metadata(
        self,
    ):
        control = Control(self.package, installed=True)
        stage = m.InstalledSources(self.package)
        for group, destination in (
            ("owner", "/dev/shm/osmo-usb-standby-aaaaaaaaaaaa"),
            ("prepared", "/dev/shm/osmo-prepared-assets-bbbbbbbbbbbb"),
            (
                "retained",
                "/dev/shm/osmo-usb-standby-aaaaaaaaaaaa/retained-assets-0e07a4db5754",
            ),
        ):
            files = {name: self.package["sources"][name] for name in m.GROUPS[group]}
            self.assertTrue(stage(control, files, destination))
        self.assertEqual(control.operations, ["status", "stage", "stage", "stage"])
        self.assertFalse(any("base64 -d" in command for command in control.commands))
        self.assertTrue(all(len(command) <= 500 for command in control.commands))
        self.assertNotIn("config.json", "".join(control.commands))

    def test_present_partial_or_foreign_manifest_never_falls_back_or_stages(self):
        for mutate in (
            lambda value: value.update(partial=True),
            lambda value: value.update(profile="ff" * 32),
            lambda value: value["manifest"].update(installer_sha256="ff" * 32),
            lambda value: value["autostart"].update(verified=False),
            lambda value: value["autostart"].update(enabled=False),
        ):
            with self.subTest(mutate=mutate):
                control = Control(self.package, installed=True)
                control.mutate = mutate
                with self.assertRaises(m.HelperInstallError):
                    m.InstalledSources(self.package).status(control)
                self.assertEqual(control.operations, ["status"])

    def test_stage_mismatch_or_lost_receipt_is_not_replayed(self):
        files = {name: self.package["sources"][name] for name in m.GROUPS["owner"]}
        for lost in (False, True):
            with self.subTest(lost=lost):
                control = Control(self.package, installed=True)
                stage = m.InstalledSources(self.package)
                stage.status(control)
                if lost:
                    control.failure = lambda command: (
                        (_ for _ in ()).throw(OSError("Lost stage receipt"))
                        if " stage " in command
                        else None
                    )
                    error = OSError
                else:
                    control.mutate = lambda value: value.update(dest="/wrong")
                    error = m.HelperInstallError
                with self.assertRaises(error) as raised:
                    stage(control, files, "/dev/shm/osmo-usb-standby-aaaaaaaaaaaa")
                if lost:
                    self.assertIs(raised.exception.helper_control, control)
                self.assertEqual(
                    sum(" stage " in command for command in control.commands), 1
                )

    def test_install_is_idempotent_and_once_nonce_bound_to_fresh_bootstrap(self):
        control = Control(self.package)
        with (
            patch.object(m, "package_profile", return_value=self.package),
            patch.object(m, "_existing_owner", return_value=None),
        ):
            first = m.install(self.camera, control=control)
            self.assertEqual(first["previous_boot_id"], BOOT1)
            self.assertTrue(first["installed"])
            count = sum("base64 -d" in command for command in control.commands)
            second = m.install(self.camera, control=control)
        self.assertTrue(second["already_installed"])
        self.assertEqual(control.operations.count("install"), 1)
        self.assertEqual(
            sum("base64 -d" in command for command in control.commands), count
        )

    def test_remove_preserves_same_control_and_owner_for_explicit_facade_reboot(self):
        control = Control(self.package, installed=True)
        snapshot = dict(
            state="SLEEP",
            nonce="aa" * 6,
            owner=dict(pid=5, start="6"),
            service=dict(pid=7, start="8"),
        )
        with (
            patch.object(m, "package_profile", return_value=self.package),
            patch(
                "osmo360_camera._stream_resume._discover",
                return_value="/dev/shm/osmo-usb-standby-" + "aa" * 6,
            ),
            patch("osmo360_camera._stream_resume._status_at", return_value=snapshot),
        ):
            result = m.remove(self.camera, control=control)
        self.assertTrue(result["requires_reboot"])
        self.assertEqual(result["previous_boot_id"], BOOT1)
        self.assertIs(result["_control"], control)
        self.assertEqual(result["owner_receipt"]["owner"], snapshot["owner"])
        self.assertEqual(control.operations, ["remove"])
        self.assertFalse(
            any(
                " reboot" in command or "umount" in command
                for command in control.commands
            )
        )

    def test_new_boot_verified_hydration_required_not_only_persistent_installed_status(
        self,
    ):
        control = Control(self.package, installed=True)
        first = [True]

        def after(command):
            if "os.unlink" in command and first:
                first.pop()
                control.boot_id, control.hydrated = BOOT2, True

        control.after_send = after
        with patch.object(m, "package_profile", return_value=self.package):
            result = m.verify_installed(
                self.camera,
                self.package["manifest"]["profile"],
                BOOT1,
                timeout=2,
                control=control,
            )
        self.assertTrue(result["autostart_hydration_verified"])
        self.assertEqual(result["boot_id"], BOOT2)
        self.assertNotIn("install", control.operations)

    def test_old_boot_or_missing_hydration_times_out_without_mutating_installation(
        self,
    ):
        control = Control(self.package, installed=True)
        with (
            patch.object(m, "package_profile", return_value=self.package),
            self.assertRaises(TimeoutError),
        ):
            m.verify_installed(
                self.camera,
                self.package["manifest"]["profile"],
                BOOT1,
                timeout=0.12,
                control=control,
            )
        self.assertEqual(set(control.operations), {"status"})
        self.assertTrue(control.installed)

    def test_remove_verification_requires_new_boot_stock_hook_ram_absence_and_role_proof(
        self,
    ):
        control = Control(self.package)
        control.boot_id = BOOT2
        result = m.verify_removed(self.camera, BOOT1, control=control)
        self.assertTrue(result["role_unshadowed"])
        self.assertTrue(result["diagnostic_cleaned"])
        self.assertTrue(
            any(
                "mountinfo" in command and "peripheral" in command
                for command in control.commands
            )
        )
        self.assertFalse(any("base64 -d" in command for command in control.commands))
        control.boot_id = BOOT1
        with self.assertRaises(m.HelperInstallError):
            m.verify_removed(self.camera, BOOT1, control=control)

    def test_late_or_exhausted_budget_never_adds_another_native_start(self):
        control = Control(self.package)
        calls = m._Calls(control, timeout=0.1, clock=control.clock)
        control.clock.now = 0.1
        with self.assertRaises(TimeoutError):
            calls.run("true")
        self.assertEqual(control.commands, [])
        control.clock.now = 0
        control.after_send = lambda command: setattr(control.clock, "now", 1)
        with self.assertRaises(TimeoutError):
            calls.run("true")
        self.assertEqual(control.commands, ["true"])

    def test_invalid_inputs_fail_before_any_camera_action_or_native_request(self):
        for timeout in (True, 0, -1, float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                m.install(self.camera, timeout=timeout)
        control = Control(self.package, installed=True)
        files = {name: self.package["sources"][name] for name in m.GROUPS["owner"]}
        with self.assertRaises(ValueError):
            m.InstalledSources(self.package)(control, files, "/tmp/foreign")
        files[next(iter(files))] = b"changed"
        with self.assertRaises(ValueError):
            m.InstalledSources(self.package)(
                control, files, "/dev/shm/osmo-usb-standby-aaaaaaaaaaaa"
            )
        self.assertEqual(control.commands, [])


if __name__ == "__main__":
    unittest.main()
