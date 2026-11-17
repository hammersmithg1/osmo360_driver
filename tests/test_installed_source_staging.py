"""Installed source capability preserves existing fresh-work/lifecycle guards.

Only fake adapters and source bytes are used. No filesystem, USB or device
installation is performed by these tests.
"""

import hashlib
import threading
import types
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _standby_control as standby
from osmo360_camera import _prepared_stream as prepared
from osmo360_camera import _retained_stream as retained
import test_standby_control as owner_fixtures
import test_prepared_stream as asset_fixtures


class InstalledStagingTests(unittest.TestCase):
    def owner(self, stager=None, *, invalid_source=False):
        scenario = owner_fixtures.Scenario()
        files = dict(owner_fixtures.FILES)
        digests = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        source_patch = patch.dict(standby.SOURCES, digests, clear=True)
        source_patch.start()
        self.addCleanup(source_patch.stop)
        if invalid_source:
            files[next(iter(files))] += b"# not approved\n"
        control = standby.StandbyControl(
            scenario.adapter(),
            source_provider=lambda: dict(files),
            source_stager=stager,
            control_lock=threading.RLock(),
            nonce=owner_fixtures.NONCE,
            clock=scenario.clock,
            sleep=scenario.clock.sleep,
            adapter_factory=scenario.adapter,
        )
        return control, scenario, files

    @staticmethod
    def starts(scenario):
        return [row[1] for row in scenario.events if row[0] == "START"]

    def launch(self, control):
        return control.launch(
            recording_state=0, battery_present=True, activation_state=1
        )

    def assets(self, module, stager):
        events = []
        camera = types.SimpleNamespace(
            _control_lock=threading.RLock(), _transport=object()
        )
        control = asset_fixtures.Control(camera, events)
        control.source_stager = stager
        # Exercise the actual boolean/locking contract over this fake adapter.
        control.stage_sources = types.MethodType(
            standby.StandbyControl.stage_sources, control
        )
        files = {
            name: ("# exact installed " + name + "\n").encode()
            for name in module.SOURCES
        }
        digests = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        source_patch = patch.dict(module.SOURCES, digests, clear=True)
        source_patch.start()
        self.addCleanup(source_patch.stop)
        if module is prepared:
            value = prepared.PreparedAssets(
                control,
                asset_fixtures.RECEIPT,
                lambda: dict(files),
                nonce="012345abcdef",
            )
        else:
            value = retained.RetainedAssets(
                control, asset_fixtures.RECEIPT, lambda: dict(files)
            )
        return value, control, files

    def test_installed_owner_skips_payload_upload_but_fresh_work_and_single_init_remain(
        self,
    ):
        calls = []

        def stage(control, files, work, *, guard):
            self.assertTrue(control.lock._is_owned())
            self.assertIsNone(guard)
            self.assertEqual(work, control.work)
            self.assertIn("mkdir " + work, self.starts(scenario)[-1])
            calls.append((control, files, work))
            return True

        control, scenario, files = self.owner(stage)
        value = self.launch(control)
        commands = self.starts(scenario)
        self.assertEqual(value["state"], "AWAKE")
        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0][0], control)
        self.assertEqual(calls[0][1], files)
        self.assertEqual(sum("mkdir " in command for command in commands), 1)
        self.assertEqual(sum(" launch " in command for command in commands), 1)
        self.assertFalse(
            any("base64 -d" in command or "set -C" in command for command in commands)
        )
        with self.assertRaises(standby.StandbyControlError):
            self.launch(control)
        self.assertEqual(len(calls), 1)

    def test_explicit_absence_has_exact_legacy_owner_upload_sequence(self):
        control, scenario, _ = self.owner()
        self.launch(control)
        legacy = self.starts(scenario)
        absent = Mock(return_value=False)
        control, scenario, _ = self.owner(absent)
        self.launch(control)
        self.assertEqual(self.starts(scenario), legacy)
        self.assertEqual(sum("base64 -d" in command for command in legacy), 2)
        absent.assert_called_once()

    def test_partial_foreign_or_nonboolean_staging_never_falls_back_or_initializes(
        self,
    ):
        for outcome in ("partial/foreign", None, 1):
            with self.subTest(outcome=outcome):
                stage = (
                    Mock(side_effect=RuntimeError(outcome))
                    if isinstance(outcome, str)
                    else Mock(return_value=outcome)
                )
                control, scenario, _ = self.owner(stage)
                with self.assertRaises((TypeError, RuntimeError)):
                    self.launch(control)
                self.assertTrue(control.launch_attempted)
                commands = self.starts(scenario)
                self.assertEqual(len(commands), 1)  # Only our fresh directory.
                self.assertFalse(
                    any(" launch " in c or "base64 -d" in c for c in commands)
                )
                with self.assertRaises(standby.StandbyControlError):
                    self.launch(control)
                stage.assert_called_once()

    def test_unapproved_local_owner_source_is_rejected_before_hook_or_mutation(self):
        stage = Mock(return_value=True)
        control, scenario, _ = self.owner(stage, invalid_source=True)
        with self.assertRaises(ValueError):
            self.launch(control)
        stage.assert_not_called()
        self.assertEqual(self.starts(scenario), [])
        self.assertFalse(control.launch_attempted)

    def test_stager_uses_shared_control_lock_and_checks_budget_before_and_after_callback(
        self,
    ):
        events = []
        expired = [False]

        def guard():
            events.append("guard")
            if expired[0]:
                raise TimeoutError("budget")

        def stage(control, files, work, *, guard):
            self.assertTrue(control.lock._is_owned())
            self.assertIs(guard, checked)
            events.append("stage")
            return True

        checked = guard
        control, _, files = self.owner(stage)
        self.assertTrue(control.stage_sources(files, control.work, guard=guard))
        self.assertEqual(events, ["guard", "stage", "guard"])
        events.clear()
        expired[0] = True
        with self.assertRaises(TimeoutError):
            control.stage_sources(files, control.work, guard=guard)
        self.assertEqual(events, ["guard"])
        events.clear()
        expired[0] = False

        def late(control, files, work, *, guard):
            events.append("stage")
            expired[0] = True
            return True

        control.source_stager = late
        with self.assertRaises(TimeoutError):
            control.stage_sources(files, control.work, guard=guard)
        self.assertEqual(events, ["guard", "stage", "guard"])

    def test_both_asset_groups_skip_payload_but_write_fresh_boot_metadata_and_modes(
        self,
    ):
        for module in (prepared, retained):
            with self.subTest(module=module.__name__):
                calls = []

                def stage(control, files, work, *, guard):
                    self.assertTrue(control.lock._is_owned())
                    self.assertTrue(
                        any("mkdir " + work in command for command in control.commands)
                    )
                    self.assertNotIn("cache.json", files)
                    calls.append((dict(files), work))
                    return True

                assets, control, files = self.assets(module, stage)
                assets.preload_assets()
                self.assertTrue(assets.staged)
                self.assertEqual(calls, [(files, assets.work)])
                self.assertFalse(
                    any("base64 -d" in command for command in control.commands)
                )
                self.assertEqual(
                    sum("chmod 400 " in command for command in control.commands),
                    len(files) + 1,
                )
                metadata = [
                    command
                    for command in control.commands
                    if "json.dump(c,f)" in command
                ]
                self.assertEqual(len(metadata), 1)
                self.assertIn("config.json", metadata[0])
                self.assertIn("boot_id", metadata[0])
                self.assertIn(control.work, metadata[0])
                self.assertIn(assets.work, metadata[0])
                before = list(control.commands)
                assets.preload_assets()
                self.assertEqual(control.commands, before)
                self.assertEqual(len(calls), 1)

    def test_asset_group_absence_uploads_each_exact_source_once(self):
        for module in (prepared, retained):
            with self.subTest(module=module.__name__):
                stage = Mock(return_value=False)
                assets, control, files = self.assets(module, stage)
                assets.preload_assets()
                self.assertEqual(
                    sum("base64 -d" in command for command in control.commands),
                    len(files),
                )
                stage.assert_called_once()
                self.assertTrue(assets.staged)

    def test_unapproved_local_asset_bytes_never_reach_installed_hook_or_mutation(self):
        for module in (prepared, retained):
            with self.subTest(module=module.__name__):
                stage = Mock(return_value=True)
                assets, control, files = self.assets(module, stage)
                files[next(iter(files))] += b"# not approved\n"
                with self.assertRaises(ValueError):
                    assets.preload_assets()
                stage.assert_not_called()
                self.assertEqual(control.commands, [])
                self.assertFalse(assets.attempted)
                self.assertFalse(assets.staged)

    def test_asset_partial_or_nonboolean_stage_never_uploads_metadata_or_retries(self):
        for module in (prepared, retained):
            for outcome in ("partial/foreign", 1):
                with self.subTest(module=module.__name__, outcome=outcome):
                    stage = (
                        Mock(side_effect=RuntimeError(outcome))
                        if isinstance(outcome, str)
                        else Mock(return_value=outcome)
                    )
                    assets, control, _ = self.assets(module, stage)
                    with self.assertRaises((TypeError, RuntimeError)):
                        assets.preload_assets()
                    self.assertFalse(assets.staged)
                    self.assertTrue(assets.attempted)
                    self.assertFalse(
                        any(
                            "base64 -d" in command or "json.dump(c,f)" in command
                            for command in control.commands
                        )
                    )
                    before = list(control.commands)
                    with self.assertRaises(prepared.PreparedStreamError):
                        assets.preload_assets()
                    self.assertEqual(control.commands, before)
                    stage.assert_called_once()


if __name__ == "__main__":
    unittest.main()
