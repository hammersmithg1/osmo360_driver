"""Offline standby-controller transactions with an already-owned fake adapter."""

import hashlib
import json
import struct
import threading
import types
import unittest
from unittest.mock import patch

from osmo360_camera import _app
from osmo360_camera import _standby_control as m

NONCE = "aabbccddeeff"
OWNER = {"pid": 100, "start": "10"}
SERVICE = {"pid": 200, "start": "20"}
FILES = {
    "usb_standby_owner.py": b"# fake source for offline protocol tests\n",
    "gui_qs_exit_forward_probe.py": b"# fake frozen dependency\n",
}


def snapshot(state="AWAKE", sequence=0):
    action = "sleep" if state in ("SLEEP", "SLEEPING") else "wake"
    level = 3 if state == "SLEEP" else 0
    value = dict(
        nonce=NONCE,
        owner=OWNER,
        service=SERVICE,
        state=state,
        retained_shadow=True,
        native_restored=False,
        periodic_reboot=False,
        owner_alive=True,
        last_sequence=sequence,
        at=dict(boottime=100, monotonic=100, realtime=1000),
        requested_readiness=dict(
            service=SERVICE,
            verified_executable=m.SYS_SHA,
            clients_ever_ready=1,
            reboot_disabled=0,
            system_power_level=level,
        ),
        shadow=dict(source='/dev/shm/osmo-usb-standby-' + NONCE + '/role.txt',
            target='/sys/devices/platform/soc/a600000.ssusb/mode', inode=[23, 26],
            mount=dict(fs='tmpfs', point='/sys/devices/platform/soc/a600000.ssusb/mode',
                       root='/osmo-usb-standby-' + NONCE + '/role.txt'),
            value='none' if state == 'SLEEP' else 'exit'),
    )
    if sequence:
        value.update(command_id=f"{sequence:016x}", action=action)
        result = dict(
            nonce=NONCE,
            owner=OWNER,
            service=SERVICE,
            command_id=value["command_id"],
            action=action,
            complete=True,
            lpc_set_calls=0,
            get_level=level,
            pulse=dict(
                key=357,
                hold_seconds=1.1 if action == "sleep" else 0.1,
                press_release_complete=True,
            ),
        )
        if action == "sleep":
            result.update(
                shadow_value="none",
                gui=dict(processing3_after_intent=True, done3_after_intent=True),
            )
        else:
            result.update(
                shadow_value="exit",
                real_exit=dict(
                    completed=True,
                    bytes_written=5,
                    literal="exit",
                    real_role_after="peripheral",
                ),
            )
        value["last_result"] = result
    return value


def proof(at=0):
    return m._EntryProof(
        at,
        0,
        True,
        1,
        True,
        True,
        _app.query_payload(),
        _app.decode_query_reply(bytes.fromhex("12004400012300415050000000000002")),
        _app.decode_query_reply(bytes.fromhex("1200550000")),
    )


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Scenario:
    def __init__(self):
        self.events = []
        self.snapshots = [snapshot()]
        self.on_run = self.on_result = self.on_pull = None
        self.clock = Clock()
        self.transport = types.SimpleNamespace(request=self.request)

    def request(self, cmdset, cmdid, payload, **options):
        self.events.append(("RESULT", cmdset, cmdid, payload))
        raw = self.on_result() if self.on_result else struct.pack("<HBB", 5, 0xE0, 0xF3)
        return types.SimpleNamespace(payload=raw)

    def adapter(self, transport=None):
        return Adapter(self, transport or self.transport)


class Adapter:
    def __init__(self, scenario, transport):
        self.scenario, self.transport = scenario, transport
        self.session = types.SimpleNamespace(
            transport=transport,
            reuse_owned_id=True,
            quarantined=False,
            owned_task_id=37,
        )

    def run_checked(self, command, **options):
        if options.get("pre_send"):
            options["pre_send"]()
        self.scenario.events.append(("START", command))
        if self.scenario.on_run:
            self.scenario.on_run(self, command)
        return dict(completed=True, success=True, result=0)

    def pull_bytes(self, path, **options):
        self.scenario.events.append(("PULL", path))
        if self.scenario.on_pull:
            self.scenario.on_pull()
        return json.dumps(
            self.scenario.snapshots.pop(0)
            if len(self.scenario.snapshots) > 1
            else self.scenario.snapshots[0]
        ).encode()


class StandbyTests(unittest.TestCase):
    def setUp(self):
        self.source_patch = patch.dict(
            m.SOURCES,
            {name: hashlib.sha256(data).hexdigest() for name, data in FILES.items()},
            clear=True,
        )
        self.source_patch.start()
        self.s = Scenario()
        self.boot = self.s.adapter()
        self.c = m.StandbyControl(
            self.boot,
            source_provider=lambda: dict(FILES),
            control_lock=threading.RLock(),
            nonce=NONCE,
            clock=self.s.clock,
            sleep=self.s.clock.sleep,
            adapter_factory=self.s.adapter,
        )

    def tearDown(self):
        self.source_patch.stop()

    def launched(self):
        self.c.launch(recording_state=0, battery_present=True, activation_state=1)
        self.s.events.clear()

    def test_launch_uploads_exact_sources_once_and_never_opens_transport(self):
        value = self.c.launch(
            recording_state=0, battery_present=True, activation_state=1
        )
        self.assertEqual(value["state"], "AWAKE")
        commands = [row[1] for row in self.s.events if row[0] == "START"]
        self.assertEqual(sum(" launch " in command for command in commands), 1)
        self.assertTrue(
            all(len(command) <= 500 and command.isascii() for command in commands)
        )
        self.assertFalse(
            any(" sleep " in command or " wake " in command for command in commands)
        )
        with self.assertRaises(m.StandbyControlError):
            self.c.launch(recording_state=0, battery_present=True, activation_state=1)
        self.assertEqual(
            commands, [row[1] for row in self.s.events if row[0] == "START"]
        )

    def test_corrupt_or_extra_source_fails_before_any_ram_command(self):
        for supplied in (
            dict(FILES, **{"usb_standby_owner.py": b"corrupt"}),
            dict(FILES, other=b"bad"),
        ):
            self.c.sources = lambda: supplied
            with self.assertRaises(ValueError):
                self.c.launch(
                    recording_state=0, battery_present=True, activation_state=1
                )
        self.assertEqual(self.s.events, [])

    def test_partial_upload_is_never_restarted_and_detach_does_not_claim_mount(self):
        def fail(boot, command):
            if "base64 -d" in command:
                boot.session.quarantined = True
                raise OSError("lost upload reply")

        self.s.on_run = fail
        with self.assertRaises(OSError):
            self.c.launch(recording_state=0, battery_present=True, activation_state=1)
        events = list(self.s.events)
        with self.assertRaises(m.StandbyControlError):
            self.c.launch(recording_state=0, battery_present=True, activation_state=1)
        self.assertEqual(events, self.s.events)
        receipt = self.c.detach()
        self.assertFalse(receipt["retained_shadow"])
        self.assertFalse(receipt["reboot_requested"])

    def test_private_sleep_requires_correlated_teardown_and_exact_next_command(self):
        self.launched()
        with self.assertRaises(TypeError):
            self.c._submit_sleep(True)
        bad = proof().__class__(
            0, 0, True, 1, True, True, b"wrong", proof().app_before, proof().app_after
        )
        with self.assertRaises(ValueError):
            self.c._submit_sleep(bad)
        result = self.c._submit_sleep(proof())
        self.assertEqual(result["command_id"], "0000000000000001")
        self.assertFalse(result["input_completion_proven"])
        commands = [event[1] for event in self.s.events if event[0] == "START"]
        sent = [command for command in commands if " sleep " in command]
        self.assertEqual(len(sent), 1)
        self.assertIn("--streams-closed --own-app-absent", sent[0])

    def test_slow_fresh_status_cannot_send_a_now_stale_sleep_proof(self):
        self.launched()
        self.s.on_pull = lambda: setattr(self.s.clock, "now", 6)
        with self.assertRaises(ValueError):
            self.c._submit_sleep(proof())
        self.assertFalse(
            any(row[0] == "START" and " sleep " in row[1] for row in self.s.events)
        )
        self.assertIsNone(self.c.pending_command)

    def test_proof_is_checked_again_after_presend_readonly_resolution_budget(self):
        self.launched()
        calls = [0]

        def resolve(**options):
            calls[0] += 1
            if calls[0] == 2:
                self.s.clock.now = (
                    6  # STATUS was fresh; later RO work consumed the proof budget.
                )

        with patch.object(self.c, "_resolve_pending", side_effect=resolve):
            with self.assertRaises(ValueError):
                self.c._submit_sleep(proof())
        self.assertFalse(
            any(row[0] == "START" and " sleep " in row[1] for row in self.s.events)
        )
        self.assertIsNone(self.c.pending_command)

    def test_lost_wake_receipt_never_resubmits_key_and_ro_terminal_precedes_new_start(
        self,
    ):
        self.launched()
        self.s.snapshots = [snapshot("SLEEP", 1), snapshot("AWAKE", 2)]

        def lost(boot, command):
            if " wake " in command:
                boot.session.quarantined = True
                raise OSError("USB reset lost reply")

        self.s.on_run = lost
        with self.assertRaises(m.StandbyUncertain) as got:
            self.c.submit_wake()
        self.assertEqual(got.exception.command_id, "0000000000000002")
        self.assertEqual(self.c.pending_command, "0000000000000002")
        self.s.on_run = None
        value = self.c.wait("0000000000000002", "AWAKE")
        self.assertEqual(value["state"], "AWAKE")
        events = self.s.events
        self.assertEqual(
            sum(row[0] == "START" and " wake " in row[1] for row in events), 1
        )
        where = next(i for i, row in enumerate(events) if row[0] == "RESULT")
        self.assertEqual(events[where][1:3], (0, 0xD1))
        self.assertTrue(
            any(
                row[0] == "START" and row[1].endswith(" status")
                for row in events[where + 1 :]
            )
        )

    def test_running_prior_task_blocks_every_new_start_and_retains_input(self):
        self.launched()
        self.boot.session.quarantined = True
        self.c.pending_command = "0000000000000002"
        self.s.on_result = lambda: struct.pack("<HBB", 5, 0, 0xF0)
        with self.assertRaises(m.StandbyUncertain):
            self.c.resolve_pending(timeout=0.1)
        self.assertTrue(self.s.events)
        self.assertTrue(all(row[0] == "RESULT" for row in self.s.events))
        self.assertEqual(self.c.pending_task, 37)
        self.assertEqual(self.c.pending_command, "0000000000000002")

    def test_late_terminal_result_does_not_authorize_new_adapter_or_start(self):
        self.launched()
        self.boot.session.quarantined = True

        def late():
            self.s.clock.now += 6
            return struct.pack("<HBB", 5, 0, 0)

        self.s.on_result = late
        with self.assertRaises(m.StandbyUncertain):
            self.c.resolve_pending(timeout=5)
        self.assertIs(self.c.bootstrap, self.boot)
        self.assertTrue(all(row[0] == "RESULT" for row in self.s.events))

    def test_wait_cannot_accept_terminal_snapshot_returned_after_deadline(self):
        self.launched()
        self.s.snapshots = [snapshot("AWAKE", 2)]
        self.s.on_pull = lambda: setattr(self.s.clock, "now", 3)
        with self.assertRaises(m.StandbyUncertain):
            self.c.wait("0000000000000002", "AWAKE", timeout=2)

    def test_external_button_level_change_rejected_before_any_qs(self):
        self.launched()
        bad = snapshot("SLEEP", 1)
        bad["requested_readiness"]["system_power_level"] = 0
        self.s.snapshots = [bad]
        with self.assertRaises(ValueError):
            self.c.submit_wake()
        self.assertFalse(
            any(row[0] == "START" and " wake " in row[1] for row in self.s.events)
        )

    def test_detach_and_explicit_reattach_never_relaunch_or_reboot(self):
        self.launched()
        self.s.snapshots = [snapshot("SLEEP", 1)]
        self.c.snapshot()
        receipt = self.c.detach()
        before = list(self.s.events)
        self.assertTrue(receipt["retained_shadow"])
        self.assertFalse(receipt["native_restored"])
        self.assertFalse(receipt["reboot_requested"])
        attached = m.StandbyControl(
            self.s.adapter(),
            source_provider=lambda: self.fail("no upload on attach"),
            control_lock=threading.RLock(),
            existing=receipt,
            clock=self.s.clock,
            sleep=self.s.clock.sleep,
            adapter_factory=self.s.adapter,
        )
        self.assertEqual(attached.snapshot()["state"], "SLEEP")
        commands = [row[1] for row in self.s.events[len(before) :] if row[0] == "START"]
        self.assertFalse(
            any(
                "mkdir" in command or " launch " in command or "finalize" in command
                for command in commands
            )
        )
        self.assertEqual(sum("sha256sum" in command for command in commands), 2)

    def test_finalization_is_one_explicit_request_never_restoration_claim(self):
        self.launched()
        result = self.c.finalize()
        self.assertTrue(result["reboot_requested"])
        self.assertFalse(result["restored"])
        with self.assertRaises(m.StandbyControlError):
            self.c.finalize()
        self.assertEqual(
            sum(
                row[0] == "START" and row[1].endswith(" finalize")
                for row in self.s.events
            ),
            1,
        )

    def test_blocked_old_task_does_not_retire_an_unsent_finalize_request(self):
        self.launched()
        self.boot.session.quarantined = True
        self.s.on_result = lambda: struct.pack("<HBB", 5, 0, 0xF0)
        with self.assertRaises(m.StandbyUncertain):
            self.c.finalize()
        self.assertFalse(self.c.finalize_attempted)
        self.assertFalse(
            any(
                row[0] == "START" and row[1].endswith(" finalize")
                for row in self.s.events
            )
        )
        self.s.on_result = None  # Old task is now conclusively missing.
        self.assertTrue(self.c.finalize()["reboot_requested"])
        self.assertEqual(
            sum(
                row[0] == "START" and row[1].endswith(" finalize")
                for row in self.s.events
            ),
            1,
        )

    def test_lost_actually_delivered_finalize_stays_retired(self):
        self.launched()

        def lost(boot, command):
            if command.endswith(" finalize"):
                boot.session.quarantined = True
                raise OSError("reply lost after START")

        self.s.on_run = lost
        with self.assertRaises(OSError):
            self.c.finalize()
        self.assertTrue(self.c.finalize_attempted)
        with self.assertRaises(m.StandbyControlError):
            self.c.finalize()
        self.assertEqual(
            sum(
                row[0] == "START" and row[1].endswith(" finalize")
                for row in self.s.events
            ),
            1,
        )


class SnapshotTests(unittest.TestCase):
    def test_actual_shape_sleep_wake_and_native_manager_guards(self):
        for original in (
            snapshot(),
            snapshot("SLEEP", 1),
            snapshot("AWAKE", 2),
            snapshot("WAKING", 2),
        ):
            self.assertEqual(
                m.parse_snapshot(json.dumps(original).encode(), nonce=NONCE), original
            )
        for modify in (
            lambda value: value.update(nonce="foreign"),
            lambda value: value.update(owner_alive=False),
            lambda value: value.update(periodic_reboot=True),
            lambda value: value.update(last_sequence=True),
            lambda value: value["requested_readiness"].update(clients_ever_ready=0),
            lambda value: value["requested_readiness"].update(clients_ever_ready=True),
            lambda value: value["requested_readiness"].update(reboot_disabled=1),
            lambda value: value["last_result"]["gui"].update(done3_after_intent=False),
            lambda value: value["last_result"].update(command_id="0000000000000002"),
        ):
            value = snapshot("SLEEP", 1)
            modify(value)
            with self.assertRaises((ValueError, m.StandbyControlError)):
                m.parse_snapshot(json.dumps(value).encode(), nonce=NONCE)

    def test_duplicate_json_nonfinite_bounds_and_owner_replacement_fail_closed(self):
        good = json.dumps(snapshot()).encode()
        bad = good[:-1] + b', "nonce": "aabbccddeeff"}'
        for data in (bad, b"{}" * 9000, good.replace(b"1000", b"NaN")):
            with self.assertRaises(ValueError):
                m.parse_snapshot(data, nonce=NONCE)
        with self.assertRaises(ValueError):
            m.parse_snapshot(good, nonce=NONCE, owner={"pid": 101, "start": "11"})


if __name__ == "__main__":
    unittest.main()
