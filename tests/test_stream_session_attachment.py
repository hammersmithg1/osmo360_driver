"""Pure NEW→SLEEP adoption: no control/native/network/data calls are permitted."""

from copy import deepcopy
import types
import unittest
from unittest.mock import Mock

from osmo360_camera import _stream_session as m
from test_standby_control import snapshot as native_snapshot
from test_stream_session import Environment


class SessionSleepAttachmentTests(unittest.TestCase):
    def setUp(self):
        self.env = Environment()
        self.value = native_snapshot("SLEEP", 1)
        self.env.control.owner, self.env.control.service = (
            self.value["owner"],
            self.value["service"],
        )
        self.env.control.last = deepcopy(self.value)
        self.env.control.launch_attempted = True
        self.env.control.pending_command = self.env.control.pending_task = None
        self.env.control._unknown_task = False
        self.env.control.bootstrap = types.SimpleNamespace(
            session=types.SimpleNamespace(quarantined=False)
        )
        self.session = self.env.session()
        self.env.clock.now = 7000  # Deliberately different from the camera epoch.
        self.prepared = types.SimpleNamespace(
            camera=self.env.camera,
            control=self.env.control,
            owner_receipt=self.session._owner_receipt(),
            prepared=True,
            closed=False,
            abandoned=False,
            arm_attempted=False,
            possible_future_apply=False,
            lease_proven=False,
            attach_attempted=False,
            imu_started=False,
            imu=types.SimpleNamespace(pid=None),
            preview=types.SimpleNamespace(_scheduled=False, _stream=None, _lease=None),
            sleep_id="0000000000000001",
            wake_id="0000000000000002",
            native_floor_ns=123456,
            report=dict(prepared=True, attached_existing=True),
        )
        self.env.control.snapshot = Mock(
            side_effect=AssertionError("Unexpected device query")
        )
        self.env.control.submit_wake = Mock(
            side_effect=AssertionError("Unexpected input")
        )
        self.env.camera.connect = Mock(
            side_effect=AssertionError("Unexpected USB open")
        )

    def attach(self, **options):
        return self.session.attach_sleep(
            self.prepared,
            native_floor_ns=options.pop("native_floor_ns", 123456),
            snapshot=options.pop("snapshot", deepcopy(self.value)),
            **options,
        )

    def test_adopts_exact_sleep_without_io_and_starts_a_new_local_host_epoch(self):
        self.assertIs(self.attach(), self.session)
        self.assertEqual(self.session.state, "SLEEP")
        self.assertIs(self.session._prepared, self.prepared)
        self.assertEqual(self.session._wake_id, "0000000000000002")
        self.assertEqual(self.session._native_floor_ns, 123456)
        self.assertEqual(self.session._stable_sleep_at, 7000)
        self.assertEqual(self.env.events, [])
        self.env.control.snapshot.assert_not_called()
        self.env.control.submit_wake.assert_not_called()
        self.env.camera.connect.assert_not_called()
        self.assertTrue(self.session.report["attached_existing"])
        self.assertTrue(self.session.owns_camera_resources)
        self.assertEqual(len(self.session.report["cycles"]), 1)
        with self.assertRaises(m.StreamSessionError):
            self.session.read_video(timeout=0)
        with self.assertRaises(m.StreamSessionError):
            self.session.read_imu(timeout=0)

    def test_current_strict_native_sleep_snapshot_and_same_owner_are_required(self):
        mutations = [
            lambda v: v.update(state="AWAKE"),
            lambda v: v["requested_readiness"].update(system_power_level=0),
            lambda v: v.update(owner=dict(pid=101, start="10")),
            lambda v: v["last_result"]["gui"].update(done3_after_intent=False),
            lambda v: v["at"].update(boottime=101),
        ]
        for change in mutations:
            value = deepcopy(self.value)
            change(value)
            with (
                self.subTest(change=change),
                self.assertRaises((ValueError, m.StreamSessionError)),
            ):
                self.attach(snapshot=value)
            self.assertEqual(self.session.state, "NEW")
        self.assertEqual(self.env.events, [])

    def test_wrong_prepared_generation_or_started_resources_are_rejected_without_io(
        self,
    ):
        for name, value in (
            ("wake_id", "0000000000000004"),
            ("sleep_id", "0000000000000003"),
            ("possible_future_apply", True),
            ("arm_attempted", True),
            ("lease_proven", True),
            ("closed", True),
            ("native_floor_ns", 123457),
        ):
            prior = getattr(self.prepared, name)
            setattr(self.prepared, name, value)
            with self.subTest(name=name), self.assertRaises(m.StreamSessionError):
                self.attach()
            setattr(self.prepared, name, prior)
            self.assertEqual(self.session.state, "NEW")
        self.prepared.owner_receipt["owner"] = dict(pid=101, start="10")
        with self.assertRaises(m.StreamSessionError):
            self.attach()
        self.assertEqual(self.env.events, [])

    def test_pending_input_or_unknown_d1_and_repeat_attach_cannot_claim_stable_sleep(
        self,
    ):
        for name, value in (
            ("pending_command", "0000000000000002"),
            ("pending_task", 22),
            ("_unknown_task", True),
        ):
            prior = getattr(self.env.control, name)
            setattr(self.env.control, name, value)
            with self.subTest(name=name), self.assertRaises(m.StreamSessionError):
                self.attach()
            setattr(self.env.control, name, prior)
        self.env.control.bootstrap.session.quarantined = True
        with self.assertRaises(m.StreamSessionError):
            self.attach()
        self.env.control.bootstrap.session.quarantined = False
        self.attach()
        with self.assertRaises(m.StreamSessionError):
            self.attach()
        self.assertEqual(self.env.events, [])
        for floor in (True, -1, 1.5):
            fresh = self.env.session()
            with self.assertRaises(ValueError):
                fresh.attach_sleep(
                    self.prepared, native_floor_ns=floor, snapshot=self.value
                )


if __name__ == "__main__":
    unittest.main()
