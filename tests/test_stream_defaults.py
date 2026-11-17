"""Default wiring/generation recovery with pure fake adapters, never USB."""

import struct
import tempfile
import threading
import types
from pathlib import Path
import unittest
from unittest.mock import patch

from osmo360_camera import _stream_defaults as m
from osmo360_camera._standby_control import StandbyUncertain


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Environment:
    def __init__(self):
        self.events, self.adapters, self.clock = [], [], Clock()
        self.activation = [b"\0\1" + bytes(58)]
        self.states = [0]
        self.old_task_running = False
        self.transport = types.SimpleNamespace(
            connected=False, _thread=None, _device_identity=None
        )

        def open():
            self.events.append("open")
            self.transport.connected = True
            self.transport._thread = object()
            self.transport._device_identity = dict(bus=1, address=2)

        def request(cmdset, cmdid, payload=b"", **options):
            if cmdid == 0x32:
                self.events.append(("activation", cmdset, payload, options["receiver"]))
                return types.SimpleNamespace(
                    payload=self.activation.pop(0)
                    if len(self.activation) > 1
                    else self.activation[0]
                )
            if cmdid == 0xD1:
                self.events.append("old.RESULT")
                return types.SimpleNamespace(
                    payload=struct.pack(
                        "<HBB", 5, 0, 0xF0 if self.old_task_running else 0
                    )
                )
            raise AssertionError("Unexpected fake protocol command")

        self.transport.open, self.transport.request = open, request

        def state(**options):
            self.events.append("state.LAST")
            value = self.states.pop(0) if len(self.states) > 1 else self.states[0]
            if isinstance(value, Exception):
                raise value
            return types.SimpleNamespace(recording_state=value)

        self.camera = types.SimpleNamespace(
            _transport=self.transport, _control_lock=threading.RLock(), state=state
        )

    def session(self, transport, **options):
        return types.SimpleNamespace(
            transport=transport,
            reuse_owned_id=True,
            quarantined=False,
            owned_task_id=37,
        )

    def bootstrap(self, transport, *, session):
        env = self

        class Bootstrap:
            def __init__(self):
                self.transport, self.session = transport, session

            def run_checked(self, command, **options):
                if options.get("pre_send"):
                    options["pre_send"]()
                env.events.append(("START", command))
                return dict(completed=True, result=0, success=True)

        value = Bootstrap()
        self.adapters.append(value)
        return value

    def ready(self, transport, **options):
        self.events.append("missing.RESULT.ready")
        return dict(ready=True)


class DefaultsTests(unittest.TestCase):
    def setUp(self):
        self.env = Environment()
        self.patches = [
            patch.object(m, "D1Session", side_effect=self.env.session),
            patch.object(m, "USBBootstrap", side_effect=self.env.bootstrap),
            patch.object(m, "wait_amt_ready", side_effect=self.env.ready),
        ]
        for item in self.patches:
            item.start()

    def tearDown(self):
        for item in reversed(self.patches):
            item.stop()

    def build(self, **options):
        value = m.build_stream_session(self.env.camera, decode=False, **options)
        manager = value.idle_probe.__self__
        manager.clock, manager.sleep = self.env.clock, self.env.clock.sleep
        value.control.clock, value.control.sleep = self.env.clock, self.env.clock.sleep
        return value, manager

    def opened(self):
        session, manager = self.build()
        self.env.transport.open()
        return session, manager

    def test_build_is_pure_and_wires_pinned_defaults(self):
        session, manager = self.build()
        self.assertEqual(self.env.events, [])
        self.assertEqual(
            len(self.env.adapters), 1
        )  # Pure construction, no native task.
        self.assertEqual(
            session.imu_options, dict(read_buffer_bytes=65536, queue_size=16384)
        )
        self.assertEqual(
            session.preview_options, dict(decode=False, queue_size=1, timeout=60)
        )
        self.assertEqual(set(session.control.sources()), set(m.OWNER_SOURCES))
        self.assertEqual(
            set(session.prepared_factory.source_provider()), set(m.WATCHER_SOURCES)
        )
        self.assertIs(session.reconnect_control.__self__, manager)

    def test_fast_sleep_is_explicit_bool_opt_in_before_device_or_adapter_work(self):
        for invalid in (1, None, "yes"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                m.build_stream_session(
                    self.env.camera, decode=False, fast_sleep=invalid
                )
        self.assertFalse(self.env.adapters)
        self.assertEqual(self.env.events, [])
        session, _ = self.build(fast_sleep=True)
        self.assertTrue(session.fast_sleep)
        default, _ = self.build()
        self.assertFalse(default.fast_sleep)

    def test_retained_flag_is_linux_fast_only_and_all_sources_validate_before_adapter(
        self,
    ):
        for options in (
            dict(retain_ecm=1),
            dict(retain_ecm=True),
            dict(retain_ecm=True, fast_sleep=True),
        ):
            with (
                self.subTest(options=options),
                patch.object(m.sys, "platform", "darwin"),
            ):
                with self.assertRaises(ValueError):
                    m.build_stream_session(self.env.camera, decode=False, **options)
        self.assertEqual(self.env.events, [])
        self.assertFalse(self.env.adapters)
        with patch.object(m.sys, "platform", "linux"):
            session, _ = self.build(fast_sleep=True, retain_ecm=True)
        self.assertTrue(session.retain_ecm)
        self.assertTrue(session.fast_sleep)
        self.assertEqual(
            set(session.retained_factory.source_provider()), set(m.RETAINED_SOURCES)
        )
        self.assertIsNone(session.retained_factory.assets)

    def test_invalid_options_and_missing_decode_dependency_fail_before_any_device_or_adapter(
        self,
    ):
        invalid = [
            dict(queue_size=True),
            dict(queue_size=0),
            dict(queue_size=257),
            dict(timeout=float("nan")),
            dict(timeout=-1),
            dict(imu_options=[]),
            dict(imu_options={"read_buffer_bytes": 123}),
            dict(imu_options={"startup_timeout": True}),
            dict(imu_options={"unknown_option": 2}),
        ]
        for options in invalid:
            with self.assertRaises((TypeError, ValueError)):
                m.build_stream_session(self.env.camera, decode=False, **options)
        with patch.object(m.importlib.util, "find_spec", return_value=None):
            with self.assertRaises(ImportError):
                m.build_stream_session(self.env.camera)
        self.assertFalse(self.env.adapters)
        self.assertEqual(self.env.events, [])

    def test_corrupt_packaged_source_fails_before_initial_adapter_or_device(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(m, "DEVICE", Path(directory)),
        ):
            for name in (*m.OWNER_SOURCES, *m.WATCHER_SOURCES):
                Path(directory, name).write_bytes(b"corrupt")
            with self.assertRaises(ValueError):
                self.build()
        self.assertFalse(self.env.adapters)
        self.assertEqual(self.env.events, [])

    def test_idle_readiness_and_fixed_battery_start_precede_record0_last(self):
        session, manager = self.opened()
        proof = session.idle_probe(self.env.camera)
        self.assertEqual(
            (proof.recording_state, proof.activation_state, proof.battery_present),
            (0, 1, True),
        )
        events = self.env.events
        self.assertLess(
            events.index("missing.RESULT.ready"),
            events.index(("START", m.BATTERY_QUERY)),
        )
        self.assertEqual(events[-1], "state.LAST")
        activation = next(
            value
            for value in events
            if isinstance(value, tuple) and value[0] == "activation"
        )
        self.assertEqual(activation[1:], (0, bytes.fromhex("310000000000"), 0x88))
        count = len(self.env.adapters)
        session.idle_probe(self.env.camera)
        self.assertEqual(
            len(self.env.adapters), count
        )  # No fresh D1 session per status.
        self.assertEqual(self.env.events.count("missing.RESULT.ready"), 1)

    def test_reader_identity_change_rebinds_even_when_bus_address_are_reused(self):
        session, manager = self.opened()
        session.idle_probe(self.env.camera)
        old = session.control.bootstrap
        self.env.transport._thread = object()
        session.idle_probe(self.env.camera)
        self.assertIsNot(session.control.bootstrap, old)
        old = session.control.bootstrap
        self.env.transport._device_identity = dict(bus=1, address=3)
        session.idle_probe(self.env.camera)
        self.assertIsNot(session.control.bootstrap, old)

    def test_unknown_old_task_is_resolved_readonly_before_battery_start_on_new_generation(
        self,
    ):
        session, manager = self.opened()
        session.idle_probe(self.env.camera)
        session.control.bootstrap.session.quarantined = True
        self.env.transport._thread = object()
        self.env.events.clear()
        session.idle_probe(self.env.camera)
        self.assertLess(
            self.env.events.index("old.RESULT"),
            self.env.events.index(("START", m.BATTERY_QUERY)),
        )
        self.assertIsNone(session.control.pending_task)

    def test_running_old_task_blocks_all_new_start_after_reconnect(self):
        session, manager = self.opened()
        session.idle_probe(self.env.camera)
        session.control.bootstrap.session.quarantined = True
        self.env.transport._thread = object()
        self.env.old_task_running = True
        self.env.events.clear()
        with self.assertRaises(StandbyUncertain):
            session.idle_probe(self.env.camera)
        self.assertFalse(
            any(isinstance(row, tuple) and row[0] == "START" for row in self.env.events)
        )
        self.assertEqual(session.control.pending_task, 37)

    def test_reconnect_callback_is_open_and_readonly_amt_only(self):
        session, manager = self.opened()
        session.idle_probe(self.env.camera)
        session.control.bootstrap.session.quarantined = True
        self.env.transport.connected = False
        self.env.events.clear()
        result = session.reconnect_control(self.env.camera)
        self.assertIs(result, session.control.bootstrap)
        self.assertEqual(self.env.events, ["open", "missing.RESULT.ready"])
        self.assertEqual(session.control.pending_task, 37)

    def test_only_activation_e3_is_retried_and_exact60_bytes_raw1_required(self):
        session, manager = self.opened()
        self.env.activation = [b"\xe3", b"\0\1" + bytes(58)]
        session.idle_probe(self.env.camera)
        self.assertEqual(
            sum(
                isinstance(row, tuple) and row[0] == "activation"
                for row in self.env.events
            ),
            2,
        )
        for wrong in (b"\xe0", b"\0\1" + bytes(57), b"\0\0" + bytes(58)):
            self.env.activation = [wrong]
            self.env.events.clear()
            with self.assertRaises(
                (m.controls.CameraReplyError, m.controls.ControlDecodeError)
            ):
                session.idle_probe(self.env.camera)
            self.assertFalse(
                any(
                    isinstance(row, tuple) and row[0] == "START"
                    for row in self.env.events
                )
            )

    def test_repeated_e3_has_absolute10s_bound_and_never_uploads_or_starts_battery(
        self,
    ):
        session, manager = self.opened()
        self.env.activation = [b"\xe3"]
        with self.assertRaises((m.RequestTimeout, m.controls.CameraReplyError)):
            session.idle_probe(self.env.camera)
        self.assertGreaterEqual(self.env.clock.now, 10)
        self.assertLessEqual(self.env.clock.now, 10.01)
        self.assertFalse(
            any(isinstance(row, tuple) and row[0] == "START" for row in self.env.events)
        )

    def test_recording_or_late_amt_reply_is_not_fresh_idle_evidence(self):
        session, manager = self.opened()
        self.env.states = [2]
        with self.assertRaises(ValueError):
            session.idle_probe(self.env.camera)
        self.env.transport._thread = object()

        def late(*args, **options):
            self.env.clock.now += 61
            return dict(ready=True)

        with patch.object(m, "wait_amt_ready", side_effect=late):
            with self.assertRaises(m.RequestTimeout):
                session.idle_probe(self.env.camera)
        self.assertFalse(manager.amt_ready)

    def test_factory_reuses_exact_assets_and_builder_keeps_runtime_keyword_arguments(
        self,
    ):
        session, manager = self.build()
        assets = types.SimpleNamespace(work="cache", staged=False)

        def stage():
            assets.staged = True
            return assets

        assets.preload_assets = stage
        assets.control = session.control
        owner = dict(work="owner", nonce="nonce", owner={}, service={})
        assets.owner_receipt = owner
        with (
            patch.object(m, "PreparedAssets", return_value=assets) as factory,
            patch.object(m, "PreparedStream") as prepared,
            patch.object(m, "build_prepared_fast_plan", return_value="plan") as builder,
        ):
            first = session.prepared_factory.preload_assets(
                self.env.camera, session.control, owner_receipt=owner
            )
            second = session.prepared_factory.preload_assets(
                self.env.camera, session.control, owner_receipt=owner
            )
            self.assertEqual(factory.call_count, 1)
            self.assertEqual(first, second)
            session.prepared_factory(
                self.env.camera,
                session.control,
                owner_receipt=owner,
                sleep_id="0000000000000001",
                wake_id="0000000000000002",
            )
            options = prepared.call_args.kwargs
            self.assertIs(options["assets"], assets)
            self.assertEqual(
                options["plan_builder"](
                    "runtime", tag="tag", nonce="nonce", port=50000
                ),
                "plan",
            )
            builder.assert_called_once_with(
                "runtime", fast=True, tag="tag", nonce="nonce", port=50000
            )

    def test_existing_owner_reuses_completed_adapter_without_new_session_or_readiness_start(
        self,
    ):
        initial, manager = self.opened()
        initial.idle_probe(self.env.camera)
        adapter = initial.control.bootstrap
        receipt = dict(
            work="/dev/shm/osmo-usb-standby-aabbccddeeff",
            nonce="aabbccddeeff",
            owner=dict(pid=100, start="10"),
            service=dict(pid=200, start="20"),
        )
        count = len(self.env.adapters)
        self.env.events.clear()
        resumed = m.build_stream_session(
            self.env.camera, decode=False, existing=receipt, bootstrap=adapter
        )
        self.assertIs(resumed.control.bootstrap, adapter)
        self.assertEqual(len(self.env.adapters), count)
        self.assertEqual(self.env.events, [])
        attached = resumed.idle_probe.__self__
        self.assertTrue(attached.amt_ready)
        self.assertIs(attached.generation[0], self.env.transport._thread)
        self.assertEqual(resumed.control.owner, receipt["owner"])
        with self.assertRaises(ValueError):
            m.build_stream_session(self.env.camera, decode=False, bootstrap=adapter)


if __name__ == "__main__":
    unittest.main()
