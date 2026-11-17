"""One helper deadline reaches readiness, discovery and actual D1 delivery."""

from dataclasses import replace
import struct
import unittest
from unittest.mock import Mock, patch

from osmo360_camera import _helper_install as m
from osmo360_camera._standby_control import StandbyControl
from osmo360_camera._usb_bootstrap import USBBootstrap
from test_helper_install import Control, BOOT1, BOOT2
from test_amt_session import FakeTransport, fake_session, MISSING, START, SUCCESS, BUSY
from test_prepared_stream import runtime
import test_standby_control as standby_fixtures
import threading
import types


class HelperBudgetTests(unittest.TestCase):
    def setUp(self):
        self.package = m.package_profile()
        self.camera = types.SimpleNamespace(_control_lock=threading.RLock())

    def test_readiness_consumes_original_budget_without_any_later_upload_or_start(self):
        control = Control(self.package)

        def late_controller(camera, timeout, *, guard):
            control.clock.now = 1
            return control

        with (
            patch.object(m.time, "monotonic", control.clock),
            patch.object(m, "_controller", late_controller),
        ):
            with self.assertRaises(TimeoutError):
                m.install(self.camera, timeout=0.5)
        self.assertEqual(control.commands, [])

    def test_discovery_budget_prevents_all_later_status_and_payload_commands(self):
        control = Control(self.package)

        def after(command):
            if command == "readonly-owner-discovery":
                control.clock.now = 1

        control.after_send = after

        def discover(scoped):
            scoped._run("readonly-owner-discovery")
            raise AssertionError("Late discovery must not return")

        with patch("osmo360_camera._stream_resume._discover", side_effect=discover):
            with self.assertRaises(TimeoutError):
                m.install(self.camera, timeout=0.5, control=control)
        self.assertEqual(control.commands[-1], "readonly-owner-discovery")
        self.assertEqual(control.operations, [])
        self.assertFalse(any("base64 -d" in command for command in control.commands))

    def test_nested_status_cannot_reset_install_budget_or_return_late_idempotent_success(
        self,
    ):
        control = Control(self.package, installed=True)
        control.after_send = lambda command: (
            setattr(control.clock, "now", 1) if " status " in command else None
        )
        with patch.object(m, "_existing_owner", return_value=None):
            with self.assertRaises(TimeoutError):
                m.install(self.camera, timeout=0.5, control=control)
        self.assertEqual(control.operations, ["status"])
        self.assertFalse(any("base64 -d" in command for command in control.commands))

    def test_late_pending_result_does_not_retire_or_send_finalize(self):
        scenario = standby_fixtures.Scenario()
        control = StandbyControl(
            scenario.adapter(),
            source_provider=lambda: {},
            control_lock=threading.RLock(),
            clock=scenario.clock,
            sleep=scenario.clock.sleep,
            adapter_factory=scenario.adapter,
        )
        control.owner, control.service, control.launch_attempted = (
            dict(pid=1, start="2"),
            dict(pid=3, start="4"),
            True,
        )
        control.pending_task = 37

        def terminal():
            scenario.clock.now = 2
            return struct.pack("<HBB", 5, 0, 0)

        scenario.on_result = terminal

        def guard():
            if scenario.clock() >= 1:
                raise TimeoutError("Outer deadline")

        with self.assertRaises(TimeoutError):
            control.finalize(guard=guard)
        self.assertFalse(control.finalize_attempted)
        self.assertFalse(any(row[0] == "START" for row in scenario.events))

    def test_actual_d1_preflight_consumes_budget_before_finalize_flag_or_start_delivery(
        self,
    ):
        fake = FakeTransport([])

        def late_missing():
            fake.now = 2
            return MISSING

        fake.replies = iter([late_missing])
        with fake_session(fake, reuse=True) as session:
            bootstrap = USBBootstrap(fake, session=session)
            control = StandbyControl(
                bootstrap,
                source_provider=lambda: {},
                control_lock=threading.RLock(),
                clock=fake.clock,
                sleep=fake.sleep,
            )
            control.owner, control.service, control.launch_attempted = (
                dict(pid=1, start="2"),
                dict(pid=3, start="4"),
                True,
            )

            def guard():
                if fake.clock() >= 1:
                    raise TimeoutError("No late native START")

            with self.assertRaises(TimeoutError):
                control.finalize(guard=guard)
            self.assertFalse(control.finalize_attempted)
            self.assertFalse(session.quarantined)
            self.assertEqual(session.operations[-1]["start_attempts"], 0)
        self.assertEqual(fake.subcommands(), [4])

    def test_reuse_busy_cleanup_cannot_send_another_start_after_delivery_guard_expires(
        self,
    ):
        fake = FakeTransport([MISSING, START, SUCCESS])
        with fake_session(fake, reuse=True) as session:
            session.run("true")

            def late_busy():
                fake.now = 0.2
                return BUSY

            fake.replies = iter([late_busy])

            def guard():
                if fake.clock() >= 0.1:
                    raise TimeoutError("No late retry START")

            with self.assertRaises(TimeoutError):
                session.run("id", pre_send=guard)
            self.assertFalse(session.quarantined)
            self.assertEqual(session.operations[-1]["start_attempts"], 1)
            self.assertFalse(session.operations[-1]["start_accepted"])
        self.assertEqual(fake.subcommands(), [4, 0, 4, 0])

    def test_initial_runtime_query_start_checks_budget_after_actual_d1_preflight(self):
        fake = FakeTransport([])

        def late_missing():
            fake.now = 2
            return MISSING

        fake.replies = iter([late_missing])
        with fake_session(fake, reuse=True) as session:
            bootstrap = USBBootstrap(fake, session=session)

            def budget():
                if fake.clock() >= 1:
                    raise TimeoutError("No late runtime query START")

            with self.assertRaises(TimeoutError):
                bootstrap.query_runtime(budget=budget)
            self.assertFalse(session.quarantined)
            self.assertEqual(session.operations[-1]["start_attempts"], 0)
        self.assertEqual(fake.subcommands(), [4])

    def test_removed_requires_same_bootstrap_native_five_function_ssp_proof(self):
        for changed in (
            replace(
                runtime(),
                original_links=runtime().original_links + ("../../functions/ecm.ecm",),
            ),
            replace(
                runtime(), original_links=tuple(reversed(runtime().original_links))
            ),
            replace(runtime(), original_max_speed="high-speed"),
        ):
            with self.subTest(runtime=changed):
                control = Control(self.package)
                control.boot_id = BOOT2
                control.bootstrap.query_runtime = Mock(return_value=changed)
                with self.assertRaises((ValueError, m.HelperInstallError)):
                    m.verify_removed(self.camera, BOOT1, control=control)
                control.bootstrap.query_runtime.assert_called_once()
                self.assertTrue(
                    callable(control.bootstrap.query_runtime.call_args.kwargs["budget"])
                )

    def test_removed_returns_plain_native_runtime_and_honors_facade_guard(self):
        control = Control(self.package)
        control.boot_id = BOOT2
        # Native configfs may spell the same canonical link through its full
        # relative gadget path; validation is about targets, not raw spelling.
        equivalent = replace(
            runtime(),
            original_links=tuple(
                "../../../../usb_gadget/g1/functions/" + name
                for name in (
                    "ffs.diag",
                    "ffs.bulk",
                    "ffs.bulk1",
                    "ffs.bulk5",
                    "ffs.bulk4",
                )
            ),
        )
        control.bootstrap.query_runtime = lambda **kw: equivalent
        guard = Mock()
        result = m.verify_removed(self.camera, BOOT1, control=control, guard=guard)
        self.assertEqual(
            result["native_runtime"]["original_max_speed"], "super-speed-plus"
        )
        self.assertEqual(
            result["native_runtime"]["original_links"], equivalent.original_links
        )
        self.assertGreater(guard.call_count, 1)
        stopped = Mock(side_effect=TimeoutError("Facade deadline"))
        before = len(control.commands)
        with self.assertRaises(TimeoutError):
            m.verify_removed(self.camera, BOOT1, control=control, guard=stopped)
        self.assertEqual(len(control.commands), before)


if __name__ == "__main__":
    unittest.main()
