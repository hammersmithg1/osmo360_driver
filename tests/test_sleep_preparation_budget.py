"""Preparation deadlines stop the next command, without touching real USB."""

import json
import unittest
from unittest.mock import patch

from osmo360_camera import _usb_bootstrap as bootstrap
from osmo360_camera import usb_preview as preview_module
from test_usb_bootstrap import FakeSession, FakeTransport, runtime_report
from test_usb_preview import harness


class RuntimeBudgetTests(unittest.TestCase):
    def adapter(self):
        transport = FakeTransport(json.dumps(runtime_report()).encode())
        return bootstrap.USBBootstrap(transport, session=FakeSession(transport))

    def test_expired_before_collector_creates_nothing_and_sends_no_start(self):
        adapter = self.adapter()

        def expired():
            raise TimeoutError("preparation expired")

        with self.assertRaises(TimeoutError):
            adapter.query_runtime(budget=expired)
        self.assertEqual(adapter.session.commands, [])
        self.assertEqual(adapter.transport.requests, [])
        self.assertFalse(adapter.last_report["cleanup_complete"])

    def test_expiry_after_mkdir_keeps_owned_ram_and_never_starts_upload_or_cleanup(
        self,
    ):
        adapter = self.adapter()

        def budget():
            if adapter.session.commands:
                raise TimeoutError("late mkdir")

        with self.assertRaises(bootstrap.BootstrapError) as caught:
            adapter.query_runtime(budget=budget)
        self.assertIsInstance(caught.exception.__cause__, TimeoutError)
        self.assertEqual(len(adapter.session.commands), 1)
        self.assertIn("mkdir ", adapter.session.commands[0][0])
        self.assertEqual(adapter.transport.requests, [])
        self.assertTrue(adapter.last_report["cleanup_deferred"])
        self.assertFalse(adapter.last_report["cleanup_complete"])

    def test_late_file_pull_cannot_start_cleanup_or_return_runtime(self):
        adapter = self.adapter()
        pulled = []

        def pull(path):
            pulled.append(path)
            return json.dumps(runtime_report()).encode()

        def budget():
            if pulled:
                raise TimeoutError("late pull")

        with patch.object(adapter, "pull_bytes", side_effect=pull):
            with self.assertRaises(bootstrap.BootstrapError) as caught:
                adapter.query_runtime(budget=budget)
        self.assertIsInstance(caught.exception.__cause__, TimeoutError)
        self.assertEqual(len(pulled), 1)
        self.assertIn(" query ", adapter.session.commands[-1][0])
        self.assertFalse(
            any(" cleanup " in command for command, _ in adapter.session.commands)
        )
        self.assertTrue(adapter.last_report["cleanup_deferred"])

    def test_successful_budgeted_runtime_keeps_verified_cleanup(self):
        adapter = self.adapter()
        checks = []
        value = adapter.query_runtime(budget=lambda: checks.append(True))
        self.assertEqual(value.host_addr, runtime_report()["host_addr"])
        self.assertTrue(adapter.last_report["cleanup_complete"])
        self.assertGreater(len(checks), len(adapter.session.commands))


class PreviewStopBudgetTests(unittest.TestCase):
    def test_expiry_immediately_before_close_retains_heartbeat_ip_and_transport(self):
        with harness() as case:
            case.preview.start()
            case.events.clear()
            checks = []

            def budget():
                checks.append(True)
                if len(checks) == 4:
                    raise TimeoutError("before CLOSE")

            with self.assertRaisesRegex(TimeoutError, "before CLOSE"):
                case.preview.stop(budget=budget)
            self.assertNotIn("lease.close", case.events)
            self.assertNotIn("network.cleanup", case.events)
            self.assertNotIn("transport.close", case.events)
            self.assertNotIn("session:rollback_once", case.events)
            self.assertTrue(case.state.lease.healthy)
            self.assertTrue(case.state.host_alias)
            self.assertTrue(case.camera._transport.connected)
            self.assertIs(case.camera._usb_preview, case.preview)
            self.assertFalse(case.preview.report["restored"])

    def test_old_stop_propagates_budget_to_collector_before_any_late_start(self):
        with harness() as case:
            case.preview.start()
            case.events.clear()
            expired = []

            def budget():
                if expired:
                    raise TimeoutError("collector deadline")

            def runtime_query(_adapter, *, budget):
                # Model the collector reaching its next START after a bounded
                # native operation consumed the last budget. Actual runtime
                # tests above exercise the same check around every command.
                expired.append(True)
                budget()
                case.events.append("forbidden.collector.START")

            with patch.object(
                preview_module.USBBootstrap, "query_runtime", runtime_query
            ):
                with self.assertRaisesRegex(TimeoutError, "collector deadline"):
                    case.preview.stop(budget=budget)
            self.assertIn("lease.close", case.events)
            self.assertNotIn("forbidden.collector.START", case.events)
            self.assertIn("network.cleanup", case.events)
            self.assertFalse(case.preview.report["restored"])


if __name__ == "__main__":
    unittest.main()
