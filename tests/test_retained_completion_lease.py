"""Completion handoff follows the live lease and original finite guard, offline."""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import socket
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[1] / "osmo360_camera/_device/retained_sleep_060.py"
spec = importlib.util.spec_from_file_location("retained_completion_device_tests", SOURCE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class CompletionLeaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.work = Path(self.tmp.name)
        self.lease = self.work / "lease"
        self.lease.mkdir()
        self.cfg = dict(
            nonce="ab" * 6,
            lease_nonce="cd" * 16,
            lease_work=str(self.lease),
            standby_owner=dict(pid=22, start="123"),
            sys_service=dict(pid=33, start="234"),
            wake_command_id="0000000000000002",
            host_sequence=6,
            host_ip="192.168.253.2",
            camera_ip="192.168.253.1",
            dev_mac="02:00:00:00:00:01",
            completion_port=54321,
        )
        pre = dict(
            udc_identity=[10, 11], ifname="usb0", ifindex=12,
            lease_sequence=7, lease_expires=101.0,
        )
        post = dict(pre, udc_identity=[10, 12])
        self.done = dict(
            nonce=self.cfg["nonce"], terminal=True, reset_attempted=1,
            standby_owner=self.cfg["standby_owner"], sys_service=self.cfg["sys_service"],
            wake_command_id=self.cfg["wake_command_id"],
            unbind_completed=True, bind_completed=True, same_six_links=True,
            lease_fresh=True, lease_sequence=7, lease_expires=101.0,
            original_udc_identity=[10, 11], active_udc_identity=[10, 12],
            pre=pre, post=post, completed_boottime=145.0,
            closed={name: dict(closed_verified_ebadf=True) for name in ("alarm90", "alarm240")},
        )
        self.state = dict(
            nonce=self.cfg["lease_nonce"], state="active", seq=8,
            updated=105.0, expires=115.0,
        )
        self.guard = dict(
            nonce=self.cfg["nonce"], owner=dict(pid=44, start="345"),
            deadline_boottime=190.0,
        )
        self.raw = m.COMPLETION_QUERY.pack(m.COMPLETION_MAGIC, m.completion_ticket(self.cfg), 42)
        self.peer = (self.cfg["host_ip"], 54322)

    def write(self, path, value):
        path.write_text(json.dumps(value))

    def base(self):
        return SimpleNamespace(
            read_json=lambda path: json.loads(path.read_text()),
            write_new=lambda path, value: self.write(path, value),
            private=lambda *args, **kwargs: None,
        )

    def test_renewed_lease_can_complete_after_immutable_reset_lease_expired(self):
        original_done, original_state = deepcopy(self.done), deepcopy(self.state)
        reply = json.loads(m.completion_reply(
            self.cfg, self.done, self.raw, self.peer, self.state, 105.0
        ))
        self.assertGreater(105.0, self.done["lease_expires"])
        self.assertEqual(reply["lease_expires"], 115.0)
        self.assertEqual(reply["challenge"], 42)
        self.assertEqual(self.done, original_done)
        self.assertEqual(self.state, original_state)  # Queries never renew or mutate a lease.

    def test_current_lease_rejects_expiry_owner_sequence_or_ttl_drift(self):
        for changes in (
            dict(expires=105.0), dict(nonce="00" * 16), dict(seq=6),
            dict(state="closed"), dict(expires=115.1), dict(updated=106.0),
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                m.completion_reply(
                    self.cfg, self.done, self.raw, self.peer,
                    dict(self.state, **changes), 105.0,
                )

    def test_terminal_reset_and_closed_rtc_proofs_remain_required(self):
        for mutate in (
            lambda value: value.update(standby_owner=dict(pid=55, start="1")),
            lambda value: value.update(sys_service=dict(pid=55, start="1")),
            lambda value: value.update(same_six_links=False),
            lambda value: value["post"].update(ifindex=99),
            lambda value: value["closed"]["alarm90"].update(closed_verified_ebadf=False),
        ):
            value = deepcopy(self.done)
            mutate(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                m.completion_reply(self.cfg, value, self.raw, self.peer, self.state, 105.0)

    def test_closed_marker_refuses_live_read_and_export_uses_renewed_state(self):
        self.write(self.lease / "lease.state", self.state)
        base = self.base()
        base.write_new = Mock()
        with patch.object(m.time, "monotonic", return_value=105.0):
            state, now = m.completion_live_lease(self.cfg, base, self.done)
            self.assertEqual((state, now), (self.state, 105.0))
            m.publish_done(self.work, self.cfg, base, self.done)
            self.assertEqual(base.write_new.call_args.args[1], self.done)
            (self.lease / "lease.closed").touch()
            with self.assertRaises(ValueError):
                m.completion_live_lease(self.cfg, base, self.done)

    def test_close_racing_the_read_is_rejected(self):
        def read(path):
            (self.lease / "lease.closed").touch()
            return self.state
        with patch.object(m.time, "monotonic", return_value=105.0):
            with self.assertRaisesRegex(ValueError, "during read"):
                m.completion_live_lease(self.cfg, SimpleNamespace(read_json=read), self.done)

    def test_absolute_guard_cannot_be_extended_by_a_fresh_lease(self):
        self.assertEqual(m.completion_deadline(self.cfg, self.guard, 150.0), 190.0)
        with self.assertRaises(TimeoutError):
            m.completion_deadline(self.cfg, self.guard, 190.0)
        for changes in (dict(nonce="00" * 6), dict(deadline_boottime=241.0)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                m.completion_deadline(self.cfg, dict(self.guard, **changes), 150.0)

    def test_signal_budget_uses_remaining_original_guard_time(self):
        self.write(self.work / "guard.json", self.guard)
        with (
            patch.object(m, "boot", return_value=150.0),
            patch.object(m.signal, "signal", return_value="old-handler"),
            patch.object(m.signal, "setitimer") as timer,
            patch.object(m, "_completion_server", return_value="finished") as server,
        ):
            self.assertEqual(m.completion_server(self.work, self.cfg), "finished")
        self.assertEqual(timer.call_args_list[0].args, (m.signal.ITIMER_REAL, 40.0))
        self.assertEqual(timer.call_args_list[1].args, (m.signal.ITIMER_REAL, 0))
        server.assert_called_once_with(self.work, self.cfg, guard=self.guard)

    def test_server_handoffs_past_old_expiry_but_exits_at_original_budget(self):
        self.write(self.work / "result.json", {k: v for k, v in self.done.items() if k != "closed"})
        self.write(self.work / "done.json", self.done)
        self.write(self.work / "guard.json", self.guard)
        self.write(self.lease / "lease.state", self.state)
        clock = SimpleNamespace(monotonic=105.0, boottime=150.0)
        sent = []
        test = self

        class FakeSocket:
            calls = 0
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def setsockopt(self, *args): pass
            def bind(self, *args): pass
            def settimeout(self, *args): pass
            def sendto(self, raw, peer): sent.append((json.loads(raw), peer))
            def recvfrom(self, size):
                self.calls += 1
                if self.calls == 1:
                    return test.raw, test.peer
                # Heartbeats can keep lease.state live, but cannot extend guard.json.
                clock.monotonic, clock.boottime = 145.0, 190.0
                test.write(test.lease / "lease.state", dict(test.state, seq=9, updated=145.0, expires=155.0))
                raise socket.timeout()

        base = self.base()
        native = SimpleNamespace(alive=lambda owner: True, proc_token=lambda: dict(pid=66, start="456"))
        with (
            patch.object(m, "modules", return_value=(SimpleNamespace(base=base), None, native)),
            patch.object(m, "boot", side_effect=lambda: clock.boottime),
            patch.object(m.time, "monotonic", side_effect=lambda: clock.monotonic),
            patch.object(m.socket, "socket", return_value=FakeSocket()),
            patch.object(m, "attribute", side_effect=lambda path: self.cfg["dev_mac"] if path.name == "address" else "12"),
            patch.object(m, "ipv4", return_value=self.cfg["camera_ip"] + "/30"),
        ):
            with self.assertRaisesRegex(TimeoutError, "Original completion guard"):
                m._completion_server(self.work, self.cfg)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][0]["lease_expires"], 115.0)
        self.assertEqual(sent[0][1], self.peer)
        self.assertEqual(json.loads((self.work / "guard.json").read_text()), self.guard)
        self.assertEqual(json.loads((self.work / "done.json").read_text()), self.done)


if __name__ == "__main__":
    unittest.main()
