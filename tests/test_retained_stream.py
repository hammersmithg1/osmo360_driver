"""Retained lease/receipt/generation tests; no USB or sockets are opened."""

from copy import deepcopy
import errno
import hashlib
import json
import socket
import struct
import shlex
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from osmo360_camera import _retained_stream as m
from osmo360_camera._standby_control import StandbyUncertain
from osmo360_camera._usb_plan import build_usb_plan
from test_prepared_stream import Control, RECEIPT, runtime, Preview, IMU, Video
from test_standby_status_retry import NativeD1


class Clock:
    now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Network:
    def __init__(self, events, generation=0):
        self.events, self.generation = events, generation
        self.identity = types.SimpleNamespace(
            name="usb" + str(generation), ifindex=10 + generation
        )
        self.report = dict(ready=dict(ok=True, ncm_interface=self.identity.name))
        self.gone, self.next = generation == 0, None
        self.deadlines = []

    def _old_index_absent(self, identity, *, deadline=None):
        self.deadlines.append(deadline)
        return self.gone, {"absent": self.gone}

    def rebind_after_usb_reset(self, **options):
        self.events.append("network.rebind." + str(self.generation))
        if self.next is None:
            raise AssertionError("unexpected reset generation")
        return self.next, self.next.report["ready"]

    def _snapshot(self):
        return {}, {}

    def _current(self, observation):
        return dict(pending_linkdown=False)

    def _unchanged_other_state(self, state):
        self.events.append("network.original_state")


class Lease:
    def __init__(self, events, config):
        self.events, self.config = events, config
        self._sequence, self.healthy, self.last_ack = 0, False, None

    def start(self, **options):
        self.events.append("lease.ACK")
        self._sequence += 1
        self.last_ack, self.healthy = dict(sequence=self._sequence, state=1), True
        return self

    def renew(self):
        self.events.append("lease.RENEW_ACK")
        self._sequence += 1
        self.last_ack = dict(sequence=self._sequence, state=1)
        return self.last_ack

    def stop(self, *, close_camera=True):
        self.events.append("lease.CLOSE" if close_camera else "lease.local_stop")
        self.healthy = False
        return dict(state=2)


class RetainedTests(unittest.TestCase):
    def setUp(self):
        self.events, self.clock = [], Clock()
        self.camera = types.SimpleNamespace(
            _control_lock=threading.RLock(),
            _transport=object(),
            _imu=None,
            _usb_preview=None,
        )
        self.control = Control(self.camera, self.events)
        self.network = Network(self.events)
        self.plan = build_usb_plan(
            runtime(), tag="retained-test", nonce="ab" * 16, port=51337
        )
        old_lease = types.SimpleNamespace(
            healthy=True,
            last_ack=dict(sequence=41, state=1),
            _sequence=44,
            stop=Mock(
                side_effect=lambda **options: self.events.append(
                    ("old_lease.stop", options)
                )
            ),
        )
        self.old = types.SimpleNamespace(
            _plan=self.plan,
            _runtime=runtime(),
            network=self.network,
            _baseline={"original": True},
            _lease=old_lease,
            _scheduled=True,
            _closed=False,
            report={},
        )
        self.camera._usb_preview = self.old
        self.assets = types.SimpleNamespace(
            staged=True,
            control=self.control,
            owner_receipt=deepcopy(RECEIPT),
            link_into=Mock(),
        )
        def pull(path, **options):
            self.events.append('endpoint.pull')
            nonce = path.rsplit('_', 1)[1].removesuffix('.json')
            guard = dict(pid=44, start='345')
            return json.dumps(dict(schema=1, nonce=nonce, guard=guard,
                worker=dict(pid=55, start='456'),
                endpoint=dict(schema=1, nonce=nonce, owner=guard, fd=7,
                    socket_identity=[9, 99], ifname='usb0', ifindex=12, port=54321),
                export_identity=dict(device=9, inode=100))).encode()
        self.control.bootstrap.pull_bytes = Mock(side_effect=pull)

    def backend(self, **options):
        return m.RetainedStream(
            self.camera,
            self.control,
            owner_receipt=RECEIPT,
            sleep_id="0000000000000001",
            wake_id="0000000000000002",
            preview=self.old,
            assets=self.assets,
            decode=False,
            preview_factory=lambda camera, **kw: Preview(camera, self.events, **kw),
            imu_factory=lambda **kw: IMU(self.events, **kw),
            video_factory=lambda **kw: Video(self.events, **kw),
            lease_factory=lambda config: Lease(self.events, config),
            clock=self.clock,
            sleep=self.clock.sleep,
            **options,
        )

    def armed(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = "SLEEP", 1
        value.arm_watcher()
        return value

    def receipt(self, value, ticket, challenge):
        return dict(
            schema=1,
            nonce=value.nonce,
            standby_owner=RECEIPT["owner"],
            sys_service=RECEIPT["service"],
            wake_command_id=value.wake_id,
            ticket=ticket.hex(),
            challenge=challenge,
            terminal=True,
            reset_attempted=1,
            udc_rebind_completed=True,
            original_plan_work=self.plan["work_dir"],
            lease_expires=109.0,
            now=100.0,
        )

    def test_stages_fresh_manifest_awake_without_apply_or_stable_sleep_guard(self):
        value = self.backend().prepare()
        self.assertEqual(value.manifest["host_sequence"], 41)
        self.assertEqual(value.manifest["plan_sha256"], self.plan["sha256"])
        self.assertNotEqual(
            value.manifest["completion_port"], self.plan["lease"]["port"]
        )
        self.assertEqual(value.manifest['completion_port'], 0)
        self.assertIsNone(value.endpoint)
        self.assertFalse(value.arm_attempted)
        self.assertFalse(value.possible_future_apply)
        self.old._lease.stop.assert_not_called()
        self.assertFalse(
            any(
                command.endswith(" launch") or "apply.sh" in command
                for command in self.control.commands
            )
        )
        self.assertEqual(value.work, "/dev/shm/osmo-retained-sleep060-" + value.nonce)
        self.assets.link_into.assert_called_once_with(value.work)

    def test_arm_is_sleep_only_and_retires_before_uncertain_input(self):
        value = self.backend().prepare()
        with self.assertRaises(m.PreparedStreamError):
            value.arm_watcher()
        self.control.state, self.control.sequence = "SLEEP", 1

        def uncertain(command):
            if command.endswith(" launch"):
                self.assertTrue(value.possible_future_apply)
                raise OSError("lost helper receipt")

        self.control.failure = uncertain
        with self.assertRaises(m.UnknownFutureApply):
            value.arm_watcher()
        with self.assertRaises(m.PreparedStreamError):
            value.arm_watcher()
        with self.assertRaises(m.UnknownFutureApply):
            value.stop()
        self.old._lease.stop.assert_not_called()
        self.assertEqual(
            sum(command.endswith(" launch") for command in self.control.commands), 1
        )

    def test_local_lease_stop_preserves_last_sent_sequence_and_never_sends_close(self):
        value = self.armed()
        self.old._lease.stop.assert_called_once_with(close_camera=False)
        self.assertEqual(value.sequence, 44)
        self.assertIs(self.camera._usb_preview, self.old)

    def test_sleep_observation_deadline_failure_never_arms_or_stops_original_lease(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = 'SLEEP', 1
        self.control.wait_sleep_snapshot = Mock(side_effect=StandbyUncertain('native observation deadline'))
        with self.assertRaises(StandbyUncertain):
            value.arm_watcher(deadline=12.25)
        self.control.wait_sleep_snapshot.assert_called_once_with(value.sleep_id, deadline=12.25,
            report=value.report['sleep_readiness'])
        self.assertFalse(value.arm_attempted)
        self.assertFalse(value.possible_future_apply)
        self.old._lease.stop.assert_not_called()
        self.assertFalse(any(command.endswith(' launch') for command in self.control.commands))

    def test_late_arm_readiness_is_rejected_at_delivery_guard_before_any_helper_start(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = 'SLEEP', 1
        def late(*args, **options):
            self.clock.now = 12.3
            return self.control.snapshot()
        self.control.wait_sleep_snapshot = late
        with self.assertRaises(m.UnknownFutureApply):
            value.arm_watcher(deadline=12.25)
        self.assertFalse(value.arm_attempted)
        self.assertFalse(value.possible_future_apply)
        self.old._lease.stop.assert_not_called()
        self.assertFalse(any(command.endswith(' launch') for command in self.control.commands))

    def test_reserved_endpoint_is_fetched_and_inode_cleaned_before_local_lease_stop(self):
        run = self.control._run
        def record(command, **options):
            if ';os.unlink(p)' in command: self.events.append('endpoint.unlink')
            return run(command, **options)
        self.control._run = record
        value = self.armed()
        output = '/blackbox/export_tmp/osmo_rebind_endpoint_' + value.nonce + '.json'
        self.control.bootstrap.pull_bytes.assert_called_once_with(output, max_bytes=4096, timeout=2)
        self.assertEqual(value.endpoint['port'], 54321)
        cleanup = next(command for command in self.control.commands if ';os.unlink(p)' in command)
        self.assertIn('(9, 100)', cleanup)
        self.assertIn(output, cleanup)
        self.assertEqual(value.manifest['completion_port'], 0)
        self.assertLess(self.events.index('endpoint.pull'), self.events.index('endpoint.unlink'))
        self.assertLess(self.events.index('endpoint.unlink'), self.events.index(('old_lease.stop', dict(close_camera=False))))
        self.assertFalse(any('UDC' in command or command.endswith(' wake') for command in self.control.commands))

    def assert_arm_failure_retains_guard(self, value):
        with self.assertRaises(m.UnknownFutureApply):
            value.arm_watcher()
        self.assertTrue(value.possible_future_apply)
        self.assertIsNone(value.endpoint)
        self.old._lease.stop.assert_not_called()
        with self.assertRaises(m.PreparedStreamError):
            value.arm_watcher()
        self.assertEqual(sum(command.endswith(' launch') for command in self.control.commands), 1)

    def test_foreign_endpoint_preserves_unknown_guard_and_old_lease(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = 'SLEEP', 1
        raw = self.control.bootstrap.pull_bytes.side_effect('unused_' + value.nonce + '.json')
        corrupt = json.loads(raw)
        corrupt['endpoint']['nonce'] = '00' * 6
        self.control.bootstrap.pull_bytes = Mock(return_value=json.dumps(corrupt).encode())
        self.assert_arm_failure_retains_guard(value)

    def test_failed_endpoint_cleanup_preserves_unknown_guard_and_old_lease(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = 'SLEEP', 1
        def reject(command):
            if ';os.unlink(p)' in command: raise OSError('cleanup receipt lost')
        self.control.failure = reject
        self.assert_arm_failure_retains_guard(value)

    def test_missing_endpoint_pull_preserves_unknown_guard_and_old_lease(self):
        value = self.backend().prepare()
        self.control.state, self.control.sequence = 'SLEEP', 1
        self.control.bootstrap.pull_bytes = Mock(side_effect=TimeoutError('endpoint pull missing'))
        self.assert_arm_failure_retains_guard(value)
        self.assertFalse(any(';os.unlink(p)' in command for command in self.control.commands))

    def test_endpoint_receipt_rejects_duplicate_foreign_or_unbounded_fields(self):
        value = self.backend().prepare()
        raw = self.control.bootstrap.pull_bytes.side_effect('unused_' + value.nonce + '.json')
        valid = json.loads(raw)
        self.assertEqual(m.decode_endpoint(raw, value.manifest)['endpoint']['port'], 54321)
        for mutate in (
            lambda item: item.update(schema=True),
            lambda item: item.update(extra=1),
            lambda item: item['endpoint'].update(port=self.plan['lease']['port']),
            lambda item: item['endpoint'].update(socket_identity=[9, True]),
            lambda item: item['endpoint'].update(owner=dict(pid=99, start='1')),
            lambda item: item['export_identity'].update(inode=0),
        ):
            corrupt = deepcopy(valid)
            mutate(corrupt)
            with self.subTest(corrupt=corrupt), self.assertRaises(ValueError):
                m.decode_endpoint(json.dumps(corrupt).encode(), value.manifest)
        for corrupt in (b'x' * 4097, raw.replace(b'"schema": 1', b'"schema": 1, "schema": 1', 1)):
            with self.subTest(raw=corrupt[:80]), self.assertRaises(ValueError):
                m.decode_endpoint(corrupt, value.manifest)

    def test_abandon_keeps_heartbeat_when_mask_is_restored_but_drain_is_unproven(self):
        for proof in (
            dict(restored=True, errors=[]),
            dict(restored=True, quiescent=False, errors=[]),
            dict(restored=True, quiescent=1, errors=[]),
        ):
            with self.subTest(proof=proof):
                self.setUp()
                value = self.backend().prepare()
                imu = value.imu = types.SimpleNamespace(pid=321, stop=Mock(return_value=proof))
                value.imu_started = True
                value.preview = types.SimpleNamespace(_stream=Mock(), _lease=Mock())
                self.camera._imu, self.camera._usb_preview = imu, value.preview
                with patch.object(m, '_collector_closed', return_value=True):
                    with self.assertRaisesRegex(m.PreparedStreamError, 'quiescence'):
                        value.abandon_local()
                value.preview._stream.stop.assert_not_called()
                value.preview._lease.stop.assert_not_called()
                self.old._lease.stop.assert_not_called()
                self.assertFalse(value.abandoned)
                self.assertIs(self.camera._imu, imu)
                self.assertIs(self.camera._usb_preview, value.preview)

    def test_abandon_allows_attempted_start_that_never_created_a_collector(self):
        value = self.backend().prepare()
        value.imu = types.SimpleNamespace(pid=None, stop=Mock(return_value=None))
        value.imu_started = True
        result = value.abandon_local()
        self.assertTrue(result['local_only'])
        self.old._lease.stop.assert_called_once_with(close_camera=False)

    def test_receipt_is_strict_and_bound_to_original_plan_owner_and_fresh_challenge(
        self,
    ):
        value = self.backend().prepare()
        ticket = m.completion_ticket(
            self.plan["lease"]["nonce"], value.nonce, value.wake_id
        )
        receipt = self.receipt(value, ticket, 42)
        self.assertEqual(
            m.decode_completion(
                json.dumps(receipt).encode(),
                manifest=value.manifest,
                ticket=ticket,
                challenge=42,
            ),
            receipt,
        )
        for field, bad in [
            ("schema", True),
            ("reset_attempted", True),
            ("challenge", 41),
            ("nonce", "00" * 6),
            ("terminal", 1),
            ("original_plan_work", "/dev/shm/foreign"),
            ("ticket", "00" * 16),
            ("lease_expires", 100.0),
            ("now", float("nan")),
        ]:
            wrong = dict(receipt, **{field: bad})
            with self.subTest(field=field), self.assertRaises(ValueError):
                m.decode_completion(
                    json.dumps(wrong).encode(),
                    manifest=value.manifest,
                    ticket=ticket,
                    challenge=42,
                )
        duplicate = json.dumps(receipt)[:-1] + ',"challenge":42}'
        with self.assertRaises(ValueError):
            m.decode_completion(
                duplicate.encode(), manifest=value.manifest, ticket=ticket, challenge=42
            )

    def test_two_generations_preserve_only_liveness_until_final_udp_and_fresh_ack(
        self,
    ):
        value = self.armed()
        middle, final = Network(self.events, 1), Network(self.events, 2)
        self.network.next, middle.next = middle, final
        test = self
        sockets = []

        class Sock:
            def __init__(self, *args):
                self.generation = len(sockets) + 1
                sockets.append(self)

            def settimeout(self, timeout):
                pass

            def sendto(self, raw, peer):
                magic, ticket, challenge = m.QUERY.unpack(raw)
                test.assertEqual(magic, m.MAGIC)
                self.raw = json.dumps(test.receipt(value, ticket, challenge)).encode()

            def recvfrom(self, limit):
                if self.generation == 1:
                    # Carry the last SENT sequence, including a heartbeat whose
                    # ACK was lost with the disappearing USB generation.
                    value._provisional_lease._sequence += 1
                    middle.gone = True
                    raise socket.timeout()
                test.assertEqual(test.events.count("lease.ACK"), 2)
                test.assertFalse(value.lease_proven)
                test.assertNotIn("video_start", test.events)
                test.assertNotIn("imu_start", test.events)
                return self.raw, (
                    test.plan["lease"]["camera_ip"],
                    value.endpoint["port"],
                )

            def close(self):
                self.closed = True

        value.socket_factory = Sock
        with (
            patch.object(m, "bind_socket") as binding,
            patch.object(
                m._linux_network,
                "evaluate",
                return_value=dict(ok=True, ncm_interface="usb2"),
            ),
        ):
            preview, imu = value.attach_data_first(native_floor_ns=1, wake_host_ns=2)
        self.assertEqual(binding.call_count, 2)
        self.assertEqual(preview._lease.last_ack["sequence"], 48)
        self.assertEqual(preview._plan["work_dir"], self.old._plan["work_dir"])
        self.assertTrue(value.lease_proven)
        self.assertFalse(value.possible_future_apply)
        self.assertIs(self.camera._usb_preview, preview)
        self.assertIs(self.camera._imu, imu)
        self.assertEqual(imu.options["queue_size"], 16384)
        self.assertEqual(imu.options["read_buffer_bytes"], 65536)
        self.assertTrue(all(sock.closed for sock in sockets))
        self.assertLess(
            self.events.index("network.original_state"), self.events.index("lease.RENEW_ACK")
        )
        self.assertLess(
            self.events.index("lease.RENEW_ACK"), self.events.index("video_start")
        )
        self.assertEqual(self.events.count("lease.local_stop"), 1)
        self.assertNotIn("lease.CLOSE", self.events)
        self.assertEqual([row["first_sent_after"] for row in value.report["lease_handoff"]], [44, 46])
        self.assertTrue(value.report["lease_handoff"][0]["stopped_locally"])
        self.assertTrue(value.report["lease_handoff"][1]["completion_proven"])
        self.assertFalse(
            any("apply.sh" in command for command in self.control.commands)
        )
        deadlines = self.network.deadlines + middle.deadlines + final.deadlines
        self.assertTrue(deadlines)
        self.assertEqual(set(deadlines), {45.0})

    def test_initial_completion_inventory_retries_unregister_under_exact_wake_deadline(self):
        # Exercise the real guard/inventory implementation against fake sysfs,
        # including the FIRST absence probe before rebind_after_usb_reset().
        from test_linux_usb_rebind import LinuxUSBRebindTests
        fixture = LinuxUSBRebindTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        value = self.armed()
        fixture.host.reset()
        value.network, value.baseline = fixture.guard, fixture.baseline
        value.clock = lambda: fixture.host.now
        value.sleep = lambda seconds: setattr(fixture.host, 'now', fixture.host.now + seconds)
        entry = fixture.host.net / 'departing'
        entry.mkdir()
        (entry / 'ifindex').write_text('20')
        read, errors, deadlines = Path.read_text, [], []
        absent = type(fixture.guard)._old_index_absent
        test = self
        def unregister(path, *args, **kwargs):
            if path == entry / 'ifindex':
                test.assertEqual(fixture.host.commands, [])
                path.unlink()
                entry.rmdir()
                errors.append(errno.EINVAL)
                raise OSError(errno.EINVAL, 'netdev unregister', str(path))
            return read(path, *args, **kwargs)
        def observe(guard, old, snapshot=None, *, deadline=None):
            deadlines.append(deadline)
            return absent(guard, old, snapshot, deadline=deadline)
        class Sock:
            def settimeout(self, timeout): pass
            def sendto(self, raw, peer):
                _, ticket, challenge = m.QUERY.unpack(raw)
                self.raw = json.dumps(test.receipt(value, ticket, challenge)).encode()
            def recvfrom(self, limit):
                return self.raw, (test.plan['lease']['camera_ip'], value.endpoint['port'])
            def close(self): pass
        value.socket_factory = lambda *args: Sock()
        with (patch.object(Path, 'read_text', unregister),
              patch.object(type(fixture.guard), '_old_index_absent', observe),
              patch.object(m, 'bind_socket')):
            value.attach_data_first(timeout=13, native_floor_ns=1, wake_host_ns=2)
        self.assertEqual(errors, [errno.EINVAL])
        self.assertEqual(deadlines[0], 13.0)
        self.assertEqual(set(deadlines), {13.0})
        self.assertAlmostEqual(fixture.host.now, .1)
        self.assertEqual(fixture.guard.report['inventory_retries']['count'], 1)
        self.assertTrue(value.lease_proven)
        self.assertEqual([index for index, _ in fixture.host.generations], [9, 9])

    def test_no_completion_stops_provisional_locally_without_media_input_or_rollback(
        self,
    ):
        value = self.armed()
        final = Network(self.events, 1)
        self.network.next = final
        commands = list(self.control.commands)
        self.control.pending_task, self.control._unknown_task = 7, True
        test = self

        class Sock:
            def settimeout(self, timeout):
                pass

            def sendto(self, raw, peer):
                pass

            def recvfrom(self, limit):
                test.clock.sleep(0.2)
                raise socket.timeout()

            def close(self):
                pass

        value.socket_factory = lambda *args: Sock()
        with patch.object(m, "bind_socket"), self.assertRaises(m.UnknownFutureApply):
            value.attach_data_first(timeout=1, native_floor_ns=1, wake_host_ns=2)
        self.assertIn("lease.ACK", self.events)
        self.assertIn("lease.local_stop", self.events)
        self.assertNotIn("lease.CLOSE", self.events)
        self.assertNotIn("video_start", self.events)
        self.assertNotIn("imu_start", self.events)
        self.assertFalse(value.lease_proven)
        self.assertTrue(value.possible_future_apply)
        self.assertIsNone(value._provisional_lease)
        self.assertEqual(self.control.commands, commands)
        self.assertEqual(self.control.pending_task, 7)
        self.assertTrue(self.control._unknown_task)
        with self.assertRaises(m.UnknownFutureApply):
            value.stop()
        self.assertIs(self.camera._usb_preview, self.old)

    def test_stalled_completion_survives_old_lease_expiry_without_starting_media(self):
        value = self.armed()
        final = Network(self.events, 1)
        test = self

        def rebind(**options):
            test.clock.sleep(4)
            return final, final.report['ready']

        self.network.rebind_after_usb_reset = rebind
        leases = []

        class TimedLease(Lease):
            def start(self, **options):
                super().start(**options)
                self.last_ack.update(camera_monotonic=test.clock(), expires=test.clock() + 10)
                return self

            def renew(self):
                result = super().renew()
                result.update(camera_monotonic=test.clock(), expires=test.clock() + 10)
                return result

        def create(config):
            lease = TimedLease(self.events, config)
            leases.append(lease)
            return lease

        value.lease_factory = create

        class Sock:
            def settimeout(self, timeout):
                pass

            def sendto(self, raw, peer):
                _, ticket, challenge = m.QUERY.unpack(raw)
                reply = test.receipt(value, ticket, challenge)
                reply.update(now=test.clock(), lease_expires=leases[-1].last_ack['expires'])
                self.raw = json.dumps(reply).encode()

            def recvfrom(self, limit):
                test.assertFalse(value.lease_proven)
                test.assertNotIn('video_start', test.events)
                test.assertNotIn('imu_start', test.events)
                test.assertLess(test.clock(), leases[-1].last_ack['expires'])
                if test.clock() < 12:
                    test.clock.sleep(1)
                    # Simulate the ordinary one-second heartbeat worker while
                    # final native proof is deliberately withheld past t10.
                    leases[-1].renew()
                    raise socket.timeout()
                return self.raw, (test.plan['lease']['camera_ip'], value.endpoint['port'])

            def close(self):
                pass

        value.socket_factory = lambda *args: Sock()
        with patch.object(m, 'bind_socket'), patch.object(
            m._linux_network, 'evaluate', return_value=dict(ok=True, ncm_interface='usb1')
        ):
            preview, imu = value.attach_data_first(timeout=20, native_floor_ns=1, wake_host_ns=2)
        self.assertGreater(self.clock(), 10)
        self.assertEqual(len(leases), 1)
        self.assertIs(preview._lease, leases[0])
        self.assertTrue(value.lease_proven)
        self.assertFalse(value.possible_future_apply)
        self.assertEqual(leases[0].last_ack, value.report['lease_handoff'][0]['adopted_ack'])
        self.assertNotIn('lease.CLOSE', self.events)
        self.assertFalse(value.imu_started)

    def test_final_completion_without_fresh_ack_keeps_unknown_guard_and_never_closes_plan(self):
        value = self.armed()
        value.network = Network(self.events, 1)
        commands = list(self.control.commands)

        def completion(remaining, *, deadline):
            ready = dict(ok=True, ncm_interface='usb1')
            value._start_provisional(ready, remaining)
            value._provisional_lease.renew = Mock(side_effect=TimeoutError('final ACK lost'))
            return ready

        value._completion = completion
        with self.assertRaises(m.UnknownFutureApply):
            value.attach_data_first(native_floor_ns=1, wake_host_ns=2)
        self.assertFalse(value.lease_proven)
        self.assertTrue(value.possible_future_apply)
        self.assertEqual(self.control.commands, commands)
        self.assertIs(self.camera._usb_preview, self.old)
        self.assertNotIn('video_start', self.events)
        self.assertNotIn('imu_start', self.events)
        self.assertNotIn('lease.CLOSE', self.events)
        self.assertIn('lease.local_stop', self.events)

    def test_late_fresh_ack_still_allows_original_plan_cleanup_without_reapply(self):
        value = self.armed()
        value.network = Network(self.events, 1)

        def completion(remaining, *, deadline):
            ready = dict(ok=True, ncm_interface="usb1")
            value._start_provisional(ready, remaining)
            return ready

        value._completion = completion
        value.lease_factory = lambda config: Lease(self.events, config)
        original = Lease.renew

        def late(lease, **options):
            result = original(lease, **options)
            self.clock.now = 2.0
            return result

        with patch.object(Lease, "renew", late):
            with self.assertRaises(TimeoutError):
                value.attach_data_first(timeout=1, native_floor_ns=1, wake_host_ns=2)
        self.assertTrue(value.lease_proven)
        value.stop()
        self.assertTrue(value.report["native_restored"])
        self.assertTrue(self.old.report["restored"])
        self.assertNotIn("video_start", self.events)
        self.assertNotIn("imu_start", self.events)

    def test_separate_asset_cache_commands_are_bounded_and_do_not_mutate_base_cache(
        self,
    ):
        files = {name: ("# exact " + name + "\n").encode() for name in m.SOURCES}
        digests = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        with patch.dict(m.SOURCES, digests, clear=True):
            assets = m.RetainedAssets(
                self.control, RECEIPT, lambda: files
            ).preload_assets()
            assets.link_into("/dev/shm/osmo-retained-sleep060-012345abcdef")
        self.assertTrue(assets.staged)
        self.assertTrue(all(len(command) <= 500 for command in self.control.commands))
        self.assertFalse(
            any("osmo-prepared-assets-" in command for command in self.control.commands)
        )
        self.assertEqual(
            sum("os.link(" in command for command in self.control.commands), 3
        )

    def cached_assets(self):
        files = {name: ("# exact " + name + "\n").encode() for name in m.SOURCES}
        hashes = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        self.addCleanup(patch.stopall)
        patch.dict(m.SOURCES, hashes, clear=True).start()
        self.control.clock = self.clock
        self.control._resolve_readonly_exit1 = Mock(return_value=True)
        return m.RetainedAssets(self.control, RECEIPT, lambda: files)

    def test_new_process_reuses_same_owner_cache_only_after_correlated_terminal_and_full_ro_proof(
        self,
    ):
        first = self.cached_assets().preload_assets()
        self.control.commands.clear()

        def existing(command):
            if command.startswith("[ ! -e "):
                raise m.BootstrapError("completed F6 test")

        self.control.failure = existing
        second = self.cached_assets().preload_assets()
        self.assertEqual(second.work, first.work)
        self.assertTrue(second.staged)
        self.assertIn(RECEIPT["work"], second.work)
        self.control._resolve_readonly_exit1.assert_called_once_with(
            "[ ! -e " + second.work + " ] && [ ! -L " + second.work + " ]", deadline=5.0
        )
        commands = self.control.commands
        self.assertFalse(
            any(
                "base64 -d" in command or "mkdir " in command or "chmod " in command
                for command in commands
            )
        )
        self.assertEqual(sum("sha256sum " in command for command in commands), 3)
        self.assertTrue(
            any("set(p.name for p in w.iterdir())" in command for command in commands)
        )
        self.assertTrue(
            any(
                "sys_service" in command and "boot_id" in command
                for command in commands
            )
        )
        self.assertTrue(all(len(command) <= 500 for command in commands))

    def test_partial_existing_cache_is_never_overwritten_or_retried(self):
        value = self.cached_assets()

        def partial(command):
            if command.startswith("[ ! -e "):
                raise m.BootstrapError("completed F6 test")
            if "set(p.name for p in w.iterdir())" in command:
                raise m.BootstrapError("partial inventory")

        self.control.failure = partial
        with self.assertRaisesRegex(m.BootstrapError, "partial inventory"):
            value.preload_assets()
        before = list(self.control.commands)
        with self.assertRaises(m.PreparedStreamError):
            value.preload_assets()
        self.assertEqual(before, self.control.commands)
        self.assertFalse(value.staged)
        self.assertFalse(
            any("mkdir " in command or "base64 -d" in command for command in before)
        )

    def test_unknown_cache_absence_task_does_not_start_validation_or_mutation(self):
        value = self.cached_assets()
        self.control._resolve_readonly_exit1.return_value = False
        self.control.failure = lambda command: (_ for _ in ()).throw(
            m.BootstrapError("unknown task")
        )
        with self.assertRaisesRegex(m.BootstrapError, "unknown task"):
            value.preload_assets()
        self.assertEqual(len(self.control.commands), 1)
        self.assertFalse(value.staged)

    def test_actual_native_f6_cache_test_requires_independent_result_before_validation(
        self,
    ):
        class CacheNative(NativeD1):
            def request(self, cmdset, cmdid, payload, **options):
                answer = super().request(cmdset, cmdid, payload, **options)
                if struct.unpack_from("<H", payload)[0] == 0:
                    command = payload[2:514].split(b"\0", 1)[0].decode("ascii")
                    if command.startswith("[ ! -e "):
                        task = struct.unpack_from("<I", payload, 518)[0]
                        self.tasks[task] = 0xF6
                        self.failed_ids.add(task)
                return answer

        files = {name: ("# exact " + name + "\n").encode() for name in m.SOURCES}
        hashes = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        for running in (False, True):
            with (
                self.subTest(running=running),
                patch.dict(m.SOURCES, hashes, clear=True),
            ):
                native = CacheNative()
                native.resolve_running = running
                control = native.control()
                owner = dict(
                    work=control.work,
                    nonce=control.nonce,
                    owner=control.owner,
                    service=control.service,
                )
                value = m.RetainedAssets(control, owner, lambda: files)
                if running:
                    with self.assertRaises(StandbyUncertain):
                        value.preload_assets()
                    self.assertFalse(value.staged)
                else:
                    value.preload_assets()
                    self.assertTrue(value.staged)
                failed = next(iter(native.failed_ids))
                start = next(
                    i
                    for i, row in enumerate(native.events)
                    if row[0] == "START" and row[2].startswith("[ ! -e ")
                )
                later = native.events[start + 1 :]
                results = [
                    i for i, row in enumerate(later) if row == ("RESULT", failed)
                ]
                self.assertGreaterEqual(len(results), 2)
                if running:
                    self.assertFalse(any(row[0] == "START" for row in later))
                else:
                    next_start = next(
                        i for i, row in enumerate(later) if row[0] == "START"
                    )
                    self.assertGreater(next_start, results[1])
                self.assertFalse(
                    any(
                        row[0] == "START"
                        and ("mkdir " in row[2] or "base64 -d" in row[2])
                        for row in native.events
                    )
                )

    def test_actual_python_cache_import_and_link_commands_never_create_pycache(self):
        # Execute only the generated source-import/link Python snippets in a
        # temporary host tree. The camera's stat/UID checks remain commands;
        # no shell, USB, source upload or camera path is executed here.
        observer = (
            "from pathlib import Path\nimport json,hashlib\nfrom types import SimpleNamespace\n"
            "import prepared_preview_wake_045\n"
            "base=SimpleNamespace(private=lambda p,**kw:Path(p).stat(),"
            "read_json=lambda p:json.loads(Path(p).read_text()),read_bytes=lambda p,n:Path(p).read_bytes())\n"
        )
        files = {
            m.HELPER: b"# helper\n",
            "prepared_preview_wake_045.py": b"VALUE=1\n",
            "prepared_preview_wake_047.py": observer.encode(),
        }
        hashes = {
            name: hashlib.sha256(data).hexdigest() for name, data in files.items()
        }
        with (
            tempfile.TemporaryDirectory() as folder,
            patch.dict(m.SOURCES, hashes, clear=True),
        ):
            root = Path(folder)
            cache = root / "cache"
            cache.mkdir()
            target = root / "fresh"
            target.mkdir()
            owner = root / "owner"
            owner.mkdir()
            config = dict(
                owner=RECEIPT["owner"],
                sys_service=RECEIPT["service"],
                nonce=RECEIPT["nonce"],
                boot_id="test-boot",
            )
            (owner / "config.json").write_text(json.dumps(config))
            (cache / "cache.json").write_text(json.dumps(config))
            boot = root / "boot-id"
            boot.write_text("test-boot\n")
            for name, data in files.items():
                (cache / name).write_bytes(data)
                (cache / name).chmod(0o400)
            assets = m.RetainedAssets(self.control, RECEIPT, lambda: files)
            assets.staged = True
            work = "/dev/shm/osmo-retained-sleep060-012345abcdef"
            assets.link_into(work)
            executed = 0
            for command in self.control.commands:
                if "import prepared_preview_wake_047 as m" not in command:
                    continue
                tokens = shlex.split(command)
                offset = tokens.index("python3")
                self.assertEqual(tokens[offset + 1 : offset + 3], ["-B", "-c"])
                program, *args = tokens[offset + 3 :]
                program = program.replace("/proc/sys/kernel/random/boot_id", str(boot))
                args = [
                    arg.replace(assets.work, str(cache))
                    .replace(work, str(target))
                    .replace(RECEIPT["work"], str(owner))
                    for arg in args
                ]
                result = subprocess.run(
                    [sys.executable, "-B", "-c", program, *args],
                    cwd=cache,
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                executed += 1
            self.assertEqual(executed, 4)
            self.assertEqual(
                set(path.name for path in cache.iterdir()), set(files) | {"cache.json"}
            )
            self.assertFalse((cache / "__pycache__").exists())
            self.assertEqual(set(path.name for path in target.iterdir()), set(files))


if __name__ == "__main__":
    unittest.main()
