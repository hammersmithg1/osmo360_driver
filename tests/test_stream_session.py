"""Pure orchestration/ownership tests; no camera, USB or network is opened."""

from collections import deque
import threading
import types
import unittest

from osmo360_camera import _app
from osmo360_camera import _stream_session as m

PRESENT = bytes.fromhex("12004400012300415050000000000002")
ABSENT = bytes.fromhex("1200550000")


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Capture:
    def __init__(self, env, kind, timestamp=100):
        self.env, self.kind = env, kind
        self.items = deque(
            [types.SimpleNamespace(timestamp_ns=timestamp, host_receive_monotonic_ns=0)]
        )
        self.stats = dict(collector_returncode=0, last_timestamp_ticks=2)
        self.report = dict(restored=False)
        self._stream = self
        self.pause_ok = self.mask_ok = True

    def start(self):
        self.env.events.append(self.kind + ".start")
        return self

    def read(self, timeout=None):
        if self.items:
            return self.items.popleft()
        self.env.clock.now += timeout or 0.1
        raise TimeoutError()

    def pause(self, **options):
        self.env.events.append("pause")
        return self.pause_ok

    def resume(self, **options):
        self.env.events.append("resume")

    def stop(self, **options):
        self.env.events.append(self.kind + ".stop")
        self.report = dict(
            restored=self.mask_ok if self.kind == "imu" else True, errors=[]
        )
        return self.report


class Control:
    def __init__(self, env):
        self.env, self.launch_attempted = env, False
        self.work, self.nonce = "/dev/shm/osmo-usb-standby-aabbccddeeff", "aabbccddeeff"
        self.owner, self.service = dict(pid=100, start="1"), dict(pid=200, start="2")
        self.last = dict(state="AWAKE", last_sequence=0)
        self.finalized = self.detached = False

    def launch(self, **options):
        self.env.events.append("owner.launch")
        self.launch_attempted = True

    def snapshot(self):
        self.env.events.append("owner.snapshot")
        return self.last

    def _submit_sleep(self, proof):
        proof.validate(self.env.clock())
        self.env.events.append("owner.sleep")
        self.last = dict(state="SLEEPING", last_sequence=self.last["last_sequence"] + 1)
        return dict(command_id=f"{self.last['last_sequence']:016x}")

    def submit_wake(self):
        self.env.events.append("owner.wake")
        self.env.wake_submitted.set()
        self.last = dict(state="WAKING", last_sequence=self.last["last_sequence"] + 1)
        return dict(command_id=f"{self.last['last_sequence']:016x}")

    def wait(self, identifier, target, **options):
        self.env.events.append("owner.wait." + target)
        self.last = dict(
            state=target, last_sequence=int(identifier, 16), command_id=identifier
        )
        return self.last

    def bind_bootstrap(self, value):
        self.env.events.append("owner.bind")

    def detach(self):
        self.env.events.append("owner.detach")
        self.detached = True
        return dict(retained_shadow=True, native_restored=False)

    def finalize(self):
        self.env.events.append("owner.finalize")
        self.finalized = True
        return dict(reboot_requested=True, restored=False)


class Witness:
    def __init__(self, floor, host):
        self.floor, self.host, self.v, self.i = floor, host, False, False

    def observe_video(self, item):
        good = (
            item.timestamp_ns > self.floor
            and item.host_receive_monotonic_ns >= self.host
        )
        self.v = self.v or good
        return good

    def observe_imu(self, item):
        good = (
            item.timestamp_ns > self.floor
            and item.host_receive_monotonic_ns >= self.host
        )
        self.i = self.i or good
        return good

    @property
    def joint_ready(self):
        return self.v and self.i

    @property
    def report(self):
        return dict(joint_ready=self.joint_ready)


class Prepared:
    def __init__(self, env, **options):
        self.env, self.options = env, options
        self.report = {}
        self.lease_proven = self.possible_future_apply = False
        self.cancel_error = None

    def prepare(self):
        self.env.events.append("prepared.prepare")
        return self

    def arm_watcher(self):
        self.env.events.append("prepared.arm")
        self.possible_future_apply = True

    def attach_data_first(self, *, timeout, native_floor_ns, wake_host_ns):
        self.env.events.append("prepared.attach")
        if not self.env.wake_submitted.wait(0.5):
            raise AssertionError("Wake was not queued")
        self.lease_proven = True
        self.witness = Witness(native_floor_ns, wake_host_ns)
        self.preview, self.imu = Capture(self.env, "preview"), Capture(self.env, "imu")
        self.preview.items = deque(
            [
                types.SimpleNamespace(
                    timestamp_ns=native_floor_ns - 1,
                    host_receive_monotonic_ns=wake_host_ns,
                ),
                types.SimpleNamespace(
                    timestamp_ns=native_floor_ns + 2,
                    host_receive_monotonic_ns=wake_host_ns,
                ),
            ]
        )
        self.imu.items = deque(
            [
                types.SimpleNamespace(
                    timestamp_ns=native_floor_ns + 1,
                    host_receive_monotonic_ns=wake_host_ns,
                )
            ]
        )
        self.fresh_video, self.fresh_imu = self.preview.items[-1], self.imu.items[-1]
        return self.preview, self.imu

    def stop(self):
        if not self.lease_proven:
            self.env.events.append("prepared.local_close")
            return self.cancel_before_apply()
        self.env.events.append("prepared.stop")
        return self.preview.stop()

    def cancel_before_apply(self):
        self.env.events.append("prepared.cancel")
        if self.cancel_error:
            raise self.cancel_error
        self.possible_future_apply = False
        return dict(no_future_apply=True)

    def abandon_local(self):
        self.env.events.append("prepared.abandon_local")
        return dict(cleanup_deferred=True, normal_rollback_sent=False)


class Environment:
    def __init__(self):
        self.events, self.clock = [], Clock()
        self.present = True
        self.wake_submitted = threading.Event()
        transport = types.SimpleNamespace(_sequence=10, _state_lock=threading.Lock())

        def request(*args, **options):
            self.events.append("app.query")
            return types.SimpleNamespace(payload=PRESENT if self.present else ABSENT)

        def write(message):
            self.events.append("app.unregister")
            self.message = message
            self.present = False

        transport.request, transport._write = request, write
        self.camera = types.SimpleNamespace(
            _control_lock=threading.RLock(),
            _transport=transport,
            _imu=None,
            _usb_preview=None,
            connect=lambda: self.events.append("camera.connect"),
            usb_preview=lambda **options: Capture(self, "preview"),
            imu=lambda **options: Capture(self, "imu"),
        )
        self.control = Control(self)
        self.prepared = None

    def prepared_factory(self, camera, control, **options):
        self.prepared = Prepared(self, **options)
        return self.prepared

    def session(self):
        return m.USBStreamSession(
            self.camera,
            control=self.control,
            prepared_factory=self.prepared_factory,
            idle_probe=lambda camera: m._IdleEvidence(self.clock(), 0, 1, True),
            reconnect_control=lambda camera: self.events.append("reconnect.RO"),
            clock=self.clock,
            sleep=self.clock.sleep,
        )


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.env = Environment()
        self.s = self.env.session()

    def test_owner_precedes_initial_capture_and_primed_data_is_preserved(self):
        self.s.start()
        events = self.env.events
        self.assertLess(events.index("owner.launch"), events.index("preview.start"))
        self.assertLess(events.index("preview.start"), events.index("imu.start"))
        self.assertEqual(self.s.read_video().timestamp_ns, 100)
        self.assertEqual(self.s.read_imu().timestamp_ns, 100)
        self.assertTrue(self.s.owns_camera_resources)

    def test_existing_camera_capture_is_rejected_before_any_owner_or_capture_mutation(
        self,
    ):
        for field in ("_imu", "_usb_preview"):
            env = Environment()
            borrowed = object()
            setattr(env.camera, field, borrowed)
            session = env.session()
            with self.assertRaises(m.StreamSessionError):
                session.start()
            self.assertNotIn("owner.launch", env.events)
            self.assertNotIn("preview.start", env.events)
            self.assertNotIn("imu.stop", env.events)
            self.assertNotIn("app.unregister", env.events)
            session.close()
            self.assertIs(getattr(env.camera, field), borrowed)

    def test_optional_immutable_assets_are_preloaded_after_owner_before_first_usb_rebind(
        self,
    ):
        env = self.env

        class Factory:
            def __call__(self, *args, **options):
                return env.prepared_factory(*args, **options)

            def preload_assets(self, camera, control, *, owner_receipt):
                env.events.append("assets.preload")
                self.receipt = owner_receipt
                return dict(verified=True)

        factory = Factory()
        self.s.prepared_factory = factory
        self.s.start()
        self.assertLess(
            env.events.index("owner.launch"), env.events.index("assets.preload")
        )
        self.assertLess(
            env.events.index("assets.preload"), env.events.index("preview.start")
        )
        self.assertEqual(factory.receipt["owner"], env.control.owner)
        self.s.sleep()
        self.assertEqual(env.events.count("assets.preload"), 1)

    def test_sleep_unregisters_once_and_restores_mask_before_usb_then_prepares(self):
        self.s.start()
        self.env.events.clear()
        self.s.sleep()
        events = self.env.events
        for first, second in [
            ("pause", "app.unregister"),
            ("app.unregister", "imu.stop"),
            ("imu.stop", "preview.stop"),
            ("preview.stop", "prepared.prepare"),
            ("prepared.prepare", "owner.sleep"),
        ]:
            self.assertLess(events.index(first), events.index(second))
        self.assertEqual(events.count("app.unregister"), 1)
        msg = self.env.message
        self.assertEqual(
            (msg.sender, msg.receiver, msg.command, msg.payload),
            (2, 0x28, (0, 0x88), _app.registration_payload(unregister=True)),
        )
        self.assertEqual(self.s.state, "SLEEP")
        self.assertFalse(any("finalize" in item for item in events))

    def test_wake_requires_joint_fresh_samples_and_returns_before_late_control(self):
        self.s.start()
        old_preview, old_imu = self.s._preview, self.s._imu
        self.s.sleep()
        self.env.clock.now += 20
        self.env.events.clear()
        self.s.wake()
        self.assertIsNot(self.s._preview, old_preview)
        self.assertIsNot(self.s._imu, old_imu)
        self.assertEqual(self.s.state, "AWAKE")
        self.assertTrue(self.env.prepared.report["media_ready_proven"])
        self.assertNotIn("owner.wait.AWAKE", self.env.events)
        self.assertIs(self.s.read_video(), self.env.prepared.fresh_video)
        self.assertIs(self.s.read_imu(), self.env.prepared.fresh_imu)
        self.s.verify_control()
        self.assertIn("owner.wait.AWAKE", self.env.events)
        self.assertEqual(self.env.events.count("owner.wake"), 1)

    def test_failed_mask_restore_keeps_lease_usb_and_session_ownership(self):
        self.s.start()
        self.s._imu.mask_ok = False
        self.env.events.clear()
        with self.assertRaises(m.StreamSessionError):
            self.s.sleep()
        self.assertNotIn("preview.stop", self.env.events)
        self.assertNotIn("prepared.prepare", self.env.events)
        self.assertNotIn("owner.sleep", self.env.events)
        self.assertTrue(self.s.owns_camera_resources)
        self.assertFalse(self.s.cleanup_safe)
        with self.assertRaises(m.StreamSessionError):
            self.s.close()
        self.assertFalse(self.env.control.detached)
        self.assertFalse(self.env.control.finalized)

    def test_default_close_stable_sleep_cancels_unarmed_plan_and_detaches_without_reboot(
        self,
    ):
        self.s.start()
        self.s.sleep()
        self.env.events.clear()
        result = self.s.close()
        self.assertIn("prepared.cancel", self.env.events)
        self.assertIn("owner.detach", self.env.events)
        self.assertNotIn("owner.wake", self.env.events)
        self.assertNotIn("owner.finalize", self.env.events)
        self.assertTrue(result["retained_shadow"])
        self.assertFalse(result["native_restored"])
        self.assertFalse(self.s.owns_camera_resources)
        self.assertTrue(self.s.cleanup_safe)

    def test_unknown_future_apply_never_triggers_early_rollback_or_implicit_reboot(
        self,
    ):
        self.s.start()
        self.s.sleep()
        self.env.events.clear()
        self.env.prepared.possible_future_apply = True
        self.env.prepared.cancel_error = RuntimeError("future apply intent uncertain")
        with self.assertRaises(m.StreamSessionError):
            self.s.close()
        self.assertNotIn("prepared.stop", self.env.events)
        self.assertNotIn("owner.finalize", self.env.events)
        self.assertNotIn("owner.detach", self.env.events)
        self.assertTrue(self.s.owns_camera_resources)
        self.assertFalse(self.s.cleanup_safe)
        self.s.recover()
        self.assertIn("owner.finalize", self.env.events)
        self.assertEqual(self.s.state, "RECOVERY_PENDING")

    def test_unknown_late_control_uses_local_abandon_not_normal_usb_stop(self):
        self.s.start()
        self.s.sleep()
        self.env.clock.now += 20
        self.s.wake()

        def unknown(*args, **options):
            raise RuntimeError("old D1 task running")

        self.env.control.wait = unknown
        self.env.events.clear()
        with self.assertRaises(m.StreamSessionError):
            self.s.close()
        self.assertIn("prepared.abandon_local", self.env.events)
        self.assertNotIn("prepared.stop", self.env.events)
        self.assertNotIn("owner.finalize", self.env.events)
        self.assertTrue(self.s.owns_camera_resources)

    def test_concurrent_floor_updates_never_regress(self):
        self.s.start()
        self.s._primed_video.clear()
        self.s._primed_imu.clear()
        low_ready, high_done, low_release = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )

        def video_read(**options):
            low_ready.set()
            self.assertTrue(low_release.wait(1))
            return types.SimpleNamespace(timestamp_ns=150)

        def imu_read(**options):
            return types.SimpleNamespace(timestamp_ns=300)

        self.s._preview.read, self.s._imu.read = video_read, imu_read
        failures = []

        def lower():
            try:
                self.s.read_video()
            except BaseException as error:
                failures.append(error)

        thread = threading.Thread(target=lower)
        thread.start()
        self.assertTrue(low_ready.wait(1))
        self.s.read_imu()
        high_done.set()
        low_release.set()
        thread.join(1)
        self.assertFalse(failures)
        self.assertEqual(self.s._native_floor_ns, 300)

    def test_borrowed_old_capture_cannot_publish_into_new_generation(self):
        self.s.start()
        self.s._primed_video.clear()
        entered, release = threading.Event(), threading.Event()

        def old_read(**options):
            entered.set()
            self.assertTrue(release.wait(1))
            return types.SimpleNamespace(timestamp_ns=999)

        self.s._preview.read = old_read
        errors = []

        def read():
            try:
                self.s.read_video()
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=read)
        thread.start()
        self.assertTrue(entered.wait(1))
        with self.s._lock:
            self.s._preview = Capture(self.env, "preview")
        release.set()
        thread.join(1)
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], m.StreamSessionError)
        self.assertEqual(self.s._native_floor_ns, 100)


if __name__ == "__main__":
    unittest.main()
