"""Owned preview lifecycle with fake control/network/decoder; no camera I/O."""

from contextlib import ExitStack, contextmanager
from dataclasses import replace
import threading
from types import SimpleNamespace
from unittest import TestCase, mock

from osmo360_camera.camera import Camera, CameraError, CameraState
from osmo360_camera._usb_bootstrap import USBRuntime, NATIVE_LINKS, PLACEHOLDER
from osmo360_camera.imu import IMUStopError
from osmo360_camera.usb_preview import USBPreview, USBPreviewError
import osmo360_camera.usb_preview as preview_module


class FakeIMU:
    pid = 123

    def __init__(self, events, name="live", fail=False):
        self.events, self.name, self.fail = events, name, fail
        self.quiescent = True

    def stop(self):
        self.events.append("imu.stop:" + self.name)
        return dict(
            restored=not self.fail, quiescent=self.quiescent,
            errors=["mask failure"] if self.fail else []
        )


class FaultedIMU:
    pid = 123

    def __init__(self, events, *, restored=True, quiescent=True, returncode=1):
        self.events = events
        self.error = IMUStopError(
            "Collector lost USB; streaming ended with returncode=1"
        )
        self.stats = {"collector_returncode": returncode}
        self.report = dict(
            restored=restored,
            quiescent=quiescent,
            errors=["DIAG read: NO_DEVICE"],
            original_last_item=8191,
            original_mask_hex="0000",
            mask_restore_rebind=dict(
                restored=restored,
                set_attempts=1,
                method="one_restore_set_and_readback",
                sample_stream_resumed=False,
            ),
        )

    def stop(self):
        self.events.append("imu.stop:faulted")
        raise self.error


class FakeTransport:
    connected = True

    def __init__(self, events):
        self.events = events
        self.charging_failure = None

    def open(self):
        self.events.append("transport.open")
        self.connected = True

    def close(self):
        self.events.append("transport.close")
        self.connected = False

    def wait_disconnected(self, timeout):
        self.events.append("transport.disconnect")
        return True

    def select_charging_only(self):
        self.events.append("transport.charging_only")
        if self.charging_failure:
            raise self.charging_failure
        return dict(
            delivered=True,
            ack_claimed=False,
            source=0x1C,
            target=0x28,
            sequence=7,
            flags=0x40,
            command_set=6,
            command=0x3A,
            payload_hex="0003",
        )


class FakeCamera:
    def __init__(self, events, record=0):
        self.events, self.record = events, record
        self._transport = FakeTransport(events)
        self._control_lock = threading.RLock()
        self._imu = None

    def connect(self):
        self.events.append("camera.connect")

    def state(self, **_):
        self.events.append("camera.state")
        return CameraState(self.record << 6, 1)


def runtime():
    return USBRuntime(
        "a600000.dwc3",
        "0x2ca3",
        "0x0025",
        "BULK",
        NATIVE_LINKS,
        "super-speed-plus",
        "aa:e4:f8:1c:10:86",
        "96:03:ae:cc:f8:37",
        None,
        None,
        True,
        PLACEHOLDER,
        (),
    )


@contextmanager
def harness(
    *, failure=None, publish_after=1, record=0, restored_runtime=None, timeout=1
):
    events = []
    camera = FakeCamera(events, record)
    original = runtime()
    plan = dict(
        commands=["upload", "verify"],
        apply_command="apply_once",
        rollback_command="rollback_once",
        work_dir="/dev/shm/owned",
        sha256={"apply.sh": "verified"},
        lease=dict(
            camera_ip="192.168.253.1",
            host_ip="192.168.253.2",
            nonce="00" * 16,
            port=50000,
        ),
    )
    state = SimpleNamespace(
        runtime_queries=0, stream=None, lease=None, host_alias=False
    )

    class Session:
        def __init__(self, *_, **__):
            pass

        def run(self, command, **_):
            events.append("session:" + command)
            return dict(completed=True, success=True, result=0)

    class Bootstrap:
        last_report = {"diagnostic_file_removed": True}

        def __init__(self, *_, **__):
            pass

        def query_runtime(self):
            events.append("runtime.query")
            state.runtime_queries += 1
            if failure == "runtime" and state.runtime_queries == 1:
                raise RuntimeError("runtime query failed")
            return (
                original
                if state.runtime_queries == 1
                else (restored_runtime or original)
            )

        def run_checked(self, command):
            events.append("bootstrap:" + command)
            if failure == command:
                raise TimeoutError("lost reply for " + command)
            return dict(success=True)

    class Network:
        def preflight(self):
            events.append("network.preflight")
            if failure == "preflight":
                raise RuntimeError("network preflight failed")
            return {"baseline": True}

        def wait_ready(self, baseline, host_mac, **_):
            events.append("network.ready")
            if failure == "network":
                raise RuntimeError("network setup failed")
            assert baseline == {"baseline": True} and host_mac == original.host_addr
            state.host_alias = True
            return dict(ok=True, ncm_interface="en9")

        def cleanup_owned(self):
            events.append("network.cleanup")
            if failure == "cleanup_network":
                raise RuntimeError("owned alias cleanup failed")
            state.host_alias = False
            return {"restored": True}

        def check_restored(self, baseline):
            events.append("network.restored")
            assert baseline == {"baseline": True}
            if failure == "restore_network":
                raise RuntimeError("network not restored")
            return {"ok": True}

    class Lease:
        healthy = True
        stats = {"acknowledged": 1}

        def __init__(self, *_, **__):
            state.lease = self

        def start(self, **_):
            events.append("lease.start")
            return self

        def stop(self):
            events.append("lease.close")
            self.healthy = False
            if failure == "lease_close":
                raise RuntimeError("close ACK lost")
            return {"state": 2}

    class Stream:
        def __init__(self, **kwargs):
            state.stream = self
            self.options = kwargs
            self.polls = 0
            self.published = False

        def start(self):
            events.append("video.start")
            return self

        @property
        def stats(self):
            self.polls += 1
            if (
                publish_after is not None
                and self.polls >= publish_after
                and not self.published
            ):
                self.published = True
                events.append("video.first-frame")
            return dict(published_frames=int(self.published), error=None, stopped=False)

        def read(self, timeout=None):
            return ("frame", timeout)

        def stop(self):
            events.append("video.stop")
            return {"stopped": True}

    with ExitStack() as stack:

        def amt_ready(*args, **kwargs):
            events.append("diagnostic.ready")
            return {"ready": True}

        for name, value in [
            ("wait_amt_ready", amt_ready),
            ("D1Session", Session),
            ("USBBootstrap", Bootstrap),
            ("PreviewLease", Lease),
            ("VideoStream", Stream),
            ("build_usb_plan", lambda _: plan),
        ]:
            stack.enter_context(mock.patch("osmo360_camera.usb_preview." + name, value))
        preview = USBPreview(
            camera, decode=False, network_guard=Network(), timeout=timeout
        )
        camera._usb_preview = preview
        yield SimpleNamespace(
            events=events, camera=camera, preview=preview, state=state, plan=plan
        )


class USBPreviewLifecycleTests(TestCase):
    def test_default_linux_guard_is_fresh_and_explicit_override_is_preserved(self):
        from osmo360_camera._linux_usb_network import LinuxUSBNetworkGuard

        with mock.patch.object(preview_module._host_network.sys, "platform", "linux"):
            first = USBPreview(FakeCamera([]), decode=False)
            second = USBPreview(FakeCamera([]), decode=False)
        self.assertIsInstance(first.network, LinuxUSBNetworkGuard)
        self.assertIsNot(first.network, second.network)
        self.assertIsNone(first.network.baseline)

        class FalseyOverride:
            def __bool__(self):
                return False

        explicit = FalseyOverride()
        with mock.patch.object(
            preview_module._host_network,
            "preview_guard",
            side_effect=AssertionError("factory called"),
        ):
            self.assertIs(
                USBPreview(
                    FakeCamera([]), decode=False, network_guard=explicit
                ).network,
                explicit,
            )

    def test_passive_guard_override_keeps_original_three_method_protocol(self):
        with harness() as case:
            owner = case.preview.network
            case.preview.network = SimpleNamespace(
                preflight=owner.preflight,
                wait_ready=owner.wait_ready,
                check_restored=owner.check_restored,
            )
            case.preview.start()
            self.assertTrue(case.preview.stop()["restored"])

    def test_charging_selection_is_once_after_idle_check_and_before_bootstrap(self):
        with harness() as case:
            case.preview.start()
            events = case.events
            self.assertEqual(events.count("transport.charging_only"), 1)
            self.assertLess(
                events.index("camera.state"), events.index("transport.charging_only")
            )
            for later in (
                "network.preflight",
                "diagnostic.ready",
                "runtime.query",
                "bootstrap:upload",
            ):
                self.assertLess(
                    events.index("transport.charging_only"), events.index(later)
                )
            self.assertEqual(
                case.preview.report["charging_only"],
                dict(
                    delivered=True,
                    ack_claimed=False,
                    source=0x1C,
                    target=0x28,
                    sequence=7,
                    flags=0x40,
                    command_set=6,
                    command=0x3A,
                    payload_hex="0003",
                ),
            )
            self.assertIs(case.preview.start(), case.preview)
            self.assertEqual(events.count("transport.charging_only"), 1)
            case.preview.stop()

    def test_failed_charging_delivery_aborts_without_retry_or_bootstrap(self):
        with harness() as case:
            error = RuntimeError("charging write failed")
            case.camera._transport.charging_failure = error
            with self.assertRaises(RuntimeError) as caught:
                case.preview.start()
            self.assertIs(caught.exception, error)
            self.assertEqual(case.events.count("transport.charging_only"), 1)
            for forbidden in (
                "network.preflight",
                "diagnostic.ready",
                "runtime.query",
                "bootstrap:upload",
            ):
                self.assertNotIn(forbidden, case.events)
            self.assertNotIn("charging_only", case.preview.report)

    def test_late_completed_bundle_never_applies_after_setup_deadline(self):
        with harness(timeout=1) as case:
            case.plan["commands"] = ["upload"]
            clock = {"now": 1.0}
            original = preview_module.USBBootstrap.run_checked

            def complete_late(bootstrap, command):
                result = original(bootstrap, command)
                clock["now"] = 3.0
                return result

            with (
                mock.patch.object(
                    preview_module.time, "monotonic", side_effect=lambda: clock["now"]
                ),
                mock.patch.object(
                    preview_module.USBBootstrap, "run_checked", complete_late
                ),
            ):
                with self.assertRaisesRegex(USBPreviewError, "deadline"):
                    case.preview.start()
            self.assertEqual(case.events.count("bootstrap:upload"), 1)
            self.assertNotIn("bootstrap:apply_once", case.events)
            self.assertNotIn("session:rollback_once", case.events)
            self.assertNotIn("transport.close", case.events)

    def test_success_waits_for_first_frame_and_verifies_restoration(self):
        with harness(publish_after=4) as case:
            preview = case.preview.start()
            case.events.append("start.returned")
            self.assertLess(
                case.events.index("video.first-frame"),
                case.events.index("start.returned"),
            )
            self.assertGreaterEqual(case.state.stream.polls, 4)
            self.assertEqual(
                case.state.stream.options["local_address"], "192.168.253.2"
            )
            self.assertEqual(case.state.stream.options["interface"], "en9")
            self.assertEqual(preview.read(0.5), ("frame", 0.5))
            report = preview.stop()
            self.assertTrue(report["restored"])
            self.assertTrue(report["network_restored"]["ok"])
            self.assertEqual(case.state.runtime_queries, 2)
            self.assertEqual(case.events.count("bootstrap:apply_once"), 1)
            self.assertNotIn("session:rollback_once", case.events)
            self.assertIsNone(case.camera._usb_preview)
            self.assertIs(preview.stop(), report)

    def test_imu_stops_only_immediately_before_apply_and_before_lease_close(self):
        with harness() as case:
            case.camera._imu = FakeIMU(case.events, "initial")
            case.preview.start()
            events = case.events
            self.assertLess(
                events.index("bootstrap:verify"), events.index("imu.stop:initial")
            )
            self.assertLess(
                events.index("imu.stop:initial"), events.index("bootstrap:apply_once")
            )
            case.camera._imu = FakeIMU(events, "live")
            case.preview.stop()
            self.assertLess(events.index("video.stop"), events.index("imu.stop:live"))
            self.assertLess(events.index("imu.stop:live"), events.index("lease.close"))
            self.assertIsNone(case.camera._imu)

    def test_unstarted_stop_and_pre_apply_failure_do_not_stop_an_existing_imu(self):
        for failure in (None, "preflight", "runtime", "upload"):
            with self.subTest(failure=failure), harness(failure=failure) as case:
                imu = case.camera._imu = FakeIMU(case.events, "unrelated")
                if failure:
                    with self.assertRaises((RuntimeError, TimeoutError)):
                        case.preview.start()
                else:
                    case.preview.stop()
                self.assertNotIn("imu.stop:unrelated", case.events)
                self.assertIs(case.camera._imu, imu)
                self.assertNotIn("bootstrap:apply_once", case.events)
                self.assertNotIn("session:rollback_once", case.events)

    def test_active_recording_blocks_before_network_or_imu_changes(self):
        with harness(record=2) as case:
            case.camera._imu = FakeIMU(case.events, "unrelated")
            with self.assertRaisesRegex(USBPreviewError, "Record0"):
                case.preview.start()
            self.assertNotIn("transport.charging_only", case.events)
            self.assertNotIn("network.preflight", case.events)
            self.assertNotIn("imu.stop:unrelated", case.events)
            self.assertNotIn("bootstrap:apply_once", case.events)

    def test_lost_apply_reply_never_retries_apply_and_still_verifies_rollback(self):
        with harness(failure="apply_once") as case:
            with self.assertRaisesRegex(TimeoutError, "lost reply"):
                case.preview.start()
            self.assertEqual(case.events.count("bootstrap:apply_once"), 1)
            self.assertEqual(case.events.count("session:rollback_once"), 1)
            self.assertTrue(case.preview.report["restored"])
            self.assertTrue(case.preview.report["errors"])

    def test_no_first_frame_times_out_and_closes_owned_lease(self):
        with harness(publish_after=None, timeout=0.025) as case:
            with self.assertRaisesRegex(USBPreviewError, "deadline"):
                case.preview.start()
            self.assertIn("video.stop", case.events)
            self.assertIn("lease.close", case.events)
            self.assertTrue(case.preview.report["restored"])

    def test_failed_video_stop_retains_metadata_diagnostic_and_restores_usb(self):
        with harness() as case:
            case.preview.start()
            cause = "Слишком короткий блок движения"
            stats = dict(
                stopped=True,
                error="Invalid preview timestamp metadata: " + cause,
                metadata_error=dict(
                    error_type="ValueError",
                    error_message=cause,
                    offending_sei={"nal_hex": "06f50180"},
                ),
            )

            def fail_stop():
                case.events.append("video.stop")
                raise RuntimeError(stats["error"])

            with (
                mock.patch.object(case.state.stream, "stop", side_effect=fail_stop),
                mock.patch.object(
                    type(case.state.stream),
                    "stats",
                    new_callable=mock.PropertyMock,
                    return_value=stats,
                ),
            ):
                with self.assertRaisesRegex(USBPreviewError, cause) as caught:
                    case.preview.stop()
            report = case.preview.report
            self.assertIs(caught.exception.report, report)
            self.assertEqual(report["video"]["metadata_error"], stats["metadata_error"])
            self.assertIsNot(report["video"], stats)
            self.assertIn("Video: " + stats["error"], report["errors"])
            self.assertTrue(report["restored"])
            self.assertTrue(report["network_restored"]["ok"])
            self.assertLess(
                case.events.index("video.stop"), case.events.index("lease.close")
            )
            self.assertEqual(case.state.runtime_queries, 2)
            self.assertIsNone(case.camera._usb_preview)

    def test_mask_restore_failure_keeps_lease_alive_and_allows_stop_retry(self):
        with harness() as case:
            case.preview.start()
            imu = case.camera._imu = FakeIMU(case.events, fail=True)
            with self.assertRaisesRegex(USBPreviewError, "IMU mask restoration"):
                case.preview.stop()
            self.assertNotIn("lease.close", case.events)
            self.assertNotIn("network.cleanup", case.events)
            self.assertTrue(case.state.lease.healthy)
            self.assertTrue(case.state.host_alias)
            self.assertFalse(case.preview.report["restored"])
            imu.fail = False
            self.assertTrue(case.preview.stop()["restored"])
            self.assertLess(
                case.events.index("imu.stop:live"), case.events.index("lease.close")
            )
            self.assertLess(
                case.events.index("lease.close"), case.events.index("network.cleanup")
            )
            self.assertFalse(case.state.host_alias)

    def test_completed_imu_fault_with_verified_mask_still_restores_usb_then_raises_original(
        self,
    ):
        with harness() as case:
            case.preview.start()
            imu = case.camera._imu = FaultedIMU(case.events)
            with self.assertRaises(IMUStopError) as caught:
                case.preview.stop()
            self.assertIs(caught.exception, imu.error)
            report = case.preview.report
            self.assertEqual(report["imu_stop"], imu.report)
            self.assertEqual(report["imu_collector_returncode"], 1)
            self.assertIn("Collector lost USB", report["imu_stop_error"])
            self.assertTrue(report["imu_stop"]["mask_restore_rebind"]["restored"])
            self.assertFalse(
                report["imu_stop"]["mask_restore_rebind"]["sample_stream_resumed"]
            )
            self.assertTrue(report["errors"])
            self.assertTrue(report["restored"])
            self.assertTrue(report["network_restored"]["ok"])
            self.assertLess(
                case.events.index("imu.stop:faulted"), case.events.index("lease.close")
            )
            self.assertLess(
                case.events.index("lease.close"), case.events.index("network.cleanup")
            )
            self.assertEqual(case.state.runtime_queries, 2)
            self.assertIsNone(case.camera._imu)
            self.assertIsNone(case.camera._usb_preview)
            self.assertIs(case.preview.stop(), report)
            self.assertEqual(case.events.count("lease.close"), 1)
            self.assertEqual(case.events.count("imu.stop:faulted"), 1)

    def test_mask_only_stop_cannot_close_lease_or_rollback_and_can_finish_once_quiet(self):
        for proof in (
            dict(restored=True, errors=[]),
            dict(restored=True, quiescent=False, errors=[]),
            dict(restored=True, quiescent=1, errors=[]),
        ):
            with self.subTest(proof=proof), harness() as case:
                case.preview.start()
                imu = case.camera._imu = FakeIMU(case.events)
                imu.stop = mock.Mock(return_value=proof)
                with self.assertRaisesRegex(USBPreviewError, "quiescence"):
                    case.preview.stop()
                self.assertIs(case.camera._imu, imu)
                self.assertIs(case.camera._usb_preview, case.preview)
                self.assertFalse(case.preview._closed)
                self.assertTrue(case.state.lease.healthy)
                self.assertTrue(case.state.host_alias)
                for forbidden in ("lease.close", "session:rollback_once", "network.cleanup"):
                    self.assertNotIn(forbidden, case.events)
                proof["quiescent"] = True
                self.assertTrue(case.preview.stop()["restored"])
                self.assertEqual(case.events.count("lease.close"), 1)

    def test_restored_fault_does_not_bypass_missing_or_failed_drain_proof(self):
        for missing, quiet in ((True, True), (False, False), (False, 1)):
            with self.subTest(missing=missing, quiet=quiet), harness() as case:
                case.preview.start()
                imu = case.camera._imu = FaultedIMU(case.events, quiescent=quiet)
                if missing:
                    del imu.report["quiescent"]
                with self.assertRaises(IMUStopError) as caught:
                    case.preview.stop()
                self.assertIs(caught.exception, imu.error)
                self.assertIs(case.camera._imu, imu)
                self.assertIs(case.camera._usb_preview, case.preview)
                self.assertFalse(case.preview._closed)
                self.assertTrue(case.state.lease.healthy)
                self.assertTrue(case.state.host_alias)
                for forbidden in ("lease.close", "session:rollback_once", "network.cleanup"):
                    self.assertNotIn(forbidden, case.events)

    def test_new_apply_requires_quiet_old_collector_not_only_restored_mask(self):
        with harness() as case:
            imu = case.camera._imu = FakeIMU(case.events, "initial")
            imu.quiescent = False
            with self.assertRaisesRegex(USBPreviewError, "quiescence"):
                case.preview.start()
            self.assertIs(case.camera._imu, imu)
            for forbidden in ("bootstrap:apply_once", "session:rollback_once", "lease.close"):
                self.assertNotIn(forbidden, case.events)

    def test_outer_camera_close_and_context_finally_preserve_unsafe_preview_owner(self):
        for cleanup in ("close", "context"):
            with self.subTest(cleanup=cleanup), harness() as case:
                case.preview.start()
                camera = Camera(transport=case.camera._transport)
                camera._usb_preview = case.preview
                case.preview.camera = camera
                imu = camera._imu = FaultedIMU(case.events, quiescent=False)
                if cleanup == "close":
                    with self.assertRaises(CameraError):
                        camera.close()
                else:
                    original = RuntimeError("application failed")
                    with self.assertRaises(RuntimeError) as caught:
                        with camera:
                            raise original
                    self.assertIs(caught.exception, original)
                    self.assertIsInstance(original.camera_cleanup_error, CameraError)
                self.assertIs(camera._imu, imu)
                self.assertIs(camera._usb_preview, case.preview)
                self.assertFalse(case.preview._closed)
                self.assertTrue(case.state.lease.healthy)
                self.assertTrue(case.state.host_alias)
                for forbidden in ("lease.close", "session:rollback_once", "network.cleanup"):
                    self.assertNotIn(forbidden, case.events)

    def test_imu_fault_without_completed_mask_proof_keeps_cleanup_dependencies(self):
        for restored, returncode in ((False, 1), (None, 1), (True, None), (True, True)):
            with (
                self.subTest(restored=restored, returncode=returncode),
                harness() as case,
            ):
                case.preview.start()
                imu = case.camera._imu = FaultedIMU(
                    case.events, restored=restored, returncode=returncode
                )
                with self.assertRaises(IMUStopError) as caught:
                    case.preview.stop()
                self.assertIs(caught.exception, imu.error)
                self.assertIs(case.camera._imu, imu)
                self.assertNotIn("lease.close", case.events)
                self.assertNotIn("network.cleanup", case.events)
                self.assertTrue(case.state.lease.healthy)
                self.assertTrue(case.state.host_alias)
                self.assertFalse(case.preview.report["restored"])

    def test_completed_imu_fault_does_not_authorize_new_preview_apply(self):
        with harness() as case:
            imu = case.camera._imu = FaultedIMU(case.events)
            with self.assertRaises(IMUStopError) as caught:
                case.preview.start()
            self.assertIs(caught.exception, imu.error)
            self.assertIs(case.camera._imu, imu)
            self.assertNotIn("bootstrap:apply_once", case.events)
            self.assertNotIn("lease.close", case.events)

    def test_usb_cleanup_failure_retains_original_completed_imu_fault(self):
        with harness() as case:
            case.preview.start()
            imu = case.camera._imu = FaultedIMU(case.events)
            cleanup_error = RuntimeError("USB control did not reconnect")
            with mock.patch.object(
                case.preview, "_reconnect", side_effect=cleanup_error
            ):
                with self.assertRaises(IMUStopError) as caught:
                    case.preview.stop()
            self.assertIs(caught.exception, imu.error)
            self.assertIs(caught.exception.__cause__, cleanup_error)
            self.assertIn(
                "USB control did not reconnect", " ".join(case.preview.report["errors"])
            )
            self.assertFalse(case.preview.report["restored"])
            self.assertIn("lease.close", case.events)
            self.assertIn("network.cleanup", case.events)
            self.assertIsNone(case.camera._imu)

    def test_restore_reconnect_failure_still_cleans_owned_alias_after_lease_close(self):
        with harness() as case:
            case.preview.start()
            failure = RuntimeError("native reconnect failed")
            with mock.patch.object(case.preview, "_reconnect", side_effect=failure):
                with self.assertRaises(RuntimeError) as caught:
                    case.preview.stop()
            self.assertIs(caught.exception, failure)
            self.assertLess(
                case.events.index("lease.close"), case.events.index("network.cleanup")
            )
            self.assertFalse(case.state.host_alias)
            self.assertTrue(case.preview.report["host_network_cleanup"]["restored"])
            self.assertFalse(case.preview.report["restored"])

    def test_cleanup_error_preserves_primary_restore_error(self):
        with harness(failure="cleanup_network") as case:
            case.preview.start()
            failure = RuntimeError("original reconnect failed")
            with mock.patch.object(case.preview, "_reconnect", side_effect=failure):
                with self.assertRaises(RuntimeError) as caught:
                    case.preview.stop()
            self.assertIs(caught.exception, failure)
            self.assertIn(
                "owned alias cleanup failed",
                case.preview.report["host_network_cleanup_error"],
            )
            self.assertFalse(case.preview.report["restored"])

    def test_owned_cleanup_failure_prevents_successful_restoration_claim(self):
        with harness(failure="cleanup_network") as case:
            case.preview.start()
            with self.assertRaisesRegex(RuntimeError, "owned alias cleanup failed"):
                case.preview.stop()
            self.assertFalse(case.preview.report["restored"])
            self.assertNotIn("network.restored", case.events)

    def test_lost_close_ack_uses_only_owned_rollback(self):
        with harness(failure="lease_close") as case:
            case.preview.start()
            report = case.preview.stop()
            self.assertEqual(case.events.count("bootstrap:apply_once"), 1)
            self.assertEqual(case.events.count("session:rollback_once"), 1)
            self.assertTrue(report["restored"])
            self.assertIn("lease_close_error", report)

    def test_changed_runtime_or_host_network_cannot_claim_restored(self):
        for kwargs in (
            {"restored_runtime": replace(runtime(), host_addr="02:00:00:00:00:01")},
            {"failure": "restore_network"},
        ):
            with self.subTest(kwargs=kwargs), harness(**kwargs) as case:
                case.preview.start()
                with self.assertRaises((USBPreviewError, RuntimeError)):
                    case.preview.stop()
                self.assertFalse(case.preview.report["restored"])
                self.assertFalse(case.state.host_alias)

    def test_read_requires_start_and_healthy_lease(self):
        with harness() as case:
            with self.assertRaises(USBPreviewError):
                case.preview.read(0)
            case.preview.start()
            case.state.lease.healthy = False
            with self.assertRaisesRegex(USBPreviewError, "heartbeat"):
                case.preview.read(0)
            case.preview.stop()
