"""ACK modes and observed USB disconnects; no camera or actual USB backend."""

import ctypes as C
from enum import IntEnum
import struct
import threading
import unittest
from unittest.mock import Mock, patch

from osmo360_camera.transport import CameraDisconnected, USBError, USBTransport
from osmo360_camera.vendor.duml import FrameReader, Message


class LoopbackTransport(USBTransport):
    """Exercises request/Future matching without opening or writing USB."""

    def __init__(self):
        super().__init__()
        self.sent = []

    @property
    def connected(self):
        return True

    def _write(self, message):
        wire = message.encode()
        self.sent.append(wire)
        request = FrameReader().feed(wire)[0]
        self._accept(request.answer(b"\0ack"))


class ReaderUSB:
    def __init__(self, status, data=b""):
        self.status = status
        self.data = data

    def libusb_bulk_transfer(self, handle, endpoint, buffer, size, count, timeout):
        if endpoint != USBTransport.IN:
            raise AssertionError("Only an offline USB read is allowed")
        C.memmove(buffer, self.data, len(self.data))
        C.cast(count, C.POINTER(C.c_int))[0] = len(self.data)
        return self.status

    def libusb_error_name(self, status):
        return f"fake USB error {status}".encode()


class InventoryUSB:
    """A read-only inventory with stable fake pointer values and descriptors."""

    def __init__(self, entries=(), *, list_status=0):
        self.entries = entries
        self.list_status = list_status
        self.list_calls = self.free_calls = self.descriptor_calls = 0
        self.arrays = []  # Keep the ctypes arrays alive through free_device_list.
        self.closed = []

    def libusb_get_device_list(self, context, result):
        self.list_calls += 1
        if self.list_status < 0:
            return self.list_status
        array = (C.c_void_p * (len(self.entries) + 1))(
            *(entry["pointer"] for entry in self.entries), None
        )
        self.arrays.append(array)
        C.cast(result, C.POINTER(C.POINTER(C.c_void_p)))[0] = C.cast(
            array, C.POINTER(C.c_void_p)
        )
        return len(self.entries)

    def libusb_free_device_list(self, devices, unref):
        self.free_calls += 1

    def entry(self, pointer):
        return next(entry for entry in self.entries if entry["pointer"] == pointer)

    def libusb_get_device_descriptor(self, pointer, target):
        self.descriptor_calls += 1
        entry = self.entry(pointer)
        status = entry.get("descriptor_status", 0)
        if status == 0:
            raw = struct.pack(
                "<BBHBBBBHHHBBBB",
                18,
                1,
                0x300,
                0,
                0,
                0,
                64,
                entry.get("vid", 0x2CA3),
                entry.get("pid", 0x0025),
                0,
                0,
                0,
                0,
                1,
            )
            C.memmove(target, raw, len(raw))
        return status

    def libusb_get_bus_number(self, pointer):
        return self.entry(pointer)["bus"]

    def libusb_get_device_address(self, pointer):
        return self.entry(pointer)["address"]

    def libusb_error_name(self, status):
        return f"fake inventory error {status}".encode()

    def libusb_release_interface(self, handle, interface):
        self.closed.append("release")

    def libusb_close(self, handle):
        self.closed.append("close")

    def libusb_exit(self, context):
        self.closed.append("exit")


class TransportRequestTests(unittest.TestCase):
    def test_default_and_explicit_ack_types_encode_real_v1_flags(self):
        transport = LoopbackTransport()
        for kwargs, expected_flags in (
            ({}, 0x40),
            ({"ack_type": 1}, 0x20),
            ({"ack_type": 2}, 0x40),
        ):
            with self.subTest(kwargs=kwargs):
                reply = transport.request(0, 1, b"probe", receiver=0x28, **kwargs)
                message = FrameReader().feed(transport.sent[-1])[0]
                self.assertEqual(
                    (message.flags, message.payload), (expected_flags, b"probe")
                )
                self.assertEqual(
                    (reply.payload, reply.source, reply.target), (b"\0ack", 0x28, 0x0A)
                )
                self.assertEqual(transport._pending, {})

    def test_ack_validation_rejects_bool_noninteger_and_other_modes_before_write(self):
        transport = LoopbackTransport()
        for value in (True, False, 0, 3, -1, 1.0, "1", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                transport.request(0, 1, ack_type=value)
        self.assertEqual(transport.sent, [])
        self.assertEqual(transport._pending, {})

    def test_integer_enum_ack_is_accepted(self):
        class Ack(IntEnum):
            IMMEDIATE = 1

        transport = LoopbackTransport()
        transport.request(0, 1, ack_type=Ack.IMMEDIATE)
        self.assertEqual(FrameReader().feed(transport.sent[-1])[0].flags, 0x20)

    def test_closed_request_does_not_load_or_open_usb(self):
        transport = USBTransport()
        with patch("osmo360_camera.transport.osmo_usb.load_usb") as load_usb:
            with self.assertRaises(CameraDisconnected):
                transport.request(0, 1, ack_type=1)
            load_usb.assert_not_called()
        self.assertEqual(transport._pending, {})

    def test_charging_only_writes_exact_gui_frame_and_delivery_report(self):
        transport = USBTransport(sender=0x0A)
        transport._handle = C.c_void_p(1)  # Connected fake; _write is replaced.
        transport._sequence = 0x1234
        with patch.object(transport, "_write") as write:
            report = transport.select_charging_only()
        write.assert_called_once()
        message = write.call_args.args[0]
        expected = Message(0x1C, 0x28, 0x1235, 0x40, 6, 0x3A, b"\0\3")
        self.assertEqual(message.encode(), expected.encode())
        decoded = FrameReader().feed(message.encode())[0]
        self.assertEqual(
            (
                decoded.sender,
                decoded.receiver,
                decoded.seq,
                decoded.flags,
                decoded.cmd_set,
                decoded.cmd_id,
                decoded.payload,
            ),
            (0x1C, 0x28, 0x1235, 0x40, 6, 0x3A, b"\0\3"),
        )
        self.assertEqual(
            report,
            dict(
                delivered=True,
                ack_claimed=False,
                source=0x1C,
                target=0x28,
                sequence=0x1235,
                flags=0x40,
                command_set=6,
                command=0x3A,
                payload_hex="0003",
            ),
        )
        self.assertEqual(transport.sender, 0x0A)
        self.assertEqual(transport._pending, {})

    def test_charging_only_does_not_create_future_request_or_pending_ack(self):
        transport = USBTransport()
        transport._handle = C.c_void_p(1)
        previous = {(42, 0, 1, 1, 0x0A): object()}
        transport._pending.update(previous)

        def write_without_ack(message):
            # No ACK is injected: successful delivery must finish on its own.
            self.assertEqual(transport._pending, previous)

        with (
            patch(
                "osmo360_camera.transport.Future",
                side_effect=AssertionError("No ACK Future"),
            ) as future,
            patch.object(
                transport, "request", side_effect=AssertionError("No request/ACK wait")
            ) as request,
            patch.object(transport, "_write", side_effect=write_without_ack) as write,
        ):
            report = transport.select_charging_only()
        self.assertTrue(report["delivered"])
        self.assertFalse(report["ack_claimed"])
        future.assert_not_called()
        request.assert_not_called()
        write.assert_called_once()
        self.assertEqual(transport._pending, previous)

    def test_charging_only_fresh_sequences_wrap_and_skip_pending_gui_reply_keys(self):
        transport = USBTransport()
        transport._handle = C.c_void_p(1)
        transport._sequence = 65535
        previous = {
            (0, 6, 0x3A, 0x28, 0x1C): object(),
            (1, 6, 0x3A, 0x28, 0x1C): object(),
            (2, 0, 1, 1, 0x0A): object(),
        }
        transport._pending.update(previous)
        with patch.object(transport, "_write") as write:
            first = transport.select_charging_only()
            second = transport.select_charging_only()
        self.assertEqual((first["sequence"], second["sequence"]), (2, 3))
        self.assertEqual([call.args[0].seq for call in write.call_args_list], [2, 3])
        self.assertEqual(transport._pending, previous)

    def test_charging_only_propagates_single_write_error_without_retry(self):
        transport = USBTransport()
        transport._handle = C.c_void_p(1)
        failure = USBError("USB control write: LIBUSB_ERROR_TIMEOUT, 0/19 bytes")
        with patch.object(transport, "_write", side_effect=failure) as write:
            with self.assertRaises(USBError) as caught:
                transport.select_charging_only()
        self.assertIs(caught.exception, failure)
        write.assert_called_once()
        self.assertEqual(transport._pending, {})
        self.assertEqual(transport.sender, 0x0A)

    def test_closed_charging_only_never_loads_opens_or_writes_usb(self):
        for stopped_handle in (False, True):
            with self.subTest(stopped_handle=stopped_handle):
                transport = USBTransport()
                if stopped_handle:
                    transport._handle = C.c_void_p(1)
                    transport._stop.set()
                with (
                    patch("osmo360_camera.transport.osmo_usb.load_usb") as load,
                    patch.object(transport, "open") as open_usb,
                    patch.object(transport, "_write") as write,
                ):
                    with self.assertRaises(CameraDisconnected):
                        transport.select_charging_only()
                load.assert_not_called()
                open_usb.assert_not_called()
                write.assert_not_called()
                self.assertEqual(transport._pending, {})

    def test_native_open_failure_is_usb_error_and_releases_initialized_resources(self):
        transport, usb = USBTransport(), Mock()
        usb.libusb_init.return_value = 0
        usb.libusb_get_device_list.return_value = -1
        usb.libusb_error_name.return_value = b"LIBUSB_ERROR_IO"
        with patch("osmo360_camera.transport.osmo_usb.load_usb", return_value=usb):
            with self.assertRaises(USBError) as caught:
                transport.open()
        self.assertIn("USB list: LIBUSB_ERROR_IO", str(caught.exception))
        self.assertIsInstance(caught.exception.__cause__, RuntimeError)
        usb.libusb_exit.assert_called_once()
        usb.libusb_close.assert_not_called()
        self.assertFalse(transport.connected)

    def test_only_no_device_read_status_marks_disconnected(self):
        for status, expected in ((-4, True), (-1, False), (-9, False)):
            with self.subTest(status=status):
                transport = USBTransport()
                transport._usb = ReaderUSB(status)
                transport._handle = C.c_void_p(1)
                transport._receive()
                self.assertIsInstance(transport.failure, CameraDisconnected)
                self.assertEqual(transport.wait_disconnected(0), expected)
                self.assertFalse(transport.connected)
                self.assertEqual(transport._read_failure_status, status)
                if expected:
                    self.assertEqual(
                        transport.disconnect_evidence["method"], "libusb_no_device"
                    )

    def test_explicit_close_does_not_claim_camera_disappeared(self):
        transport = USBTransport()
        self.assertFalse(transport.wait_disconnected(0))
        transport.close()
        self.assertFalse(transport.wait_disconnected(0))

    def test_disconnect_observation_survives_trailing_data_handler_error(self):
        transport = USBTransport()
        trailing = Message(1, 10, 1, 0x80, 0, 1, b"\0").encode()
        transport._usb = ReaderUSB(-4, trailing)
        transport._handle = C.c_void_p(1)
        with patch.object(
            transport, "_accept", side_effect=RuntimeError("handler failed")
        ):
            transport._receive()
        self.assertTrue(transport.wait_disconnected(0))
        transport._usb = Mock()
        transport.close()
        self.assertTrue(transport.wait_disconnected(0))

    def test_successful_open_clears_previous_disconnect_before_reader_start(self):
        transport, usb, thread = USBTransport(), Mock(), Mock()
        transport._disconnected.set()
        transport._read_failure_status = -1
        transport._device_identity = {"pointer": 999, "bus": 4, "address": 9}
        transport._disconnect_evidence = {"method": "previous"}
        transport._last_inventory_error = "previous inventory error"
        device_array = (C.c_void_p * 1)(123)

        def device_list(context, result):
            C.cast(result, C.POINTER(C.POINTER(C.c_void_p)))[0] = C.cast(
                device_array, C.POINTER(C.c_void_p)
            )
            return 1

        def set_pointer(value):
            def assign(*args):
                C.cast(args[-1], C.POINTER(C.c_void_p))[0] = C.c_void_p(value)
                return 0

            return assign

        def descriptor(device, target):
            raw = struct.pack(
                "<BBHBBBBHHHBBBB",
                18,
                1,
                0x300,
                0,
                0,
                0,
                64,
                transport.VID,
                transport.PID,
                0,
                0,
                0,
                0,
                1,
            )
            C.memmove(target, raw, len(raw))
            return 0

        usb.libusb_init.side_effect = set_pointer(2)
        usb.libusb_get_device_list.side_effect = device_list
        usb.libusb_get_device_descriptor.side_effect = descriptor
        usb.libusb_open.side_effect = set_pointer(3)
        usb.libusb_claim_interface.return_value = 0
        usb.libusb_get_device_speed.return_value = 3
        usb.libusb_get_bus_number.return_value = 2
        usb.libusb_get_device_address.return_value = 41
        thread.is_alive.return_value = False
        thread.start.side_effect = lambda: self.assertFalse(
            transport.wait_disconnected(0)
        )
        with (
            patch("osmo360_camera.transport.osmo_usb.load_usb", return_value=usb),
            patch("osmo360_camera.transport.threading.Thread", return_value=thread),
        ):
            self.assertIs(transport.open(), transport)
            self.assertFalse(transport.wait_disconnected(0))
            self.assertIsNone(transport._read_failure_status)
            self.assertIsNone(transport.disconnect_evidence)
            self.assertIsNone(transport.last_inventory_error)
            self.assertEqual(
                transport._device_identity,
                dict(pointer=123, bus=2, address=41, vid=0x2CA3, pid=0x0025),
            )
            thread.start.assert_called_once_with()
            transport.close()
        self.assertFalse(transport.wait_disconnected(0))


class TransportInventoryTests(unittest.TestCase):
    def transport(self, usb, *, status=-1, identity=True):
        result = USBTransport()
        result._usb = usb
        result._handle, result._context = C.c_void_p(1), C.c_void_p(2)
        if identity:
            result._device_identity = dict(
                pointer=123, bus=2, address=41, vid=0x2CA3, pid=0x0025
            )
        result._note_read_status(status)
        return result

    def test_io_alone_does_not_prove_absence_but_complete_inventory_does(self):
        usb = InventoryUSB([dict(pointer=124, bus=2, address=42)])
        transport = self.transport(usb)
        self.assertFalse(transport.wait_disconnected(0))
        self.assertIsNone(transport.disconnect_evidence)
        self.assertTrue(transport.wait_disconnected(0.1))
        evidence = transport.disconnect_evidence
        self.assertEqual(evidence["method"], "device_inventory_absent")
        self.assertEqual(evidence["read_status"], -1)
        self.assertEqual(evidence["old_device"]["pointer"], 123)
        self.assertEqual(evidence["inventory_count"], 1)
        evidence["old_device"]["pointer"] = 999
        self.assertEqual(transport.disconnect_evidence["old_device"]["pointer"], 123)
        self.assertEqual(
            (usb.list_calls, usb.free_calls, usb.descriptor_calls), (1, 1, 1)
        )

    def test_successful_empty_inventory_is_evidence(self):
        usb = InventoryUSB()
        transport = self.transport(usb)
        self.assertTrue(transport.wait_disconnected(0.1))
        self.assertEqual(transport.disconnect_evidence["inventory_count"], 0)
        self.assertEqual(usb.free_calls, 1)

    def test_not_found_and_other_fatal_read_errors_require_independent_absence(self):
        # Actual second public reboot returned -5/NOT_FOUND, not -1/IO.
        for status in (-5, -9, -12, -99):
            with self.subTest(status=status):
                usb = InventoryUSB([dict(pointer=124, bus=2, address=42)])
                transport = self.transport(usb, status=status)
                self.assertFalse(transport.wait_disconnected(0))
                self.assertIsNone(transport.disconnect_evidence)
                self.assertTrue(transport.wait_disconnected(0.1))
                self.assertEqual(transport.disconnect_evidence["read_status"], status)
                self.assertEqual(transport.read_failure_status, status)
                self.assertEqual(
                    transport.disconnect_evidence["method"], "device_inventory_absent"
                )

    def test_not_found_does_not_override_present_device_or_inventory_errors(self):
        for usb in (
            InventoryUSB([dict(pointer=123, bus=2, address=41)]),
            InventoryUSB(list_status=-1),
            InventoryUSB([dict(pointer=124, bus=2, address=42, descriptor_status=-1)]),
        ):
            with self.subTest(entries=usb.entries, list_status=usb.list_status):
                transport = self.transport(usb, status=-5)
                self.assertFalse(transport.wait_disconnected(0.001))
                self.assertIsNone(transport.disconnect_evidence)
                self.assertEqual(transport.read_failure_status, -5)
                if usb.list_status < 0 or any(
                    e.get("descriptor_status", 0) < 0 for e in usb.entries
                ):
                    self.assertIn(
                        "fake inventory error", transport.last_inventory_error
                    )

    def test_old_pointer_or_old_location_still_present_even_if_descriptor_changed(self):
        for entry in (
            dict(pointer=123, bus=3, address=42),
            dict(pointer=999, bus=2, address=41, vid=1, pid=2),
        ):
            with self.subTest(entry=entry):
                usb = InventoryUSB([entry])
                transport = self.transport(usb)
                self.assertFalse(transport.wait_disconnected(0.001))
                self.assertIsNone(transport.disconnect_evidence)
                self.assertIsNone(transport.last_inventory_error)

    def test_enumeration_or_any_descriptor_error_is_inconclusive(self):
        old = dict(pointer=123, bus=2, address=41)
        broken = dict(pointer=124, bus=2, address=42, descriptor_status=-1)
        for usb in (
            InventoryUSB(list_status=-1),
            InventoryUSB([broken]),
            InventoryUSB([old, broken]),
        ):
            with self.subTest(entries=usb.entries):
                transport = self.transport(usb)
                self.assertFalse(transport.wait_disconnected(0.001))
                self.assertIsNone(transport.disconnect_evidence)
                self.assertIn("fake inventory error", transport.last_inventory_error)
                if usb.list_status >= 0:
                    self.assertEqual(usb.free_calls, usb.list_calls)
                    self.assertEqual(
                        usb.descriptor_calls, len(usb.entries) * usb.list_calls
                    )

    def test_missing_old_identity_cannot_turn_successful_inventory_into_evidence(self):
        usb = InventoryUSB()
        transport = self.transport(usb, identity=False)
        self.assertFalse(transport.wait_disconnected(0.001))
        self.assertIsNone(transport.disconnect_evidence)
        self.assertIn("identity", transport.last_inventory_error)
        self.assertEqual(usb.list_calls, 0)

    def test_timeout_does_not_poll_and_close_cannot_fabricate_absence(self):
        for status in (-7, -1, -5, -9):
            with self.subTest(status=status):
                usb = InventoryUSB()
                transport = self.transport(usb, status=status)
                if status == -7:
                    self.assertFalse(transport.wait_disconnected(0.001))
                transport.close()
                self.assertFalse(transport.wait_disconnected(0.001))
                self.assertIsNone(transport.disconnect_evidence)
                self.assertEqual(usb.list_calls, 0)

    def test_polling_uses_one_absolute_deadline(self):
        usb = InventoryUSB([dict(pointer=123, bus=2, address=41)])
        transport = self.transport(usb)
        clock = {"now": 10.0}
        waits = []

        def wait(seconds):
            waits.append(seconds)
            clock["now"] += seconds
            return False

        with (
            patch(
                "osmo360_camera.transport.time.monotonic",
                side_effect=lambda: clock["now"],
            ),
            patch.object(transport._disconnected, "wait", side_effect=wait),
        ):
            self.assertFalse(transport.wait_disconnected(0.12))
        self.assertAlmostEqual(clock["now"], 10.12)
        self.assertEqual(usb.list_calls, 3)
        self.assertTrue(all(0 < delay <= 0.05 for delay in waits))

    def test_inventory_completed_after_deadline_does_not_publish_absence(self):
        usb = InventoryUSB()
        transport = self.transport(usb)
        clock = {"now": 10.0}
        enumerate_devices = usb.libusb_get_device_list

        def slow_inventory(*args):
            result = enumerate_devices(*args)
            clock["now"] += 0.2
            return result

        usb.libusb_get_device_list = slow_inventory
        with patch(
            "osmo360_camera.transport.time.monotonic", side_effect=lambda: clock["now"]
        ):
            self.assertFalse(transport.wait_disconnected(0.1))
        self.assertFalse(transport._disconnected.is_set())
        self.assertIsNone(transport.disconnect_evidence)
        self.assertIn("deadline", transport.last_inventory_error)
        self.assertEqual(usb.free_calls, 1)

    def test_inventory_holds_close_lock_until_its_device_list_is_freed(self):
        usb = InventoryUSB([dict(pointer=123, bus=2, address=41)])
        transport = self.transport(usb)
        entered, release, closed = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )
        enumerate_devices = usb.libusb_get_device_list

        def blocked_inventory(*args):
            entered.set()
            if not release.wait(1):
                raise AssertionError("Offline inventory test was not released")
            return enumerate_devices(*args)

        usb.libusb_get_device_list = blocked_inventory
        poll = threading.Thread(target=transport._inventory_old_device_absent)
        close = threading.Thread(target=lambda: (transport.close(), closed.set()))
        poll.start()
        try:
            self.assertTrue(entered.wait(1))
            close.start()
            self.assertFalse(closed.wait(0.01))
            self.assertEqual(usb.closed, [])
        finally:
            release.set()
            poll.join(1)
            if close.ident is not None:
                close.join(1)
        self.assertFalse(poll.is_alive())
        self.assertFalse(close.is_alive())
        self.assertEqual(usb.free_calls, 1)
        self.assertEqual(usb.closed, ["release", "close", "exit"])
        self.assertFalse(transport.wait_disconnected(0))


if __name__ == "__main__":
    unittest.main()
