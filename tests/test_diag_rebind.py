"""Restore-only DIAG recovery with fake inventories/handles and mask replies."""

import ctypes as C
import json
import struct
import unittest
from unittest.mock import Mock

from osmo360_camera._diag_collector import _DiagRebindGuard, _restore_only_after_reset


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, value):
        self.now += value


class Guard:
    def __init__(self, clock):
        self.clock = clock
        self.absent = True
        self.match = True
        self.mask = (7, b"\x04")
        self.events = []
        self.set_error = False
        self.late_read = False
        self.read_override = None

    def old_absent(self):
        self.events.append("inventory")
        if isinstance(self.absent, Exception):
            raise self.absent
        return self.absent, {"method": "complete_fake_inventory"}

    def close_old(self):
        self.events.append("close_old")

    def open_matching(self):
        self.events.append("open_same_identity")
        if isinstance(self.match, Exception):
            raise self.match
        return self.match

    def read_mask(self, deadline):
        self.events.append(("GET", deadline))
        if self.late_read:
            self.clock.now = deadline + 0.1
        return self.read_override if self.read_override is not None else self.mask

    def write_mask(self, last, value, deadline):
        self.events.append(("SET", last, value, deadline))
        self.mask = (last, value)
        if self.set_error:
            self.clock.now = deadline  # Simulate waiting out the lost receipt budget.
            raise RuntimeError("SET receipt timed out after application")

    def close_rebound(self):
        self.events.append("close_rebound")


class RestoreOnlyTests(unittest.TestCase):
    def run_restore(self, guard):
        return _restore_only_after_reset(
            guard, 7, b"\0", b"\x04", clock=guard.clock, sleep=guard.clock.sleep
        )

    def test_observed_old_absence_and_new_owned_mask_restore_once(self):
        guard = Guard(Clock())
        report = self.run_restore(guard)
        self.assertTrue(report["restored"])
        self.assertFalse(report["sample_stream_resumed"])
        self.assertEqual(report["set_attempts"], 1)
        self.assertEqual(
            [x[0] for x in guard.events if isinstance(x, tuple)], ["GET", "SET", "GET"]
        )
        self.assertLess(
            guard.events.index("close_old"), guard.events.index("open_same_identity")
        )
        self.assertEqual(guard.events[-1], "close_rebound")

    def test_already_original_mask_is_readback_proof_without_a_set(self):
        guard = Guard(Clock())
        guard.mask = (7, b"\0")
        report = self.run_restore(guard)
        self.assertTrue(report["restored"])
        self.assertEqual(report["method"], "original_mask_already_observed")
        self.assertEqual(report["set_attempts"], 0)
        self.assertFalse(
            any(isinstance(x, tuple) and x[0] == "SET" for x in guard.events)
        )

    def test_still_present_or_failed_inventory_never_closes_or_opens_another_handle(
        self,
    ):
        for absent in (False, RuntimeError("Inventory incomplete")):
            guard = Guard(Clock())
            guard.absent = absent
            report = self.run_restore(guard)
            self.assertFalse(report["restored"])
            self.assertNotIn("disconnect_evidence", report)
            self.assertNotIn("close_old", guard.events)
            self.assertNotIn("open_same_identity", guard.events)
            self.assertEqual(report["set_attempts"], 0)

    def test_replacement_ambiguity_or_claim_failure_does_not_adopt_a_new_baseline(self):
        for reason in ("Identity differs", "Ambiguous port", "Interface busy"):
            guard = Guard(Clock())
            guard.match = RuntimeError(reason)
            report = self.run_restore(guard)
            self.assertFalse(report["restored"])
            self.assertIn(reason, report["error"])
            self.assertFalse(any(isinstance(x, tuple) for x in guard.events))

    def test_foreign_mask_or_last_item_refuses_every_set(self):
        for mask in ((8, b"\0\0"), (7, b"\x08"), (7, b"\x05")):
            guard = Guard(Clock())
            guard.mask = mask
            report = self.run_restore(guard)
            self.assertFalse(report["restored"])
            self.assertEqual(report["set_attempts"], 0)
            self.assertFalse(
                any(isinstance(x, tuple) and x[0] == "SET" for x in guard.events)
            )

    def test_lost_set_receipt_reserves_time_for_get_and_never_replays_set(self):
        guard = Guard(Clock())
        guard.set_error = True
        report = self.run_restore(guard)
        self.assertTrue(report["restored"])
        self.assertIn("set_receipt_error", report)
        self.assertEqual(report["set_attempts"], 1)
        operations = [x for x in guard.events if isinstance(x, tuple)]
        self.assertLess(operations[1][-1], operations[2][-1])
        self.assertEqual([x[0] for x in operations], ["GET", "SET", "GET"])

    def test_late_original_readback_is_not_success(self):
        guard = Guard(Clock())
        guard.mask = (7, b"\0")
        guard.late_read = True
        report = self.run_restore(guard)
        self.assertFalse(report["restored"])
        self.assertEqual(report["set_attempts"], 0)


class FakeUSB:
    def __init__(self):
        self.devices = {
            11: dict(
                bus=1,
                address=7,
                ports=(1, 2, 4),
                product=b"Osmo 360",
                serial=b"private-unit",
            ),
            22: dict(
                bus=1,
                address=8,
                ports=(1, 2, 4),
                product=b"Osmo 360",
                serial=b"private-unit",
            ),
        }
        self.listed = [11]
        self.arrays = []
        self.libusb_get_bus_number = Mock(
            side_effect=lambda device: self.devices[device]["bus"]
        )
        self.libusb_get_device_address = Mock(
            side_effect=lambda device: self.devices[device]["address"]
        )
        self.libusb_get_port_numbers = Mock(side_effect=self.ports)
        self.libusb_get_device_descriptor = Mock(side_effect=self.descriptor)
        self.libusb_get_string_descriptor_ascii = Mock(side_effect=self.string)
        self.libusb_get_device_list = Mock(side_effect=self.inventory)
        self.libusb_free_device_list = Mock()
        self.libusb_open = Mock(side_effect=self.open)
        self.libusb_claim_interface = Mock(return_value=0)
        self.libusb_release_interface = Mock(return_value=0)
        self.libusb_close = Mock()
        self.libusb_error_name = Mock(return_value=b"FAKE_ERROR")

    def ports(self, device, output, size):
        values = self.devices[device]["ports"]
        for index, value in enumerate(values):
            output[index] = value
        return len(values)

    def descriptor(self, device, output):
        value = bytearray(18)
        value[0:2] = b"\x12\x01"
        struct.pack_into("<HHH", value, 8, 0x2CA3, 0x0025, 0x0100)
        value[15] = 2
        value[16] = 3 if self.devices[device]["serial"] is not None else 0
        C.memmove(output, bytes(value), 18)
        return 0

    def string(self, handle, index, output, capacity):
        value = self.devices[handle.value - 1000]["product" if index == 2 else "serial"]
        C.memmove(output, value, len(value))
        return len(value)

    def inventory(self, context, output):
        values = (C.c_void_p * len(self.listed))(*self.listed)
        self.arrays.append(values)
        C.cast(output, C.POINTER(C.POINTER(C.c_void_p)))[0] = C.cast(
            values, C.POINTER(C.c_void_p)
        )
        return len(self.listed)

    def open(self, device, output):
        C.cast(output, C.POINTER(C.c_void_p))[0] = C.c_void_p(device + 1000)
        return 0


class RebindIdentityTests(unittest.TestCase):
    def guard(self, usb):
        return _DiagRebindGuard(usb, C.c_void_p(1), 11, C.c_void_p(1011), Mock())

    def test_complete_inventory_distinguishes_old_presence_from_renumbered_device(self):
        usb = FakeUSB()
        guard = self.guard(usb)
        self.assertFalse(guard.old_absent()[0])
        usb.listed = [22]
        self.assertTrue(guard.old_absent()[0])
        self.assertTrue(guard.open_matching())
        usb.libusb_claim_interface.assert_called_once()
        self.assertEqual(usb.libusb_claim_interface.call_args.args[1], 0)
        guard.close_rebound()
        usb.libusb_close.assert_called_once()

    def test_failed_descriptor_inventory_never_becomes_absence(self):
        usb = FakeUSB()
        guard = self.guard(usb)
        usb.libusb_get_device_descriptor.return_value = -1
        usb.libusb_get_device_descriptor.side_effect = None
        with self.assertRaises(RuntimeError):
            guard.old_absent()
        usb.libusb_open.assert_not_called()

    def test_different_port_product_or_serial_never_claims_diag(self):
        for field, value in (
            ("ports", (1, 2, 3)),
            ("product", b"Other"),
            ("serial", b"Another-unit"),
            ("serial", None),
        ):
            usb = FakeUSB()
            guard = self.guard(usb)
            usb.listed = [22]
            usb.devices[22][field] = value
            if field == "ports":
                self.assertFalse(guard.open_matching())
            else:
                with self.assertRaisesRegex(RuntimeError, "identity differs"):
                    guard.open_matching()
            usb.libusb_claim_interface.assert_not_called()

    def test_serial_absent_on_both_devices_keeps_port_and_product_pin(self):
        usb = FakeUSB()
        usb.devices[11]["serial"] = usb.devices[22]["serial"] = None
        guard = self.guard(usb)
        usb.listed = [22]
        guard.read_mask = Mock(return_value=(7, b"\0"))
        guard.write_mask = Mock()
        clock = Clock()
        report = _restore_only_after_reset(
            guard, 7, b"\0", b"\x04", clock=clock, sleep=clock.sleep
        )
        self.assertTrue(report["restored"])
        guard.write_mask.assert_not_called()
        # Real recovery diagnostics contain proof booleans, not raw private
        # strings or either identity digest held inside the actual USB guard.
        public = json.dumps(report)
        self.assertNotIn("Osmo 360", public)
        self.assertNotIn(guard.fingerprint[1].hex(), public)


if __name__ == "__main__":
    unittest.main()
