"""Real camera ACK fixtures and a fake USB backend; never opens hardware."""

import ctypes as C
from dataclasses import replace
import json
from pathlib import Path
import queue
import threading
import time
import unittest

from osmo360_camera.transport import CameraDisconnected, Reply, RequestTimeout
from osmo360_camera.controls import CameraCommand
from osmo360_camera.v3 import (
    MACHINE_ALIAS,
    PC_HOST,
    MixedFrameReader,
    V3Message,
    V3Reply,
    V3USBTransport,
    pack_v3,
    parse_v3,
)
from osmo360_camera.vendor.duml import Message, crc8, crc16

FIXTURE = json.loads(
    (Path(__file__).parent / "fixtures/duml_v3_real_acks.json").read_text()
)


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("Timed out waiting for fake USB reader")
        time.sleep(0.002)


class V3CodecTests(unittest.TestCase):
    def test_real_captured_acks_and_exact_reencoding(self):
        for record in FIXTURE["frames"]:
            with self.subTest(source=record["source"]):
                frame = bytes.fromhex(record["hex"])
                message = parse_v3(frame)
                self.assertEqual(message.sender, record["source"])
                self.assertEqual(message.receiver, record["target"])
                self.assertEqual(message.seq, record["sequence"])
                self.assertEqual(
                    message.command, (record["command_set"], record["command"])
                )
                self.assertEqual(message.source_machine, record["source_machine"])
                self.assertEqual(message.target_machine, record["target_machine"])
                self.assertEqual(message.payload.hex(), record["payload_hex"])
                self.assertEqual(message.ack_type, 2)  # Native ACKs retain ACKtype2.
                self.assertTrue(message.is_reply)
                self.assertEqual(message.ttl, record["ttl"])
                self.assertEqual(message.encode(), frame)

    def test_native_index_beyond_v1_and_codec_bounds(self):
        message = parse_v3(pack_v3(PC_HOST, 0x0143, 0, 1, seq=65535))
        self.assertEqual(
            (message.receiver, message.source_machine, message.target_machine),
            (0x0143, MACHINE_ALIAS, MACHINE_ALIAS),
        )
        self.assertEqual(
            parse_v3(
                pack_v3(PC_HOST, 0x0100, 2, CameraCommand.GET_VIDEO_FORMAT)
            ).cmd_id,
            int(CameraCommand.GET_VIDEO_FORMAT),
        )
        for kwargs in (
            {"receiver": 0x0180},
            {"sender": -1},
            {"seq": 65536},
            {"cmd_id": 256},
            {"source_machine": 0x800},
            {"target_component": 16},
            {"ttl": 32},
            {"priority": 4},
            {"payload": bytes(65515)},
            {"payload": bytearray()},
            {"seq": True},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                replace(message, **kwargs).encode()

    def test_corrupt_header_and_frame_crc_rejected_then_resync(self):
        good = bytes.fromhex(FIXTURE["frames"][0]["hex"])
        for index in (5, len(good) - 1):
            bad = bytearray(good)
            bad[index] ^= 0x40
            with self.subTest(index=index), self.assertRaises(ValueError):
                parse_v3(bytes(bad))
            self.assertEqual(
                MixedFrameReader().feed(bytes(bad) + good), [parse_v3(good)]
            )

    def test_fragmented_v1_v3_interleave_uses_outer_frame_boundaries(self):
        v1 = Message(1, 10, 7, 0xC0, 2, 0x70, b"\0state")
        captured = bytes.fromhex(FIXTURE["frames"][1]["hex"])
        outer = V3Message(0x0808, PC_HOST, 8, 0, 1, v1.encode(), is_reply=True)
        stream = b"noise" + v1.encode() + captured + outer.encode() + v1.encode()
        for chunk in (1, 2, 5, 17, 64, len(stream)):
            reader, messages = MixedFrameReader(), []
            for offset in range(0, len(stream), chunk):
                messages.extend(reader.feed(stream[offset : offset + chunk]))
            with self.subTest(chunk=chunk):
                self.assertEqual(messages, [v1, parse_v3(captured), outer, v1])

    def test_crc_valid_truncated_address_field_is_not_a_message(self):
        frame = bytearray(pack_v3(PC_HOST, 0x0808, 0, 1))
        del frame[17:19]  # Retain explicit-target-MID flag, omit its host bytes.
        frame[3:5] = len(frame).to_bytes(2, "little")
        frame[5] = crc8(frame[:5])
        frame[-2:] = crc16(frame[:-2]).to_bytes(2, "little")
        with self.assertRaises(ValueError):
            parse_v3(bytes(frame))
        self.assertEqual(MixedFrameReader().feed(bytes(frame)), [])


class FakeUSB:
    def __init__(self):
        self.incoming = queue.Queue()
        self.outgoing = queue.Queue()
        self.closed = []

    def libusb_bulk_transfer(self, handle, endpoint, buffer, size, count, timeout):
        if endpoint == 0x05:
            self.outgoing.put(bytes(buffer.raw[:size]))
            C.cast(count, C.POINTER(C.c_int))[0] = size
            return 0
        if endpoint != 0x85:
            raise AssertionError(f"Unexpected fake endpoint {endpoint}")
        try:
            status, data = self.incoming.get(timeout=0.01)
        except queue.Empty:
            status, data = -7, b""
        assert len(data) <= size
        C.memmove(buffer, data, len(data))
        C.cast(count, C.POINTER(C.c_int))[0] = len(data)
        return status

    def libusb_error_name(self, status):
        return b"fake USB disconnected"

    def libusb_release_interface(self, handle, interface):
        self.closed.append(("release", interface))

    def libusb_close(self, handle):
        self.closed.append(("close",))

    def libusb_exit(self, context):
        self.closed.append(("exit",))


class V3TransportTests(unittest.TestCase):
    def setUp(self):
        self.transport = V3USBTransport()
        self.usb = FakeUSB()
        self.transport._usb = self.usb
        self.transport._context = C.c_void_p(2)
        self.transport._handle = C.c_void_p(1)
        self.transport._thread = threading.Thread(
            target=self.transport._receive, daemon=True
        )
        self.transport._thread.start()
        self.workers = []

    def tearDown(self):
        self.transport.close()
        for worker in self.workers:
            worker.join(2)
            self.assertFalse(worker.is_alive())

    def request(self, *, v3=True, receiver=0x0143, timeout=2):
        result = {}

        def run():
            try:
                if v3:
                    result["reply"] = self.transport.request_v3(
                        0, 1, b"", receiver, timeout=timeout
                    )
                else:
                    result["reply"] = self.transport.request(
                        0, 1, receiver=1, timeout=timeout
                    )
            except Exception as error:
                result["error"] = error

        thread = threading.Thread(target=run)
        self.workers.append(thread)
        thread.start()
        packet = self.usb.outgoing.get(timeout=2)
        request = MixedFrameReader().feed(packet)[0]
        return thread, request, result

    def feed(self, message, *, status=0):
        self.usb.incoming.put((status, message.encode()))

    def test_actual_camera_ack_resolves_its_exact_request(self):
        record = FIXTURE["frames"][0]
        self.transport._sequence = record["sequence"] - 1
        thread, request, result = self.request(receiver=record["source"])
        self.assertEqual(request.seq, record["sequence"])
        self.usb.incoming.put((0, bytes.fromhex(record["hex"])))
        thread.join(2)
        reply = result["reply"]
        self.assertEqual(reply.payload.hex(), record["payload_hex"])
        self.assertEqual(
            (reply.source_machine, reply.target_machine), (MACHINE_ALIAS, MACHINE_ALIAS)
        )

    def test_v1_and_v3_requests_share_reader_without_cross_matching(self):
        v1thread, v1, v1result = self.request(v3=False)
        v3thread, v3, v3result = self.request()
        self.assertEqual(v3.sender, PC_HOST)
        self.assertEqual(
            (v3.source_machine, v3.target_machine), (MACHINE_ALIAS, MACHINE_ALIAS)
        )
        self.assertEqual(v1.sender, 0x0A)
        self.assertTrue(v1thread.is_alive())
        notification = V3Message(0x0808, PC_HOST, 123, 2, 0x70, b"notice")
        v1notice = Message(1, 10, 124, 0x80, 2, 0x70, b"V1 notice")
        reply = V3Message(v3.receiver, v3.sender, v3.seq, 0, 1, b"\0v3", is_reply=True)
        wire = (
            notification.encode()
            + v1notice.encode()
            + v1.answer(b"\0v1").encode()
            + reply.encode()
        )
        for offset in range(0, len(wire), 7):
            self.usb.incoming.put(
                (-7, wire[offset : offset + 7])
            )  # Partial data on timeout.
        v1thread.join(2)
        v3thread.join(2)
        self.assertIsInstance(v1result.get("reply"), Reply)
        self.assertNotIsInstance(v1result["reply"], V3Reply)
        self.assertIsInstance(v3result.get("reply"), V3Reply)
        self.assertEqual(v1result["reply"].payload, b"\0v1")
        self.assertEqual(v3result["reply"].payload, b"\0v3")
        self.assertEqual(v3result["reply"].source, 0x0143)
        self.assertEqual([message for _, message in self.transport.events], [v1notice])
        self.assertEqual(self.transport.v3_events[0][1], notification)
        self.assertEqual(self.transport._pending, {})

    def test_wrong_native_address_machine_and_component_do_not_resolve(self):
        thread, request, result = self.request()
        reply = V3Message(
            request.receiver,
            request.sender,
            request.seq,
            0,
            1,
            b"\0good",
            is_reply=True,
        )
        variants = (
            {"sender": 0x0808},
            {"receiver": 0x0A01},
            {"source_machine": 0},
            {"target_machine": 0},
            {"source_component": 1},
            {"target_component": 1},
            {"seq": (request.seq + 1) & 65535},
            {"cmd_id": 2},
            {"is_reply": False},
        )
        for kwargs in variants:
            self.feed(replace(reply, **kwargs))
        wait_for(lambda: len(self.transport.v3_events) == len(variants))
        self.assertTrue(thread.is_alive())
        self.assertFalse(result)
        self.feed(reply)
        thread.join(2)
        self.assertEqual(result["reply"].payload, b"\0good")

    def test_inherited_close_fails_both_protocol_futures(self):
        first, _, v1 = self.request(v3=False)
        second, _, v3 = self.request()
        self.transport.close()
        first.join(2)
        second.join(2)
        self.assertIsInstance(v1.get("error"), CameraDisconnected)
        self.assertIsInstance(v3.get("error"), CameraDisconnected)
        self.assertEqual(self.transport._pending, {})
        self.assertEqual(self.usb.closed, [("release", 4), ("close",), ("exit",)])

    def test_reader_failure_uses_inherited_fail_pending(self):
        first, _, v1 = self.request(v3=False)
        second, _, v3 = self.request()
        self.usb.incoming.put((-4, b""))
        first.join(2)
        second.join(2)
        self.assertIsInstance(v1.get("error"), CameraDisconnected)
        self.assertIsInstance(v3.get("error"), CameraDisconnected)
        self.assertIsInstance(self.transport.failure, CameraDisconnected)
        self.assertFalse(self.transport.connected)
        self.assertTrue(self.transport.wait_disconnected(0))

    def test_timeout_removes_v3_pending_request(self):
        thread, _, result = self.request(timeout=0.02)
        thread.join(2)
        self.assertIsInstance(result.get("error"), RequestTimeout)
        self.assertEqual(self.transport._pending, {})


if __name__ == "__main__":
    unittest.main()
