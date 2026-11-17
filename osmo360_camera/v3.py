"""Optional native-address DUML V3 on the existing USB control interface.

V1 and V3 share one USB reader and the base transport's pending-request
lifecycle. V3 requests always originate from PC0A00 with source/target machine
alias7FE, as verified by captured read-only GetVersion ACKs. This module does
not negotiate modes, open AMT, change settings, or replace Camera's default.
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, TimeoutError as FutureTimeout
import ctypes as C
from dataclasses import dataclass
import math
import struct
import time

from .transport import CameraDisconnected, Reply, RequestTimeout, USBError, USBTransport
from .vendor.duml import FrameReader, crc8, crc16

PC_HOST = 0x0A00
MACHINE_ALIAS = 0x7FE
V3_OVERHEAD = 21  # Explicit source/target machine IDs, no OOB, CRC included.


def _uint(value, maximum, name):
    if (
        not isinstance(value, int)
        or isinstance(value, bool)
        or not 0 <= value <= maximum
    ):
        raise ValueError(f"{name} must be an integer in 0..{maximum}")
    return value


def _host(value, name):
    _uint(value, 0xFFFF, name)
    if value & 0x80:
        raise ValueError(f"{name} must have a single 7-bit host index")
    return value


@dataclass(frozen=True)
class V3Message:
    sender: int
    receiver: int
    seq: int
    cmd_set: int
    cmd_id: int
    payload: bytes = b""
    source_machine: int = MACHINE_ALIAS
    target_machine: int = MACHINE_ALIAS
    source_component: int = 0
    target_component: int = 0
    ttl: int = 15
    ack_type: int = 2
    is_reply: bool = False
    priority: int = 0
    encrypt_type: int = 0
    oob_value: int | None = None

    @property
    def command(self):
        return self.cmd_set, self.cmd_id

    def encode(self):
        """Encode single-host, explicit-MID, unencrypted V3 without OOB data."""
        _host(self.sender, "sender")
        _host(self.receiver, "receiver")
        for name in ("cmd_set", "cmd_id"):
            _uint(getattr(self, name), 255, name)
        _uint(self.seq, 65535, "seq")
        for name in ("source_machine", "target_machine"):
            _uint(getattr(self, name), 0x7FF, name)
        for name in ("source_component", "target_component"):
            _uint(getattr(self, name), 15, name)
        _uint(self.ttl, 31, "ttl")
        _uint(self.ack_type, 3, "ack_type")
        _uint(self.priority, 3, "priority")
        if type(self.is_reply) is not bool:
            raise ValueError("is_reply must be bool")
        if self.encrypt_type != 0 or self.oob_value is not None:
            raise ValueError("Encoding encrypted/OOB V3 messages is unsupported")
        if (
            not isinstance(self.payload, bytes)
            or len(self.payload) > 65535 - V3_OVERHEAD
        ):
            raise ValueError("payload must be bytes, up to 65514 bytes")
        size = V3_OVERHEAD + len(self.payload)
        frame = bytearray(
            (
                0x55,
                int(self.is_reply) | self.priority << 3,
                3 << 2 | self.ack_type,
                size & 255,
                size >> 8,
                0,
                self.cmd_set,
                self.cmd_id,
                6 | self.ttl << 3,
                self.seq & 255,
                self.seq >> 8,
            )
        )
        frame += struct.pack(
            "<HBBHBB",
            self.source_machine | self.source_component << 11,
            self.sender >> 8,
            self.sender & 127,
            self.target_machine | self.target_component << 11,
            self.receiver >> 8,
            self.receiver & 127,
        )
        frame[5] = crc8(frame[:5])
        frame += self.payload
        frame += struct.pack("<H", crc16(frame))
        return bytes(frame)


def pack_v3(
    source,
    target,
    cmd_set,
    cmd_id,
    payload=b"",
    *,
    seq=1,
    source_machine=MACHINE_ALIAS,
    target_machine=MACHINE_ALIAS,
    ttl=15,
    ack_type=2,
    is_reply=False,
    priority=0,
):
    """Small codec helper; V3USBTransport itself fixes the PC address/MIDs."""
    return V3Message(
        source,
        target,
        seq,
        cmd_set,
        cmd_id,
        payload,
        source_machine,
        target_machine,
        ttl=ttl,
        ack_type=ack_type,
        is_reply=is_reply,
        priority=priority,
    ).encode()


def parse_v3(frame: bytes) -> V3Message:
    """Decode one complete CRC-checked unicast V3 frame, preserving MIDs.

    Multiple-machine/host address lists are unsupported. OOB and encryption
    flags are retained for events, but cannot satisfy a plain request Future.
    """
    if (
        not isinstance(frame, bytes)
        or len(frame) < 19
        or frame[0] != 0x55
        or (frame[2] >> 2 & 7) != 3
        or int.from_bytes(frame[3:5], "little") != len(frame)
    ):
        raise ValueError("Not a complete DUML V3 frame")
    if crc8(frame[:5]) != frame[5] or crc16(frame[:-2]) != int.from_bytes(
        frame[-2:], "little"
    ):
        raise ValueError("DUML V3 CRC mismatch")
    offset, end = 11, len(frame) - 2
    oob = None
    if frame[8] & 1:
        oob, shift = 0, 0
        while True:
            if offset >= end or shift >= 70:
                raise ValueError("Truncated/oversized V3 OOB field")
            byte = frame[offset]
            offset += 1
            oob |= (byte & 127) << shift
            if not byte & 128:
                break
            shift += 7
    if not frame[8] & 2:
        raise ValueError("V3 source machine ID is required")
    addresses, machines, components = [], [], []
    for has_machine in (True, bool(frame[8] & 4)):
        if has_machine:
            if offset + 2 > end:
                raise ValueError("Truncated V3 machine ID")
            value = int.from_bytes(frame[offset : offset + 2], "little")
            offset += 2
            if value & 0x8000:
                raise ValueError("V3 multiple machine IDs are unsupported")
            machine, component = value & 0x7FF, value >> 11 & 15
        else:
            machine, component = 0x7FD, 0
        if offset + 2 > end:
            raise ValueError("Truncated V3 host ID")
        host_type, index = frame[offset : offset + 2]
        offset += 2
        if index & 128:
            raise ValueError("V3 multiple host IDs are unsupported")
        addresses.append(host_type << 8 | index)
        machines.append(machine)
        components.append(component)
    return V3Message(
        *addresses,
        int.from_bytes(frame[9:11], "little"),
        frame[6],
        frame[7],
        frame[offset:end],
        *machines,
        *components,
        ttl=frame[8] >> 3,
        ack_type=frame[2] & 3,
        is_reply=bool(frame[1] & 1),
        priority=frame[1] >> 3 & 3,
        encrypt_type=frame[1] >> 5,
        oob_value=oob,
    )


class MixedFrameReader:
    """One framing buffer; a V3 payload is never scanned as independent V1."""

    def __init__(self):
        self._pending = bytearray()

    def feed(self, data):
        self._pending.extend(data)
        messages = []
        buf = self._pending
        while True:
            start = buf.find(0x55)
            if start < 0:
                buf.clear()
                break
            del buf[:start]
            if len(buf) < 4:
                break
            version = buf[2] >> 2 & 7
            if version == 1 and buf[2] >> 2 == 1:
                if crc8(buf[:3]) != buf[3]:
                    del buf[0]
                    continue
                size = int.from_bytes(buf[1:3], "little") & 0x3FF
                minimum = 13
            elif version == 3:
                if len(buf) < 6:
                    break
                if crc8(buf[:5]) != buf[5]:
                    del buf[0]
                    continue
                size, minimum = int.from_bytes(buf[3:5], "little"), 19
            else:
                del buf[0]
                continue
            if size < minimum:
                del buf[0]
                continue
            if len(buf) < size:
                break
            candidate = bytes(buf[:size])
            if crc16(candidate[:-2]) != int.from_bytes(candidate[-2:], "little"):
                del buf[0]
                continue
            del buf[:size]
            if version == 3:
                try:
                    messages.append(parse_v3(candidate))
                except ValueError:
                    continue
            else:
                messages.extend(FrameReader().feed(candidate))
        return messages


@dataclass(frozen=True)
class V3Reply(Reply):
    """Reply.source/target are native 16-bit hosts, not V1 packed addresses."""

    source_machine: int
    target_machine: int
    source_component: int
    target_component: int


class V3USBTransport(USBTransport):
    """Optional V1+V3 USB client with a fixed PC identity and inherited close.

    ``request(...)`` keeps the base V1 API. For full native addresses use
    ``request_v3(0, 1, b'', receiver_native16=0x0143)``. Callers choose commands;
    construction/open performs no V3 handshake or application request.
    """

    def __init__(self):
        super().__init__(sender=0x0A)
        self.v3_events = deque(maxlen=256)

    @staticmethod
    def _v3_key(message):
        return (
            "v3",
            message.seq,
            message.cmd_set,
            message.cmd_id,
            message.sender,
            message.receiver,
            message.source_machine,
            message.target_machine,
            message.source_component,
            message.target_component,
        )

    def request_v3(
        self, command_set, command, payload=b"", receiver_native16=0x0100, *, timeout=2
    ) -> V3Reply:
        if not self.connected:
            raise CameraDisconnected(str(self.failure or "Call open() first"))
        _host(receiver_native16, "receiver_native16")
        _uint(command_set, 255, "command_set")
        _uint(command, 255, "command")
        if not isinstance(payload, bytes) or len(payload) > 65535 - V3_OVERHEAD:
            raise ValueError("payload must be bytes, up to 65514 bytes")
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("timeout must be finite and positive")
        with self._state_lock:
            for _ in range(65536):
                self._sequence = (self._sequence + 1) & 65535
                expected = V3Message(
                    receiver_native16,
                    PC_HOST,
                    self._sequence,
                    command_set,
                    command,
                    is_reply=True,
                )
                key = self._v3_key(expected)
                if key not in self._pending:
                    break
            else:
                raise USBError("All V3 sequence IDs are busy")
            future = Future()
            self._pending[key] = future
            sequence = self._sequence
        try:
            self._write(
                V3Message(
                    PC_HOST, receiver_native16, sequence, command_set, command, payload
                )
            )
            return future.result(timeout)
        except FutureTimeout as exc:
            raise RequestTimeout(
                f"No V3 ACK {command_set:02x}/{command:02x} "
                f"from {receiver_native16:04x} in {timeout}s"
            ) from exc
        finally:
            with self._state_lock:
                self._pending.pop(key, None)

    def _accept_v3(self, message):
        received = time.monotonic_ns()
        if message.is_reply and message.encrypt_type == 0 and message.oob_value is None:
            with self._state_lock:
                future = self._pending.pop(self._v3_key(message), None)
            if future is not None:
                future.set_result(
                    V3Reply(
                        message.payload,
                        message.sender,
                        message.receiver,
                        message.cmd_set,
                        message.cmd_id,
                        message.seq,
                        received,
                        message.source_machine,
                        message.target_machine,
                        message.source_component,
                        message.target_component,
                    )
                )
                return
        self.v3_events.append((received, message))

    def _receive(self):
        reader, buffer = MixedFrameReader(), C.create_string_buffer(65536)
        try:
            while not self._stop.is_set():
                count = C.c_int()
                status = self._usb.libusb_bulk_transfer(
                    self._handle, self.IN, buffer, len(buffer), C.byref(count), 100
                )
                self._note_read_status(status)
                if count.value:
                    self.received_bytes += count.value
                    for message in reader.feed(buffer.raw[: count.value]):
                        if isinstance(message, V3Message):
                            self._accept_v3(message)
                        else:
                            self._accept(message)
                if status not in (0, -7):
                    raise CameraDisconnected(
                        self._usb.libusb_error_name(status).decode()
                    )
        except Exception as exc:
            self.failure = exc
            self._stop.set()
            self._fail_pending(exc)
