"""Один владелец USB control interface 4, без reset/configuration/Wi-Fi.

Ответ связывается с запросом по sequence, команде и двум адресам. Отдельный
reader позволяет камере спрашивать identity, пока пользователь ждёт свой ACK.
Этот класс не открывает AMT, не включает запись и не захватывает DIAG interface0.
"""

from collections import deque
from concurrent.futures import Future, TimeoutError as FutureTimeout
import ctypes as C
from dataclasses import dataclass
import math
import struct
import threading
import time

from . import _usb as osmo_usb
from .vendor.duml import FrameReader, Message


class USBError(RuntimeError):
    """Ошибка соединения или передачи USB."""


class CameraDisconnected(USBError):
    pass


class RequestTimeout(USBError):
    pass


@dataclass(frozen=True)
class Reply:
    payload: bytes
    source: int
    target: int
    command_set: int
    command: int
    sequence: int
    received_monotonic_ns: int


class USBTransport:
    VID, PID = 0x2CA3, 0x0025
    INTERFACE, OUT, IN = 4, 0x05, 0x85
    APP_IDENTITY = b"\0APP" + bytes(37) + b"\2" + bytes(8) + b"\2\10" + bytes(10)

    def __init__(self, *, sender=0x0A):
        if not 0 <= sender <= 255:
            raise ValueError("sender должен быть uint8")
        self.sender = sender
        self._usb = self._context = self._handle = None
        self._thread = None
        self._stop = threading.Event()
        self._disconnected = threading.Event()
        self._read_failure_status = None
        self._device_identity = None
        self._disconnect_evidence = None
        self._last_inventory_error = None
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._pending = {}
        self._sequence = time.monotonic_ns() & 65535
        self.events = deque(maxlen=256)
        self.link_speed = None
        self.received_bytes = 0
        self.failure = None

    @property
    def connected(self):
        return self._handle is not None and not self._stop.is_set()

    def wait_disconnected(self, timeout=None):
        """Wait for an observed disconnect of the device opened by this client.

        NO_DEVICE is direct evidence. After any fatal reader error, a complete
        successful inventory can also prove that both the old device pointer
        and its old bus/address disappeared. Read errors alone, inventory errors, and
        explicit close are not evidence. One deadline covers all 50ms polls.
        """
        if timeout is not None and (
            not isinstance(timeout, (int, float)) or not math.isfinite(timeout)
        ):
            raise ValueError("disconnect timeout must be finite, or None")
        deadline = None if timeout is None else time.monotonic() + max(0, timeout)
        while not self._disconnected.is_set():
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return self._disconnected.is_set()
            with self._state_lock:
                read_status = self._read_failure_status
            if (
                read_status is not None
                and read_status < 0
                and read_status not in (-4, -7)
                and self._inventory_old_device_absent(deadline=deadline)
            ):
                return True
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return self._disconnected.is_set()
            self._disconnected.wait(0.05 if remaining is None else min(0.05, remaining))
        return True

    @property
    def disconnect_evidence(self):
        """Copy of the last verified disconnect observation, or None."""
        with self._state_lock:
            if self._disconnect_evidence is None:
                return None
            result = dict(self._disconnect_evidence)
            if result.get("old_device") is not None:
                result["old_device"] = dict(result["old_device"])
            return result

    @property
    def read_failure_status(self):
        """Last fatal libusb IN-read status, or None since the last open."""
        with self._state_lock:
            return self._read_failure_status

    @property
    def last_inventory_error(self):
        with self._state_lock:
            return self._last_inventory_error

    def _note_read_status(self, status):
        """Record the actual libusb read status; shared with optional readers."""
        if status in (0, -7):
            return
        with self._state_lock:
            self._read_failure_status = status
            if status == -4:
                self._disconnect_evidence = {
                    "method": "libusb_no_device",
                    "read_status": status,
                    "observed_monotonic_ns": time.monotonic_ns(),
                    "old_device": dict(self._device_identity)
                    if self._device_identity
                    else None,
                }
                self._disconnected.set()

    def _device_location(self, device):
        bus = self._usb.libusb_get_bus_number(device)
        address = self._usb.libusb_get_device_address(device)
        if any(type(v) is not int or not 0 <= v <= 255 for v in (bus, address)):
            raise USBError("libusb did not return a valid bus/address")
        return bus, address

    def _inventory_old_device_absent(self, *, deadline=None):
        # The old handle owns a device reference. Protect it and the context
        # against close(), and free the inventory before releasing this lock.
        with self._write_lock:
            with self._state_lock:
                identity = (
                    dict(self._device_identity) if self._device_identity else None
                )
            if identity is None:
                with self._state_lock:
                    self._last_inventory_error = (
                        "Old USB device identity is unavailable"
                    )
                return False
            if self._handle is None or self._context is None:
                return False
            devices, count = C.POINTER(C.c_void_p)(), -1
            try:
                count = osmo_usb.check(
                    self._usb,
                    self._usb.libusb_get_device_list(self._context, C.byref(devices)),
                    "Disconnect inventory",
                )
                present = False
                for index in range(count):
                    device = devices[index]
                    raw = C.create_string_buffer(18)
                    osmo_usb.check(
                        self._usb,
                        self._usb.libusb_get_device_descriptor(device, raw),
                        "Disconnect inventory descriptor",
                    )
                    bus, address = self._device_location(device)
                    # Either match is conservatively still present, including
                    # changed descriptors or a changed pointer at this address.
                    present |= device == identity["pointer"] or (bus, address) == (
                        identity["bus"],
                        identity["address"],
                    )
                with self._state_lock:
                    self._last_inventory_error = None
                    if not present:
                        if deadline is not None and time.monotonic() > deadline:
                            self._last_inventory_error = (
                                "USB inventory finished after disconnect deadline"
                            )
                            return False
                        self._disconnect_evidence = {
                            "method": "device_inventory_absent",
                            "read_status": self._read_failure_status,
                            "observed_monotonic_ns": time.monotonic_ns(),
                            "old_device": identity,
                            "inventory_count": count,
                        }
                        self._disconnected.set()
                return not present
            except Exception as error:
                with self._state_lock:
                    self._last_inventory_error = f"{type(error).__name__}: {error}"
                return False
            finally:
                if count >= 0:
                    self._usb.libusb_free_device_list(devices, 1)

    def open(self):
        if self.connected:
            return self
        if self._handle is not None:
            self.close()
        self._usb = u = osmo_usb.load_usb()
        context, handle = C.c_void_p(), C.c_void_p()
        devices = C.POINTER(C.c_void_p)()
        initialized = listed = claimed = False

        def check(code, operation):
            # Keep native open/claim failures in the transport's USBError
            # contract; callers may wait for a camera still enumerating.
            try:
                return osmo_usb.check(u, code, operation)
            except RuntimeError as error:
                raise USBError(str(error)) from error

        try:
            check(u.libusb_init(C.byref(context)), "libusb init")
            initialized = True
            count = check(
                u.libusb_get_device_list(context, C.byref(devices)), "USB list"
            )
            listed = True
            matches = []
            for i in range(count):
                raw = C.create_string_buffer(18)
                check(u.libusb_get_device_descriptor(devices[i], raw), "USB descriptor")
                d = struct.unpack("<BBHBBBBHHHBBBB", raw.raw)
                if (d[7], d[8]) == (self.VID, self.PID):
                    matches.append(devices[i])
            if len(matches) != 1:
                raise USBError(
                    "Нужна одна Osmo360 в режиме съёмки / «Только зарядка» (2ca3:0025)"
                )
            check(u.libusb_open(matches[0], C.byref(handle)), "Open camera")
            check(
                u.libusb_claim_interface(handle, self.INTERFACE),
                "Claim control interface4",
            )
            claimed = True
            u.libusb_get_device_speed.argtypes = [C.c_void_p]
            u.libusb_get_device_speed.restype = C.c_int
            for name in ("libusb_get_bus_number", "libusb_get_device_address"):
                function = getattr(u, name)
                function.argtypes, function.restype = [C.c_void_p], C.c_uint8
            bus, address = self._device_location(matches[0])
            self.link_speed = {
                1: "1.5Mb/s",
                2: "12Mb/s",
                3: "480Mb/s",
                4: "5Gb/s",
                5: "10Gb/s",
            }.get(u.libusb_get_device_speed(matches[0]), "unknown")
            self._context, self._handle = context, handle
            self._stop.clear()
            with self._state_lock:
                self._device_identity = {
                    "pointer": matches[0],
                    "bus": bus,
                    "address": address,
                    "vid": self.VID,
                    "pid": self.PID,
                }
                self._read_failure_status = None
                self._disconnect_evidence = self._last_inventory_error = None
                self._disconnected.clear()
            self.failure = None
            self._thread = threading.Thread(
                target=self._receive, name="osmo-usb-control", daemon=True
            )
            self._thread.start()
            return self
        except Exception:
            if claimed:
                u.libusb_release_interface(handle, self.INTERFACE)
            if handle:
                u.libusb_close(handle)
            if listed:
                u.libusb_free_device_list(devices, 1)
                listed = False
            if initialized:
                u.libusb_exit(context)
            self._context = self._handle = None
            raise
        finally:
            if listed:
                u.libusb_free_device_list(devices, 1)

    def _write(self, message):
        with self._write_lock:
            if not self.connected:
                raise CameraDisconnected(str(self.failure or "USB control закрыт"))
            packet = message.encode()
            buffer, count = C.create_string_buffer(packet), C.c_int()
            status = self._usb.libusb_bulk_transfer(
                self._handle, self.OUT, buffer, len(packet), C.byref(count), 500
            )
            if status < 0 or count.value != len(packet):
                reason = (
                    self._usb.libusb_error_name(status).decode()
                    if status < 0
                    else "partial write"
                )
                raise USBError(
                    f"USB control write: {reason}, {count.value}/{len(packet)} bytes"
                )

    def select_charging_only(self):
        """Deliver one native GUI Charging3 frame; do not wait for its ACK.

        This runtime USB-mode selection keeps preview bootstrap out of file
        transfer mode. It requires an already connected camera; delivery is
        not mode-completion or power-on evidence. A lost write is not retried.
        """
        if not self.connected:
            raise CameraDisconnected(str(self.failure or "Сначала вызовите open()"))
        with self._state_lock:
            for _ in range(65536):
                self._sequence = (self._sequence + 1) & 65535
                if (self._sequence, 6, 0x3A, 0x28, 0x1C) not in self._pending:
                    break
            else:
                raise USBError("Все sequence IDs заняты")
            sequence = self._sequence
        self._write(Message(0x1C, 0x28, sequence, 0x40, 6, 0x3A, b"\0\3"))
        return dict(
            delivered=True,
            ack_claimed=False,
            source=0x1C,
            target=0x28,
            sequence=sequence,
            flags=0x40,
            command_set=6,
            command=0x3A,
            payload_hex="0003",
        )

    def request(
        self, command_set, command, payload=b"", *, receiver=1, timeout=2, ack_type=2
    ):
        if not self.connected:
            raise CameraDisconnected(str(self.failure or "Сначала вызовите open()"))
        if any(
            not isinstance(v, int) or not 0 <= v <= 255
            for v in (command_set, command, receiver)
        ):
            raise ValueError("DUML адреса и команды должны быть uint8")
        if not isinstance(payload, bytes) or len(payload) > 1010 or timeout <= 0:
            raise ValueError("payload: bytes до1010; timeout > 0")
        if (
            not isinstance(ack_type, int)
            or isinstance(ack_type, bool)
            or ack_type not in (1, 2)
        ):
            raise ValueError("ack_type должен быть 1 или 2")
        with self._state_lock:
            for _ in range(65536):
                self._sequence = (self._sequence + 1) & 65535
                key = (self._sequence, command_set, command, receiver, self.sender)
                if key not in self._pending:
                    break
            else:
                raise USBError("Все sequence IDs заняты")
            future = Future()
            self._pending[key] = future
            sequence = self._sequence
        try:
            self._write(
                Message(
                    self.sender,
                    receiver,
                    sequence,
                    ack_type << 5,
                    command_set,
                    command,
                    payload,
                )
            )
            return future.result(timeout)
        except FutureTimeout as exc:
            raise RequestTimeout(
                f"Нет ACK {command_set:02x}/{command:02x} от{receiver:02x} за{timeout}s"
            ) from exc
        finally:
            with self._state_lock:
                self._pending.pop(key, None)

    def _accept(self, message):
        received = time.monotonic_ns()
        if message.is_reply:
            key = (
                message.seq,
                message.cmd_set,
                message.cmd_id,
                message.sender,
                message.receiver,
            )
            with self._state_lock:
                future = self._pending.pop(key, None)
            if future is not None:
                future.set_result(
                    Reply(
                        message.payload,
                        message.sender,
                        message.receiver,
                        message.cmd_set,
                        message.cmd_id,
                        message.seq,
                        received,
                    )
                )
                return
        self.events.append((received, message))
        if message.needs_reply and message.receiver == self.sender:
            if message.command == (0, 0x81):
                self._write(message.answer(self.APP_IDENTITY, flags=0xC0))
            elif message.command == (0, 0x88) and message.payload[:1] == b"\x19":
                self._write(message.answer(bytes.fromhex("1a00000000"), flags=0xC0))

    def _receive(self):
        reader, buffer = FrameReader(), C.create_string_buffer(65536)
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
                        self._accept(message)
                if status not in (0, -7):
                    raise CameraDisconnected(
                        self._usb.libusb_error_name(status).decode()
                    )
        except Exception as exc:
            self.failure = exc
            self._stop.set()
            self._fail_pending(exc)

    def _fail_pending(self, error):
        with self._state_lock:
            futures, self._pending = list(self._pending.values()), {}
        for future in futures:
            if not future.done():
                future.set_exception(error)

    def close(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            if self._thread is threading.current_thread():
                raise USBError("Нельзя закрыть transport из его reader thread")
            self._thread.join(2)
            if self._thread.is_alive():
                raise USBError("USB reader не завершился; handle оставлен открытым")
        self._fail_pending(CameraDisconnected("USB control закрыт"))
        with self._write_lock:
            if self._handle is not None:
                self._usb.libusb_release_interface(self._handle, self.INTERFACE)
                self._usb.libusb_close(self._handle)
                self._usb.libusb_exit(self._context)
            self._context = self._handle = self._thread = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
