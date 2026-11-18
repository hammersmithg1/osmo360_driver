"""Raw gyro/accel collector for the private IMU subprocess.

Reads SSC sensor events through USB DIAG, pairs gyro and accel by their exact
hardware timestamp, and restores the original logging mask on normal exit.
"""

import argparse
import binascii
from collections import Counter
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import socket
import struct
import time

from . import _usb as osmo_usb
from ._imu_wire import PACKET, pack_sample
from ._ssc import decode_diag_packet
from ._ssc_fast import decode_raw_xyz_packet


BIT_REVERSE_TABLE = bytes(int(f"{i:08b}"[::-1], 2) for i in range(256))
HDLC_ESCAPE = re.compile(rb"\x7d(.)", re.DOTALL)
UNESCAPED_BYTES = tuple(bytes([i ^ 0x20]) for i in range(256))
LIVE_PREFIX = 'OSMO_IMU_STATUS '


def _drain_until_quiet(receive, snapshot, *, timeout=3.0, quiet=0.30,
                       clock=time.monotonic):
    """Observe the SAME DIAG reader idle before releasing its USB interface.

    LOG_CONFIG ACK/GET proves a local mask, not completion of already queued
    camera USB writes. Keep consuming those writes with the existing reader.
    Two empty USB timeouts spanning ``quiet`` seconds are the bounded host
    barrier; this does not claim a remote DSP ACK or inspect native memory.
    A restored nonzero mask may keep producing traffic and fail this barrier.
    """
    if (type(timeout) not in (int, float) or not math.isfinite(timeout)
            or type(quiet) not in (int, float) or not math.isfinite(quiet)
            or not 0 < quiet < timeout):
        raise ValueError('Finite0<quiet<timeout required for DIAG drain')
    began = clock()
    deadline, last_data = began + timeout, began
    initial = dict(snapshot())
    previous = initial
    evidence = dict(verified=False, timeout_s=timeout, required_quiet_s=quiet,
                    elapsed_s=0.0, quiet_s=0.0, reads=0, bytes=0, timeouts=0,
                    consecutive_empty_timeouts=0, pending_fragment_bytes=0,
                    last_read_status=None)
    try:
        while clock() < deadline:
            receive()  # One existing150ms bulk read; never SET/open/rebind.
            now = clock()
            current = dict(snapshot())
            count = current['usb_reads'] - previous['usb_reads']
            size = current['usb_bytes'] - previous['usb_bytes']
            timeouts = current['usb_timeouts'] - previous['usb_timeouts']
            status = current['last_read_status']
            fragment = current['pending_fragment_bytes']
            if (count != 1 or size < 0 or timeouts not in (0, 1)
                    or status not in (0, -7)
                    or type(fragment) is not int or fragment < 0):
                raise RuntimeError('DIAG drain reader/counters changed')
            evidence['reads'] += count
            evidence['bytes'] += size
            evidence['timeouts'] += timeouts
            evidence['pending_fragment_bytes'] = fragment
            evidence['last_read_status'] = status
            if size:
                last_data = now
            if status == -7 and size == 0:
                evidence['consecutive_empty_timeouts'] += 1
            else:
                evidence['consecutive_empty_timeouts'] = 0
            evidence['elapsed_s'] = max(0, now - began)
            evidence['quiet_s'] = max(0, now - last_data)
            previous = current
            if now >= deadline:
                break  # A late idle observation cannot authorize handoff.
            if (evidence['consecutive_empty_timeouts'] >= 2
                    and evidence['quiet_s'] >= quiet and fragment == 0):
                evidence['verified'] = True
                return evidence
        evidence['error'] = 'DIAG did not become quiescent before the drain deadline'
    except BaseException as error:
        # A second signal/USB failure must not turn mask restoration into a
        # successful handoff. Preserve evidence while outer finally releases
        # local handles; the session retains its lease/USB owner on failure.
        evidence['elapsed_s'] = max(0, clock() - began)
        evidence['error'] = type(error).__name__ + ': ' + str(error)
        try:
            current = snapshot()
            evidence.update(reads=max(0, current['usb_reads'] - initial['usb_reads']),
                            bytes=max(0, current['usb_bytes'] - initial['usb_bytes']),
                            timeouts=max(0, current['usb_timeouts'] - initial['usb_timeouts']),
                            last_read_status=current['last_read_status'],
                            pending_fragment_bytes=current['pending_fragment_bytes'])
        except Exception:
            pass  # The original read/diagnostic failure remains authoritative.
    return evidence


def _drain_restored_reader(report, receive, snapshot, *, original_mask,
                           fatal_read_status=None, clock=time.monotonic):
    """Keep mask restoration and same-generation USB quiescence distinct."""
    report['quiescent'] = False
    if report.get('restored') is not True:
        report['drain'] = dict(verified=False, skipped='Original mask not restored')
        return
    if fatal_read_status is not None:
        report['drain'] = dict(verified=False, skipped='Original DIAG reader failed',
                               fatal_read_status=fatal_read_status)
    else:
        report['drain'] = _drain_until_quiet(receive, snapshot, clock=clock)
        report['quiescent'] = report['drain']['verified'] is True
    if any(original_mask):
        # A sparse pre-existing log can be silent for this whole observation
        # and resume after the reader closes. Restore it unchanged, but never
        # interpret that short interval as permission for a USB transition.
        report['quiescent'] = False
        report['drain']['handoff_blocked_reason'] = (
            'Original equipment1 DIAG mask has enabled logs; '
            'observed quiet cannot exclude later traffic')
    if not report['quiescent']:
        report['errors'].append('DIAG USB handoff blocked: ' + str(
            report['drain'].get('handoff_blocked_reason')
            or report['drain'].get('error') or report['drain'].get('skipped')))


class _LiveStatus:
    """Bounded host-only evidence; never performs USB or changes recovery."""

    def __init__(self, snapshot, *, clock=None, output=None):
        self.snapshot = snapshot
        self.clock = clock or time.monotonic_ns
        self.output = output or (lambda line: print(line, flush=True))
        self.phase = 'starting'
        self.times = dict(mask_verified_ns=None, first_ssc_ns=None, first_pair_ns=None,
                          fatal_read_ns=None, fatal_read_status=None, fatal_read_phase=None)
        self.events = []
        self.latest = None
        self.last_output_ns = None
        self.output_failed = False
        self._fatal_phases = set()

    def mark(self, event, *, at_ns=None, status=None, **fields):
        now = self.clock() if at_ns is None else at_ns
        if event in ('mask_verified', 'first_ssc', 'first_pair'):
            key = event + '_ns'
            if self.times[key] is not None:
                return
            self.times[key] = now
        elif event == 'fatal_read':
            if self.phase in self._fatal_phases:
                return
            self._fatal_phases.add(self.phase)
            if self.times['fatal_read_ns'] is None:
                self.times.update(fatal_read_ns=now, fatal_read_status=status,
                                  fatal_read_phase=self.phase)
        self._emit(event, now, **fields)

    def progress(self):
        now = self.clock()
        if self.last_output_ns is None or now - self.last_output_ns >= 1_000_000_000:
            self._emit('progress', now)

    def _emit(self, event, now, **fields):
        value = dict(schema=1, event=event, monotonic_ns=now, phase=self.phase,
                     **self.snapshot(), **self.times, **fields)
        self.latest = value
        self.last_output_ns = now
        if event != 'progress':
            self.events.append(value)
            del self.events[:-12]
        if not self.output_failed:
            try:
                self.output(LIVE_PREFIX + json.dumps(value, separators=(',', ':'), allow_nan=False))
            except (OSError, ValueError):
                # Losing the diagnostic pipe cannot bypass mask restoration.
                self.output_failed = True


def checksum(data):
    # crc_hqx is non-reflected CCITT; reversing input/result bits gives X25.
    value = binascii.crc_hqx(bytes(data).translate(BIT_REVERSE_TABLE), 0xFFFF)
    return (
        (BIT_REVERSE_TABLE[value & 255] << 8) | BIT_REVERSE_TABLE[value >> 8]
    ) ^ 0xFFFF


def _unescape_match(match):
    return UNESCAPED_BYTES[match[1][0]]


def frame(body):
    data = body + struct.pack("<H", checksum(body))
    return (
        b"".join(
            bytes([0x7D, x ^ 0x20]) if x in (0x7D, 0x7E) else bytes([x]) for x in data
        )
        + b"\x7e"
    )


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            end = self.buffer.find(b"\x7e")
            if end < 0:
                return
            encoded = bytes(self.buffer[:end])
            del self.buffer[: end + 1]
            # An odd trailing escape run is incomplete; paired 7D7D is valid.
            if (
                encoded.endswith(b"\x7d")
                and (len(encoded) - len(encoded.rstrip(b"\x7d"))) % 2
            ):
                continue
            decoded = (
                HDLC_ESCAPE.sub(_unescape_match, encoded)
                if b"\x7d" in encoded
                else encoded
            )
            if len(decoded) < 3:
                continue
            body, received_checksum = (
                decoded[:-2],
                int.from_bytes(decoded[-2:], "little"),
            )
            if received_checksum != checksum(body):
                self.bad_crc += 1
                continue
            yield body


def _sample_outputs(output, *, write_json=False, udp_format=None):
    """Serialize only the requested sinks; binary-only avoids JSON entirely."""
    if udp_format not in (None, "json", "binary"):
        raise ValueError("Unknown UDP format")
    line = (
        json.dumps(output, separators=(",", ":")) + "\n"
        if write_json or udp_format == "json"
        else None
    )
    datagram = (
        pack_sample(output)
        if udp_format == "binary"
        else line.encode()
        if udp_format == "json"
        else None
    )
    return line, datagram


class _UDPSampleBatch:
    """One capture read's pending unchanged binary records; never a timer.

    send/drop accounting remains samples. Clear the pending block before the
    send: an interrupted/uncertain UDP send must never be replayed by finally.
    The caller flushes the tail before this same receive() returns or raises.
    """
    def __init__(self, udp, address, max_samples):
        if type(max_samples) is not int or not 1 <= max_samples <= 64:
            raise ValueError('UDP batch must contain1..64 samples')
        self.udp, self.address, self.max_samples = udp, address, max_samples
        self.pending = []
        self.sent = self.dropped = 0

    def add(self, record):
        if not isinstance(record, bytes) or len(record) != PACKET.size:
            raise ValueError('UDP batch requires unchanged binary sample records')
        self.pending.append(record)
        if len(self.pending) == self.max_samples:
            self.flush()

    def flush(self):
        if not self.pending:
            return
        count = len(self.pending)
        payload = b''.join(self.pending)
        self.pending.clear()
        try:
            self.udp.sendto(payload, self.address)
        except (BlockingIOError, OSError):
            self.dropped += count
        else:
            self.sent += count


def config(operation, data):
    return b"\x73" + struct.pack("<3xI", operation) + data


def parse_mask(body, operation):
    if len(body) < 20 or body[0] != 0x73:
        raise RuntimeError("Не получен полный ответ LOG_CONFIG")
    op, status, equipment, last_item = struct.unpack_from("<4I", body, 4)
    if op != operation or status or equipment != 1:
        raise RuntimeError(
            f"LOG_CONFIG: op={op}, status={status}, equipment={equipment}"
        )
    length = (last_item + 8) // 8
    mask = body[20:]
    if len(mask) != length or not 1 <= last_item <= 4095:
        raise RuntimeError(
            f"Некорректный размер маски: last={last_item}, bytes={len(mask)}"
        )
    return last_item, mask


class _DiagRebindGuard:
    """Private identity pin and mask-only client; never resumes sample output."""

    def __init__(self, usb, context, device, handle, close_old):
        self.usb, self.context, self.device = usb, context, device
        self._close_old_callback = close_old
        self.rebound = None
        self.decoder = Decoder()
        self.discarded_packets = 0
        for name, args, result in (
            ("get_bus_number", [C.c_void_p], C.c_uint8),
            ("get_device_address", [C.c_void_p], C.c_uint8),
            ("get_port_numbers", [C.c_void_p, C.POINTER(C.c_uint8), C.c_int], C.c_int),
            (
                "get_string_descriptor_ascii",
                [C.c_void_p, C.c_uint8, C.c_void_p, C.c_int],
                C.c_int,
            ),
        ):
            function = getattr(usb, "libusb_" + name)
            function.argtypes, function.restype = args, result
        self.old_location = self._location(device)
        self.port = self._port(device)
        self.fingerprint = self._fingerprint(handle, self._descriptor(device))
        self.identity_strength = (
            "physical_port_product_and_serial"
            if self.fingerprint[2] is not None
            else "physical_port_product_without_usb_serial"
        )

    def _location(self, device):
        value = (
            self.usb.libusb_get_bus_number(device),
            self.usb.libusb_get_device_address(device),
        )
        if not all(value):
            raise RuntimeError("Rebind identity requires a valid USB bus/address")
        return value

    def _port(self, device):
        ports = (C.c_uint8 * 8)()
        count = osmo_usb.check(
            self.usb,
            self.usb.libusb_get_port_numbers(device, ports, 8),
            "USB port path",
        )
        if not 1 <= count <= 8 or not all(ports[:count]):
            raise RuntimeError("Rebind identity requires a physical USB port path")
        return self._location(device)[0], tuple(ports[:count])

    def _descriptor(self, device):
        raw = C.create_string_buffer(18)
        osmo_usb.check(
            self.usb,
            self.usb.libusb_get_device_descriptor(device, raw),
            "Rebind USB descriptor",
        )
        return raw.raw

    def _string_hash(self, handle, index):
        if not index:
            raise RuntimeError("Rebind identity requires the product string")
        raw = C.create_string_buffer(256)
        length = osmo_usb.check(
            self.usb,
            self.usb.libusb_get_string_descriptor_ascii(handle, index, raw, len(raw)),
            "Rebind private string descriptor",
        )
        if not 0 < length <= len(raw):
            raise RuntimeError("Rebind identity string is unavailable")
        return hashlib.sha256(raw.raw[:length]).digest()

    def _fingerprint(self, handle, descriptor):
        # Raw product/serial strings and their digests stay private, not reports.
        return (
            descriptor[8:14],
            self._string_hash(handle, descriptor[15]),
            self._string_hash(handle, descriptor[16]) if descriptor[16] else None,
        )

    def _inventory(self, visit):
        devices, count = C.POINTER(C.c_void_p)(), -1
        try:
            count = osmo_usb.check(
                self.usb,
                self.usb.libusb_get_device_list(self.context, C.byref(devices)),
                "Rebind device inventory",
            )
            return visit(devices, count)
        finally:
            if count >= 0:
                self.usb.libusb_free_device_list(devices, 1)

    def old_absent(self):
        def inspect(devices, count):
            present = False
            for i in range(count):
                device = devices[i]
                self._descriptor(
                    device
                )  # A failed/incomplete inventory is not absence.
                present |= (
                    device == self.device or self._location(device) == self.old_location
                )
            return not present, dict(
                method="complete_usb_inventory",
                inventory_count=count,
                old_bus=self.old_location[0],
                old_address=self.old_location[1],
            )

        return self._inventory(inspect)

    def close_old(self):
        self._close_old_callback()

    def open_matching(self):
        def inspect(devices, count):
            matches = []
            for i in range(count):
                device = devices[i]
                descriptor = self._descriptor(device)
                if (
                    struct.unpack_from("<HH", descriptor, 8) == (0x2CA3, 0x0025)
                    and self._port(device) == self.port
                ):
                    matches.append((device, descriptor))
            if not matches:
                return False
            if len(matches) != 1:
                raise RuntimeError("Rebind physical port is ambiguous")
            device, descriptor = matches[0]
            handle = C.c_void_p()
            claimed = False
            try:
                osmo_usb.check(
                    self.usb,
                    self.usb.libusb_open(device, C.byref(handle)),
                    "Rebind open",
                )
                if self._fingerprint(handle, descriptor) != self.fingerprint:
                    raise RuntimeError("Rebind product/serial identity differs")
                osmo_usb.check(
                    self.usb,
                    self.usb.libusb_claim_interface(handle, 0),
                    "Rebind claim DIAG0",
                )
                claimed = True
                self.rebound = handle
                self.new_location = self._location(device)
                return True
            finally:
                if not self.rebound and handle:
                    if claimed:
                        self.usb.libusb_release_interface(handle, 0)
                    self.usb.libusb_close(handle)

        return self._inventory(inspect)

    def _request(self, operation, payload, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("DIAG rebind restoration deadline before request")
        wire, written = frame(config(operation, payload)), C.c_int()
        buffer = C.create_string_buffer(wire)
        osmo_usb.check(
            self.usb,
            self.usb.libusb_bulk_transfer(
                self.rebound,
                0x01,
                buffer,
                len(wire),
                C.byref(written),
                max(1, min(1000, int(remaining * 1000))),
            ),
            "Rebind DIAG write",
        )
        if written.value != len(wire):
            raise RuntimeError("Partial rebind DIAG write")
        buffer = C.create_string_buffer(4096)
        while time.monotonic() < deadline:
            received = C.c_int()
            timeout = max(1, min(150, int((deadline - time.monotonic()) * 1000)))
            status = self.usb.libusb_bulk_transfer(
                self.rebound, 0x81, buffer, len(buffer), C.byref(received), timeout
            )
            if status not in (0, -7):
                osmo_usb.check(self.usb, status, "Rebind DIAG read")
            for body in self.decoder.feed(buffer.raw[: received.value]):
                if (
                    len(body) >= 12
                    and body[0] == 0x73
                    and struct.unpack_from("<I", body, 4)[0] == operation
                ):
                    if time.monotonic() >= deadline:
                        raise RuntimeError("DIAG rebind reply arrived after deadline")
                    return parse_mask(body, operation)
                self.discarded_packets += 1
        raise RuntimeError("DIAG rebind mask reply timed out")

    def read_mask(self, deadline):
        return self._request(4, struct.pack("<I", 1), deadline)

    def write_mask(self, last, mask, deadline):
        return self._request(3, struct.pack("<II", 1, last) + mask, deadline)

    def close_rebound(self):
        if self.rebound:
            self.usb.libusb_release_interface(self.rebound, 0)
            self.usb.libusb_close(self.rebound)
            self.rebound = None


def _restore_only_after_reset(
    guard, last, original, owned, *, timeout=8, clock=time.monotonic, sleep=time.sleep
):
    """One bounded, identity-pinned cleanup; a successful result is NOT streaming recovery."""
    deadline = clock() + timeout
    report = dict(restored=False, set_attempts=0, sample_stream_resumed=False)
    try:
        while clock() < deadline:
            try:
                absent, evidence = guard.old_absent()
                if absent and clock() < deadline:
                    report["disconnect_evidence"] = evidence
                    break
            except Exception as error:
                report["last_inventory_error"] = (
                    type(error).__name__ + ": " + str(error)
                )
            sleep(min(0.05, max(0, deadline - clock())))
        else:
            raise RuntimeError("Original USB device disappearance was not verified")
        guard.close_old()
        while clock() < deadline:
            if guard.open_matching():
                if clock() >= deadline:
                    raise RuntimeError("Rebind identity completed after deadline")
                report["identity_matched"] = True
                report["identity_strength"] = getattr(
                    guard, "identity_strength", "provided_guard"
                )
                break
            sleep(min(0.05, max(0, deadline - clock())))
        else:
            raise RuntimeError("Matching USB DIAG device did not reappear")
        current_last, current = guard.read_mask(deadline)
        if clock() >= deadline or current_last != last:
            raise RuntimeError("Rebind mask last-item/deadline differs")
        if current == original:
            report.update(restored=True, method="original_mask_already_observed")
        elif current == owned:
            report["set_attempts"] = 1
            try:
                # Preserve time for a fresh GET if the SET receipt is lost.
                receipt_deadline = clock() + (deadline - clock()) / 2
                guard.write_mask(last, original, receipt_deadline)
            except Exception as error:
                # A lost SET receipt is uncertain: read back, never repeat SET.
                report["set_receipt_error"] = type(error).__name__ + ": " + str(error)
            checked_last, checked = guard.read_mask(deadline)
            if clock() >= deadline or (checked_last, checked) != (last, original):
                raise RuntimeError(
                    "Original mask readback after rebind was not verified"
                )
            report.update(restored=True, method="one_restore_set_and_readback")
        else:
            raise RuntimeError("Foreign DIAG mask after rebind; refusing overwrite")
    except BaseException as error:
        report["error"] = type(error).__name__ + ": " + str(error)
    finally:
        try:
            guard.close_rebound()
        except BaseException as error:
            report["close_error"] = type(error).__name__ + ": " + str(error)
        report["discarded_cleanup_packets"] = getattr(guard, "discarded_packets", 0)
        report["cleanup_bad_crc"] = getattr(
            getattr(guard, "decoder", None), "bad_crc", 0
        )
    return report


def _restore_original_mask(report, request, last, original, *, rebind=None):
    """Keep retry diagnostics separate from the final restoration outcome."""
    report["restore_attempt_errors"] = []
    for retry in range(3):
        try:
            request(config(3, struct.pack("<II", 1, last) + original), 3)
            current_last, current_mask = parse_mask(
                request(config(4, struct.pack("<I", 1)), 4), 4
            )
            if (current_last, current_mask) != (last, original):
                raise RuntimeError("Маска после восстановления отличается")
            report["restored"] = True
            break
        except (OSError, RuntimeError) as error:
            report["restore_attempt_errors"].append(
                f"restore attempt {retry + 1}: {error}"
            )
    if not report["restored"]:
        result = rebind() if rebind is not None else None
        if result is None:
            report["errors"].append(
                "Original DIAG mask restoration was not verified after 3 attempts"
            )
        else:
            report["mask_restore_rebind"] = result
            report["restored"] = result["restored"]
            if not result["restored"] or result.get("close_error"):
                report["errors"].append(
                    "Rebind restore: "
                    + str(result.get("error") or result.get("close_error"))
                )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--profile", choices=["ssc-raw"], default="ssc-raw")
    parser.add_argument(
        "--read-buffer-bytes",
        type=int,
        default=65536,
        help="USB read buffer: 1024..65536 bytes, multiple of 1024 (default: 65536)",
    )
    parser.add_argument(
        "--samples-jsonl",
        type=Path,
        help="Писать каждый RAW XYZ сразу, по строке на образец",
    )
    parser.add_argument(
        "--udp-port", type=int, help="Передавать RAW XYZ как JSON на 127.0.0.1:PORT"
    )
    parser.add_argument(
        "--udp-format",
        choices=("json", "binary"),
        default="json",
        help="UDP encoding; binary is internal paired IMU (default: json)",
    )
    parser.add_argument(
        '--udp-batch-size', type=int, default=1,
        help='Private binary samples per datagram; flush tail at each USB read (1..64, default1)',
    )
    parser.add_argument(
        "--no-packet-jsonl",
        action="store_true",
        help="Не сохранять подробный журнал DIAG",
    )
    parser.add_argument(
        "--no-raw", action="store_true", help="Не сохранять дублирующий USB .bin"
    )
    parser.add_argument(
        "--paired",
        action="store_true",
        help="Выдавать gx/gy/gz/ax/ay/az с точно совпавшими аппаратными timestamps",
    )
    args = parser.parse_args()
    items = [0x9CB]
    if not 0 <= args.seconds <= 60:
        parser.error("seconds: 0 (до Ctrl-C) или 1..60")
    if args.udp_port is not None and not 1 <= args.udp_port <= 65535:
        parser.error("udp-port: 1..65535")
    if args.udp_format == "binary" and (not args.paired or args.udp_port is None):
        parser.error("binary UDP requires --paired and --udp-port")
    if not 1 <= args.udp_batch_size <= 64:
        parser.error('udp-batch-size:1..64')
    if args.udp_batch_size > 1 and (args.udp_format != 'binary' or not args.paired or args.udp_port is None):
        parser.error('Batched UDP requires binary paired samples and --udp-port')
    if not 1024 <= args.read_buffer_bytes <= 65536 or args.read_buffer_bytes % 1024:
        parser.error("read-buffer-bytes: 1024..65536, multiple of 1024")
    paths = {
        name: Path(f"{args.prefix}.{name}") for name in ["bin", "packets.jsonl", "json"]
    }
    if (
        any(p.exists() for p in paths.values())
        or args.samples_jsonl
        and args.samples_jsonl.exists()
    ):
        parser.error("Используйте новое имя prefix: файлы не перезаписываются")

    usb = osmo_usb.load_usb()
    context, handle, devices = C.c_void_p(), C.c_void_p(), C.POINTER(C.c_void_p)()
    count, claimed, attempted = -1, False, False
    old_mask, old_last, owned_mask = None, None, None
    rebind_guard, fatal_read_status = None, None
    decoder, opcodes, log_codes = Decoder(), Counter(), Counter()
    report = {
        "transport": "USB DIAG 2ca3:0025 interface 0",
        "seconds": args.seconds,
        "profile": args.profile,
        "udp_format": args.udp_format if args.udp_port is not None else None,
        'udp_batch_size': args.udp_batch_size,
        "read_buffer_bytes": args.read_buffer_bytes,
        "requested_sensor_codes": [f"{0x1000 + n:04x}" for n in items],
        "restored": False,
        "quiescent": False,
        "drain": None,
        "errors": [],
    }
    raw, packets, sample_file, udp = None, None, None, None
    sample_counts, sample_decode_errors, udp_drops, udp_sent = Counter(), 0, 0, 0
    pending_pairs, paired_samples, evicted_unpaired = {}, 0, 0
    streaming_enabled = False
    decode_samples = None
    binary_raw = args.paired and args.udp_format == "binary" and not args.samples_jsonl
    if args.samples_jsonl or args.udp_port:
        decode_samples = decode_raw_xyz_packet if binary_raw else decode_diag_packet
    # Reuse the host buffer; Decoder retains packets split across USB reads.
    read_buffer = C.create_string_buffer(args.read_buffer_bytes)
    io_counts = dict(usb_reads=0, usb_bytes=0, usb_timeouts=0, last_read_status=None)

    def live_snapshot():
        return dict(io_counts, ssc_packets=log_codes['19cb'],
                    accel_samples=sample_counts['accel'], gyro_samples=sample_counts['gyro'],
                    paired_samples=paired_samples, sample_decode_errors=sample_decode_errors,
                    udp_sent=udp_sent, udp_drops=udp_drops, bad_crc=decoder.bad_crc,
                    unpaired_pending=len(pending_pairs))

    live = _LiveStatus(live_snapshot)

    def close_original():
        nonlocal claimed, handle
        old_handle, was_claimed = handle, claimed
        handle, claimed = C.c_void_p(), False
        try:
            if was_claimed:
                usb.libusb_release_interface(old_handle, 0)
        finally:
            if old_handle:
                usb.libusb_close(old_handle)

    def send(body):
        wire, written = frame(body), C.c_int()
        buf = C.create_string_buffer(wire)
        osmo_usb.check(
            usb,
            usb.libusb_bulk_transfer(
                handle, 0x01, buf, len(wire), C.byref(written), 1000
            ),
            "DIAG write",
        )
        if written.value != len(wire):
            raise RuntimeError("Неполная запись DIAG")

    def receive():
        nonlocal \
            sample_decode_errors, \
            udp_drops, \
            udp_sent, \
            paired_samples, \
            evicted_unpaired, \
            fatal_read_status
        received = C.c_int()
        code = usb.libusb_bulk_transfer(
            handle, 0x81, read_buffer, len(read_buffer), C.byref(received), 150
        )
        received_at_ns = time.monotonic_ns()
        io_counts['usb_reads'] += 1
        io_counts['usb_bytes'] += received.value
        io_counts['usb_timeouts'] += code == -7
        io_counts['last_read_status'] = code
        if code not in (0, -7):
            fatal_read_status = code
            live.mark('fatal_read', at_ns=received_at_ns, status=code)
            osmo_usb.check(usb, code, "DIAG read")
        data = read_buffer.raw[: received.value]
        if raw:
            raw.write(data)
        decoded = list(decoder.feed(data))
        udp_batch = (_UDPSampleBatch(udp, ('127.0.0.1', args.udp_port), args.udp_batch_size)
                     if udp is not None and args.udp_format == 'binary' and args.udp_batch_size > 1
                     else None)
        try:
            for body in decoded:
                opcodes[f"{body[0]:02x}"] += 1
                log_code = None
                if body[0] == 0x10 and len(body) >= 16:
                    log_code = struct.unpack_from("<H", body, 6)[0]
                    log_codes[f"{log_code:04x}"] += 1
                    if (log_code == 0x19CB and streaming_enabled
                            and live.times['first_ssc_ns'] is None):
                        live.mark('first_ssc', at_ns=received_at_ns)
                if packets:
                    entry = {
                        "host_monotonic_s": time.monotonic(),
                        "opcode": body[0],
                        "hex": body.hex(),
                    }
                    if log_code is not None:
                        entry["log_code"] = f"{log_code:04x}"
                    packets.write(json.dumps(entry) + "\n")
                if decode_samples and streaming_enabled:
                    try:
                        samples = decode_samples(body)
                    except ValueError:
                        sample_decode_errors += 1
                        continue
                    for sample in samples:
                        if binary_raw:
                            sensor, tick, _, _, _, _, _ = sample
                        else:
                            if sample["kind"] != "raw" or sample["sensor_type"] not in (
                                "accel", "gyro"
                            ):
                                continue
                            sample["host_receive_monotonic_ns"] = received_at_ns
                            sensor, tick = sample["sensor_type"], sample["timestamp_ticks"]
                        sample_counts[sensor] += 1
                        output = sample
                        if args.paired:
                            pair = pending_pairs.setdefault(tick, {})
                            pair[sensor] = sample
                            if len(pair) < 2:
                                if len(pending_pairs) > 4096:
                                    del pending_pairs[next(iter(pending_pairs))]
                                    evicted_unpaired += 1
                                continue
                            a, g = pair["accel"], pair["gyro"]
                            if binary_raw:
                                _, _, ax, ay, az, astatus, asuid = a
                                _, _, gx, gy, gz, gstatus, gsuid = g
                            else:
                                ax, ay, az, astatus, asuid = (
                                    a["x"], a["y"], a["z"], a["status"], a["suid"]
                                )
                                gx, gy, gz, gstatus, gsuid = (
                                    g["x"], g["y"], g["z"], g["status"], g["suid"]
                                )
                            output = {
                                "sensor_type": "imu",
                                "timestamp_ticks": tick,
                                "tick_hz": 19200000,
                                "ax": ax,
                                "ay": ay,
                                "az": az,
                                "gx": gx,
                                "gy": gy,
                                "gz": gz,
                                "accel_unit": "m/s^2",
                                "gyro_unit": "rad/s",
                                "accel_status": astatus,
                                "gyro_status": gstatus,
                                "accel_suid": asuid,
                                "gyro_suid": gsuid,
                                "host_receive_monotonic_ns": received_at_ns,
                            }
                            del pending_pairs[tick]
                            paired_samples += 1
                            if paired_samples == 1:
                                live.mark('first_pair', at_ns=received_at_ns)
                        line, datagram = _sample_outputs(
                            output,
                            write_json=sample_file is not None,
                            udp_format=args.udp_format if udp else None,
                        )
                        if sample_file:
                            sample_file.write(line)
                        if udp:
                            if udp_batch is not None:
                                udp_batch.add(datagram)
                            else:
                                try:
                                    udp.sendto(datagram, ("127.0.0.1", args.udp_port))
                                    udp_sent += 1
                                except (BlockingIOError, OSError):
                                    udp_drops += 1
        finally:
            if udp_batch is not None:
                try:
                    udp_batch.flush()
                finally:
                    udp_sent += udp_batch.sent
                    udp_drops += udp_batch.dropped
        live.progress()
        return decoded

    def request(body, operation, timeout=3):
        send(body)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for reply in receive():
                if reply[0] in (0x13, 0x14, 0x15):
                    raise RuntimeError(f"DIAG отверг запрос: {reply[:24].hex()}")
                if (
                    len(reply) >= 12
                    and reply[0] == 0x73
                    and struct.unpack_from("<I", reply, 4)[0] == operation
                ):
                    return reply
        raise RuntimeError(f"Нет ответа LOG_CONFIG op={operation}")

    def interrupt(signum, unused):
        raise KeyboardInterrupt(f"signal {signum}")

    old_term = signal.signal(signal.SIGTERM, interrupt)
    try:
        raw = paths["bin"].open("xb") if not args.no_raw else None
        packets = paths["packets.jsonl"].open("x") if not args.no_packet_jsonl else None
        if args.samples_jsonl:
            sample_file = args.samples_jsonl.open("x", buffering=1)
        if args.udp_port:
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.setblocking(False)
        osmo_usb.check(usb, usb.libusb_init(C.byref(context)), "libusb init")
        count = osmo_usb.check(
            usb, usb.libusb_get_device_list(context, C.byref(devices)), "device list"
        )
        matches = []
        for i in range(count):
            desc = C.create_string_buffer(18)
            osmo_usb.check(
                usb, usb.libusb_get_device_descriptor(devices[i], desc), "descriptor"
            )
            vid, pid = struct.unpack_from("<HH", desc.raw, 8)
            if (vid, pid) == (0x2CA3, 0x0025):
                matches.append(devices[i])
        if len(matches) != 1:
            raise RuntimeError("Нужна одна Osmo в USB-режиме съёмки 2ca3:0025")
        osmo_usb.check(usb, usb.libusb_open(matches[0], C.byref(handle)), "open")
        osmo_usb.check(usb, usb.libusb_claim_interface(handle, 0), "claim DIAG")
        claimed = True
        try:
            rebind_guard = _DiagRebindGuard(
                usb, context, matches[0], handle, close_original
            )
            report["rebind_identity_ready"] = True
        except (AttributeError, OSError, RuntimeError) as error:
            # Normal capture does not require recovery support. Without the
            # original identity pin a later reconnect is never adopted.
            report["rebind_guard_error"] = type(error).__name__ + ": " + str(error)
        old_last, old_mask = parse_mask(request(config(4, struct.pack("<I", 1)), 4), 4)
        report.update(original_last_item=old_last, original_mask_hex=old_mask.hex())
        # Сохраняем исходную маску на диске до первого изменения.
        paths["json"].write_text(json.dumps(report, indent=2) + "\n")
        new_mask = bytearray(old_mask)
        for item in items:
            if item > old_last:
                raise RuntimeError(f"Недоступный item {item:03x}")
            new_mask[item // 8] |= 1 << (item % 8)
        owned_mask = bytes(new_mask)
        attempted = True
        parse_mask(request(config(3, struct.pack("<II", 1, old_last) + new_mask), 3), 3)
        read_last, read_mask = parse_mask(
            request(config(4, struct.pack("<I", 1)), 4), 4
        )
        report["enabled_mask_verified"] = read_last == old_last and read_mask == bytes(
            new_mask
        )
        if not report["enabled_mask_verified"]:
            raise RuntimeError("Камера не подтвердила запрошенную маску логирования")
        # Не выдаём старые логи, прочитанные вместе с исходной маской до запуска.
        streaming_enabled = True
        live.phase = 'capture'
        live.mark('mask_verified')
        print("Sensor log mask enabled; capture started", flush=True)
        deadline = time.monotonic() + args.seconds if args.seconds else float("inf")
        while time.monotonic() < deadline:
            receive()
    except KeyboardInterrupt as error:
        report["stop_reason"] = str(error) or "Ctrl-C"
    except (OSError, RuntimeError) as error:
        report["errors"].append(str(error) or "interrupted")
    finally:
        live.phase = 'cleanup'
        live.mark('cleanup_started')
        streaming_enabled = False
        if attempted:

            def restore_rebound():
                # A fatal read may first appear while normal restore awaits its ACK.
                if fatal_read_status is None or rebind_guard is None:
                    return None
                report["discarded_decoder_bytes_at_reset"] = len(decoder.buffer)
                report["unpaired_samples_at_reset"] = len(pending_pairs)
                return _restore_only_after_reset(
                    rebind_guard, old_last, old_mask, owned_mask
                )

            try:
                _restore_original_mask(
                    report, request, old_last, old_mask, rebind=restore_rebound
                )
            except BaseException as error:
                report['errors'].append('Original mask cleanup interrupted: '
                                        + type(error).__name__ + ': ' + str(error))
            _drain_restored_reader(
                report, receive,
                lambda: dict(io_counts, pending_fragment_bytes=len(decoder.buffer)),
                original_mask=old_mask,
                fatal_read_status=fatal_read_status,
                clock=time.monotonic,
            )
            live.mark('drain_verified' if report['quiescent'] else 'drain_failed',
                      quiescent=report['quiescent'])
            if fatal_read_status is not None:
                report["fatal_read_status"] = fatal_read_status
        if claimed:
            usb.libusb_release_interface(handle, 0)
        if handle:
            usb.libusb_close(handle)
        if count >= 0:
            usb.libusb_free_device_list(devices, 1)
        if context:
            usb.libusb_exit(context)
        signal.signal(signal.SIGTERM, old_term)
        if raw:
            raw.close()
        if packets:
            packets.close()
        if sample_file:
            sample_file.close()
        if udp:
            udp.close()
        report.update(
            opcodes=dict(opcodes),
            log_codes=dict(log_codes),
            bad_crc=decoder.bad_crc,
            live_samples=dict(sample_counts),
            sample_decode_errors=sample_decode_errors,
            udp_drops=udp_drops,
            udp_port=args.udp_port,
            paired_samples=paired_samples,
            unpaired_pending=len(pending_pairs),
            evicted_unpaired=evicted_unpaired,
            samples_jsonl=str(args.samples_jsonl) if args.samples_jsonl else None,
        )
        live.phase = 'finished'
        live.mark('collector_finished', restored=bool(report['restored']),
                  quiescent=bool(report['quiescent']), error_count=len(report['errors']))
        report.update(collector_live=live.latest, collector_events=live.events,
                      live_output_failed=live.output_failed)
        paths["json"].write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in report.items() if k != "original_mask_hex"}, indent=2
        )
    )
    return int(bool(report["errors"]) or attempted and (
        report["restored"] is not True or report["quiescent"] is not True))


if __name__ == "__main__":
    raise SystemExit(main())
