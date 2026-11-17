"""Pure Qualcomm SEE/SSC DIAG sensor decoding with original hardware ticks.

RAW and API events are distinguished so the collector can exclude duplicate
API samples. Sensor values and axes are preserved without interpolation.
"""

from collections import defaultdict
import math
import struct


def varint(data, position):
    value = 0
    for shift in range(0, 70, 7):
        if position >= len(data):
            raise ValueError("Неполный varint protobuf")
        byte = data[position]
        position += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, position
    raise ValueError("Слишком длинный varint protobuf")


def protobuf(data):
    result, position = defaultdict(list), 0
    while position < len(data):
        key, position = varint(data, position)
        tag, wire = key >> 3, key & 7
        if tag == 0:
            raise ValueError("Нулевой номер поля protobuf")
        if wire == 0:
            value, position = varint(data, position)
        elif wire in (1, 2, 5):
            if wire == 2:
                length, position = varint(data, position)
            else:
                length = 8 if wire == 1 else 4
            value = data[position : position + length]
            position += length
            if len(value) != length:
                raise ValueError("Неполное поле protobuf")
        else:
            raise ValueError(f"Неизвестный wire type {wire}")
        result[tag].append((wire, value))
    return dict(result)


def scalar(message, tag, wire, default):
    if tag not in message:
        return default
    actual, value = message[tag][-1]
    if actual != wire:
        raise ValueError(f"Поле {tag}: wire {actual} вместо {wire}")
    return value


def nested(message, tag):
    return protobuf(scalar(message, tag, 2, b""))


def fixed(message, tag, size, default=0):
    value = scalar(message, tag, 5 if size == 4 else 1, bytes(size))
    return int.from_bytes(value, "little") if tag in message else default


def floats(message, tag):
    values = []
    for wire, data in message.get(tag, []):
        if wire == 5:
            values.append(struct.unpack("<f", data)[0])
        elif wire == 2 and len(data) % 4 == 0:
            values.extend(struct.unpack("<" + "f" * (len(data) // 4), data))
        else:
            raise ValueError("Invalid packed float field")
    return values


def uid(message):
    return f"{fixed(message, 2, 8):016x}{fixed(message, 1, 8):016x}"


def decode_diag_packet(body):
    """Return XYZ samples from one HDLC-decoded DIAG_LOG_F body, no I/O.

    Keys: sensor_type, timestamp_ticks, x/y/z, status, kind ('raw' or 'api'),
    batch_type, suid, instance_id, log_code, log_timestamp_ticks.
    Ticks have NOT been converted to seconds. Neither axes nor values change.
    Non-sensor records return []; malformed sensor packets raise ValueError.
    """
    if len(body) < 16 or body[0] != 0x10:
        return []
    code = struct.unpack_from("<H", body, 6)[0]
    if code not in (0x19C7, 0x19C9, 0x19CA, 0x19CB):
        return []
    length = struct.unpack_from("<H", body, 4)[0]
    if length < 12 or length + 4 > len(body):
        raise ValueError("Truncated DIAG log")
    message = protobuf(body[16 : 4 + length])
    sensor_type = scalar(message, 4, 2, b"").decode("utf-8", errors="replace")
    common = {
        "sensor_type": sensor_type,
        "suid": uid(nested(message, 3)),
        "instance_id": fixed(message, 5, 8),
        "log_code": f"{code:04x}",
        "log_timestamp_ticks": fixed(message, 2, 8),
    }
    samples = []

    def add(timestamp, values, status, kind, batch_type=""):
        if len(values) != 3:
            return
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite sensor XYZ")
        samples.append(
            {
                **common,
                "timestamp_ticks": timestamp,
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "status": status,
                "kind": kind,
                "batch_type": batch_type,
            }
        )

    if 103 in message:
        raw = nested(message, 103)
        for wire, data in raw.get(1, []):
            if wire != 2:
                raise ValueError("Invalid raw batch sample")
            sample = protobuf(data)
            add(
                fixed(sample, 2, 8),
                floats(sample, 3),
                scalar(sample, 4, 0, 0),
                "raw",
                scalar(sample, 1, 0, 0),
            )
    elif 101 in message:
        api = nested(message, 101)
        if fixed(api, 1, 4) == 1025:
            payload = scalar(nested(api, 100), 1, 2, b"")
            event = protobuf(payload)
            add(fixed(api, 2, 8), floats(event, 1), scalar(event, 2, 0, 0), "api")
    elif 102 in message:
        client = nested(message, 102)
        events = nested(client, 101)
        for wire, data in events.get(2, []):
            if wire != 2:
                raise ValueError("Invalid client event")
            event = protobuf(data)
            if fixed(event, 1, 4) == 1025:
                payload = protobuf(scalar(event, 3, 2, b""))
                add(
                    fixed(event, 2, 8),
                    floats(payload, 1),
                    scalar(payload, 2, 0, 0),
                    "api",
                )
    return samples
