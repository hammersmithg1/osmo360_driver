"""Typed RAW SSC decoding for the private binary collector hot path.

The public dictionary/JSON decoder stays in _ssc. This parser preserves its
wire validation, scalar-last semantics and float32 values, but avoids building
protobuf dictionaries/lists for every 8 kHz sensor event. No parse error falls
back to a more permissive decoder. Non-RAW records explicitly use the public
validator because they are outside this optimized path.
"""
from functools import lru_cache
import math
import struct
from ._ssc import decode_diag_packet, varint


def _fields(data):
    pos, size = 0, len(data)
    while pos < size:
        key = data[pos]
        pos += 1
        if key >= 128:
            key, pos = varint(data, pos - 1)
        tag, wire = key >> 3, key & 7
        if not tag:
            raise ValueError('Zero protobuf field')
        if wire == 0:
            if pos >= size:
                raise ValueError('Truncated varint')
            value = data[pos]
            pos += 1
            if value >= 128:
                value, pos = varint(data, pos - 1)
        elif wire in (1, 2, 5):
            if wire == 2:
                if pos >= size:
                    raise ValueError('Truncated length')
                length = data[pos]
                pos += 1
                if length >= 128:
                    length, pos = varint(data, pos - 1)
            else:
                length = 8 if wire == 1 else 4
            end = pos + length
            if end > size:
                raise ValueError('Truncated protobuf field')
            value, pos = data[pos:end], end
        else:
            raise ValueError('Unsupported protobuf wire')
        yield tag, wire, value


# Camera sensor identity is immutable; cap the cache for foreign/mutated logs.
@lru_cache(maxsize=32)
def _suid(data):
    low, high = b'\0' * 8, b'\0' * 8
    low_wire = high_wire = 1
    for tag, wire, value in _fields(data):
        if tag == 1:
            low_wire, low = wire, value
        elif tag == 2:
            high_wire, high = wire, value
    if low_wire != 1 or high_wire != 1:
        raise ValueError('Invalid SUID wire')
    # Same two fixed64 words/order as _ssc.uid(), without integer formatting.
    return high[::-1].hex() + low[::-1].hex()


def _raw_sample(data):
    # Exactly: batch-type(varint), ticks(fixed64), XYZ(12 packed bytes),
    # status(varint). Every byte is accounted for. Other layouts, including
    # repeated/unknown fields, use the strict scanner below.
    if (len(data) == 27 and data[0] == 8 and data[1] < 128
            and data[2] == 17 and data[11:13] == b'\x1a\x0c'
            and data[25] == 32 and data[26] < 128):
        tick = struct.unpack_from('<Q', data, 3)[0]
        xyz = struct.unpack_from('<3f', data, 13)
        status = data[26]
    else:
        tick = status = 0
        tick_wire, status_wire, batch_wire = 1, 0, 0
        xyz = []
        for tag, wire, value in _fields(data):
            if tag == 1:
                batch_wire = wire
            elif tag == 2:
                tick_wire, tick = wire, value
            elif tag == 4:
                status_wire, status = wire, value
            elif tag == 3:
                if wire == 5:
                    xyz.append(struct.unpack('<f', value)[0])
                elif wire == 2 and len(value) % 4 == 0:
                    xyz.extend(struct.unpack('<' + 'f' * (len(value) // 4), value))
                else:
                    raise ValueError('Invalid float wire')
        if tick_wire != 1 or status_wire != 0 or batch_wire != 0:
            raise ValueError('Invalid raw scalar wire')
        if isinstance(tick, (bytes, bytearray)):
            tick = int.from_bytes(tick, 'little')
        if len(xyz) != 3:
            return None
    if not (math.isfinite(xyz[0]) and math.isfinite(xyz[1]) and math.isfinite(xyz[2])):
        raise ValueError('Nonfinite XYZ')
    return tick, xyz[0], xyz[1], xyz[2], status


def decode_raw_xyz_packet(body):
    """Return (sensor, ticks, x, y, z, status, suid) tuples, no I/O.

    Only RAW accel/gyro are returned. Other sensor RAW records are validated
    before filtering, just like the generic collector. All timestamps remain
    integers; axes retain the source float32 values without transformation.
    """
    if len(body) < 16 or body[0] != 0x10:
        return []
    code = struct.unpack_from('<H', body, 6)[0]
    if code not in (0x19c7, 0x19c9, 0x19ca, 0x19cb):
        return []
    length = struct.unpack_from('<H', body, 4)[0]
    if length < 12 or length + 4 > len(body):
        raise ValueError('Truncated DIAG log')
    sensor, sensor_wire = b'', 2
    uid, uid_wire = b'', 2
    raw, raw_wire = None, None
    log_wire = instance_wire = 1
    for tag, wire, value in _fields(body[16:4 + length]):
        if tag == 4:
            sensor_wire, sensor = wire, value
        elif tag == 3:
            uid_wire, uid = wire, value
        elif tag == 2:
            log_wire = wire
        elif tag == 5:
            instance_wire = wire
        elif tag == 103:
            raw_wire, raw = wire, value
    if sensor_wire != 2 or uid_wire != 2 or log_wire != 1 or instance_wire != 1:
        raise ValueError('Invalid SSC metadata wire')
    identity = _suid(bytes(uid))
    if raw_wire is None:
        # Explicit non-RAW path, never exception-based recovery of a bad RAW.
        decode_diag_packet(body)
        return []
    if raw_wire != 2:
        raise ValueError('Invalid RAW batch wire')
    result = []
    sensor_name = sensor.decode('utf8', errors='replace')
    wanted = sensor_name in ('accel', 'gyro')
    for tag, wire, value in _fields(raw):
        if tag == 1:
            if wire != 2:
                raise ValueError('Invalid raw sample wire')
            value = _raw_sample(value)
            if wanted and value is not None:
                result.append((sensor_name, *value, identity))
    return result
