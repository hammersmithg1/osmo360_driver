"""Private collector→IMUStream wire format, unrelated to SSC sensor decoding.

Version1 fixes SI units and the 19.2MHz camera clock. Float64 preserves the
already-decoded SSC numbers exactly; SUID bytes preserve the existing128 bits.
External collector UDP remains JSON unless binary is explicitly selected.
"""

from functools import lru_cache
import math
import struct


MAGIC = b"OSMOIMU\x01"
BINARY_PREFIX = MAGIC[:-1]
TICK_HZ = 19_200_000
PACKET = struct.Struct("<8sQQ6dII16s16s")
AXES = ("ax", "ay", "az", "gx", "gy", "gz")


@lru_cache(maxsize=32)
def _suid_bytes(value):
    raw = bytes.fromhex(value)
    if len(raw) != 16:
        raise ValueError("SUID must contain exactly128 bits")
    return raw


@lru_cache(maxsize=32)
def _suid_hex(value):
    return value.hex()


def pack_sample(record):
    """Encode a validated paired record; uint bounds and all axes are strict."""
    if (
        record.get("sensor_type") != "imu"
        or record.get("tick_hz") != TICK_HZ
        or record.get("accel_unit") != "m/s^2"
        or record.get("gyro_unit") != "rad/s"
    ):
        raise ValueError("Binary IMU requires paired SI samples at19.2MHz")
    integers = []
    for name, limit in (
        ("timestamp_ticks", 1 << 64),
        ("host_receive_monotonic_ns", 1 << 64),
        ("accel_status", 1 << 32),
        ("gyro_status", 1 << 32),
    ):
        value = record.get(name)
        if type(value) is not int or not 0 <= value < limit:
            raise ValueError("Invalid binary IMU " + name)
        integers.append(value)
    axes = []
    for name in AXES:
        value = record.get(name)
        if type(value) not in (int, float):
            raise ValueError("Invalid binary IMU " + name)
        try:
            value = float(value)
        except OverflowError as error:
            raise ValueError("Invalid binary IMU " + name) from error
        if not math.isfinite(value):
            raise ValueError("Non-finite binary IMU " + name)
        axes.append(value)
    suids = []
    for name in ("accel_suid", "gyro_suid"):
        value = record.get(name)
        if type(value) is not str or len(value) != 32:
            raise ValueError("Invalid binary IMU " + name)
        suids.append(_suid_bytes(value))
    tick, host, accel_status, gyro_status = integers
    return PACKET.pack(MAGIC, tick, host, *axes, accel_status, gyro_status, *suids)


def unpack_sample(data):
    """Return IMUSample fields, rejecting unknown versions, sizes and NaN/Inf."""
    if len(data) != PACKET.size:
        raise ValueError("Invalid binary IMU datagram length")
    magic, tick, host, ax, ay, az, gx, gy, gz, astatus, gstatus, asuid, gsuid = (
        PACKET.unpack(data)
    )
    if magic != MAGIC:
        raise ValueError("Unknown binary IMU magic/version")
    if not all(math.isfinite(value) for value in (ax, ay, az, gx, gy, gz)):
        raise ValueError("Non-finite binary IMU axis")
    return dict(
        timestamp_ticks=tick,
        host_receive_monotonic_ns=host,
        ax=ax,
        ay=ay,
        az=az,
        gx=gx,
        gy=gy,
        gz=gz,
        accel_status=astatus,
        gyro_status=gstatus,
        accel_suid=_suid_hex(asuid),
        gyro_suid=_suid_hex(gsuid),
        tick_hz=TICK_HZ,
    )
