"""Pure codecs for native OQ101 temperature queries; no transport or hardware I/O.

The useful query is system 00/44, version1/subcommand15. Its native callback
reads cached temperature groups by name; it does not alter thermal control.
The wire layout follows retail firmware10.00.25.29.
"""

from dataclasses import dataclass
import struct
from typing import Iterable


SYSTEM_DESTINATION = 0x28
COMMAND_SET = 0x00
COMMAND_ID = 0x44
TEMPERATURE_SUBCOMMAND = 0x4F  # version1 in bits7:6, native read subcommand15
SOURCE_FIRMWARE = "OQ101 module10.00.25.29 (static analysis)"
MAX_GROUPS = 32


@dataclass(frozen=True)
class SensorMetadata:
    """A thermal group, which can represent several underlying channels."""

    query_name: str
    description: str
    channel_count: int | None
    native_node: str | None


# Short unique names deliberately match the native strstr(group_name, query).
# They remain useful if a firmware changes the suffix of a configured group.
SENSORS = {
    "ntc": SensorMetadata(
        "ntc", "OQ101 NTC thermistor group", 1, "/dev/thermal/ntc/OQ101"
    ),
    "qcs8550": SensorMetadata(
        "qcs8550", "QCS8550 SoC thermal group", 4, "/dev/thermal/tsens"
    ),
    "ddr": SensorMetadata("ddr", "DDR thermal group", 1, "/dev/thermal/ddr"),
    "sens": SensorMetadata("sens", "Image-sensor thermal group", None, None),
    "bat": SensorMetadata("bat", "Battery thermal group", 1, "/dev/thermal/battery"),
}
DEFAULT_SENSORS = tuple(SENSORS)


@dataclass(frozen=True)
class TemperatureReading:
    sensor: str
    raw_tenths_celsius: int

    @property
    def celsius(self) -> float:
        """Preserve native values, including any uninitialized/error sentinel."""
        return self.raw_tenths_celsius / 10.0


class TemperatureProtocolError(ValueError):
    def __init__(self, message: str, return_code: int | None = None):
        super().__init__(message)
        self.return_code = return_code


def _names(sensor_names: Iterable[str]) -> tuple[str, ...]:
    names = tuple(sensor_names)
    if not 1 <= len(names) <= MAX_GROUPS or len(set(names)) != len(names):
        raise ValueError("Choose1..32 distinct thermal group names")
    for name in names:
        if not isinstance(name, str):
            raise ValueError("Thermal group names must be strings")
        try:
            encoded = name.encode("ascii")
        except UnicodeEncodeError as error:
            raise ValueError("Thermal group names must be ASCII") from error
        if not 1 <= len(encoded) <= 31 or any(c < 0x21 or c > 0x7E for c in encoded):
            raise ValueError(
                "Thermal group names must be1..31 printable ASCII bytes without spaces"
            )
    return names


def _request(subcommand: int, sensor_names: Iterable[str]) -> bytes:
    names = _names(sensor_names)
    return bytes((subcommand, len(names))) + b"".join(
        bytes((index,)) + name.encode("ascii").ljust(32, b"\0")
        for index, name in enumerate(names)
    )


def build_temperature_request(sensor_names: Iterable[str] = DEFAULT_SENSORS) -> bytes:
    """Payload only, for a system-destination00/44 request with reply requested."""
    return _request(TEMPERATURE_SUBCOMMAND, sensor_names)


def _body(payload: bytes, names: tuple[str, ...]) -> bytes:
    if not payload:
        raise TemperatureProtocolError("Empty temperature response")
    if payload[0] != 0:
        raise TemperatureProtocolError(f"Camera returned0x{payload[0]:02x}", payload[0])
    if len(payload) != 2 + 2 * len(names) or payload[1] != len(names):
        raise TemperatureProtocolError("Temperature response count/length mismatch")
    return payload[2:]


def decode_temperature_response(
    payload: bytes,
    sensor_names: Iterable[str] = DEFAULT_SENSORS,
) -> tuple[TemperatureReading, ...]:
    """Decode ret,count,int16LE[count]; values are cached group readings in0.1°C.

    A nonzero return code (e.g. DF for a missing requested group) is an error;
    a partial native reply must never be presented as a successful reading.
    The protocol carries no sample timestamp or channel-level breakdown here.
    """
    names = _names(sensor_names)
    values = struct.unpack("<" + "h" * len(names), _body(payload, names))
    return tuple(TemperatureReading(name, value) for name, value in zip(names, values))
