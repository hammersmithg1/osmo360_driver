"""Pure OQ101 device-notify codec for our APP preview registration.

No transport or camera I/O. Values follow the matching25.29 native headers
and handlers.
An accepted registration ACK only means queued, so verify state with QUERY.
"""

from dataclasses import dataclass
import struct


class AppProtocolError(ValueError):
    """A reply cannot safely establish the expected APP state."""


def query_payload():
    """DN0011: query SURFACE3/APP2, user_type1. QUERY uses reversed nibbles."""
    return bytes.fromhex("1100320001")


def registration_payload(*, unregister=False):
    """Preserve the proven14byte identity; change only the dev_reg bit.

    The bytes46/23/7c/41 are flags/type/reserved/name, not a session UUID.
    This describes APP source02 -> system28; this codec sends nothing.
    """
    if type(unregister) is not bool:
        raise TypeError("unregister must be boolean")
    return bytes.fromhex(
        "170047237c415050000000000002" if unregister else "170046237c415050000000000002"
    )


def _bytes(payload):
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload must be bytes-like")
    return bytes(payload)


@dataclass(frozen=True)
class SurfaceRecord:
    raw: bytes
    type_raw: int
    device_type: int
    sub_type: int
    reserved_type: int
    host_name: bytes
    host_id: int

    @property
    def is_own_app(self):
        # Reserved bits have no identity meaning; keep them in the raw fields.
        return (
            self.device_type == 3
            and self.sub_type == 2
            and self.host_name == b"APP" + bytes(5)
            and self.host_id == 2
        )


@dataclass(frozen=True)
class QueryReply:
    raw: bytes
    action: int
    reserved: int
    count: int
    records: tuple[SurfaceRecord, ...]

    @property
    def own_records(self):
        """Callers require exactly one before unregister, then none afterwards."""
        return tuple(record for record in self.records if record.is_own_app)

    @property
    def all_apps_absent(self):
        """Canonical OTHER55/count0, distinct from an unrelated APP remaining."""
        return self.action == 0x55


def decode_query_reply(payload):
    """Decode only the exact QUERY32 response; reject uncertain absence.

    Header: u16command12, u8action, u8reserved, u8count. Each APP record is
    u16type/subtype, name8, host_id1. There is no generic leading return byte.
    """
    raw = _bytes(payload)
    if len(raw) < 5 or raw[:2] != b"\x12\0":
        raise AppProtocolError("Expected DN0012 APP query header")
    action, reserved, count = raw[2:5]
    if reserved != 0 or action not in (0x44, 0x55):
        raise AppProtocolError("Unknown APP query status/reserved field")
    if len(raw) != 5 + 11 * count:
        raise AppProtocolError("APP query count does not match11byte records")
    if (action == 0x55) != (count == 0):
        raise AppProtocolError("Inconsistent APP query action/count")
    records = []
    for offset in range(5, len(raw), 11):
        row = raw[offset : offset + 11]
        type_raw = struct.unpack_from("<H", row)[0]
        device_type, sub_type = type_raw & 15, (type_raw >> 4) & 15
        if (device_type, sub_type) != (3, 2):
            raise AppProtocolError(
                "Record does not match the requested SURFACE3/APP2 filter"
            )
        records.append(
            SurfaceRecord(
                row, type_raw, device_type, sub_type, type_raw >> 8, row[2:10], row[10]
            )
        )
    return QueryReply(raw, action, reserved, count, tuple(records))


@dataclass(frozen=True)
class RegistrationReply:
    raw: bytes
    return_code: int

    @property
    def accepted(self):
        """Queued acceptance only; this does not prove removal or capture."""
        return self.return_code == 0


def decode_registration_reply(payload):
    """DN0018 ACK: exactly u16command18 +u16return code, without a prefix."""
    raw = _bytes(payload)
    if len(raw) != 4 or raw[:2] != b"\x18\0":
        raise AppProtocolError("Expected four-byte DN0018 registration ACK")
    return RegistrationReply(raw, struct.unpack_from("<H", raw, 2)[0])
