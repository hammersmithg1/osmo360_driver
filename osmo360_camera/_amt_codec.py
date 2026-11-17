"""Pure AMT task-result/cancellation codecs and correlated reply decoding."""

from dataclasses import dataclass
import struct


def _task_id(value):
    if type(value) is not int or not 1 <= value <= 0xFFFFFFFF:
        raise ValueError("task_id must be an integer in 1..0xffffffff")
    return value


def result_payload(task_id):
    return struct.pack("<HI", 4, _task_id(task_id))


def stop_payload(task_id):
    return struct.pack("<HI", 2, _task_id(task_id))


@dataclass(frozen=True)
class Reply:
    subcommand: int
    status: int
    result: int | None
    raw: bytes


def decode_reply(payload, *, expected_subcommand):
    """No leading generic return byte: native uses resp_data_without_retcode."""
    if expected_subcommand not in (1, 3, 5):
        raise ValueError("expected response subcommand must be1,3,5")
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("reply must be bytes-like")
    payload = bytes(payload)
    expected_length = 3 if expected_subcommand == 1 else 4
    if len(payload) != expected_length:
        raise ValueError("invalid D1 reply length: " + payload.hex())
    subcommand, status = struct.unpack_from("<HB", payload)
    if subcommand != expected_subcommand:
        raise ValueError("uncorrelated D1 response subcommand: " + payload.hex())
    return Reply(subcommand, status, payload[3] if len(payload) == 4 else None, payload)
