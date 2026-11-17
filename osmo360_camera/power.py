"""Read-only logical system-mode codec; no USB, wake or power-off operations.

The native system00/44 v1 subcommand6 returns a fixed64-byte C-string buffer.
It describes logical state/scene, not proof of camera-frame or IMU production.
"""

from dataclasses import dataclass

from .controls import CameraReplyError, ControlDecodeError


SYSTEM_DESTINATION = 0x28
COMMAND_SET = 0
COMMAND_ID = 0x44


@dataclass(frozen=True)
class SystemMode:
    state: str
    scene: str
    raw: bytes


def build_system_mode_request() -> bytes:
    """Payload for system00/44: exact native read-only860000 query."""
    return bytes.fromhex("860000")


def decode_system_mode_response(payload: bytes) -> SystemMode:
    """Decode status+64-byte state/scene reply, retaining unknown names/padding."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("reply must be bytes-like")
    payload = bytes(payload)
    if not payload:
        raise ControlDecodeError("Empty system-mode reply")
    if payload[0]:
        raise CameraReplyError(payload[0], payload)
    if len(payload) != 65:
        raise ControlDecodeError("System-mode reply must be status+64 bytes")
    body = payload[1:]
    terminator = body.find(b"\0")
    if terminator < 0:
        raise ControlDecodeError("System-mode C string is not terminated")
    text = body[:terminator]
    if any(byte < 0x20 or byte > 0x7E for byte in text):
        raise ControlDecodeError("System-mode description is not printable ASCII")
    state, separator, scene = text.decode("ascii").partition(" - ")
    if not separator or not state or not scene:
        raise ControlDecodeError("Expected 'state - scene' system-mode description")
    # Real captured responses contain nonzero bytes after the first NUL.
    # Padding must not be decoded as text, required to be zero, or discarded.
    return SystemMode(state, scene, body)
