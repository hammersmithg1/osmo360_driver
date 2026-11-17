"""Private, pure stock DUML diagnostic-file pull; no USB owner or CLI."""

import hashlib
from pathlib import PurePosixPath
import struct

HARD_MAX = 4 * 1024 * 1024
PACK_SIZE = 512
FILE_COMMAND = (0, 0x2A)


def checked_path(path):
    """Use literal absolute paths; never normalize traversal or expand a shell."""
    data = path.encode("utf-8")
    if (
        not path.startswith("/")
        or "\x00" in path
        or ".." in path
        or not 1 <= len(data) <= 127
    ):
        raise ValueError("Path must be absolute, 1..127 UTF-8 bytes, without NUL or ..")
    return data


class PullReceiver:
    """Pure packet state machine; emits ACK messages, never remote write requests.

    Native sender sets window/pack/checksum from its metadata ACK. A one-packet
    window makes an empty lost-list ACK (return_code + highest seq) sufficient.
    Only an exact requested basename and the chosen transport property are accepted.
    Encrypted mode saves verified ciphertext; it does not claim to decrypt DJI logs.
    """

    def __init__(
        self,
        path,
        sender=0x0A,
        receiver=0x28,
        max_bytes=HARD_MAX,
        metadata_only=False,
        encrypted=False,
    ):
        checked_path(path)
        if not 1 <= max_bytes <= HARD_MAX:
            raise ValueError("max_bytes must be 1..4 MiB")
        self.path, self.sender, self.receiver = path, sender, receiver
        self.max_bytes, self.metadata_only = max_bytes, metadata_only
        self.encrypted = encrypted
        self.metadata = None
        self.data = bytearray()
        self.next_sequence = 0
        self.done = False
        self.checksum_verified = False
        self.error = None
        self.seen_metadata = False

    def request_payload(self):
        path = checked_path(self.path)
        return (
            bytes([8, len(path)]) + path + (b"\x00\x01\x04" if self.encrypted else b"")
        )

    def _reject(self, message, reason):
        self.error = reason
        # D6 is a native transfer failure response. It declines this read session.
        return [message.answer(b"\xd6")]

    def feed(self, message):
        if (
            message.command != FILE_COMMAND
            or message.is_reply
            or message.sender != self.receiver
            or message.receiver != self.sender
        ):
            return []
        if not message.payload:
            return self._reject(message, "Empty transfer request")
        if self.error:
            return [message.answer(b"\xd6")]
        op, payload = message.payload[0], message.payload
        if op in (1, 11):
            self.seen_metadata = True
            width = 4 if op == 1 else 8
            if len(payload) < 2 + width:
                return self._reject(message, "Truncated metadata")
            size = int.from_bytes(payload[1 : 1 + width], "little")
            length = payload[1 + width]
            end = 2 + width + length
            if length == 0 or end > len(payload):
                return self._reject(message, "Invalid metadata filename length")
            name = payload[2 + width : end].decode("utf-8", errors="replace")
            if name != PurePosixPath(self.path).name:
                return self._reject(message, f"Unexpected remote basename: {name!r}")
            # The stock sender adds NUL, extension length=1, and property byte.
            extra = payload[end:]
            valid_extra = (
                (b"\x00\x01\x05",)
                if self.encrypted
                else (b"", b"\x00", b"\x00\x01\x01")
            )
            if extra not in valid_extra:
                return self._reject(
                    message, f"Unsupported metadata extension: {extra.hex()}"
                )
            metadata = {
                "operation": op,
                "size": size,
                "name": name,
                "extension_hex": extra.hex(),
                "encrypted_transport": self.encrypted,
            }
            if self.metadata is not None and metadata != self.metadata:
                return self._reject(message, "Metadata changed during transfer")
            self.metadata = metadata
            if self.metadata_only:
                # Do not grant a data window: this stage observes metadata only.
                self.done = True
                return [message.answer(b"\xd6")]
            if not 0 < size <= self.max_bytes:
                return self._reject(
                    message, f"Remote size {size} exceeds 1..{self.max_bytes}"
                )
            return [message.answer(b"\x00" + struct.pack("<HHBB", PACK_SIZE, 1, 1, 0))]
        if op in (2, 4):
            if self.metadata is None or len(payload) < 6:
                return self._reject(
                    message, "Data arrived without metadata or sequence"
                )
            sequence = struct.unpack_from("<I", payload, 1)[0]
            body = payload[5:]
            if sequence > self.next_sequence:
                return self._reject(
                    message,
                    f"Out-of-order sequence {sequence}; expected {self.next_sequence}",
                )
            offset = sequence * PACK_SIZE
            expected = min(PACK_SIZE, self.metadata["size"] - offset)
            if expected <= 0 or len(body) != expected:
                return self._reject(
                    message,
                    f"Unexpected data length {len(body)} at offset {offset}; expected {expected}",
                )
            if sequence < self.next_sequence:
                if self.data[offset : offset + expected] != body:
                    return self._reject(message, "Conflicting duplicate data")
            else:
                if len(self.data) + len(body) > self.max_bytes:
                    return self._reject(message, "Host receive cap exceeded")
                self.data.extend(body)
                self.next_sequence += 1
            return [message.answer(b"\x00" + struct.pack("<I", self.next_sequence - 1))]
        if op == 3:
            if self.metadata is None or len(self.data) != self.metadata["size"]:
                return self._reject(message, "Checksum arrived before complete file")
            if len(payload) != 17:
                return self._reject(message, "Expected op03 plus 16-byte MD5")
            if hashlib.md5(self.data).digest() != payload[1:]:
                return self._reject(message, "Remote MD5 mismatch")
            self.checksum_verified = self.done = True
            return [message.answer(b"\x00")]
        return self._reject(message, f"Unexpected camera transfer operation 0x{op:02x}")

    def result(self):
        return {
            "metadata": self.metadata,
            "received_bytes": len(self.data),
            "complete": self.done,
            "checksum_verified": self.checksum_verified,
            "encrypted_transport": self.encrypted,
            "sha256": hashlib.sha256(self.data).hexdigest()
            if self.checksum_verified
            else None,
            "error": self.error,
        }
