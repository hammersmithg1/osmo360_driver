"""Deterministic DIAG framing parity and malformed-stream recovery."""

import random
import struct
import unittest

from osmo360_camera._diag_collector import Decoder, checksum, frame


def reference_checksum(data):
    """Independent bitwise CRC16/X25, without crc_hqx or reversal tables."""
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value ^ 0xFFFF


def reference_frame(body, *, escape_all=False):
    data = body + struct.pack("<H", reference_checksum(body))
    # A literal 7E remains a delimiter even immediately after 7D; avoid that
    # invalid encoding when deliberately escaping otherwise ordinary bytes.
    return (
        b"".join(
            bytes((0x7D, byte ^ 0x20))
            if byte in (0x7D, 0x7E) or escape_all and byte ^ 0x20 != 0x7E
            else bytes((byte,))
            for byte in data
        )
        + b"\x7e"
    )


class ReferenceDecoder:
    """Original streaming state machine; deliberately retains per-byte loops."""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        while 0x7E in self.buffer:
            end = self.buffer.index(0x7E)
            encoded = bytes(self.buffer[:end])
            del self.buffer[: end + 1]
            decoded, escaped = bytearray(), False
            for byte in encoded:
                if escaped:
                    decoded.append(byte ^ 0x20)
                    escaped = False
                elif byte == 0x7D:
                    escaped = True
                else:
                    decoded.append(byte)
            if len(decoded) < 3 or escaped:
                continue
            body = bytes(decoded[:-2])
            if int.from_bytes(decoded[-2:], "little") != reference_checksum(body):
                self.bad_crc += 1
                continue
            yield body


class DiagFramingTests(unittest.TestCase):
    def test_x25_known_vector_and_all_byte_values(self):
        self.assertEqual(checksum(b""), 0)
        self.assertEqual(checksum(b"123456789"), 0x906E)
        self.assertEqual(frame(b"123456789"), b"123456789\x6e\x90\x7e")
        values = bytes(range(256))
        for data in (values, bytearray(values), memoryview(values), list(values)):
            self.assertEqual(checksum(data), reference_checksum(values))
        rng = random.Random(22)
        for size in (1, 2, 3, 31, 127, 256, 1024):
            for _ in range(5):
                data = rng.randbytes(size)
                self.assertEqual(checksum(data), reference_checksum(data))
                self.assertEqual(frame(data), reference_frame(data))

    def test_noncanonical_escapes_include_newline_and_paired_trailing_7d(self):
        body = b"\x5d\x7e" + bytes(range(256))
        encoded = reference_frame(body, escape_all=True)
        self.assertIn(b"\x7d\x0a", encoded)  # DOTALL: encodes the ordinary '*'.
        self.assertIn(b"\x7d\x7d\x7d\x5e", encoded)
        self.assertEqual(list(Decoder().feed(encoded)), [body])

        # A valid CRC whose final decoded byte is 5D can end in encoded7D7D.
        body = next(
            bytes((i,))
            for i in range(256)
            if reference_checksum(bytes((i,))) >> 8 == 0x5D
        )
        encoded = reference_frame(body, escape_all=True)
        self.assertTrue(encoded.endswith(b"\x7d\x7d\x7e"))
        self.assertEqual(list(Decoder().feed(encoded)), [body])

    def test_empty_short_and_dangling_frames_do_not_count_as_crc_errors(self):
        invalid = b"\x7e\x7e\x01\x7e" + reference_frame(b"")
        invalid += b"\x01\x02\x03\x7d\x7e\x7d\x7e\x7d\x7d\x7e"
        decoder = Decoder()
        self.assertEqual(list(decoder.feed(invalid)), [])
        self.assertEqual(decoder.bad_crc, 0)
        self.assertEqual(decoder.buffer, bytearray())
        # The escape before a raw delimiter does not escape that delimiter.
        self.assertEqual(list(decoder.feed(b"bad\x7d\x7e" + frame(b"next"))), [b"next"])
        self.assertEqual(decoder.bad_crc, 0)

    def test_chunk_boundaries_preserve_escape_and_incomplete_frame(self):
        values = [b"x", bytes(range(256)), b"\x5d\x7e\x7d\x2a"]
        wire = b"\x7e" + b"".join(reference_frame(x, escape_all=True) for x in values)
        wire += b"\x01\x02\x03\x7d\x7e" + b"incomplete\x7d"
        for size in (1, 2, 3, 7, 64, 4096):
            with self.subTest(chunk_size=size):
                decoder = Decoder()
                packets = []
                for i in range(0, len(wire), size):
                    packets.extend(decoder.feed(wire[i : i + size]))
                self.assertEqual(packets, values)
                self.assertEqual(decoder.bad_crc, 0)
                self.assertEqual(decoder.buffer, b"incomplete\x7d")

    def test_partial_generator_keeps_later_frames_in_the_buffer(self):
        decoder = Decoder()
        first, second = frame(b"first"), frame(b"second")
        pending = decoder.feed(first + second)
        self.assertEqual(decoder.buffer, b"")  # Generator has not started.
        self.assertEqual(next(pending), b"first")
        self.assertEqual(decoder.buffer, second)
        pending.close()
        self.assertEqual(list(decoder.feed(b"")), [b"second"])

    def test_bit_flips_count_crc_errors_and_next_frame_recovers(self):
        body = b"123456789"
        for byte_index in range(len(body) + 2):
            for bit in range(8):
                changed = bytearray(body + struct.pack("<H", reference_checksum(body)))
                changed[byte_index] ^= 1 << bit
                corrupted = (
                    b"".join(
                        bytes((0x7D, x ^ 0x20)) if x in (0x7D, 0x7E) else bytes((x,))
                        for x in changed
                    )
                    + b"\x7e"
                )
                decoder = Decoder()
                self.assertEqual(list(decoder.feed(corrupted + frame(b"ok"))), [b"ok"])
                self.assertEqual(decoder.bad_crc, 1)

    def test_random_stream_matches_original_state_at_each_chunk(self):
        rng = random.Random(42844)
        wire = (
            b"".join(
                rng.randbytes(113) + reference_frame(rng.randbytes(55), escape_all=True)
                for _ in range(40)
            )
            + b"\x7d"
        )
        for size in (1, 2, 31, 4096):
            with self.subTest(chunk_size=size):
                actual, expected = Decoder(), ReferenceDecoder()
                for i in range(0, len(wire), size):
                    chunk = wire[i : i + size]
                    self.assertEqual(
                        list(actual.feed(chunk)), list(expected.feed(chunk))
                    )
                    self.assertEqual(actual.bad_crc, expected.bad_crc)
                    self.assertEqual(actual.buffer, expected.buffer)


if __name__ == "__main__":
    unittest.main()
