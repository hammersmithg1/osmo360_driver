"""Native DN protocol vectors only; no USB, camera or network access."""

import unittest

from osmo360_camera import _app as app


# Captured in hardware041: QUERY32 returned this one APP, then EMPTY_REPLY
# after unregister. USB ACK was not observed; absence is independent evidence.
# Response records use type23, unlike the QUERY32 request nibble order.
OWN_REPLY = bytes.fromhex("12004400012300415050000000000002")
EMPTY_REPLY = bytes.fromhex("1200550000")


class AppProtocolTests(unittest.TestCase):
    def test_query_and_registration_native_vectors_have_different_nibble_order(self):
        self.assertEqual(app.query_payload(), bytes.fromhex("1100320001"))
        register = app.registration_payload()
        unregister = app.registration_payload(unregister=True)
        self.assertEqual(register, bytes.fromhex("170046237c415050000000000002"))
        self.assertEqual(unregister, bytes.fromhex("170047237c415050000000000002"))
        self.assertEqual(
            [
                i
                for i, pair in enumerate(zip(register, unregister))
                if pair[0] != pair[1]
            ],
            [2],
        )
        with self.assertRaises(TypeError):
            app.registration_payload(unregister=1)

    def test_documented_populated_reply_retains_all_native_fields(self):
        result = app.decode_query_reply(OWN_REPLY)
        self.assertEqual(
            (result.raw, result.action, result.reserved, result.count),
            (OWN_REPLY, 0x44, 0, 1),
        )
        (record,) = result.records
        self.assertEqual(record.raw, OWN_REPLY[5:])
        self.assertEqual(
            (
                record.type_raw,
                record.device_type,
                record.sub_type,
                record.reserved_type,
                record.host_name,
                record.host_id,
            ),
            (0x23, 3, 2, 0, b"APP\0\0\0\0\0", 2),
        )
        self.assertEqual(result.own_records, (record,))
        self.assertFalse(result.all_apps_absent)

    def test_only_canonical_other_empty_reply_proves_all_apps_absent(self):
        result = app.decode_query_reply(EMPTY_REPLY)
        self.assertTrue(result.all_apps_absent)
        self.assertEqual(
            (result.records, result.own_records, result.count), ((), (), 0)
        )
        for malformed in (
            b"\x12\0\x44\0\0",
            b"\x12\0\x55\1\0",
            b"\x12\0\x11\0\0",
            b"\x12\0\x55\0\1" + OWN_REPLY[5:],
        ):
            with (
                self.subTest(raw=malformed.hex()),
                self.assertRaises(app.AppProtocolError),
            ):
                app.decode_query_reply(malformed)

    def test_another_app_is_not_owned_and_is_not_global_absence(self):
        another = b"\x23\0ROBOT\0\0\0\x0a"
        result = app.decode_query_reply(b"\x12\0\x44\0\1" + another)
        self.assertEqual(result.records[0].host_name, b"ROBOT\0\0\0")
        self.assertFalse(result.records[0].is_own_app)
        self.assertFalse(result.all_apps_absent)
        self.assertEqual(result.own_records, ())

    def test_multiple_records_do_not_hide_duplicate_own_identity(self):
        result = app.decode_query_reply(b"\x12\0\x44\0\2" + OWN_REPLY[5:] * 2)
        self.assertEqual(len(result.own_records), 2)
        # A caller must not call this a unique owned registration.
        self.assertEqual(result.count, 2)

    def test_reserved_record_bits_are_retained_without_inventing_a_uuid(self):
        raw = bytearray(OWN_REPLY)
        raw[6] = 0x7C
        (record,) = app.decode_query_reply(raw).records
        self.assertEqual((record.type_raw, record.reserved_type), (0x7C23, 0x7C))
        self.assertTrue(record.is_own_app)

    def test_wrong_query_filter_or_own_metadata_cannot_prove_own_presence(self):
        wrong_filter = bytearray(OWN_REPLY)
        wrong_filter[5] = 0x32
        with self.assertRaises(app.AppProtocolError):
            app.decode_query_reply(wrong_filter)
        for index, value in ((7, ord("a")), (10, 32), (15, 10)):
            raw = bytearray(OWN_REPLY)
            raw[index] = value
            result = app.decode_query_reply(raw)
            self.assertEqual(result.own_records, ())
            self.assertFalse(result.all_apps_absent)

    def test_malformed_reply_lengths_headers_and_count_are_rejected(self):
        malformed = [
            b"",
            OWN_REPLY[:4],
            OWN_REPLY[:-1],
            OWN_REPLY + b"\0",
            b"\0" + OWN_REPLY,
            OWN_REPLY[:4] + b"\2" + OWN_REPLY[5:],
            b"\x13\0" + OWN_REPLY[2:],
            OWN_REPLY[:3] + b"\1" + OWN_REPLY[4:],
        ]
        for raw in malformed:
            with self.subTest(raw=raw.hex()), self.assertRaises(app.AppProtocolError):
                app.decode_query_reply(raw)

    def test_decoding_copies_mutable_wire_data(self):
        source = bytearray(OWN_REPLY)
        reply = app.decode_query_reply(memoryview(source))
        source[:] = bytes(len(source))
        self.assertEqual(reply.raw, OWN_REPLY)
        self.assertTrue(reply.records[0].is_own_app)

    def test_registration_ack_decodes_rejection_without_claiming_completion(self):
        accepted = app.decode_registration_reply(bytes.fromhex("18000000"))
        self.assertTrue(accepted.accepted)
        self.assertEqual(
            (accepted.raw, accepted.return_code), (bytes.fromhex("18000000"), 0)
        )
        rejected = app.decode_registration_reply(bytes.fromhex("18000500"))
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.return_code, 5)

    def test_registration_ack_correlation_and_length_are_strict(self):
        for raw in (b"", b"\x18\0\0", b"\0\x18\0\0\0", b"\x17\0\0\0", b"\x18\0\0\0\0"):
            with self.subTest(raw=raw.hex()), self.assertRaises(app.AppProtocolError):
                app.decode_registration_reply(raw)

    def test_non_wire_inputs_are_not_silently_converted_to_bytes(self):
        for decoder in (app.decode_query_reply, app.decode_registration_reply):
            for value in (None, 5, "1200550000", [0x12, 0, 0x55, 0, 0]):
                with (
                    self.subTest(decoder=decoder.__name__, value=value),
                    self.assertRaises(TypeError),
                ):
                    decoder(value)


if __name__ == "__main__":
    unittest.main()
