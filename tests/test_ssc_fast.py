"""Typed collector decoder must agree with the independent generic SSC path."""

import itertools
import json
from pathlib import Path
import random
import struct
import tempfile
import unittest
from unittest.mock import patch

from osmo360_camera import _diag_collector as collector
from osmo360_camera import _ssc_fast as fast
from osmo360_camera._ssc import decode_diag_packet
from test_ssc_decoder import field, fixed, packet
import test_imu_live_status as integration


def body(message):
    header = bytearray(16)
    header[0] = 0x10
    struct.pack_into('<HH', header, 4, len(message) + 12, 0x19cb)
    return bytes(header) + message


def message(samples, *, extra=b'', uid=None):
    if uid is None:
        uid = fixed(1, 123) + fixed(2, 456)
    return (fixed(2, 999) + field(3, 2, uid) + field(4, 2, b'accel') + fixed(5, 42)
            + field(103, 2, b''.join(field(1, 2, value) for value in samples)) + extra)


def cases():
    parts = [field(1, 0, 1), fixed(2, 2**53 + 1),
             field(3, 2, struct.pack('<3f', 1.25, -0.0, 9.75)), field(4, 0, 3)]
    values = [packet(sensor, [(10, (1.25, -2.5, 9.75))], kind=kind)
              for sensor in ('accel', 'gyro', 'other') for kind in ('raw', 'api', 'client')]
    values += [body(message([b''.join(layout)])) for layout in itertools.permutations(parts)]
    for xyz in ([], [1.25], [1.25, 2.5], [1.25, 2.5, -0.0], [1., 2., 3., 4.],
                [float('nan'), 2., 3.], [float('inf'), 2., 3.], [float('nan'), 2.]):
        for packed in (True, False):
            floats = (field(3, 2, struct.pack('<' + 'f' * len(xyz), *xyz)) if packed
                      else b''.join(field(3, 5, struct.pack('<f', f)) for f in xyz))
            for batch, tick, status in [
                (b'', b'', b''), (parts[0], parts[1], parts[3]),
                (field(1, 2, b'bad') + parts[0], parts[1], parts[3]),
                (parts[0], parts[1] + field(2, 0, 1), parts[3]),
                (parts[0], parts[1], field(4, 1, b'\0' * 8) + parts[3]),
            ]:
                values.append(body(message([batch + tick + floats + status])))
    for extra in (field(7, 0, 2**65), field(7, 1, b'abcdefgh'), field(7, 2, b'anything'),
                  field(7, 5, b'abcd'), b'\x00', b'\x3b', b'\xff' * 12):
        values += [body(message([b''.join(parts) + extra])),
                   body(message([b''.join(parts)], extra=extra))]
    for uid in (b'', fixed(2, 456) + fixed(1, 123), field(1, 0, 2),
                field(1, 0, 2) + fixed(1, 123), fixed(1, 123) + field(7, 2, b'text'), b'\x00'):
        values.append(body(message([b''.join(parts)], uid=uid)))
    for size in (0, 1, 3, 7, 8, 16):
        values.append(body(message([parts[0] + parts[1] + field(3, 2, b'x' * size) + parts[3]])))
    # Repeated packed and unpacked XYZ fields concatenate, rather than last-win.
    values.append(body(message([parts[0] + parts[1] + field(3, 5, struct.pack('<f', 1.))
                               + field(3, 2, struct.pack('<2f', -0., 3.)) + parts[3]])))
    # Only the final RAW batch is decoded, as with nested(scalar(...)).
    values.append(body(message([b'\x00'], extra=field(103, 2, field(1, 2, b''.join(parts))))))
    return values


def reference(data):
    return [(v['sensor_type'], v['timestamp_ticks'], v['x'], v['y'], v['z'], v['status'], v['suid'])
            for v in decode_diag_packet(data)
            if v['kind'] == 'raw' and v['sensor_type'] in ('accel', 'gyro')]


class FastSSCDecoderTests(unittest.TestCase):
    def agrees(self, data):
        results = []
        for decoder in (reference, fast.decode_raw_xyz_packet):
            try:
                # Equality includes signed zero and every preserved float bit.
                results.append(('ok', [(sensor, tick, struct.pack('<3d', x, y, z), status, uid)
                                       for sensor, tick, x, y, z, status, uid in decoder(data)]))
            except ValueError:
                results.append(('invalid',))
        self.assertEqual(results[0], results[1], data.hex())

    def test_field_order_duplicates_packed_unknown_and_nonfinite_semantics(self):
        for data in cases():
            self.agrees(data)

    def test_native_usb_fixture_matches_saved_generic_fields_and_raw_bits(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/ssc_raw_native.json').read_text())
        self.assertEqual(fixture['schema'], 1)
        self.assertEqual(len(fixture['raw_sha256']), 64)
        self.assertGreater(fixture['raw_bytes'], 0)
        self.assertEqual(len(fixture['packets']), 64)
        sensors = []
        for record in fixture['packets']:
            data = bytes.fromhex(record['body_hex'])
            # Frozen capture output, independent of both current decoders.
            self.assertEqual(decode_diag_packet(data), record['decoded'])
            self.agrees(data)
            sensors.extend(value[0] for value in fast.decode_raw_xyz_packet(data))
        self.assertEqual(sensors.count('accel'), 32)
        self.assertEqual(sensors.count('gyro'), 32)

    def test_every_prefix_is_validated_without_partial_output(self):
        for data in cases():
            for size in range(len(data) + 1):
                self.agrees(data[:size])
                if size >= 16:
                    # Also make the outer DIAG length valid, so truncation
                    # reaches every inner protobuf boundary and scalar.
                    shortened = bytearray(data[:size])
                    struct.pack_into('<H', shortened, 4, size - 4)
                    self.agrees(bytes(shortened))

    def test_deterministic_single_and_multiple_byte_corruption(self):
        samples = cases()
        for data in samples[:12]:
            for pos in range(len(data)):
                for byte in (0, 1, 7, 8, 16, 17, 26, 32, 127, 128, 255):
                    changed = bytearray(data)
                    changed[pos] = byte
                    self.agrees(bytes(changed))
        rng = random.Random(84520)
        for _ in range(15000):
            changed = bytearray(rng.choice(samples))
            for _ in range(rng.randrange(1, 4)):
                changed[rng.randrange(len(changed))] = rng.randrange(256)
            self.agrees(bytes(changed))

    def test_raw_parse_failure_never_falls_back_and_nonraw_explicitly_validates(self):
        valid = packet('gyro', [(2**53 + 1, (1.25, -0.0, 9.75))])
        invalid = body(message([b'\x00']))
        with patch.object(fast, 'decode_diag_packet', side_effect=AssertionError('RAW fallback')):
            self.assertEqual(fast.decode_raw_xyz_packet(valid)[0][1], 2**53 + 1)
            with self.assertRaises(ValueError):
                fast.decode_raw_xyz_packet(invalid)
        with patch.object(fast, 'decode_diag_packet', wraps=decode_diag_packet) as generic:
            self.assertEqual(fast.decode_raw_xyz_packet(packet('gyro', [(1, (1., 2., 3.))], kind='api')), [])
        self.assertEqual(generic.call_count, 1)

    def test_sensor_identity_cache_is_bounded_and_preserves_raw_ids(self):
        fast._suid.cache_clear()
        for identity in range(100):
            raw = body(message([field(3, 2, struct.pack('<3f', 1., 2., 3.))],
                               uid=fixed(1, identity) + fixed(2, 456)))
            self.agrees(raw)
        self.assertEqual(fast._suid.cache_info().maxsize, 32)
        self.assertLessEqual(fast._suid.cache_info().currsize, 32)

    def test_large_batches_all_sensor_codes_and_bytearrays_preserve_every_sample(self):
        samples = [(2**53 + n * 2400, (1.25, -0., -3.5)) for n in range(1000)]
        for sensor in ('accel', 'gyro', 'unselected'):
            for code in (0x19c7, 0x19c9, 0x19ca, 0x19cb, 0x19cd):
                data = packet(sensor, samples, code=code)
                self.agrees(data)
                self.assertEqual(fast.decode_raw_xyz_packet(bytearray(data)), reference(data))
        # Noncanonical scalar varints and repeated fields use the same strict
        # parser, with no assumption that a USB/SSC packet has one sample.
        for data in cases():
            self.agrees(bytearray(data))

    def test_real_collector_binary_matches_generic_diagnostic_path_byte_for_byte(self):
        run = integration.LiveCollectorTests.run_collector
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(collector, 'decode_raw_xyz_packet', wraps=fast.decode_raw_xyz_packet) as typed:
                fast_result, _, fast_udp, _, fast_report = run(self, directory)
        self.assertGreater(typed.call_count, 0)
        parse = collector.argparse.ArgumentParser.parse_args
        with tempfile.TemporaryDirectory() as directory:
            def diagnostic(parser, *args, **kwargs):
                options = parse(parser, *args, **kwargs)
                options.samples_jsonl = Path(directory) / 'samples.jsonl'
                return options

            with patch.object(collector.argparse.ArgumentParser, 'parse_args', diagnostic), patch.object(
                collector, 'decode_raw_xyz_packet', side_effect=AssertionError('JSON path changed')
            ):
                result, _, udp, _, report = run(self, directory)
            self.assertEqual(len((Path(directory) / 'samples.jsonl').read_text().splitlines()), 1000)
        self.assertEqual(fast_udp.sent, udp.sent)
        self.assertEqual(fast_result, result)
        self.assertEqual(fast_report['restored'], report['restored'])
        for key in ('paired_samples', 'udp_sent', 'udp_drops', 'sample_decode_errors'):
            self.assertEqual(fast_report['collector_live'][key], report['collector_live'][key])


if __name__ == '__main__':
    unittest.main()
