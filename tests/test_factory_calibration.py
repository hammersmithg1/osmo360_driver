"""Factory SEI ABI, opaque-error isolation, and camera-free export regressions."""

from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera.calibration import (
    CalibrationDecodeError, decode_factory_calibration, uniform_resize_seed,
)
from osmo360_camera.vendor.preview_metadata import records_from_video
from osmo360_driver.calibration import extract_calibration, main
from osmo360_driver.runtime import OsmoCamera


FIXTURE = json.loads((Path(__file__).parent / 'fixtures/preview_sei_native.json').read_text())
HEADER_SHA256 = 'b91d33e01871062ce7c630a70fcb6dfaf05de35624ea73606637707626e267e6'
INTRINSIC_KEYS = {'fx', 'fy', 'cx', 'cy', *(f'k{i}' for i in range(1, 10)), 'p1', 'p2', 'xi', 's'}


def captured_nal():
    return b'\0\0\0\1' + bytes.fromhex(FIXTURE['current'][0]['nal_hex'])


def captured_row():
    return next(records_from_video(captured_nal()))


def captured_value():
    return bytes.fromhex(captured_row()['calibration_value_hex'])


def payload_size(value):
    return b'\xff' * (value // 255) + bytes([value % 255])


def sei(tlvs):
    # Ordinary SEI escaping, independent of the camera-specific field decoder.
    rbsp = payload_size(245) + payload_size(len(tlvs)) + tlvs + b'\x80'
    escaped = bytearray()
    zeros = 0
    for byte in rbsp:
        if zeros >= 2 and byte <= 3:
            escaped.append(3)
            zeros = 0
        escaped.append(byte)
        zeros = zeros + 1 if byte == 0 else 0
    return b'\0\0\0\1\x06' + escaped


def replace_calibration(value):
    """Keep the captured motion TLV byte-for-byte while changing tag13 only."""
    original = bytes.fromhex(captured_row()['raw_tlv_hex'])
    result = bytearray()
    cursor = 0
    while cursor < len(original):
        tag, length = struct.unpack_from('<HH', original, cursor)
        old = original[cursor + 4:cursor + 4 + length]
        selected = value if tag == 13 else old
        result.extend(struct.pack('<HH', tag, len(selected)) + selected)
        cursor += 4 + length
    return sei(result)


def synthetic_v2(stream_type=2):
    """Independently pack the native v2 offsets, not a resized v1 prefix."""
    body = bytearray(287)
    body[0] = stream_type
    body[1:137] = captured_value()[6:142]
    struct.pack_into('<11f', body, 137, 11, 12, 13,
                     0.5, 0.25, -0.5, 0.75, 1, 2, 3, 4)
    struct.pack_into('<11f', body, 181, 21, 22, 23,
                     -0.5, 0.5, 0.25, -0.25, 5, 6, 7, 8)
    struct.pack_into('<QHf8I2fIf', body, 225,
                     0x0102030405060708, 1000, 59.94,
                     0xFFFFFFFF, 0x80000001, 4000, 3000, 17, 19, 41, 43,
                     3840, 4096, 99, -12.5)
    return struct.pack('<BI', 2, 287) + body


class FactoryCalibrationTests(unittest.TestCase):
    def test_current_native_v1_golden_fields_and_original_bytes(self):
        value = captured_value()
        decoded = decode_factory_calibration(value)
        self.assertEqual((decoded['version'], decoded['declared_body_bytes'], decoded['actual_body_bytes']),
                         (1, 219, 287))
        self.assertEqual(decoded['layout_header_sha256'], HEADER_SHA256)
        self.assertEqual(decoded['original_value_hex'], value.hex())
        self.assertEqual(decoded['payload_sha256'], hashlib.sha256(value).hexdigest())
        self.assertEqual((decoded['stream_frame_type'], decoded['stream_frame_type_name']), (1, 'dual_fisheye'))
        self.assertEqual(decoded['calibration_image_size'], [3840., 3840.])
        self.assertEqual(decoded['sensor_image_size'], [1920, 1920])
        self.assertEqual(decoded['rolling_shutter_grid'], [1920, 1920])
        self.assertEqual(decoded['panorama_grid'], [480, 240])
        self.assertEqual(decoded['cam_lens_type'], 8)
        self.assertEqual(decoded['unused_union_tail_hex'], bytes(68).hex())
        self.assertIsNone(decoded['temperature'])
        first, second = decoded['lenses']
        self.assertEqual((first['native_id'], second['native_id']), ('intrinsic00', 'intrinsic01'))
        self.assertEqual(set(first['intrinsic']), INTRINSIC_KEYS)
        self.assertEqual(set(second['intrinsic']), INTRINSIC_KEYS)
        self.assertEqual(first['intrinsic']['fx'], 1049.9498291015625)
        self.assertEqual(first['intrinsic']['fy'], 1049.541748046875)
        self.assertEqual(first['intrinsic']['cx'], 1917.8426513671875)
        self.assertEqual(first['intrinsic']['cy'], 1921.01611328125)
        self.assertEqual(first['intrinsic']['k5'], 0.0011231499956920743)
        self.assertEqual(first['intrinsic']['p1'], -0.0004904349916614592)
        self.assertEqual(first['intrinsic']['p2'], 0.001168790040537715)
        self.assertEqual(second['intrinsic']['fx'], 1053.026123046875)
        self.assertEqual(second['intrinsic']['cx'], 1911.816650390625)
        self.assertEqual(first['extrinsic'], dict(yaw=-179.976806640625,
            pitch=90.6150131225586, roll=-0.8166033625602722))
        self.assertEqual(second['extrinsic'], dict(yaw=0.007939563132822514,
            pitch=90.78720092773438, roll=-0.4377053380012512))

    def test_header_proven_v1_tail_keeps_itd_unsigned_and_units_unverified(self):
        timing = decode_factory_calibration(captured_value())['timing']
        self.assertEqual(timing, dict(read_out_time=23955120, imu_sampling_rate=1000,
            sensor_frame_rate=29.985559463500977, itd_value_u32=4294967029,
            lro_value_u32=4753, units_verified=False))

    def test_legacy_219_byte_v1_body_has_distinct_real_intrinsics(self):
        body = bytes.fromhex(FIXTURE['old']['expected']['calibration_hex'])
        decoded = decode_factory_calibration(struct.pack('<BI', 1, 219) + body)
        self.assertEqual(decoded['actual_body_bytes'], 219)
        self.assertEqual(decoded['unused_union_tail_hex'], '')
        self.assertEqual(decoded['sensor_image_size'], [0, 0])
        self.assertEqual(decoded['lenses'][0]['intrinsic']['fx'], 1052.5404052734375)
        self.assertNotEqual(decoded['lenses'][0]['intrinsic']['fx'],
                            decode_factory_calibration(captured_value())['lenses'][0]['intrinsic']['fx'])

    def test_nonzero_and_nan_v1_union_storage_cannot_select_v2(self):
        value = bytearray(captured_value())
        # This lies outside the selected 219-byte body, in union capacity only.
        value[224:] = (bytes.fromhex('0000c07f') * 17)
        decoded = decode_factory_calibration(value)
        self.assertEqual(decoded['unused_union_tail_hex'], value[224:].hex())
        self.assertIsNone(decoded['temperature'])
        self.assertEqual(decoded['lenses'], decode_factory_calibration(captured_value())['lenses'])
        for lens in decoded['lenses']:
            self.assertEqual(set(lens['extrinsic']), {'yaw', 'pitch', 'roll'})

    def test_xi_and_s_are_preserved_and_block_kb5_compatibility(self):
        value = bytearray(captured_value())
        struct.pack_into('<2f', value, 5 + 61, 0.125, -0.25)
        struct.pack_into('<2f', value, 5 + 129, 0.5, 0.75)
        decoded = decode_factory_calibration(value)
        self.assertEqual([(lens['intrinsic']['xi'], lens['intrinsic']['s']) for lens in decoded['lenses']],
                         [(0.125, -0.25), (0.5, 0.75)])
        self.assertFalse(decoded['projection_interpretation']['coefficients_compatible'])
        self.assertIsNone(decoded['projection_interpretation']['candidate_model'])

    def test_v2_has_wxyz_quaternions_temperature_and_shifted_tail(self):
        decoded = decode_factory_calibration(synthetic_v2())
        self.assertEqual((decoded['version'], decoded['declared_body_bytes'], decoded['actual_body_bytes']),
                         (2, 287, 287))
        self.assertEqual(decoded['unused_union_tail_hex'], '')
        self.assertEqual(decoded['stream_frame_type_name'], 'dual_hemisphere')
        first, second = decoded['lenses']
        self.assertEqual(first['extrinsic'], dict(yaw=11., pitch=12., roll=13.,
            cam_imu_extri_q_wxyz=[0.5, 0.25, -0.5, 0.75], cam_extri_q_wxyz=[1., 2., 3., 4.]))
        self.assertEqual(second['extrinsic'], dict(yaw=21., pitch=22., roll=23.,
            cam_imu_extri_q_wxyz=[-0.5, 0.5, 0.25, -0.25], cam_extri_q_wxyz=[5., 6., 7., 8.]))
        self.assertEqual(decoded['temperature'], -12.5)
        self.assertEqual(decoded['timing']['read_out_time'], 0x0102030405060708)
        self.assertEqual(decoded['timing']['itd_value_u32'], 0xFFFFFFFF)
        self.assertEqual(decoded['timing']['lro_value_u32'], 0x80000001)
        self.assertAlmostEqual(decoded['timing']['sensor_frame_rate'], 59.94, places=5)
        self.assertEqual(decoded['sensor_image_size'], [4000, 3000])
        self.assertEqual(decoded['rolling_shutter_grid'], [17, 19])
        self.assertEqual(decoded['panorama_grid'], [41, 43])
        self.assertEqual(decoded['calibration_image_size'], [3840., 4096.])
        self.assertEqual(decoded['cam_lens_type'], 99)
        self.assertFalse(decoded['camera_imu_transform_verified'])

    def test_unknown_stream_type_is_preserved_without_guessing_projection(self):
        decoded = decode_factory_calibration(synthetic_v2(stream_type=203))
        self.assertEqual(decoded['stream_frame_type'], 203)
        self.assertEqual(decoded['stream_frame_type_name'], 'unknown')
        self.assertFalse(decoded['projection_interpretation']['coefficients_compatible'])
        self.assertIsNone(decoded['projection_interpretation']['candidate_model'])

    def test_native_layout_does_not_certify_usb_geometry_or_imu_transform(self):
        decoded = decode_factory_calibration(captured_value())
        self.assertTrue(decoded['native_layout_verified'])
        self.assertTrue(decoded['projection_interpretation']['coefficients_compatible'])
        self.assertEqual(decoded['projection_interpretation']['candidate_model'], 'dji_kb5_brown_tangential')
        self.assertFalse(decoded['projection_interpretation']['verified_for_usb'])
        for field in ('stream_mapping_verified', 'camera_imu_transform_verified',
                      'extrinsic_angle_units_verified', 'extrinsic_rotation_convention_verified', 'aprilgrid_validated'):
            self.assertFalse(decoded[field], field)

    def test_unknown_version_mismatched_sizes_and_truncations_fail_explicitly(self):
        good = captured_value()
        malformed = [b'', good[:4], bytes([0]) + good[1:], bytes([99]) + good[1:],
            good[:1] + struct.pack('<I', 218) + good[5:],
            good[:1] + struct.pack('<I', 287) + good[5:],
            struct.pack('<BI', 2, 219) + good[5:],
            good[:223], good[:225], good[:-1], good + b'\0', synthetic_v2()[:-1]]
        for value in malformed:
            with self.subTest(prefix=value[:5].hex(), length=len(value)), self.assertRaises(CalibrationDecodeError):
                decode_factory_calibration(value)

    def test_every_float_group_rejects_nonfinite_values_in_selected_layout(self):
        # Body offsets independently follow the packed native ABI. Exercise
        # xi/s and both quaternion groups as well as the common optical fields.
        for source, offsets in (
            (captured_value(), [*range(1, 137, 4), *range(137, 161, 4), 171, 207, 211]),
            (synthetic_v2(), [*range(137, 225, 4), 235, 271, 275, 283]),
        ):
            for offset in offsets:
                for number in (math.nan, math.inf, -math.inf):
                    value = bytearray(source)
                    struct.pack_into('<f', value, 5 + offset, number)
                    with self.subTest(version=source[0], offset=offset, number=number), self.assertRaises(CalibrationDecodeError):
                        decode_factory_calibration(value)

    def test_nonpositive_focal_lengths_and_calibration_dimensions_are_rejected(self):
        for offset in (1, 5, 69, 73, 207, 211):
            for number in (0., -1.):
                value = bytearray(captured_value())
                struct.pack_into('<f', value, offset + 5, number)
                with self.subTest(offset=offset, value=number), self.assertRaises(CalibrationDecodeError):
                    decode_factory_calibration(value)

    def test_supported_bytes_containers_and_type_errors(self):
        value = captured_value()
        for wrapped in (value, bytearray(value), memoryview(value)):
            self.assertEqual(decode_factory_calibration(wrapped)['original_value_hex'], value.hex())
        for unsupported in (None, value.hex(), list(value), 292):
            with self.subTest(type=type(unsupported).__name__), self.assertRaises(TypeError):
                decode_factory_calibration(unsupported)

    def test_bad_calibration_never_hides_valid_motion_timestamp_or_original_tlv(self):
        expected = captured_row()
        nonfinite = bytearray(captured_value())
        struct.pack_into('<f', nonfinite, 6, math.nan)
        unknown = bytes([99]) + captured_value()[1:]
        truncated = captured_value()[:223]
        for value in (unknown, nonfinite, truncated):
            with self.subTest(prefix=bytes(value[:9]).hex()):
                row = next(records_from_video(replace_calibration(value)))
                self.assertEqual(row['frame_clock_ns_raw'], expected['frame_clock_ns_raw'])
                self.assertEqual(row['imu_clock_us_raw'], expected['imu_clock_us_raw'])
                self.assertEqual(row['quaternions'], expected['quaternions'])
                self.assertEqual(row['calibration_value_hex'], bytes(value).hex())
                self.assertIn('calibration_decode_error', row)
                self.assertNotIn('calibration_decoded', row)
                self.assertIn(bytes(value).hex(), row['raw_tlv_hex'])

    def test_complete_short_tag13_envelope_keeps_following_motion(self):
        expected = captured_row()
        for length in range(5):
            value = captured_value()[:length]
            with self.subTest(length=length):
                row = next(records_from_video(replace_calibration(value)))
                self.assertEqual(row['calibration_value_hex'], value.hex())
                self.assertIn('Truncated', row['calibration_decode_error'])
                self.assertNotIn('calibration_decoded', row)
                self.assertNotIn('calibration_version', row)
                self.assertEqual(row['frame_clock_ns_raw'], expected['frame_clock_ns_raw'])
                self.assertEqual(row['quaternions'], expected['quaternions'])

    def test_repeated_bad_tag13_cannot_retain_an_earlier_decoded_calibration(self):
        original = bytes.fromhex(captured_row()['raw_tlv_hex'])
        for value in (b'\x01', b'\x63' + captured_value()[1:]):
            with self.subTest(length=len(value)):
                row = next(records_from_video(sei(original + struct.pack('<HH', 13, len(value)) + value)))
                self.assertEqual(row['calibration_value_hex'], value.hex())
                self.assertIn('calibration_decode_error', row)
                self.assertNotIn('calibration_decoded', row)
                self.assertEqual(row['frame_clock_ns_raw'], FIXTURE['current'][0]['expected']['frame_clock_ns_raw'])

    def test_truncated_outer_tlv_is_fatal_instead_of_being_skipped_as_bad_calibration(self):
        with self.assertRaises(ValueError):
            next(records_from_video(sei(struct.pack('<HH', 13, 5) + b'\x01\0')))


class FactorySeedTests(unittest.TestCase):
    def test_anisotropic_seed_keeps_every_distortion_term_and_native_source(self):
        native = decode_factory_calibration(captured_value())
        original = deepcopy(native)
        seed = uniform_resize_seed(native, (1024, 768))
        self.assertEqual(native, original)
        self.assertEqual(seed['purpose'], 'unverified_aprilgrid_seed')
        self.assertEqual(seed['native_to_image_affine'], [[1024 / 3840, 0, 0], [0, 768 / 3840, 0], [0, 0, 1]])
        self.assertEqual(seed['lenses'][1]['assumed_image_roi_xywh'], [1024, 0, 1024, 768])
        for index, lens in enumerate(seed['lenses']):
            expected = native['lenses'][index]['intrinsic']
            for name in INTRINSIC_KEYS:
                factor = 1024 / 3840 if name in ('fx', 'cx') else 768 / 3840 if name in ('fy', 'cy') else 1
                self.assertEqual(lens['intrinsic'][name], expected[name] * factor, name)
        for field in ('stream_mapping_verified', 'aprilgrid_validated', 'kalibr_equi_exact_export'):
            self.assertFalse(seed[field], field)
        self.assertEqual(seed['source_payload_sha256'], native['payload_sha256'])
        self.assertEqual(len(seed['assumptions']), 3)
        seed['lenses'][0]['intrinsic']['k5'] = 0
        self.assertEqual(native, original)

    def test_seed_size_requires_two_positive_integers(self):
        native = decode_factory_calibration(captured_value())
        for size in ((0, 1024), (-1, 1024), (1024, 0), (True, 1024), (1024., 1024), (1024,), (1, 2, 3)):
            with self.subTest(size=size), self.assertRaises(ValueError):
                uniform_resize_seed(native, size)


class FactoryCalibrationCLITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.input = self.root / 'status.json'
        self.input.write_text(json.dumps({'calibration': captured_row()}))
        self.output = self.root / 'factory.json'

    def run_main(self, *extra):
        stdout = io.StringIO()
        with redirect_stdout(stdout), patch('osmo360_camera.Camera.connect', side_effect=AssertionError('offline export opened USB')):
            code = main(['--input', str(self.input), '--output', str(self.output), *extra])
        return code, json.loads(stdout.getvalue())

    def test_extracts_status_manifest_decoded_export_and_legacy_body_json(self):
        value = captured_value()
        body = value[5:]
        documents = [
            {'osmo360': {'calibration': {'calibration_value_hex': value.hex()}}},
            {'sources': {'osmo360': {'status': {'calibration': captured_row()}}}},
            {'capture_start': {'osmo360': {'factory': decode_factory_calibration(value)}}},
            {'metadata': {'calibration_hex': body.hex(), 'calibration_version': 1,
                          'calibration_declared_body_length': 219}},
        ]
        for index, document in enumerate(documents):
            path = self.root / f'input-{index}.json'
            path.write_text(json.dumps(document))
            with self.subTest(index=index), patch('osmo360_camera.Camera.connect', side_effect=AssertionError('USB')):
                decoded, provenance = extract_calibration(path)
                self.assertEqual(decoded['original_value_hex'], value.hex())
                self.assertEqual(provenance['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())

    def test_jsonl_stops_after_first_calibration_instead_of_parsing_later_damage(self):
        path = self.root / 'video-index.jsonl'
        path.write_text(json.dumps({'sequence': 0}) + '\n' +
            json.dumps({'sequence': 1, 'metadata': captured_row()}) + '\n{incomplete later line\n')
        decoded, provenance = extract_calibration(path)
        self.assertEqual(decoded['original_value_hex'], captured_value().hex())
        self.assertEqual(provenance['format'], 'json_or_jsonl')

    def test_original_h264_fixture_exports_without_video_decoder_or_camera(self):
        self.input = self.root / 'first.h264'
        raw = b'\0\0\0\1\x09\x10' + captured_nal()
        self.input.write_bytes(raw)
        code, printed = self.run_main()
        native = json.loads(self.output.read_text())
        self.assertEqual(code, 0)
        self.assertEqual(printed['version'], 1)
        self.assertEqual(native['original_value_hex'], captured_value().hex())
        self.assertEqual(native['input_provenance']['format'], 'h264')
        self.assertEqual(native['input_provenance']['prefix_sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(self.input.read_bytes(), raw)

    def test_native_export_does_not_create_a_resize_seed_implicitly(self):
        original = self.input.read_bytes()
        code, printed = self.run_main('--lens-width', '512', '--lens-height', '256')
        native = json.loads(self.output.read_text())
        self.assertEqual(code, 0)
        self.assertEqual(printed['calibration_image_size'], [3840., 3840.])
        self.assertEqual(native['lenses'][0]['intrinsic']['fx'], 1049.9498291015625)
        self.assertNotIn('assumed_image_roi_xywh', native['lenses'][0])
        self.assertEqual({path.name for path in self.root.iterdir()}, {'status.json', 'factory.json'})
        self.assertEqual(self.input.read_bytes(), original)

    def test_explicit_seed_is_separate_unverified_and_uses_requested_lens_size(self):
        seed_path = self.root / 'aprilgrid-seed.json'
        code, _ = self.run_main('--resize-seed', str(seed_path), '--lens-width', '512', '--lens-height', '256')
        self.assertEqual(code, 0)
        native, seed = json.loads(self.output.read_text()), json.loads(seed_path.read_text())
        self.assertEqual(native['calibration_image_size'], [3840., 3840.])
        self.assertEqual(seed['lens_image_size'], [512, 256])
        self.assertEqual(seed['purpose'], 'unverified_aprilgrid_seed')
        self.assertFalse(seed['stream_mapping_verified'])
        self.assertFalse(seed['aprilgrid_validated'])
        self.assertFalse(seed['kalibr_equi_exact_export'])
        self.assertEqual(seed['source_payload_sha256'], native['payload_sha256'])

    def test_existing_native_or_seed_output_is_never_overwritten(self):
        seed_path = self.root / 'seed.json'
        for occupied in (self.output, seed_path):
            occupied.write_text('keep original\n')
            try:
                with self.subTest(path=occupied.name), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    self.run_main('--resize-seed', str(seed_path))
                self.assertEqual(error.exception.code, 2)
                self.assertEqual(occupied.read_text(), 'keep original\n')
                other = seed_path if occupied == self.output else self.output
                self.assertFalse(other.exists())
            finally:
                occupied.unlink()

    def test_seed_same_path_and_invalid_dimensions_leave_no_output(self):
        for extra in (('--resize-seed', str(self.output)),
                      ('--resize-seed', str(self.root / 'seed.json'), '--lens-width', '0')):
            with self.subTest(extra=extra), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                self.run_main(*extra)
            self.assertEqual(error.exception.code, 2)
            self.assertFalse(self.output.exists())
            self.assertFalse((self.root / 'seed.json').exists())

    def test_missing_seed_parent_does_not_leave_a_partial_native_output(self):
        seed_path = self.root / 'missing-parent' / 'seed.json'
        original = self.input.read_bytes()
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            self.run_main('--resize-seed', str(seed_path))
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(self.output.exists())
        self.assertFalse(seed_path.exists())
        self.assertEqual(self.input.read_bytes(), original)

    def test_competing_seed_creation_is_not_overwritten_or_deleted_during_rollback(self):
        seed_path = self.root / 'seed.json'
        original_open = Path.open

        def racing_open(path, mode='r', *args, **kwargs):
            if path == seed_path and mode == 'x':
                with original_open(seed_path, 'w') as stream:
                    stream.write('other writer\n')
            return original_open(path, mode, *args, **kwargs)

        with patch.object(Path, 'open', new=racing_open), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            self.run_main('--resize-seed', str(seed_path))
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(self.output.exists())
        self.assertEqual(seed_path.read_text(), 'other writer\n')

    def test_unsupported_calibration_cli_fails_without_output_or_usb(self):
        self.input.write_text(json.dumps({'calibration_value_hex': (b'\x63' + captured_value()[1:]).hex()}))
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            self.run_main()
        self.assertEqual(error.exception.code, 2)
        self.assertFalse(self.output.exists())

    def test_no_calibration_and_unknown_format_report_explicit_errors(self):
        self.input.write_text(json.dumps({'status': 'no calibration yet'}))
        with self.assertRaisesRegex(ValueError, 'No original factory tag13'):
            extract_calibration(self.input)
        with self.assertRaisesRegex(ValueError, 'Format must be'):
            extract_calibration(self.input, format='guess')

    def test_unknown_calibration_in_complete_h264_keeps_specific_schema_error(self):
        path = self.root / 'unsupported.h264'
        path.write_bytes(replace_calibration(b'\x63' + captured_value()[1:]))
        with self.assertRaisesRegex(CalibrationDecodeError, 'Unsupported factory calibration version 99'):
            extract_calibration(path)


class RuntimeCalibrationObservationTests(unittest.TestCase):
    @staticmethod
    def owner():
        with patch('osmo360_driver.runtime.threading.Thread'):
            return OsmoCamera(camera_factory=Mock(side_effect=AssertionError('no camera I/O')))

    def test_cache_and_status_are_independent_copies_with_original_source_times(self):
        owner = self.owner()
        metadata = captured_row()
        original = deepcopy(metadata)
        frame = SimpleNamespace(metadata=metadata, timestamp_ns=123456789,
                                host_receive_monotonic_ns=987654321)
        owner.observe_calibration(frame)
        status = owner.status()
        self.assertTrue(status['calibration_available'])
        self.assertEqual(status['calibration']['calibration_source_native_ns'], 123456789)
        self.assertEqual(status['calibration']['calibration_source_host_receive_monotonic_ns'], 987654321)
        self.assertEqual(status['calibration']['calibration_value_hex'], original['calibration_value_hex'])
        self.assertNotIn('quaternions', status['calibration'])
        metadata['calibration_decoded']['lenses'][0]['intrinsic']['fx'] = 1
        status['calibration']['calibration_decoded']['lenses'][0]['intrinsic']['fx'] = 2
        self.assertEqual(owner.status()['calibration']['calibration_decoded']['lenses'][0]['intrinsic']['fx'],
                         1049.9498291015625)
        owner.observe_calibration(SimpleNamespace(metadata={}, timestamp_ns=999, host_receive_monotonic_ns=999))
        self.assertEqual(owner.status()['calibration']['calibration_source_native_ns'], 123456789)
        owner.camera_factory.assert_not_called()

    def test_raw_first_au_calibration_is_observed_before_archive_and_without_preview(self):
        owner = self.owner()
        frame = SimpleNamespace(metadata=captured_row(), timestamp_ns=123456,
                                host_receive_monotonic_ns=987654, raw_h264=captured_nal())
        observed = []

        def append(raw, stamp):
            observed.append((raw, stamp, deepcopy(owner.calibration)))

        owner.archive = SimpleNamespace(append=append)
        owner.raw_video(frame)
        self.assertEqual(len(observed), 1)
        self.assertIs(observed[0][0], frame)
        self.assertIn('calibration_decoded', observed[0][2])
        self.assertEqual(observed[0][2]['calibration_source_native_ns'], 123456)
        self.assertEqual(owner.frames, 0)
        owner.camera_factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
