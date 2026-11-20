"""Byte preservation and native fusion metadata must not imply EIS-off/SLAM geometry."""

import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_camera.vendor.preview_metadata import records_from_video
from osmo360_driver.acquisition import raw_video_acquisition
from osmo360_driver.runtime import OsmoCamera
from osmo360_driver.video_processing import video_processing_contract


class VideoProcessingTests(unittest.TestCase):
    def assert_unknown_camera_processing(self, contract):
        self.assertFalse(contract['sensor_raw'])
        self.assertFalse(contract['host_stabilization_applied'])
        self.assertEqual(contract['onboard_stabilization'], 'unknown')
        self.assertFalse(contract['onboard_stabilization_disabled_verified'])
        self.assertFalse(contract['fixed_geometry_verified'])
        self.assertFalse(contract['slam_intrinsics_verified'])

    def owner(self):
        # Allocate the actor state without starting its command worker or USB.
        with patch('osmo360_driver.runtime.threading.Thread'):
            return OsmoCamera()

    def test_each_consumer_receives_independent_serializable_facts(self):
        first = video_processing_contract()
        first.update(onboard_stabilization='off', fixed_geometry_verified=True)
        second = video_processing_contract()
        self.assert_unknown_camera_processing(second)
        self.assertEqual(json.loads(json.dumps(second)), second)

    def test_original_encoded_bytes_do_not_assert_sensor_raw_or_fixed_geometry(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/preview_sei_native.json').read_text())
        original = b'\0\0\0\1' + bytes.fromhex(fixture['current'][0]['nal_hex'])
        metadata = next(records_from_video(original))
        # Real captured metadata contains unit fusion quaternions and an opaque
        # calibration blob. Their presence does not establish an applied warp.
        self.assertGreater(metadata['sample_count'], 0)
        self.assertIn('quaternions', metadata)
        self.assertIn('calibration_hex', metadata)
        frame = SimpleNamespace(raw_h264=original, timestamp_ns=metadata['frame_clock_ns_raw'],
                                host_receive_monotonic_ns=100, metadata=metadata)
        self.assertEqual(raw_video_acquisition(frame, 200, 0)[5], original)
        contract = video_processing_contract()
        self.assertTrue(contract['original_encoded_h264_preserved'])
        self.assertEqual(contract['encoded_source'], 'camera_liveview_encoder')
        self.assert_unknown_camera_processing(contract)

    def test_runtime_never_promotes_opaque_tlv_or_fusion_fields_to_verified_isp_state(self):
        owner = self.owner()
        owner.calibration = {'calibration_hex': '01020304'}
        owner.capture_stats = {'preview': {'video': {'metadata': {
            'raw_tlv16_hex': '01080000000100000000000000',
            'sample_count': 33, 'quaternions': [[1., 0., 0., 0.]],
        }}}}
        first = owner.status()['video_processing']
        self.assert_unknown_camera_processing(first)
        first['onboard_stabilization_disabled_verified'] = True
        first['slam_intrinsics_verified'] = True
        self.assert_unknown_camera_processing(owner.status()['video_processing'])


if __name__ == '__main__':
    unittest.main()
