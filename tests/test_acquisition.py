"""Immutable IPC acquisition schema and exact shared ROS messages, offline."""

import json
import pickle
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from osmo360_driver.acquisition import (
    imu_acquisition, video_acquisition, materialize_acquisition,
)
from test_ros_topics import FakeNode, load_fake_ros


class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        topics, self.created = load_fake_ros()
        self.types = topics(FakeNode()).message_types
        self.sample = SimpleNamespace(
            gyro=[.1, .2, .3], accel=[0., 0., 9.81], timestamp_ns=101,
            host_receive_monotonic_ns=202, timestamp_ticks=3,
            accel_status=4, gyro_status=5,
        )
        self.frame = SimpleNamespace(
            timestamp_ns=101, host_receive_monotonic_ns=202,
            raw_h264=bytearray(b'original AU'), image=object(),
            metadata={'native': b'\x00\xff', 'nested': [1, 2]},
        )

    def test_builders_need_no_ros_and_detach_mutable_inputs_for_pickle_ipc(self):
        jpeg = bytearray(b'jpeg')
        with patch.dict(sys.modules, {'sensor_msgs': None, 'sensor_msgs.msg': None,
                                      'std_msgs': None, 'std_msgs.msg': None}):
            imu = imu_acquisition(self.sample, 1000)
            video = video_acquisition(self.frame, jpeg, 1001, 7)
            self.assertEqual(materialize_acquisition(imu, topics=()), ())
        self.sample.gyro[0] = 999
        self.frame.metadata['nested'].append(3)
        self.frame.raw_h264[:] = b'changed'
        jpeg[:] = b'changed'
        self.assertEqual(imu, ('imu', 1000, 101, 202, 3, (.1, .2, .3), (0., 0., 9.81), 4, 5))
        self.assertEqual(video[:7], ('video', 1001, 101, 202, 7, b'original AU', b'jpeg'))
        metadata = json.loads(video[7])
        self.assertEqual(metadata['nested'], [1, 2])
        self.assertEqual(metadata['native'], {'encoding': 'hex', 'data': '00ff'})
        self.assertEqual(metadata['_rover_timing'], {'valid_native': True, 'stamp_source': 'camera_native'})
        self.assertEqual(pickle.loads(pickle.dumps((imu, video))), (imu, video))
        self.assertEqual(self.created, {})

    def test_materialized_native_messages_keep_original_headers_units_and_companion_timing(self):
        stamp = 1_700_000_000_000_000_123
        entries = materialize_acquisition(imu_acquisition(self.sample, stamp), message_types=self.types)
        self.assertEqual([topic for topic, _, _ in entries], ['/osmo360/imu', '/osmo360/imu/timing'])
        message, timing = (row[1] for row in entries)
        self.assertEqual(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec, stamp)
        self.assertEqual(message.header.frame_id, 'osmo360_imu_native')
        self.assertEqual((message.angular_velocity.x, message.angular_velocity.z), (.1, .3))
        self.assertEqual(message.linear_acceleration.z, 9.81)
        self.assertEqual(message.orientation_covariance, [-1.] + [0.] * 8)
        self.assertEqual(timing.data, [stamp, 101, 202, 3, 4, 5])
        video = materialize_acquisition(video_acquisition(self.frame, b'jpeg', stamp, 8), message_types=self.types)
        messages = {topic: message for topic, message, _ in video}
        self.assertEqual(messages['/osmo360/image/compressed'].data, b'jpeg')
        self.assertEqual(messages['/osmo360/video/h264'].data, b'original AU')
        self.assertEqual(messages['/osmo360/video/timing'].data, [stamp, 101, 202, 8, 1])
        self.assertEqual({row[2] for row in video}, {stamp})

    def test_missing_native_clock_keeps_original_au_without_synchronized_jpeg(self):
        self.frame.timestamp_ns = None
        dto = video_acquisition(self.frame, b'live only', 1000, 9)
        entries = materialize_acquisition(dto, message_types=self.types)
        messages = {topic: message for topic, message, _ in entries}
        self.assertNotIn('/osmo360/image/compressed', messages)
        self.assertEqual(messages['/osmo360/video/h264'].data, b'original AU')
        self.assertEqual(messages['/osmo360/video/timing'].data, [1000, 0, 202, 9, 0])
        self.assertEqual(json.loads(messages['/osmo360/video/metadata'].data)['_rover_timing'],
                         {'valid_native': False, 'stamp_source': 'host_receive'})

    def test_only_selected_ros_type_is_constructed_and_unknown_schema_is_rejected(self):
        dto = imu_acquisition(self.sample, 1000)
        entries = materialize_acquisition(dto, {'/osmo360/imu/timing'},
            message_types={'UInt64MultiArray': self.types['UInt64MultiArray']})
        self.assertEqual(len(entries), 1)
        self.assertEqual(self.created, {'UInt64MultiArray': 1})
        for invalid in ([], (), ('imu', 1), ('video', 1), ('webcam', 1)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                materialize_acquisition(invalid)


if __name__ == '__main__':
    unittest.main()
