"""Demand/recording boundaries with fake ROS types; no ROS or USB required."""

from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def load_fake_ros():
    created = Counter()

    class Message:
        def __init__(self, **values):
            created[type(self).__name__] += 1
            self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
            self.__dict__.update(values)

    class Imu(Message):
        def __init__(self):
            super().__init__()
            self.angular_velocity = SimpleNamespace(x=0., y=0., z=0.)
            self.linear_acceleration = SimpleNamespace(x=0., y=0., z=0.)
            self.orientation_covariance = [0.] * 9

    class CompressedImage(Message):
        pass

    class String(Message):
        pass

    class UInt64MultiArray(Message):
        pass

    modules = {name: ModuleType(name) for name in
               ('rclpy', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.qos'].QoSProfile = lambda **kw: SimpleNamespace(**kw)
    modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    modules['sensor_msgs.msg'].CompressedImage = CompressedImage
    modules['sensor_msgs.msg'].Imu = Imu
    modules['std_msgs.msg'].String = String
    modules['std_msgs.msg'].UInt64MultiArray = UInt64MultiArray
    name = 'osmo360_driver._ros_topics_test'
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).parents[1] / 'osmo360_driver/ros.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.OsmoTopics, created


class FakePublisher:
    def __init__(self):
        self.count = 0
        self.queries = 0
        self.messages = []

    def get_subscription_count(self):
        self.queries += 1
        return self.count

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, kind, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


class RosTopicsTests(unittest.TestCase):
    def setUp(self):
        self.OsmoTopics, self.created = load_fake_ros()
        self.node = FakeNode()
        self.sample = SimpleNamespace(gyro=(.1, .2, .3), accel=(0., 0., 9.81),
            timestamp_ns=100, host_receive_monotonic_ns=200, timestamp_ticks=2,
            accel_status=0, gyro_status=0)
        self.frame = SimpleNamespace(raw_h264=b'original Annex-B', timestamp_ns=100,
                                     host_receive_monotonic_ns=200, metadata={'native': b'\x00\xff'})

    def test_idle_8khz_input_neither_allocates_ros_messages_nor_queries_graph(self):
        sink = Mock()
        topics = self.OsmoTopics(self.node, sink=sink, has_sink_consumers=lambda: False)
        for stamp in range(8000):
            topics.imu(object(), stamp)  # No sample fields are needed without consumers.
        topics.video(self.frame, b'jpeg', 8001)
        topics.video_unstamped(SimpleNamespace(timestamp_ns=None), b'jpeg', 8002)
        self.assertEqual(self.created, {})
        self.assertEqual(topics.sequence, 2)
        sink.assert_not_called()
        self.assertTrue(all(p.queries == 1 for p in self.node.publishers.values()))

    def test_recording_demand_applies_to_next_sample_without_graph_refresh(self):
        recording = [False]
        saved = []
        topics = self.OsmoTopics(self.node, sink=lambda *args: saved.append(args),
                                 has_sink_consumers=lambda: recording[0])
        topics.imu(self.sample, 0)
        recording[0] = True
        for stamp in range(1, 8001):
            topics.imu(self.sample, stamp)
        recording[0] = False
        topics.imu(self.sample, 8001)
        self.assertEqual(len(saved), 16000)
        self.assertEqual([stamp for _, topic, _, stamp in saved if topic == '/osmo360/imu'],
                         list(range(1, 8001)))
        timings = [message.data for _, topic, message, _ in saved if topic == '/osmo360/imu/timing']
        self.assertEqual([value[0] for value in timings], list(range(1, 8001)))
        self.assertTrue(all(value[1:] == [100, 200, 2, 0, 0] for value in timings))
        self.assertTrue(all(p.queries == 1 for p in self.node.publishers.values()))

    def test_only_subscribed_type_is_constructed_and_graph_poll_is_external(self):
        topics = self.OsmoTopics(self.node)
        publisher = self.node.publishers['/osmo360/imu/timing']
        publisher.count = 1
        topics.imu(self.sample, 1000)
        self.assertEqual(self.created, {})
        topics.refresh_subscribers()
        topics.imu(self.sample, 1001)
        self.assertEqual(self.created, {'UInt64MultiArray': 1})
        self.assertEqual(publisher.messages[0].data, [1001, 100, 200, 2, 0, 0])
        publisher.count = 0
        topics.refresh_subscribers()
        topics.imu(self.sample, 1002)
        self.assertEqual(len(publisher.messages), 1)

    def test_batch_sink_keeps_whole_native_unstamped_and_imu_groups_before_dds(self):
        saved, order, legacy = [], [], Mock()

        def batch(source, entries):
            saved.append((source, tuple(entries)))
            order.append('batch')

        topics = self.OsmoTopics(self.node, sink=legacy, batch_sink=batch)
        for publisher in self.node.publishers.values():
            publisher.count = 1
            publisher.publish = lambda message: order.append('dds')
        topics.refresh_subscribers()
        topics.video(self.frame, b'jpeg', 1000)
        topics.video_unstamped(SimpleNamespace(raw_h264=b'original P', timestamp_ns=None,
            host_receive_monotonic_ns=240, metadata={}), b'unknown jpeg', 1040)
        topics.imu(self.sample, 1080)
        self.assertEqual(order, ['batch', *['dds'] * 4, 'batch', *['dds'] * 3, 'batch', 'dds', 'dds'])
        legacy.assert_not_called()  # A supplied batch sink owns archival delivery.
        self.assertEqual([source for source, _ in saved], ['osmo360'] * 3)
        self.assertEqual([len(entries) for _, entries in saved], [4, 3, 2])
        self.assertEqual([{stamp for _, _, stamp in entries} for _, entries in saved], [{1000}, {1040}, {1080}])
        unknown = {topic: message for topic, message, _ in saved[1][1]}
        self.assertNotIn('/osmo360/image/compressed', unknown)
        self.assertEqual(unknown['/osmo360/video/h264'].data, b'original P')
        self.assertEqual(unknown['/osmo360/video/timing'].data, [1040, 0, 240, 1, 0])
        self.assertEqual(topics.sequence, 2)

    def test_batch_demand_starts_on_next_sample_and_preserves_every_imu_pair(self):
        active, groups = [False], []
        topics = self.OsmoTopics(self.node, batch_sink=lambda source, entries: groups.append(tuple(entries)),
                                 has_sink_consumers=lambda: active[0])
        topics.imu(object(), 0)
        self.assertEqual(self.created, {})
        active[0] = True
        for stamp in range(1, 8001):
            topics.imu(self.sample, stamp)
        active[0] = False
        topics.imu(object(), 8001)
        self.assertEqual(len(groups), 8000)
        self.assertTrue(all([topic for topic, _, _ in entries] == ['/osmo360/imu', '/osmo360/imu/timing'] for entries in groups))
        self.assertEqual([entries[0][2] for entries in groups], list(range(1, 8001)))
        self.assertTrue(all(entries[0][2] == entries[1][2] == entries[1][1].data[0] for entries in groups))
        self.assertTrue(all(p.queries == 1 for p in self.node.publishers.values()))

    def test_raw_acquisition_sink_records_8000_pairs_without_ros_construction_or_graph_queries(self):
        active, acquisitions = [False], []
        legacy, batch = Mock(), Mock()
        topics = self.OsmoTopics(self.node, sink=legacy, batch_sink=batch,
            acquisition_sink=lambda source, dto: acquisitions.append((source, dto)),
            has_sink_consumers=lambda: active[0])
        topics.imu(object(), 0)
        active[0] = True
        for stamp in range(1, 8001):
            topics.imu(self.sample, stamp)
        active[0] = False
        topics.imu(object(), 8001)
        self.assertEqual(self.created, {})
        self.assertEqual(len(acquisitions), 8000)
        self.assertEqual([dto[1] for _, dto in acquisitions], list(range(1, 8001)))
        self.assertTrue(all(source == 'osmo360' and dto[0] == 'imu' for source, dto in acquisitions))
        self.assertTrue(all(dto[2:] == (100, 200, 2, (.1, .2, .3), (0., 0., 9.81), 0, 0)
                            for _, dto in acquisitions))
        legacy.assert_not_called()
        batch.assert_not_called()
        self.assertTrue(all(p.queries == 1 for p in self.node.publishers.values()))

    def test_raw_acquisition_precedes_only_subscribed_dds_and_keeps_missing_clock_semantics(self):
        order, saved, before = [], [], []

        def acquire(source, dto):
            before.append(dict(self.created))
            saved.append(dto)
            order.append('raw')

        topics = self.OsmoTopics(self.node, acquisition_sink=acquire)
        publisher = self.node.publishers['/osmo360/imu/timing']
        publisher.count = 1
        publisher.publish = lambda message: order.append('dds')
        topics.refresh_subscribers()
        topics.imu(self.sample, 1000)
        self.assertEqual(order, ['raw', 'dds'])
        self.assertEqual(before, [{}])
        self.assertEqual(self.created, {'UInt64MultiArray': 1})
        frame = SimpleNamespace(raw_h264=b'original P', timestamp_ns=None,
            host_receive_monotonic_ns=240, metadata={})
        topics.video_unstamped(frame, b'viewing JPEG', 1040)
        self.assertEqual(self.created, {'UInt64MultiArray': 1})
        self.assertEqual(saved[-1][:7], ('video', 1040, None, 240, 0, b'original P', b'viewing JPEG'))
        self.assertEqual(json.loads(saved[-1][7])['_rover_timing'],
                         {'valid_native': False, 'stamp_source': 'host_receive'})

    def test_default_sink_keeps_every_topic_and_native_data_without_dds_subscribers(self):
        saved = []
        topics = self.OsmoTopics(self.node, sink=lambda *args: saved.append(args))
        topics.imu(self.sample, 1_700_000_000_000_000_123)
        topics.video(self.frame, b'jpeg', 1_700_000_000_000_000_124)
        self.assertEqual(len(saved), 6)
        messages = {topic: message for _, topic, message, _ in saved}
        self.assertEqual(messages['/osmo360/video/h264'].data, b'original Annex-B')
        self.assertEqual(messages['/osmo360/imu'].angular_velocity.z, .3)
        self.assertEqual(messages['/osmo360/imu'].linear_acceleration.z, 9.81)
        self.assertEqual(messages['/osmo360/imu'].orientation_covariance[0], -1.)
        header = messages['/osmo360/imu'].header
        self.assertEqual(header.stamp.sec * 10**9 + header.stamp.nanosec, 1_700_000_000_000_000_123)
        self.assertIn('00ff', messages['/osmo360/video/metadata'].data)
        self.assertTrue(all(not p.messages for p in self.node.publishers.values()))

    def test_sink_precedes_dds_and_recording_works_before_next_discovery_poll(self):
        order = []
        active = [False]
        topics = self.OsmoTopics(self.node, sink=lambda *args: order.append('sink'),
                                 has_sink_consumers=lambda: active[0])
        publisher = self.node.publishers['/osmo360/imu']
        publisher.count = 1
        topics.refresh_subscribers()
        publisher.publish = lambda message: order.append('dds')
        active[0] = True
        topics.imu(self.sample, 1000)
        self.assertEqual(order, ['sink', 'dds', 'sink'])

    def test_unstamped_au_keeps_original_h264_chain_but_cannot_enter_slam_images(self):
        saved = []
        topics = self.OsmoTopics(self.node, sink=lambda *args: saved.append(args))
        for publisher in self.node.publishers.values():
            publisher.count = 1
        topics.refresh_subscribers()
        # Distinct Annex-B units represent an IDR followed by two dependent
        # pictures. Dropping the middle unit because SEI time is missing would
        # also remove bytes required to decode subsequent pictures.
        access_units = [b'\0\0\0\1\x65IDR', b'\0\0\0\1\x41P1', b'\0\0\0\1\x41P2']
        frames = [SimpleNamespace(raw_h264=raw, timestamp_ns=native,
                                 host_receive_monotonic_ns=received, metadata={'sequence': index})
                  for index, (raw, native, received) in enumerate(zip(
                      access_units, [100, None, 180], [1000, 1040, 1080]))]
        topics.video(frames[0], b'jpeg0', 10_100)
        topics.video_unstamped(frames[1], b'jpeg1', 10_140)
        topics.video(frames[2], b'jpeg2', 10_180)
        archived = {topic: [(message, stamp) for _, name, message, stamp in saved if name == topic]
                    for topic in self.node.publishers}
        for records in (archived['/osmo360/video/h264'],
                        [(message, None) for message in self.node.publishers['/osmo360/video/h264'].messages]):
            self.assertEqual([message.data for message, _ in records], access_units)
            self.assertEqual(b''.join(message.data for message, _ in records), b''.join(access_units))
        self.assertEqual([stamp for _, stamp in archived['/osmo360/video/h264']], [10_100, 10_140, 10_180])
        for messages in ([message for message, _ in archived['/osmo360/image/compressed']],
                         self.node.publishers['/osmo360/image/compressed'].messages):
            self.assertEqual([message.data for message in messages], [b'jpeg0', b'jpeg2'])
        expected_timing = [[10_100, 100, 1000, 0, 1], [10_140, 0, 1040, 1, 0], [10_180, 180, 1080, 2, 1]]
        self.assertEqual([message.data for message, _ in archived['/osmo360/video/timing']], expected_timing)
        self.assertEqual([message.data for message in self.node.publishers['/osmo360/video/timing'].messages], expected_timing)
        metadata = [json.loads(message.data) for message, _ in archived['/osmo360/video/metadata']]
        self.assertEqual([value['sequence'] for value in metadata], [0, 1, 2])
        self.assertEqual([value['_rover_timing'] for value in metadata], [
            {'valid_native': True, 'stamp_source': 'camera_native'},
            {'valid_native': False, 'stamp_source': 'host_receive'},
            {'valid_native': True, 'stamp_source': 'camera_native'},
        ])
        self.assertTrue(all('_rover_timing' not in frame.metadata for frame in frames))
        self.assertEqual(topics.sequence, 3)

    def test_unstamped_frame_creates_no_slam_message_even_with_slam_only_subscriber(self):
        topics = self.OsmoTopics(self.node)
        publisher = self.node.publishers['/osmo360/image/compressed']
        publisher.count = 1
        topics.refresh_subscribers()
        frame = SimpleNamespace(timestamp_ns=None)
        topics.video_unstamped(frame, b'jpeg', 1000)
        self.assertEqual(self.created, {})
        self.assertEqual(publisher.messages, [])
        self.assertEqual(topics.sequence, 1)
        with self.assertRaisesRegex(ValueError, 'video_unstamped'):
            topics.video(frame, b'jpeg', 1000)
        self.assertEqual(publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
