"""Demand/recording boundaries with fake ROS types; no ROS or USB required."""

from collections import Counter
import importlib.util
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
        topics.video(object(), b'jpeg', 8001)
        self.assertEqual(self.created, {})
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


if __name__ == '__main__':
    unittest.main()
