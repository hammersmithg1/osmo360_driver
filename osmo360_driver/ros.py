"""Standard ROS messages. The optional sink receives EVERY sample before DDS.

Browser JPEG is a viewing copy. /video/h264 retains the original Annex-B
access units, including DJI metadata. SLAM can consume /image/compressed;
offline tools can decode the original H.264 without the extra JPEG generation.
"""

import json

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String, UInt64MultiArray

from .runtime import json_value
from .topics import TOPICS


def set_header(message, stamp_ns, frame_id):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.header.frame_id = frame_id
    return message


def compressed_image(data, stamp_ns, frame_id, format='jpeg'):
    message = set_header(CompressedImage(), stamp_ns, frame_id)
    message.format, message.data = format, data
    return message


class OsmoTopics:
    def __init__(self, node, sink=None, *, has_sink_consumers=None):
        self.sink = sink
        # Recording demand is checked for each sample, independently of ROS
        # discovery. A newly started recording must not wait for the timer.
        self.has_sink_consumers = has_sink_consumers or (lambda: sink is not None)
        types = {'sensor_msgs/msg/CompressedImage': CompressedImage, 'sensor_msgs/msg/Imu': Imu,
                 'std_msgs/msg/String': String, 'std_msgs/msg/UInt64MultiArray': UInt64MultiArray}
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publishers = {topic: node.create_publisher(types[kind], topic, qos) for topic, kind in TOPICS.items()}
        self._subscribed_topics = frozenset()
        self.refresh_subscribers()
        self.sequence = 0

    def refresh_subscribers(self):
        """Call from the ROS executor timer, not the 8 kHz capture thread."""
        self._subscribed_topics = frozenset(
            topic for topic, publisher in self.publishers.items()
            if publisher.get_subscription_count())

    def emit(self, topic, message, stamp_ns, sink_active, subscribers):
        if sink_active:
            self.sink('osmo360', topic, message, stamp_ns)
        if topic in subscribers:
            self.publishers[topic].publish(message)

    def video(self, frame, jpeg, stamp_ns):
        subscribers = self._subscribed_topics
        sink_active = self.sink is not None and self.has_sink_consumers()
        if sink_active or '/osmo360/image/compressed' in subscribers:
            self.emit('/osmo360/image/compressed', compressed_image(
                jpeg, stamp_ns, 'osmo360_dual_fisheye', 'bgr8; jpeg compressed bgr8'),
                stamp_ns, sink_active, subscribers)
        if sink_active or '/osmo360/video/h264' in subscribers:
            self.emit('/osmo360/video/h264', compressed_image(
                frame.raw_h264, stamp_ns, 'osmo360_dual_fisheye', 'h264'),
                stamp_ns, sink_active, subscribers)
        if sink_active or '/osmo360/video/timing' in subscribers:
            self.emit('/osmo360/video/timing', UInt64MultiArray(data=[
                stamp_ns, frame.timestamp_ns, frame.host_receive_monotonic_ns, self.sequence]),
                stamp_ns, sink_active, subscribers)
        if sink_active or '/osmo360/video/metadata' in subscribers:
            self.emit('/osmo360/video/metadata', String(data=json.dumps(
                json_value(frame.metadata), allow_nan=False, separators=(',', ':'))),
                stamp_ns, sink_active, subscribers)
        self.sequence += 1

    def imu(self, sample, stamp_ns):
        subscribers = self._subscribed_topics
        sink_active = self.sink is not None and self.has_sink_consumers()
        if sink_active or '/osmo360/imu' in subscribers:
            message = set_header(Imu(), stamp_ns, 'osmo360_imu_native')
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = sample.gyro
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = sample.accel
            # No fused orientation. Zero covariance means unknown (not calibrated).
            message.orientation_covariance[0] = -1.0
            self.emit('/osmo360/imu', message, stamp_ns, sink_active, subscribers)
        if sink_active or '/osmo360/imu/timing' in subscribers:
            self.emit('/osmo360/imu/timing', UInt64MultiArray(data=[
                stamp_ns, sample.timestamp_ns, sample.host_receive_monotonic_ns,
                sample.timestamp_ticks, sample.accel_status, sample.gyro_status]),
                stamp_ns, sink_active, subscribers)
