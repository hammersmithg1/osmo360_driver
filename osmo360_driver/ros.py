"""Standard ROS messages. Optional sinks receive EVERY sample before DDS.

batch_sink(source, entries) receives one acquisition as (topic, message, stamp)
entries, so recording start/stop and overflow cannot separate its companion
timing/metadata messages. The original per-message sink remains supported.
acquisition_sink(source, immutable_tuple) runs before ROS construction, allowing
a separate recording process to use acquisition.materialize_acquisition().
Archival priority is acquisition_sink, then batch_sink, then the legacy sink.

JPEG images and native timing provide independent frames for ROS/SLAM.
/video/h264 retains arriving Annex-B access units, including DJI metadata.
A recording that joins an existing stream may need earlier parameter sets
and reference pictures; standalone H.264 decoding is not verified here.
Access units without a native timestamp stay in the same H.264 sequence, but
their host-arrival-stamped JPEG is excluded from the SLAM image topic.
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String, UInt64MultiArray

from .acquisition import (
    IMU_TOPICS, VIDEO_TOPICS, imu_acquisition, video_acquisition,
    materialize_acquisition, set_header, compressed_image as _compressed_image,
)
from .topics import TOPICS


def compressed_image(data, stamp_ns, frame_id, format='jpeg'):
    return _compressed_image(data, stamp_ns, frame_id, format, message_type=CompressedImage)


class OsmoTopics:
    def __init__(self, node, sink=None, *, has_sink_consumers=None, batch_sink=None,
                 acquisition_sink=None):
        self.sink, self.batch_sink = sink, batch_sink
        self.acquisition_sink = acquisition_sink
        # Recording demand is checked for each sample, independently of ROS
        # discovery. A newly started recording must not wait for the timer.
        self.has_sink_consumers = has_sink_consumers or (
            lambda: sink is not None or batch_sink is not None or acquisition_sink is not None)
        self.message_types = dict(CompressedImage=CompressedImage, Imu=Imu,
                                  String=String, UInt64MultiArray=UInt64MultiArray)
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

    def emit_group(self, entries, sink_active, subscribers):
        # One acquisition and all its timing/metadata enter a recording together.
        # The legacy per-message sink keeps its original callback ordering.
        if sink_active and self.batch_sink is not None:
            self.batch_sink('osmo360', entries)
            sink_active = False
        for topic, message, stamp_ns in entries:
            self.emit(topic, message, stamp_ns, sink_active, subscribers)

    def video(self, frame, jpeg, stamp_ns):
        if frame.timestamp_ns is None:
            raise ValueError('Use video_unstamped for an access unit without native time')
        self._video(frame, jpeg, stamp_ns, valid_native=True)

    def video_unstamped(self, frame, jpeg, stamp_ns):
        """Keep the AU for decoding; stamp_ns is mapped host receive time."""
        if frame.timestamp_ns is not None:
            raise ValueError('video_unstamped requires a missing native timestamp')
        self._video(frame, jpeg, stamp_ns, valid_native=False)

    def _video(self, frame, jpeg, stamp_ns, *, valid_native):
        subscribers = self._subscribed_topics
        sink_active = any(callback is not None for callback in (
            self.sink, self.batch_sink, self.acquisition_sink)) and self.has_sink_consumers()
        selected = VIDEO_TOPICS.intersection(subscribers)
        if not valid_native:
            selected = selected - {'/osmo360/image/compressed'}
        if sink_active or selected:
            acquisition = video_acquisition(frame, jpeg, stamp_ns, self.sequence)
            raw_sink = sink_active and self.acquisition_sink is not None
            if raw_sink:
                self.acquisition_sink('osmo360', acquisition)
            legacy_sink = sink_active and not raw_sink
            entries = materialize_acquisition(acquisition, None if legacy_sink else selected,
                                               message_types=self.message_types)
            self.emit_group(entries, legacy_sink, subscribers)
        self.sequence += 1

    def imu(self, sample, stamp_ns):
        subscribers = self._subscribed_topics
        sink_active = any(callback is not None for callback in (
            self.sink, self.batch_sink, self.acquisition_sink)) and self.has_sink_consumers()
        selected = IMU_TOPICS.intersection(subscribers)
        if not sink_active and not selected:
            return
        acquisition = imu_acquisition(sample, stamp_ns)
        raw_sink = sink_active and self.acquisition_sink is not None
        if raw_sink:
            self.acquisition_sink('osmo360', acquisition)
        legacy_sink = sink_active and not raw_sink
        entries = materialize_acquisition(acquisition, None if legacy_sink else selected,
                                           message_types=self.message_types)
        self.emit_group(entries, legacy_sink, subscribers)
