"""Immutable Osmo acquisitions and shared ROS materialization.

Builders do not import ROS or serialize CDR. A recording worker can receive
the compact tuples through IPC and materialize the same standard messages.
All clocks are already supplied by acquisition; no timestamp is invented here.
"""

import json

from .runtime import json_value


IMU_TOPICS = frozenset(('/osmo360/imu', '/osmo360/imu/timing'))
VIDEO_TOPICS = frozenset(('/osmo360/image/compressed', '/osmo360/video/h264',
                          '/osmo360/video/timing', '/osmo360/video/metadata'))


def imu_acquisition(sample, stamp_ns):
    """('imu', mapped_ns, native_ns, host_ns, ticks, gyro3, accel3, astatus, gstatus)."""
    return ('imu', stamp_ns, sample.timestamp_ns, sample.host_receive_monotonic_ns,
            sample.timestamp_ticks, tuple(sample.gyro), tuple(sample.accel),
            sample.accel_status, sample.gyro_status)


def video_acquisition(frame, jpeg, stamp_ns, sequence):
    """('video', mapped_ns, native_ns|None, host_ns, seq, h264, jpeg, metadata_json)."""
    valid_native = frame.timestamp_ns is not None
    metadata = dict(frame.metadata, _rover_timing={
        'valid_native': valid_native,
        'stamp_source': 'camera_native' if valid_native else 'host_receive',
    })
    encoded = json.dumps(json_value(metadata), allow_nan=False, separators=(',', ':'))
    return ('video', stamp_ns, frame.timestamp_ns, frame.host_receive_monotonic_ns,
            sequence, bytes(frame.raw_h264), bytes(jpeg), encoded)


def set_header(message, stamp_ns, frame_id):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.header.frame_id = frame_id
    return message


def compressed_image(data, stamp_ns, frame_id, format='jpeg', *, message_type=None):
    if message_type is None:
        from sensor_msgs.msg import CompressedImage
        message_type = CompressedImage
    message = set_header(message_type(), stamp_ns, frame_id)
    message.format, message.data = format, data
    return message


def materialize_acquisition(acquisition, topics=None, *, message_types=None):
    """Return (topic, ROS message, mapped_ns) entries, optionally selecting topics.

    ROS imports are lazy. An existing ROS adapter can supply its already loaded
    message types; an IPC recording worker uses the default standard types.
    Missing native video clocks remain excluded from the synchronized JPEG topic.
    """
    if not isinstance(acquisition, tuple) or not acquisition:
        raise ValueError('Expected an immutable Osmo acquisition tuple')
    kind = acquisition[0]
    if kind not in ('imu', 'video') or len(acquisition) != (9 if kind == 'imu' else 8):
        raise ValueError('Unknown Osmo acquisition schema')
    selected = IMU_TOPICS if kind == 'imu' else VIDEO_TOPICS
    if kind == 'video' and acquisition[2] is None:
        selected = selected - {'/osmo360/image/compressed'}
    if topics is not None:
        selected = selected.intersection(topics)
    if not selected:
        return ()
    if message_types is None:
        from sensor_msgs.msg import CompressedImage, Imu
        from std_msgs.msg import String, UInt64MultiArray
        message_types = dict(CompressedImage=CompressedImage, Imu=Imu,
                             String=String, UInt64MultiArray=UInt64MultiArray)
    stamp = acquisition[1]
    entries = []
    if kind == 'imu':
        _, _, native, host, ticks, gyro, accel, accel_status, gyro_status = acquisition
        if '/osmo360/imu' in selected:
            message = set_header(message_types['Imu'](), stamp, 'osmo360_imu_native')
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = accel
            message.orientation_covariance[0] = -1.0
            entries.append(('/osmo360/imu', message, stamp))
        if '/osmo360/imu/timing' in selected:
            entries.append(('/osmo360/imu/timing', message_types['UInt64MultiArray'](
                data=[stamp, native, host, ticks, accel_status, gyro_status]), stamp))
    else:
        _, _, native, host, sequence, raw, jpeg, metadata_json = acquisition
        if '/osmo360/image/compressed' in selected:
            entries.append(('/osmo360/image/compressed', compressed_image(
                jpeg, stamp, 'osmo360_dual_fisheye', 'bgr8; jpeg compressed bgr8',
                message_type=message_types['CompressedImage']), stamp))
        if '/osmo360/video/h264' in selected:
            entries.append(('/osmo360/video/h264', compressed_image(
                raw, stamp, 'osmo360_dual_fisheye', 'h264',
                message_type=message_types['CompressedImage']), stamp))
        if '/osmo360/video/timing' in selected:
            entries.append(('/osmo360/video/timing', message_types['UInt64MultiArray'](
                data=[stamp, native if native is not None else 0, host, sequence,
                      int(native is not None)]), stamp))
        if '/osmo360/video/metadata' in selected:
            entries.append(('/osmo360/video/metadata', message_types['String'](data=metadata_json), stamp))
    return tuple(entries)
