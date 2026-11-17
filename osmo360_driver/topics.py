"""Topic names and ROS types, usable without a ROS installation."""

TOPICS = {
    '/osmo360/image/compressed': 'sensor_msgs/msg/CompressedImage',
    '/osmo360/video/h264': 'sensor_msgs/msg/CompressedImage',
    '/osmo360/imu': 'sensor_msgs/msg/Imu',
    '/osmo360/video/timing': 'std_msgs/msg/UInt64MultiArray',
    '/osmo360/imu/timing': 'std_msgs/msg/UInt64MultiArray',
    '/osmo360/video/metadata': 'std_msgs/msg/String',
}
