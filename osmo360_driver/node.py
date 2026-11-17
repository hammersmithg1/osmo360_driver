"""Standalone driver. Rover composes the same owner with its HTTP/recorder."""

import json
import signal

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .runtime import OsmoCamera
from .ros import OsmoTopics


class OsmoNode(Node):
    def __init__(self):
        super().__init__('osmo360')
        self.declare_parameter('enabled', False)
        self.topics = OsmoTopics(self)
        self.camera = OsmoCamera(self.topics.video, self.topics.imu,
                                 lambda error: self.get_logger().error(error))
        self.status_publisher = self.create_publisher(String, '~/status', 1)
        self.create_timer(1, self.update_status)
        self.create_service(SetBool, '~/stream', self.stream)
        for action in ('wake', 'sleep', 'reboot', 'refresh'):
            self.create_service(Trigger, '~/' + action, self.trigger(action))
        # These are requested values; ~/status.settings reports actual readback.
        for key, default in dict(exposure_mode=0, iso=0, white_balance=0, color=0,
                                 frame_rate=14, shutter_denominator=250).items():
            self.declare_parameter(key, default)
        self.add_on_set_parameters_callback(self.configure)
        if self.get_parameter('enabled').value:
            self.camera.command('start')

    def update_status(self):
        self.topics.refresh_subscribers()
        self.status_publisher.publish(String(data=json.dumps(self.camera.status())))

    def stream(self, request, response):
        return self.enqueue('start' if request.data else 'stop', response)

    def trigger(self, action):
        return lambda request, response: self.enqueue(action, response)

    def enqueue(self, action, response):
        try:
            self.camera.command(action)
            response.success, response.message = True, 'Queued; observe ~/status for completion'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def configure(self, parameters):
        try:
            self.camera.command('configure', {p.name: p.value for p in parameters})
            return SetParametersResult(successful=True, reason='Queued; observe ~/status.settings and error')
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def destroy_node(self):
        try:
            self.camera.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OsmoNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
