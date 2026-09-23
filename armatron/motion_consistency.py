"""Detect persistent disagreement between commanded step motion and RF2O."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class MotionConsistencyMonitor(Node):
    def __init__(self):
        super().__init__('motion_consistency_monitor')
        self.command = Twist()
        self.drive_speed = (0.0, 0.0, 0.0)
        self.rf2o_speed = (0.0, 0.0)
        self.last_rf2o = None
        self.last_rf2o_time = None
        self.disagreement_since = None
        self.latched = False

        self.command_threshold = self.declare_parameter('command_threshold', 0.10).value
        self.drive_threshold = self.declare_parameter('drive_threshold', 0.10).value
        self.rf2o_stationary_threshold = self.declare_parameter('rf2o_stationary_threshold', 0.04).value
        self.persistence_seconds = self.declare_parameter('persistence_seconds', 0.75).value

        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(Odometry, '/odom/drive_raw', self.on_drive, 10)
        self.create_subscription(Odometry, '/odom/rf2o', self.on_rf2o, 10)
        self.fault_pub = self.create_publisher(Bool, '/drive/safety_inhibited', 1)
        self.status_pub = self.create_publisher(String, '/motion_consistency/status', 1)
        self.stop_client = self.create_client(Empty, '/drive/safety_stop')
        self.reset_client = self.create_client(Empty, '/drive/safety_reset')
        self.create_service(Empty, '/motion_consistency/reset', self.reset)
        self.create_timer(0.1, self.evaluate)

    def on_command(self, message):
        self.command = message

    def on_drive(self, message):
        twist = message.twist.twist
        self.drive_speed = (twist.linear.x, twist.linear.y, twist.angular.z)

    def on_rf2o(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))
        now = time.monotonic()
        if self.last_rf2o is not None:
            elapsed = now - self.last_rf2o_time
            if elapsed > 0.02:
                translation = math.hypot(position.x - self.last_rf2o[0],
                                         position.y - self.last_rf2o[1]) / elapsed
                yaw_delta = (yaw - self.last_rf2o[2] + math.pi) % (2.0 * math.pi) - math.pi
                self.rf2o_speed = (translation, abs(yaw_delta) / elapsed)
        self.last_rf2o = (position.x, position.y, yaw)
        self.last_rf2o_time = now

    def evaluate(self):
        if self.latched:
            self.publish('STALLED: drive safety inhibit latched')
            return
        command_speed = math.hypot(self.command.linear.x, self.command.linear.y)
        drive_speed = math.hypot(self.drive_speed[0], self.drive_speed[1])
        now = time.monotonic()
        translational_disagreement = (
            command_speed >= self.command_threshold and
            drive_speed >= self.drive_threshold and
            self.rf2o_speed[0] <= self.rf2o_stationary_threshold
        )
        rotational_disagreement = (
            abs(self.command.angular.z) >= self.command_threshold and
            abs(self.drive_speed[2]) >= self.drive_threshold and
            self.rf2o_speed[1] <= self.rf2o_stationary_threshold
        )
        # An absent/stalled scanner is not evidence of a motor stall.
        disagreement = (translational_disagreement or rotational_disagreement) and (
            self.last_rf2o_time is not None and now - self.last_rf2o_time < 0.5)
        if disagreement:
            self.disagreement_since = self.disagreement_since or now
            if now - self.disagreement_since >= self.persistence_seconds:
                self.latched = True
                self.stop_client.call_async(Empty.Request())
                self.get_logger().error('Persistent drive/RF2O disagreement; safety inhibit latched')
                self.publish('STALLED: persistent drive/RF2O disagreement')
                return
            self.publish('SUSPECT: drive motion is not visible to RF2O')
        else:
            self.disagreement_since = None
            self.publish('NORMAL')

    def publish(self, text):
        self.fault_pub.publish(Bool(data=self.latched))
        self.status_pub.publish(String(data=text))

    def reset(self, request, response):
        self.latched = False
        self.disagreement_since = None
        self.reset_client.call_async(Empty.Request())
        self.get_logger().warn('Motion consistency fault reset by operator')
        self.publish('NORMAL')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionConsistencyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
