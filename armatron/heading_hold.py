import math
import threading, time

import numpy as np
import pyquaternion as pq

import rclpy
import tf2_ros
import tf2_geometry_msgs

from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, TransformStamped, Vector3

class HeadingHoldController(Node):
    def __init__(self):
        super().__init__("goto_pose")

        self.base_frame = self.declare_parameter(
          'base_frame', 'base_link').get_parameter_value().string_value
        self.map_frame = self.declare_parameter(
          'map_frame', 'map').get_parameter_value().string_value

        self.vel_publisher = self.create_subscription(Twist, "/cmd_vel_abs", self.on_vel_msg_received)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_vel_msg_received(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.map_frame, rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame} to {self.pose.header.frame_id}: {ex}')
            return
        
        rot = transform.transform.rotation
        quat = pq.Quaternion(rot.w, rot.x, rot.y, rot.z)
        quat.rotate((msg.translation.x, msg.translation.y, msg.translation.z))
        
        self.tf_buffer.transform()


def main(args=None):
    rclpy.init(args=args)

    drive_node = HeadingHoldController()

    try:
        rclpy.spin(drive_node)
    finally:
        drive_node.stop()
        drive_node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()