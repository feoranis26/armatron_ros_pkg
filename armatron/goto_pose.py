import math
import socket, threading, time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener
from tf2_ros import TransformException

class GotoPoseNode(Node):
    def __init__(self):
        super().__init__("goto_pose")

        self.source_frame = self.declare_parameter(
          'source_frame', 'base_link').get_parameter_value().string_value

        self.target_frame = self.declare_parameter(
          'target_frame', 'target').get_parameter_value().string_value

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return

        msg = Twist()

        speed_x = min(max(-0.15, transform.transform.translation.x), 0.15)
        speed_y = min(max(-0.15, transform.transform.translation.y), 0.15)
        speed_th = min(max(-0.1, transform.transform.rotation.z), 0.1)

        msg.linear.x = float(speed_x)
        msg.linear.y = float(speed_y)
        msg.angular.z = float(speed_th)

        self.vel_publisher.publish(msg)
        

    
def main(args=None):
    rclpy.init(args=args)

    drive_node = GotoPoseNode()

    rclpy.spin(drive_node)

    drive_node.stop()
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()