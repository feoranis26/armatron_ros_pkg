import math
import socket, threading, time

import rclpy
import tf2_geometry_msgs

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

        self.base_frame = self.declare_parameter(
          'base_frame', 'base_link').get_parameter_value().string_value

        self.pose_subscriber = self.create_subscription(PoseStamped, "/goal_pose", self.pose_received)

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.update)

        self.pose = None

    def update(self):
        if self.pose == None:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.pose.header.frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame} to {self.pose.header.frame_id}: {ex}')
            return

        pose_transformed = tf2_geometry_msgs.do_transform_pose(self.pose, transform)

        msg = Twist()

        speed_x = min(max(-0.15, pose_transformed.position.x), 0.15)
        speed_y = min(max(-0.15, pose_transformed.positionn.y), 0.15)
        speed_th = min(max(-0.1, pose_transformed.rotation.z), 0.1)

        msg.linear.x = float(speed_x)
        msg.linear.y = float(speed_y)
        msg.angular.z = float(speed_th)

        self.vel_publisher.publish(msg)
    
    def pose_received(self, msg):
        self.pose = msg

    
def main(args=None):
    rclpy.init(args=args)

    drive_node = GotoPoseNode()

    rclpy.spin(drive_node)

    drive_node.stop()
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()