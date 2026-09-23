"""Persist a recent map-frame pose as a restart hint, never as ground truth."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .map_state import load_state, save_state


class PosePersistence(Node):
    def __init__(self):
        super().__init__('map_pose_persistence')
        self.last_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/slam_toolbox/pose',
                                 self.on_pose, 10)
        self.create_timer(5.0, self.persist)

    def on_pose(self, message):
        pose = message.pose.pose
        sin_yaw = 2.0 * (pose.orientation.w * pose.orientation.z +
                         pose.orientation.x * pose.orientation.y)
        cos_yaw = 1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2)
        self.last_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': math.atan2(sin_yaw, cos_yaw),
        }

    def persist(self):
        if self.last_pose is None:
            return
        state = load_state()
        if not state['active_profile']:
            return
        state['last_pose'] = self.last_pose
        save_state(state)


def main(args=None):
    rclpy.init(args=args)
    node = PosePersistence()
    try:
        rclpy.spin(node)
    finally:
        node.persist()
        node.destroy_node()
        rclpy.shutdown()
