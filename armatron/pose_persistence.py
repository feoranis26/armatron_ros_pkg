"""Persist a recent map-frame pose as a restart hint, never as ground truth."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .map_state import load_state, save_state, state_root


class PosePersistence(Node):
    def __init__(self):
        super().__init__('map_pose_persistence')
        self.last_pose = None
        self.profile = load_state()['active_profile']
        topic = self.declare_parameter('pose_topic', '/pose').value
        self.create_subscription(PoseWithCovarianceStamped, topic,
                                 self.on_pose, 10)
        self.create_timer(5.0, self.persist)

    def on_pose(self, message):
        if message.header.frame_id != 'map':
            return
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            return
        sin_yaw = 2.0 * (pose.orientation.w * pose.orientation.z +
                         pose.orientation.x * pose.orientation.y)
        cos_yaw = 1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2)
        first = self.last_pose is None
        self.last_pose = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': math.atan2(sin_yaw, cos_yaw),
        }
        if first:
            self.persist()

    def persist(self):
        if self.last_pose is None:
            return
        state = load_state()
        if not self.profile or state['active_profile'] != self.profile:
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
