"""Pause on heading loss and automatically resume after fresh observations."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from robot_localization.srv import ToggleFilterProcessing, SetPose

from .map_state import state_root


class HeadingWatchdog:
    def __init__(self, now):
        self.received = None
        self.stamp = -math.inf
        self.healthy_since = None

    def sample(self, stamp, ros_now, now):
        if math.isfinite(stamp) and stamp > self.stamp and -0.1 <= ros_now-stamp <= 0.5:
            if self.received is None or now-self.received > 1.0:
                self.healthy_since = now
            self.stamp, self.received = stamp, now

    def ready(self, now):
        return (self.received is not None and now-self.received <= 1.0 and
                now-self.healthy_since >= 0.3)


class HeadingGuard(Node):
    def __init__(self):
        super().__init__('heading_guard')
        # Obsolete latch files no longer control operation or map persistence.
        marker = state_root() / 'heading_fault.json'
        if marker.exists():
            try:
                marker.unlink()
            except OSError as error:
                self.get_logger().warning(f'Ignoring obsolete heading fault marker: {error}')
        self.watchdog = HeadingWatchdog(time.monotonic())
        self.phase = 'STOP'
        self.pending = None
        self.pending_client = None
        self.request_at = -math.inf
        self.odom = None
        self.odom_at = -math.inf
        self.last_status = None
        self.ready_pub = self.create_publisher(Bool, '/odometry/heading_ready', 1)
        self.status_pub = self.create_publisher(String, '/odometry/heading_status', 1)
        self.toggle = self.create_client(ToggleFilterProcessing, '/toggle')
        self.set_pose = self.create_client(SetPose, '/set_pose')
        self.create_subscription(Imu, '/imu/gyro', self.on_imu, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 1)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        if msg.header.frame_id == 'odom':
            self.odom = msg
            self.odom_at = time.monotonic()

    def on_imu(self, msg):
        q = msg.orientation
        if (msg.header.frame_id != 'base_link' or msg.orientation_covariance[0] < 0 or
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            return
        self.watchdog.sample(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                             self.get_clock().now().nanoseconds*1e-9, time.monotonic())

    def request(self, client, request, now):
        if client.service_is_ready():
            self.pending_client = client
            self.pending = client.call_async(request)
            self.request_at = now

    def tick(self):
        now = time.monotonic()
        healthy = self.watchdog.ready(now)
        if self.pending is not None:
            if self.pending.done():
                try:
                    self.pending.result()
                    # Toggle's status=false means already in the requested state.
                    self.phase = {'STOP': 'PAUSED', 'RESET': 'START', 'START': 'RUNNING'}[self.phase]
                except Exception as error:
                    self.get_logger().error(f'Heading recovery service failed: {error}')
                    self.phase = 'STOP'
                self.pending = None
            elif now-self.request_at > 2.0:
                self.pending_client.remove_pending_request(self.pending)
                self.pending = None
                self.phase = 'STOP'
                self.get_logger().warning('Heading recovery service timed out; retrying pause')
        if self.pending is None:
            if not healthy and self.phase in ('START', 'RUNNING', 'RESET'):
                self.phase = 'STOP'
            if self.phase == 'STOP':
                req = ToggleFilterProcessing.Request()
                req.on = False
                self.request(self.toggle, req, now)
            elif self.phase == 'PAUSED' and healthy:
                # Held filtered pose is still published while toggled off. At
                # first startup the filter may not yet have initialized.
                self.phase = 'RESET' if self.odom is not None else 'START'
            elif self.phase == 'RESET' and now-self.odom_at <= 0.5:
                req = SetPose.Request()
                req.pose.header = copy.deepcopy(self.odom.header)
                req.pose.header.stamp = self.get_clock().now().to_msg()
                req.pose.pose = copy.deepcopy(self.odom.pose)
                # SetPose preserves pose but clears velocity/acceleration and
                # queued observations. IMU yaw is already in the odom reference.
                self.request(self.set_pose, req, now)
            elif self.phase == 'START':
                req = ToggleFilterProcessing.Request()
                req.on = True
                self.request(self.toggle, req, now)
        ready = healthy and self.phase == 'RUNNING'
        self.ready_pub.publish(Bool(data=ready))
        status = 'OK' if ready else ('WAITING: fresh gyro data' if not healthy else
                                     'RECOVERING: ' + self.phase.lower())
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = HeadingGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
