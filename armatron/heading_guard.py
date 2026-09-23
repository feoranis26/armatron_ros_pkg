"""Latch real heading loss; never infer sensor failure from motion disagreement."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from robot_localization.srv import ToggleFilterProcessing

from .map_state import state_root


class HeadingWatchdog:
    def __init__(self, now, latched=False):
        self.started = now
        self.received = None
        self.stamp = -math.inf
        self.latched = latched

    def sample(self, stamp, ros_now, now):
        self.ready(now)  # Detect the gap even if recovery arrives before the timer.
        # Duplicates or replayed messages must not feed the watchdog.
        if math.isfinite(stamp) and stamp > self.stamp and -0.1 <= ros_now-stamp <= 0.5:
            self.stamp, self.received = stamp, now

    def ready(self, now):
        if ((self.received is None and now-self.started > 5.0) or
                (self.received is not None and now-self.received > 1.0)):
            self.latched = True
        return not self.latched and self.received is not None


class HeadingGuard(Node):
    def __init__(self):
        super().__init__('heading_guard')
        self.marker = state_root() / 'heading_fault.json'
        self.watchdog = HeadingWatchdog(time.monotonic(), self.marker.exists())
        self.recorded = self.marker.exists()
        self.pending = None
        self.last_request = -math.inf
        self.last_log = -math.inf
        self.ready_pub = self.create_publisher(Bool, '/odometry/heading_ready', 1)
        self.stop_pub = self.create_publisher(Bool, '/drive/inhibit_request', 1)
        self.status_pub = self.create_publisher(String, '/odometry/heading_status', 1)
        self.toggle = self.create_client(ToggleFilterProcessing, '/toggle')
        self.create_subscription(Imu, '/imu/gyro', self.on_imu, 10)
        self.create_timer(0.05, self.tick)

    def on_imu(self, msg):
        q = msg.orientation
        if (msg.header.frame_id != 'base_link' or msg.orientation_covariance[0] < 0 or
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
            return
        self.watchdog.sample(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                             self.get_clock().now().nanoseconds*1e-9, time.monotonic())

    def tick(self):
        now = time.monotonic()
        ready = self.watchdog.ready(now)
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data='OK' if ready else
            'FAULT_LATCHED: gyro lost; scans blocked; repair and restart required'
            if self.watchdog.latched else 'WAITING: gyro startup'))
        if not self.watchdog.latched:
            return
        self.stop_pub.publish(Bool(data=True))
        if not self.recorded:
            try:
                self.marker.parent.mkdir(parents=True, exist_ok=True)
                self.marker.write_text(json.dumps({'reason': 'heading loss', 'time': time.time()}))
                self.recorded = True
            except OSError as error:
                if now-self.last_log > 5:
                    self.get_logger().error(f'Cannot persist heading fault: {error}')
        if now-self.last_log > 5:
            self.get_logger().error('Heading lost: drive inhibited, scans blocked, EKF pause requested. '
                                    'Recovery requires stopping navigation and repairing gyro.')
            self.last_log = now
        if self.pending is not None:
            if not self.pending.done():
                if now-self.last_request > 2.0:
                    self.toggle.remove_pending_request(self.pending)
                    self.pending = None
                return
            try:
                self.pending.result()
            except Exception as error:
                self.get_logger().error(f'EKF pause failed: {error}')
            self.pending = None
        # Repeat the idempotent OFF request, including after an EKF restart.
        if now-self.last_request >= 1.0 and self.toggle.service_is_ready():
            request = ToggleFilterProcessing.Request()
            request.on = False
            self.pending = self.toggle.call_async(request)
            self.last_request = now


def main(args=None):
    rclpy.init(args=args)
    node = HeadingGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
