"""Diagnose disagreement and gate optional wheel fusion, never propulsion."""
import copy
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .motion_window import MotionWindow


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


class MotionConsistencyMonitor(Node):
    def __init__(self):
        super().__init__('motion_consistency_monitor')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.window = MotionWindow(param('distance_floor', 0.015),
                                   param('relative_error', 0.4), param('angle_floor', 0.10),
                                   param('timing_tolerance', 0.2),
                                   param('rotation_translation_tolerance', 0.125),
                                   param('stationary_distance', 0.08),
                                   param('stationary_angle', 0.25))
        self.use_drive = param('use_drive_fusion', False)
        self.agreement_seconds = param('agreement_seconds', 3.0)
        if not math.isfinite(self.agreement_seconds) or self.agreement_seconds <= 0:
            raise ValueError('agreement_seconds must be positive and finite')
        self.agreement_since = None
        self.drive_variance = param('drive_velocity_variance', 0.0025)
        self.rf_position_variance = param('rf_position_variance', 0.0004)
        self.rf_yaw_variance = param('rf_yaw_variance', 0.0025)
        for value in (self.drive_variance, self.rf_position_variance, self.rf_yaw_variance):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Measurement variances must be positive and finite')
        self.drive_at = self.rf_at = self.ack_at = self.gyro_at = float('-inf')
        self.gyro_ok = False
        self.ack = None
        self.gate = False
        self.last_status = None
        self.confidence = None
        self.confidence_at = float('-inf')
        self.create_subscription(Odometry, '/odom/drive_raw', self.on_drive, 10)
        self.create_subscription(Odometry, '/odom/rf2o', self.on_rf, 10)
        self.create_subscription(Bool, '/drive/safety_inhibited', self.on_ack, 10)
        self.create_subscription(String, '/gyro/status', self.on_gyro, 10)
        self.create_subscription(String, '/lidar/confidence', self.on_confidence, 10)
        self.status_pub = self.create_publisher(String, '/motion_consistency/status', 10)
        self.valid_pub = self.create_publisher(Bool, '/drive/odometry_valid', 10)
        self.drive_pub = self.create_publisher(Odometry, '/odom/drive_validated', 10)
        self.rf_pub = self.create_publisher(Odometry, '/odom/rf2o_fusion', 10)
        self.create_timer(0.05, self.evaluate)

    def on_ack(self, msg):
        self.ack, self.ack_at = msg.data, time.monotonic()

    def on_gyro(self, msg):
        self.gyro_ok, self.gyro_at = msg.data == 'OK', time.monotonic()

    def on_confidence(self, msg):
        try:
            data = json.loads(msg.data)
            allowed = {'CONSISTENT', 'LIDAR_UNDERCONSTRAINED', 'MOTION_CONTRADICTED',
                       'TRACKING_UNRELIABLE', 'UNAVAILABLE'}
            age = self.get_clock().now().nanoseconds/1e9 - float(data['stamp'])
            if (data.get('schema') != 1 or data['state'] not in allowed or
                    data.get('candidate_state') not in allowed or not -0.1 <= age <= 0.8):
                return
            self.confidence = data
            self.confidence_at = time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def on_drive(self, msg):
        if not self.current_measurement(msg):
            return
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.angular.z)
        if not self.window.add('drive', stamp(msg), values):
            return
        self.drive_at = time.monotonic()
        self.evaluate()
        if self.gate and self.use_drive:
            out = copy.deepcopy(msg)
            out.twist.covariance = [0.]*36
            for i in (0,7,14,21,28,35):
                out.twist.covariance[i] = self.drive_variance if i in (0,7) else 1e6
            self.drive_pub.publish(out)

    def on_rf(self, msg):
        if not self.current_measurement(msg):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not self.window.add('rf', stamp(msg), (p.x,p.y,yaw)):
            return
        self.rf_at = time.monotonic()
        # Preserve raw data; give the EKF explicit configurable uncertainty.
        out = copy.deepcopy(msg)
        out.pose.covariance = [0.]*36
        for i in (0,7,14,21,28,35):
            out.pose.covariance[i] = (self.rf_position_variance if i in (0,7) else
                                      self.rf_yaw_variance if i == 35 else 1e6)
        self.rf_pub.publish(out)
        self.evaluate()

    def current_measurement(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - stamp(msg)
        return math.isfinite(age) and -0.1 <= age <= 0.5

    def evaluate(self):
        now = time.monotonic()
        short, long = self.window.compare(0.75), self.window.compare(2.5)
        fresh = (now-self.drive_at < 0.5 and now-self.rf_at < 0.5 and
                 now-self.ack_at < 0.5 and now-self.gyro_at < 1.5 and self.gyro_ok)
        healthy = fresh and short is not None
        bad = healthy and any(result and result['bad'] for result in (short,long))
        # Optional wheel fusion drops immediately, but returns only after a
        # sustained agreement interval. Neither condition commands the motors.
        evidence_fresh = self.confidence is not None and now-self.confidence_at < 0.8
        state = self.confidence['state'] if evidence_fresh and healthy else 'UNAVAILABLE'
        consistent = (healthy and not bad and self.ack is False and state == 'CONSISTENT'
                      and self.confidence['candidate_state'] == 'CONSISTENT')
        if not consistent:
            self.agreement_since = None
        elif self.agreement_since is None:
            self.agreement_since = now
        self.gate = (self.use_drive and consistent and self.agreement_since is not None
                     and now-self.agreement_since >= self.agreement_seconds)
        detail = ('sensors unavailable/warming up' if not healthy else
                  'lidar confidence unavailable' if not evidence_fresh else
                  self.confidence.get('reason', state))
        detail += '; raw estimates disagree=' + str(bool(bad))
        if evidence_fresh:
            detail += '; candidate=' + self.confidence['candidate_state']
        status = f'{state}: {detail}; wheel gate={self.gate}; Pi inhibit={self.ack if now-self.ack_at<0.5 else "UNKNOWN"}'
        self.status_pub.publish(String(data=status))
        self.valid_pub.publish(Bool(data=self.gate and self.use_drive))
        if state != self.last_status:
            self.get_logger().info(status)
            self.last_status = state

def main(args=None):
    rclpy.init(args=args)
    node = MotionConsistencyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
