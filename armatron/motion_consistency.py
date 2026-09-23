"""Diagnose motion and weight lidar/step translation, never propulsion."""
import copy
import json
import math
import time
import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .motion_window import MotionWindow
from .fusion_weights import FusionWeights, PoseVelocity, twist_covariance


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
        self.adaptive = param('adaptive_fusion', True)
        self.fallback_enabled = param('wheel_fallback', True)
        self.weights = FusionWeights(param('rf_velocity_variance', 0.0025),
                                     param('rf_weak_velocity_variance', 1.0),
                                     param('wheel_fallback_variance', 0.04),
                                     param('fusion_weak_ratio', 0.20))
        self.velocity = PoseVelocity()
        self.rf_yaw = 0.
        self.rf_stamp = float('-inf')
        self.fusion_mode = 'NO_CONFIDENCE'
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
        self.fusion_pub = self.create_publisher(String, '/odometry/fusion_status', 10)
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
            if (data.get('schema') not in (1, 2) or data['state'] not in allowed or
                    data.get('candidate_state') not in allowed or not -0.1 <= age <= 0.8):
                return
            self.confidence = data
            self.confidence_at = time.monotonic()
        except (ValueError, TypeError, KeyError):
            return

    def on_drive(self, msg):
        if msg.child_frame_id != 'base_link' or not self.current_measurement(msg):
            return
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.angular.z)
        if not self.window.add('drive', stamp(msg), values):
            return
        self.drive_at = time.monotonic()
        self.evaluate()
        _, wheel, _ = self.fusion_weights(stamp(msg))
        if self.gate and wheel is not None:
            out = copy.deepcopy(msg)
            out.twist.covariance = twist_covariance(wheel)
            self.drive_pub.publish(out)

    def on_rf(self, msg):
        if msg.child_frame_id != 'base_link' or not self.current_measurement(msg):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not self.window.add('rf', stamp(msg), (p.x,p.y,yaw)):
            return
        self.rf_at = time.monotonic()
        self.rf_yaw, self.rf_stamp = yaw, stamp(msg)
        velocity = self.velocity.update(stamp(msg), (p.x, p.y), yaw,
                                        (msg.header.frame_id, msg.child_frame_id))
        rf, _, self.fusion_mode = self.fusion_weights(stamp(msg))
        if velocity is not None:
            out = copy.deepcopy(msg)
            out.pose.covariance = (np.eye(6)*1e6).ravel().tolist()
            out.twist.twist.linear.x = float(velocity[0])
            out.twist.twist.linear.y = float(velocity[1])
            out.twist.twist.angular.z = 0.0
            out.twist.covariance = twist_covariance(rf)
            self.rf_pub.publish(out)
        self.evaluate()

    def fusion_weights(self, timestamp):
        if not self.adaptive:
            return np.eye(2)*self.weights.lidar_variance, None, 'LIDAR_FIXED'
        evidence = self.confidence if time.monotonic()-self.confidence_at < 0.8 else None
        return self.weights.get(evidence, self.rf_yaw, timestamp)

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
        # Geometry controls estimator weights only, never motor permissions.
        evidence_fresh = self.confidence is not None and now-self.confidence_at < 0.8
        state = self.confidence['state'] if evidence_fresh and healthy else 'UNAVAILABLE'
        _, wheel, self.fusion_mode = self.fusion_weights(
            self.get_clock().now().nanoseconds/1e9)
        # A mismatch in a weak direction is the reason for fallback, not a veto.
        # Explicit drive inhibition and stale telemetry still suppress its input.
        self.gate = (self.fallback_enabled and healthy and self.ack is False and wheel is not None)
        if self.fusion_mode == 'WHEEL_FALLBACK' and not self.gate:
            self.fusion_mode = 'LIDAR_WEAK'
        detail = ('sensors unavailable/warming up' if not healthy else
                  'lidar confidence unavailable' if not evidence_fresh else
                  self.confidence.get('reason', state))
        detail += '; raw estimates disagree=' + str(bool(bad))
        if evidence_fresh:
            detail += '; candidate=' + self.confidence['candidate_state']
        status = f'{state}: {detail}; wheel gate={self.gate}; Pi inhibit={self.ack if now-self.ack_at<0.5 else "UNKNOWN"}'
        self.status_pub.publish(String(data=status))
        self.valid_pub.publish(Bool(data=self.gate))
        self.fusion_pub.publish(String(data=f'{self.fusion_mode}; wheel fallback={self.gate}'))
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
