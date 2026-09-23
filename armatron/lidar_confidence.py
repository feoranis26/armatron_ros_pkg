"""Timestamp-aligned scan hypotheses and diagnostic-only confidence output."""
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .motion_hypotheses import History, hypotheses
from .scan_evidence import ScanEvidence, EvidenceDwell, rotation


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class LidarConfidence(Node):
    def __init__(self):
        super().__init__('lidar_confidence')
        def param(name, value):
            return self.declare_parameter(name, value).value
        self.base_frame = param('base_frame', 'base_link')
        self.interval = float(param('comparison_seconds', 0.6))
        if not 0.3 <= self.interval <= 1.5:
            raise ValueError('comparison_seconds must be in [0.3, 1.5]')
        self.engine = ScanEvidence(**{k: param(k, v) for k, v in {
            'max_points': 240, 'min_points': 30, 'max_gap': 0.5,
            'match_distance': 0.4, 'residual_cap': 0.25, 'min_overlap': 0.55,
            'max_error': 0.12, 'weak_ratio': 0.03, 'min_strength': 0.005,
            'rotation_scale': 2.0, 'score_margin': 0.025, 'probe_distance': 0.10}.items()})
        self.dwell = EvidenceDwell()
        self.drive, self.gyro, self.rf = History(), History(), History()
        self.scans, self.solver = deque(), deque()
        self.done = -float('inf')
        self.last_output = -float('inf')
        self.last_reason = 'waiting for scans and telemetry'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(String, '/lidar/confidence', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/drive_raw', self.on_drive, 30)
        self.create_subscription(Odometry, '/odom/rf2o', self.on_rf, 30)
        self.create_subscription(Imu, '/imu/gyro', self.on_imu, 30)
        self.create_subscription(DiagnosticArray, '/rf2o/solver_diagnostics', self.on_solver, 10)
        self.create_timer(0.1, self.evaluate)

    def on_drive(self, msg):
        if msg.child_frame_id != self.base_frame:
            return
        v = msg.twist.twist.linear
        self.drive.add(stamp(msg), (v.x, v.y, 0.))

    def on_rf(self, msg):
        if msg.child_frame_id != self.base_frame:
            return
        p = msg.pose.pose.position
        self.rf.add(stamp(msg), (p.x, p.y, yaw(msg.pose.pose.orientation)))

    def on_imu(self, msg):
        if msg.header.frame_id == self.base_frame and msg.orientation_covariance[0] >= 0:
            self.gyro.add(stamp(msg), (0., 0., yaw(msg.orientation)))

    def on_solver(self, msg):
        for status in msg.status:
            if status.name == 'rf2o_solver':
                self.solver.append((stamp(msg), {v.key: v.value for v in status.values}))
        while len(self.solver) > 40:
            self.solver.popleft()

    def on_scan(self, msg):
        t = stamp(msg)
        if self.scans and t <= self.scans[-1][0]:
            return
        try:
            transform = self.tf.lookup_transform(self.base_frame, msg.header.frame_id,
                                                  Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            self.last_reason = 'scan-to-base TF unavailable'
            return
        q = transform.rotation
        # Scan geometry is planar: reject tilted mounting rather than silently
        # flattening a non-planar cloud. Robot roll/pitch in the world is unknown.
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if abs(norm-1.) > 0.01 or abs(q.x) > 0.01 or abs(q.y) > 0.01:
            self.last_reason = 'scan mounting is not planar'
            return
        ranges = np.array(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges[~valid] = np.nan
        points = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles)))
        points = points @ rotation(yaw(q)).T + [transform.translation.x, transform.translation.y]
        self.scans.append((t, points))
        while self.scans and self.scans[0][0] < t-3.0:
            self.scans.popleft()

    def emit(self, result):
        self.publisher.publish(String(data=json.dumps(result, allow_nan=False)))

    def evaluate(self):
        ros_now = self.get_clock().now().nanoseconds*1e-9
        mono_now = time.monotonic()
        # Choose the newest alignable scan; tolerate callback ordering without
        # extrapolating telemetry or consuming a scan before RF2O processes it.
        for t, points in reversed(self.scans):
            if t <= self.done or not -0.1 <= ros_now-t <= 0.8:
                continue
            refs = [r for r in self.scans if self.interval <= t-r[0] <= self.interval+0.2]
            if not refs:
                self.last_reason = 'waiting for reference scan'
                continue
            a, reference = refs[-1]
            poses = hypotheses(self.drive, self.gyro, self.rf, a, t)
            if poses is None:
                self.last_reason = 'telemetry does not bracket scan interval'
                continue
            native = [(abs(s-t), values) for s, values in self.solver if abs(s-t) < 0.00001]
            if not native:
                self.last_reason = 'RF2O solver diagnostics missing; build the RF2O confidence-diagnostics fork branch'
                continue
            metrics = min(native, key=lambda row: row[0])[1]
            start = time.monotonic()
            result = self.engine.analyze(reference, points, poses)
            if metrics.get('valid') != '1':
                result.update(state='TRACKING_UNRELIABLE', reason='RF2O solver invalid')
            # Expose native metrics for comparison/calibration. The independent
            # geometric test owns classification; internal weights are not a
            # calibrated probability or physical pose covariance.
            result.update(schema=1, stamp=t, reference_stamp=a, frame_id=self.base_frame,
                          hypotheses={k: v.tolist() for k, v in poses.items()}, solver=metrics)
            result['candidate_state'] = result['state']
            result['state'] = self.dwell.update(t, result['candidate_state'])
            result['evaluation_ms'] = (time.monotonic()-start)*1000
            self.emit(result)
            self.done, self.last_output = t, mono_now
            return
        if mono_now-self.last_output >= 0.5:
            self.dwell = EvidenceDwell()
            self.emit({'schema': 1, 'stamp': ros_now, 'state': 'UNAVAILABLE',
                       'candidate_state': 'UNAVAILABLE', 'reason': self.last_reason})
            self.last_output = mono_now


def main(args=None):
    rclpy.init(args=args)
    node = LidarConfidence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
