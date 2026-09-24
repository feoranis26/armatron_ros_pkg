"""Read-only receipt-time diagnostics; no TF restamping or drive commands."""
import argparse
import json
import math
import time


class Timing:
    def __init__(self):
        self.ages = []
        self.gaps = []
        self.last_at = self.last_stamp = None
        self.repeated = self.backward = 0

    def add(self, stamp, ros_now, monotonic_now):
        if not all(math.isfinite(v) for v in (stamp, ros_now, monotonic_now)):
            return
        self.ages.append(ros_now-stamp)
        if self.last_at is not None:
            self.gaps.append(monotonic_now-self.last_at)
            self.repeated += stamp == self.last_stamp
            self.backward += stamp < self.last_stamp
        self.last_stamp, self.last_at = stamp, monotonic_now

    def report(self, ros_now, monotonic_now):
        if not self.ages:
            return {'samples': 0}
        ordered = sorted(self.ages)
        return {
            'samples': len(ordered),
            'receipt_age_min_s': round(ordered[0], 4),
            'receipt_age_p50_s': round(ordered[(len(ordered)-1)//2], 4),
            'receipt_age_p95_s': round(ordered[math.ceil(.95*len(ordered))-1], 4),
            'receipt_age_max_s': round(ordered[-1], 4),
            'max_receipt_gap_s': round(max(self.gaps, default=0.), 4),
            'seconds_since_last_receipt': round(monotonic_now-self.last_at, 4),
            'last_stamp_age_now_s': round(ros_now-self.last_stamp, 4),
            'repeated_stamps': self.repeated, 'backward_stamps': self.backward,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 300:
        parser.error('duration must be between 1 and 300 seconds')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init(args=[])
    node = Node('armatron_tf_timing')
    streams = {name: Timing() for name in (
        'tf:map->odom', 'tf:odom->base_link', '/scan', '/pose', '/odometry/filtered')}
    def observe(name, stamp):
        streams.setdefault(name, Timing()).add(
            stamp.sec+stamp.nanosec*1e-9,
            node.get_clock().now().nanoseconds*1e-9, time.monotonic())
    def transforms(msg):
        for transform in msg.transforms:
            observe('tf:'+transform.header.frame_id.lstrip('/')+'->'+
                    transform.child_frame_id.lstrip('/'), transform.header.stamp)
    subscriptions = [node.create_subscription(TFMessage, '/tf', transforms, qos_profile_sensor_data)]
    for kind, topic in ((LaserScan, '/scan'), (PoseWithCovarianceStamped, '/pose'),
                        (Odometry, '/odometry/filtered')):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, name=topic: observe(name, msg.header.stamp),
            qos_profile_sensor_data))
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        ros_now = node.get_clock().now().nanoseconds*1e-9
        now = time.monotonic()
        print(json.dumps({
            'duration_s': round(now-started, 3),
            'note': 'Receipt ages include source timestamp policy and delivery delay. '
                    'Negative map->odom age is expected with forward-dated TF. '
                    'These are this probe receipts, not the Nav2 TF cache.',
            'streams': {name: samples.report(ros_now, now) for name, samples in streams.items()},
            'tf_publishers': [{'name': info.node_name, 'namespace': info.node_namespace}
                              for info in node.get_publishers_info_by_topic('/tf')],
        }, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
