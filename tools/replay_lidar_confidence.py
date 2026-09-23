#!/usr/bin/env python3
"""Replay a ROS 2 bag through the real confidence node without motor nodes.

Run in a separate ROS_DOMAIN_ID. Publishes only sensor/TF inputs and writes
confidence outputs as JSONL; never publishes commands or drive inhibit requests.
Requires a bag with /scan, /odom/drive_raw, /odom/rf2o, /imu/gyro,
/rf2o/solver_diagnostics and /tf_static. Bag time is driven by recorded /clock
generated here from storage timestamps, so wall-clock sensor age is irrelevant.
"""
import argparse
import json
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from armatron.lidar_confidence import LidarConfidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--output', required=True)
    parser.add_argument('--rate', type=float, default=1.)
    args = parser.parse_args()
    if not 0 < args.rate <= 2:
        parser.error('--rate must be in (0, 2]')
    import os
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', '67'):
        parser.error('Use an isolated ROS_DOMAIN_ID (for example 68), not the robot domain')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id=''), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    required = {'/scan', '/odom/drive_raw', '/odom/rf2o', '/imu/gyro',
                '/rf2o/solver_diagnostics', '/tf_static'}
    if not required <= types.keys():
        parser.error('Missing required topics: '+', '.join(sorted(required-types.keys())))
    rclpy.init()
    node = LidarConfidence()
    node.set_parameters([Parameter('use_sim_time', value=True)])
    feeder = rclpy.create_node('confidence_replay_inputs')
    clock_pub = feeder.create_publisher(Clock, '/clock', 10)
    publishers = {}
    for topic in required | ({'/tf'} & types.keys()):
        qos = (QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic == '/tf_static' else qos_profile_sensor_data if topic == '/scan' else 100)
        publishers[topic] = feeder.create_publisher(get_message(types[topic]), topic, qos)
    counts = {}
    # Exclusive creation protects previous diagnostic recordings.
    with open(args.output, 'x') as output:
        from std_msgs.msg import String
        def receive(msg):
            row = json.loads(msg.data)
            counts[row['state']] = counts.get(row['state'], 0)+1
            output.write(msg.data+'\n')
        feeder.create_subscription(String, '/lidar/confidence', receive, 100)
        def spin():
            rclpy.spin_once(feeder, timeout_sec=0.)
            rclpy.spin_once(node, timeout_sec=0.)
        # Allow DDS discovery before publishing the initial TF and sensor data.
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            spin()
            time.sleep(0.01)
        first = None
        wall = time.monotonic()
        while reader.has_next():
            topic, serialized, timestamp = reader.read_next()
            if topic not in publishers:
                continue
            if first is None:
                first = timestamp
            due = wall+(timestamp-first)*1e-9/args.rate
            while time.monotonic() < due:
                spin()
                time.sleep(0.001)
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(timestamp, 1000000000)
            clock_pub.publish(clock)
            publishers[topic].publish(deserialize_message(serialized, get_message(types[topic])))
            spin()
        for _ in range(100):
            spin()
            time.sleep(0.005)
    node.destroy_node()
    feeder.destroy_node()
    rclpy.shutdown()
    print(json.dumps(counts, indent=2))


if __name__ == '__main__':
    main()
