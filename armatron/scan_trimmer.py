import math
import socket, threading, time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class ScanTrimmer(Node):
    def __init__(self):
        super().__init__("scan_trimmer")

        self.raw_scan_topic = self.declare_parameter(
          'raw_scan_topic', 'scan_raw').get_parameter_value().string_value
        
        self.processed_scan_topic = self.declare_parameter(
          'processed_scan_topic', 'scan').get_parameter_value().string_value

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.raw_scan_topic,
            self.on_scan_received,
            1)

        self.scan_publisher = self.create_publisher(LaserScan, self.processed_scan_topic, 1)

    def on_scan_received(self, msg):
        scan = LaserScan()
        # print(f"Received message with {len(msg.ranges)}")

        scan.header = msg.header
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment

        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time

        scan.range_min = 0.35
        scan.range_max = msg.range_max

        for i in msg.ranges:
            if i > 0.35:
                scan.ranges.append(i)
            else:
                scan.ranges.append(float("inf"))

        # print(f"Publishing message with {len(scan.ranges)}")
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    trim_node = ScanTrimmer()

    rclpy.spin(trim_node)

    trim_node.stop()
    trim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()