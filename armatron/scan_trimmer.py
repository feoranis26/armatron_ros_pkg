import math
import socket, threading, time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class ScanTrimmer(Node):
    def __init__(self):
        super().__init__("scan_trimmer")

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.on_scan_received,
            1)
        
        self.scan_publisher = self.create_publisher(LaserScan, "scan_trimmed", 1)

        
    def on_scan_received(self, msg):
        scan = LaserScan()

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