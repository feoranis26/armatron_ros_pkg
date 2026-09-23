import rclpy
import time
import copy
import json
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .scan_geometry import mask_ranges, validate_boxes

class ScanTrimmer(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.heading_ready = False
        self.heading_at = float('-inf')
        self.create_subscription(Bool, '/odometry/heading_ready', self.on_heading, 1)
        self.boxes = validate_boxes(json.loads(self.declare_parameter('profile_boxes', '[]').value))
        self.mask_frame = self.declare_parameter('mask_frame', '').value
        self.padding = float(self.declare_parameter('mask_padding', 0.005).value)
        if not math.isfinite(self.padding) or self.padding < 0:
            raise ValueError('mask_padding must be finite and nonnegative')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.last_mask_warning = float('-inf')

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

    def on_heading(self, msg):
        self.heading_ready, self.heading_at = msg.data, time.monotonic()

    def on_scan_received(self, msg):
        if not self.heading_ready or time.monotonic()-self.heading_at > 0.3:
            return
        scan = copy.deepcopy(msg)
        transform = (0., 0., 0.)
        boxes = self.boxes
        if boxes and self.mask_frame and self.mask_frame != msg.header.frame_id:
            try:
                t = self.tf.lookup_transform(self.mask_frame, msg.header.frame_id,
                                             Time.from_msg(msg.header.stamp)).transform
                q = t.rotation
                if abs(q.x) > .01 or abs(q.y) > .01:
                    raise ValueError('Mask transform must be planar')
                transform = (t.translation.x, t.translation.y,
                             math.atan2(2*q.w*q.z, 1-2*q.z*q.z))
            except (TransformException, ValueError) as error:
                # Keep returns if a mask cannot be located; never erase guessed zones.
                boxes = []
                if time.monotonic()-self.last_mask_warning > 5:
                    self.get_logger().error(f'Self-mask unavailable; forwarding unmasked scan: {error}')
                    self.last_mask_warning = time.monotonic()
        scan.ranges = mask_ranges(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max, boxes, self.padding, transform)
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    trim_node = ScanTrimmer()

    try:
        rclpy.spin(trim_node)
    finally:
        trim_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
