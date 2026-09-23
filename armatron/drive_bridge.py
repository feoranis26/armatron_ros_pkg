import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from .gyro_imu import GyroImuAdapter

from .drive_protocol import WheelDriver
from .gyro_protocol import UDPGyro


class ArmatronDrive(Node):
    def __init__(self):
        super().__init__("armatron_drive")
        self.powerstate = False
        self.speed = Twist()
        self.odom_speed = [0.0, 0.0]

        self.position = Point()
        self.heading = 0

        self.last_position = [float("nan"), float("nan")]

        self.base_frame_id = "base_link"
        self.odom_frame_id = "odom"

        self.tgt_speed = [0, 0, 0]

        self.lastSpeedReceived = time.time()

        self.vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_vel_msg_received,
            1)

        self.hold_subscriber = self.create_subscription(
            Float64,
            '/hold_heading',
            self.on_hold_msg_received,
            1)

        self.hold_service = self.create_service(
            Empty,
            "/hold_heading",
            self.on_hold_service_called)

        self.reset_service = self.create_service(
            Empty,
            "/odom_reset",
            self.on_reset_service_called)

        # This is an observation from open-loop step pulses, never the
        # authoritative odom -> base_link transform.
        self.odom_publisher = self.create_publisher(Odometry, "/odom/drive_raw", 10)
        self.gyro_status_publisher = self.create_publisher(String, '/gyro/status', 10)
        self.gyro_imu_publisher = self.create_publisher(Imu, '/imu/gyro', 10)
        # Initial uncertainty assumption (~2.9 degree standard deviation), not
        # a claim of measured BNO accuracy. Keep configurable for validation.
        self.gyro_imu = GyroImuAdapter(
            self.declare_parameter('gyro_yaw_variance', 0.0025).value, Imu)
        self.gyro_was_fresh = None
        self.last_gyro_warning = float('-inf')
        self.safety_stop_service = self.create_service(
            Empty, "/drive/safety_stop", self.on_safety_stop)
        self.safety_reset_service = self.create_service(
            Empty, "/drive/safety_reset", self.on_safety_reset)

        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.print_status)

        self.absolute = False

        controller_host = self.declare_parameter('controller_host', '10.8.3.56').value
        drive_port = self.declare_parameter('drive_port', 11753).value
        drive_listen_port = self.declare_parameter('drive_listen_port', 11754).value
        gyro_port = self.declare_parameter('gyro_port', 11755).value
        gyro_listen_port = self.declare_parameter('gyro_listen_port', 11757).value

        self.driver = WheelDriver(controller_host, drive_port, drive_listen_port)
        self.driver.start()

        #self.i2c = board.I2C()
        #self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
        #self.imu.mode = adafruit_bno055.IMUPLUS_MODE
            #exit()

        self.gyro = UDPGyro(controller_host, gyro_port, gyro_listen_port)
        self.gyro.start()
        
        #package_share_directory = get_package_share_directory('mpu9250_ros')
        #self.imu.loadCalibDataFromFile(package_share_directory + "/calib.json")


        self.heading = 0.0
        self.angular_speed = 0.0
        self.hold = 0


    def stop(self):
        self.driver.stop()
        self.gyro.stop()
        #super().stop()

    def tick(self):
        if time.time() - self.lastSpeedReceived > 1:
            self.set_speed(Twist())

        #read_yaw = self.imu.euler[0]
        read_yaw = self.gyro.angle
        if read_yaw is not None:
            self.heading = -math.radians(read_yaw)

        imu = self.gyro_imu.message(
            self.gyro.sample, time.monotonic(),
            self.get_clock().now().to_msg(), self.base_frame_id)
        if imu is not None:
            self.gyro_imu_publisher.publish(imu)

        #quat = self.imu.quaternion
        quaternion = Quaternion()

        #if quat is not None:
        try:
            #quaternion.x = quat[0]
            #quaternion.y = quat[1]
            #quaternion.z = quat[2]
            #quaternion.w = quat[3]
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = math.sin(self.heading / 2)
            quaternion.w = math.cos(self.heading / 2)

            self.orientation = quaternion
        except AssertionError:
            pass

        if self.absolute or self.hold != 0:
            self.hdg_compensation = self.heading - self.hold
            self.speed_x = self.tgt_speed[0] * math.cos(self.hdg_compensation) + self.tgt_speed[1] * math.sin(self.hdg_compensation)
            self.speed_y = self.tgt_speed[1] * math.cos(self.hdg_compensation) + -self.tgt_speed[0] * math.sin(self.hdg_compensation)
        else:
            self.speed_x = self.tgt_speed[0]
            self.speed_y = self.tgt_speed[1]

        if self.hold != 0.0:
            self.speed_th = min(max((self.hold - self.heading), -0.2), 0.2)
        else:
            self.speed_th = self.tgt_speed[2]

        if math.isnan(self.last_position[0]) and math.isnan(self.last_position[1]):
            self.last_position[0] = self.driver.position[0]
            self.last_position[1] = self.driver.position[1]

        pos_diff = [self.last_position[0] - self.driver.position[0], self.last_position[1] - self.driver.position[1]]
        if self.last_position[0] == 0 and self.last_position[1] == 0:
            pos_diff = [0, 0]

        self.last_position[0] = self.driver.position[0]
        self.last_position[1] = self.driver.position[1]

        self.position.x += pos_diff[0] * math.cos(self.heading) + -pos_diff[1] * math.sin(self.heading)
        self.position.y += pos_diff[1] * math.cos(self.heading) + pos_diff[0] * math.sin(self.heading)

        # Odometry.twist is expressed in child_frame_id (base_link), not odom.
        self.odom_speed[0] = self.driver.speed[0]
        self.odom_speed[1] = self.driver.speed[1]
        self.odom_update()

        self.driver.drive(self.speed_x, self.speed_y, self.speed_th)
        self.driver.update()

    def print_status(self):
        fresh = self.gyro.angle is not None
        self.gyro_status_publisher.publish(String(data=(
            'OK' if fresh else 'STALE: no valid gyro angle within 1 second; raw pose heading held')))
        now = time.monotonic()
        if not fresh and now - self.last_gyro_warning >= 5.0:
            self.get_logger().error(
                'Gyro data missing/stale. Raw odometry heading is held, not measured. '
                'Check Pi armatron-gyro.service and UDP 11755/11757.')
            self.last_gyro_warning = now
        elif fresh and self.gyro_was_fresh is not True:
            self.get_logger().info('Valid gyro telemetry received')
        self.gyro_was_fresh = fresh
        self.get_logger().debug(f"Heading:\t\t {self.heading}")
        self.get_logger().debug(f"Position:\t\t {self.position}")
        self.get_logger().debug(f"Tgt speed:\t\t {self.tgt_speed}")
        self.get_logger().debug(f"Odom speed: {self.odom_speed}")


    def odom_update(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.position.x
        odom.pose.pose.position.y = self.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.orientation
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.odom_speed[0]
        odom.twist.twist.linear.y = self.odom_speed[1]
        odom.twist.twist.angular.z = float(self.driver.speed[2] or 0.0)
        self.odom_publisher.publish(odom)

    def on_safety_stop(self, request, response):
        self.driver.safety_stop()
        self.get_logger().error("Drive safety inhibit requested")
        return response

    def on_safety_reset(self, request, response):
        self.driver.safety_reset()
        self.get_logger().warn("Drive safety inhibit reset requested")
        return response

    def on_vel_msg_received(self, msg):
        self.get_logger().debug(f"Received spd msg l x: {msg.linear.x} y: {msg.linear.y} z: {msg.linear.z} a x: {msg.angular.x} y: {msg.angular.y} z: {msg.angular.z}")
        self.set_speed(msg)
        self.lastSpeedReceived = time.time()

    def on_hold_msg_received(self, msg):
        self.hold = msg.value

    def on_hold_service_called(self, request, response):
        self.get_logger().info("Hold heading!")
        if self.hold == 0.0:
            self.hold = self.heading
        else:
            self.hold = 0.0

        return response

    def on_reset_service_called(self, request, response):
        self.get_logger().info("Reset odometry!")
        
        self.position = Point()
        self.heading = 0

        self.last_position = [0, 0]

        return response

    def set_speed(self, speed):
        self.tgt_speed[0] = speed.linear.x
        self.tgt_speed[1] = speed.linear.y
        self.tgt_speed[2] = speed.angular.z


def main(args=None):
    rclpy.init(args=args)

    drive_node = ArmatronDrive()

    try:
        rclpy.spin(drive_node)
    finally:
        drive_node.stop()
        drive_node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
