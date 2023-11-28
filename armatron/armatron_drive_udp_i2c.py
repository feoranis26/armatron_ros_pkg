import math
import socket, threading, time
from simple_pid import PID

from smbus import SMBus
import numpy as np
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .whl_udp import WheelDriver
from .imu_udp import IMU

class ArmatronDrive(Node):
    def __init__(self):
        super().__init__("armatron_drive")
        self.powerstate = False
        self.speed = Twist()
        self.odom_speed = [0.0, 0.0]

        self.position = Point()
        self.pose = Pose()
        self.heading = 0

        self.last_position = [0, 0]

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

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.tick)
        self.timer = self.create_timer(0.2, self.drive_tick)

        self.absolute = False

        self.driver = WheelDriver("10.8.1.2", 11753)
        self.bus = SMBus(1)
        self.imu = MPU9250.MPU9250(self.bus, 0x68)
        self.imu.begin()

        self.sensorfusion = kalman.Kalman()
        
        #package_share_directory = get_package_share_directory('mpu9250_ros')
        #self.imu.loadCalibDataFromFile(package_share_directory + "/calib.json")

        self.stop_event = threading.Event()
        self.imu_daemon = threading.Thread(target=self.gyro_integrate, daemon=True)
        self.imu_daemon.start()

        self.heading = 0.0
        self.angular_speed = 0.0
        self.hold = 0

        self.pid = PID(0.2, 0.05, 0.0, setpoint=0)

    def stop(self):
        self.driver.drive(0, 0, 0)
        self.stop_event.set()
        super().stop()

    def tick(self):
        #if time.time() - self.lastSpeedReceived > 1 and self.speed != Twist():
        #    self.set_speed(Twist())

        self.odom_update()

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

        pos_diff = [self.last_position[0] - self.driver.position[0], self.last_position[1] - self.driver.position[1]]
        if self.last_position[0] == 0 and self.last_position[1] == 0:
            pos_diff = [0, 0]

        self.last_position[0] = self.driver.position[0]
        self.last_position[1] = self.driver.position[1]

        self.position.x += pos_diff[0] * math.cos(self.heading) + -pos_diff[1] * math.sin(self.heading)
        self.position.y += pos_diff[1] * math.cos(self.heading) + pos_diff[0] * math.sin(self.heading)

        self.odom_speed[0] = self.driver.speed[0] * math.cos(self.heading) + -self.driver.speed[1] * math.sin(self.heading)
        self.odom_speed[1] = self.driver.speed[1] * math.cos(self.heading) + self.driver.speed[0] * math.sin(self.heading)

    def drive_tick(self):
        print(self.heading)
        print(self.position)
        print(self.tgt_speed)
        print(self.odom_speed)
        print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw))

        self.driver.drive(self.speed_x, self.speed_y, self.speed_th)# self.tgt_speed[2])

    def odom_update(self):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.heading / 2)
        quaternion.w = math.cos(self.heading / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = (self.get_clock().now() + Duration(seconds=0.2)).to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.position.x
        transform_stamped_msg.transform.translation.y = self.position.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.position.x
        odom.pose.pose.position.y = self.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.odom_speed[0]
        odom.twist.twist.linear.y = self.odom_speed[1]
        odom.twist.twist.angular.z = self.angular_speed
        self.odom_publisher.publish(odom)

    def on_vel_msg_received(self, msg):
        self.set_speed(msg)
        self.lastSpeedReceived = time.time()

    def on_hold_msg_received(self, msg):
        self.hold = msg.value

    def on_hold_service_called(self, request, response):
        print("Hold heading!")
        if self.hold == 0.0:
            self.hold = self.heading
        else:
            self.hold = 0.0

        return response

    def set_speed(self, speed):
        self.tgt_speed[0] = speed.linear.x
        self.tgt_speed[1] = speed.linear.y
        self.tgt_speed[2] = min(max(speed.angular.z, -0.25), 0.25)

    def gyro_integrate(self):
        last = time.time()
        while not self.stop_event.is_set():
            self.imu.readSensor()

            current = time.time()
            delta = current - last
            last = current

            self.imu.computeOrientation()
            self.sensorfusion.computeAndUpdateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2], self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], delta)



            self.angular_speed = -self.imu.GyroVals[2] + 0.00055
            self.heading += self.angular_speed * delta


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    drive_node = ArmatronDrive()

    rclpy.spin(drive_node)

    drive_node.stop()
    drive_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()