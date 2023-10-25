import math
import socket, threading, time
from simple_pid import PID

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
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

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.tick)

        self.absolute = False

        self.driver = WheelDriver("10.8.1.2", 11753)
        self.gyro = IMU("10.8.1.3", 11752)
        self.gyro.calibrate()
        self.gyro.set_yaw(0)

        self.pid = PID(0.2, 0.05, 0.0, setpoint=0)

    def stop(self):
        super().stop()
        self.driver.drive(0, 0, 0)

    def tick(self):
        #if time.time() - self.lastSpeedReceived > 1 and self.speed != Twist():
        #    self.set_speed(Twist())

        self.odom_update()

        self.heading = -self.gyro.yaw * math.pi / 180
        print(self.heading)
        print(self.position)
        print(self.tgt_speed)
        print(self.odom_speed)

        if self.absolute:
            speed_x = self.tgt_speed[0] * math.cos(self.heading) + self.tgt_speed[1] * math.sin(self.heading)
            speed_y = self.tgt_speed[1] * math.cos(self.heading) + -self.tgt_speed[0] * math.sin(self.heading)
        else:
            speed_x = self.tgt_speed[0]
            speed_y = self.tgt_speed[1]
        
        self.driver.drive(speed_x, speed_y, self.tgt_speed[2])


        pos_diff = [self.last_position[0] - self.driver.position[0], self.last_position[1] - self.driver.position[1]]
        self.last_position[0] = self.driver.position[0]
        self.last_position[1] = self.driver.position[1]

        self.position.x += pos_diff[0] * math.cos(self.heading) + -pos_diff[1] * math.sin(self.heading)
        self.position.y += pos_diff[1] * math.cos(self.heading) + pos_diff[0] * math.sin(self.heading)

        self.odom_speed[0] = self.driver.speed[0] * math.cos(self.heading) + -self.driver.speed[1] * math.sin(self.heading)
        self.odom_speed[1] = self.driver.speed[1] * math.cos(self.heading) + self.driver.speed[0] * math.sin(self.heading)


        #self.odom_speeds =

    def odom_update(self):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.heading / 2)
        quaternion.w = math.cos(self.heading / 2)


        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = (self.get_clock().now() + Duration(seconds=0.25)).to_msg()
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
        odom.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(odom)

    def on_vel_msg_received(self, msg):
        self.set_speed(msg)
        self.lastSpeedReceived = time.time()

    def set_speed(self, speed):
        self.tgt_speed[0] = speed.linear.x
        self.tgt_speed[1] = speed.linear.y
        self.tgt_speed[2] = speed.angular.z


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