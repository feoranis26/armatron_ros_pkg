import math
import socket, threading, time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ArmatronDrive(Node):
    def __init__(self):
        super().__init__("armatron_drive")

        self.wheeldriver_ip = ""
        self.powermanager_ip = ""

        self.wheeldriver_connection = None
        self.powermanager_connection = None

        self.broadcast_udp_port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tcp_port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.start_daemons()

        self.powerstate = False
        self.speed = Twist()

        self.lastSpeedReceived = time.time()
        self.lastSpeedSent = time.time()

        self.position = Point()
        self.pose = Pose()
        self.headingDegrees = 0
        self.heading = 0

        self.base_frame_id = "base_link"
        self.odom_frame_id = "odom"


        self.power_subscriber = self.create_subscription(
            Bool,
            '/power_12v_state',
            self.on_power_msg_received,
            1)

        self.vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_vel_msg_received,
            1)

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.on_odom_goto_msg_received,
            1)

        self.od_cmd_subscriber = self.create_subscription(
            String,
            '/od_cmd',
            self.on_od_cmd_msg_received,
            1)

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        if time.time() - self.lastSpeedReceived > 1 and self.speed != Twist():
            self.set_speed(Twist())

        self.odom_update()

    def odom_update(self):
        self.heading = self.headingDegrees * math.pi / 180

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
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(odom)

    def on_power_msg_received(self, msg):
        self.set_power(msg.data)

    def on_vel_msg_received(self, msg):
        if not (self.wheeldriver_connection == None):
            self.set_speed(msg)
            self.lastSpeedReceived = time.time()

    def on_odom_goto_msg_received(self, msg):
        return
        #self.goto(msg.pose)

    def on_od_cmd_msg_received(self, msg):
        self.send_to(self.wheeldriver_connection, msg)


    def receive_broadcasts(self):
        self.broadcast_udp_port.bind(("", 11750));

        print("Receiving broadcasts...")

        while True:
            data, sender = self.broadcast_udp_port.recvfrom(1024)

            self.process_udp(data, sender)

    def receive_tcp(self):
        self.tcp_port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_port.bind(("", 11751));
        self.tcp_port.listen(1)

        print("Starting TCP host...")

        while True:
            connection, sender = self.tcp_port.accept()

            if sender[0] == self.wheeldriver_ip:
                print("Wheeldriver connected!")

                self.wheeldriver_connection = connection

                self.send_to(self.wheeldriver_connection, "relative_mode")
                self.send_to(self.wheeldriver_connection, "no_hold_heading")
                
                if not self.wheeldrv_comms_thread.is_alive():
                    self.wheeldrv_comms_thread.start()

            elif sender[0] == self.powermanager_ip:
                print("Powermanager connected!")

                self.powermanager_connection = connection

    def wheeldrv_comms(self):
        while True:
            data = self.wheeldriver_connection.recv(1024)
            data = data.decode("UTF-8")

            spl = data.split('\r')

            for d in spl:
                self.wheeldrv_decode(d)

    def wheeldrv_decode(self, message):
        spl = message.split(':')

        try:
            if spl[0] == "position":
                coords = spl[1].split(',')
                self.position.x = float(coords[1])
                self.position.y = -float(coords[0])
            elif spl[0] == "heading":
                self.headingDegrees = float(spl[1])
        except:
            print("Error parsing message: \"" + message + "\"")

    def process_udp(self, data, sender):
        data = data.decode("UTF-8")
        spl = data.split(' ')

        print(data)

        if spl[0] == "fComms":
            if spl[1] == "ID:WHEELDRV":
                self.wheeldriver_ip = sender[0]

                print("Wheeldriver ip: " + self.wheeldriver_ip)
                self.send_connect_request(self.wheeldriver_ip)

            elif spl[1] == "ID:POWERMGR":
                self.powermanager_ip = sender[0]

                print("Powermanager ip: " + self.powermanager_ip)
                self.send_connect_request(self.powermanager_ip)

    def send_connect_request(self, ip):
        self.broadcast_udp_port.sendto(bytes("CONNECT", "utf-8"), (ip, 11752))

    def send_to(self, sendto, message):
        print("Sending: " + message)
        sendto.sendall(bytes(message + "\r", "utf-8"))



    def start_daemons(self):
        self.broadcast_thread = threading.Thread(target=self.receive_broadcasts, args=(), daemon=True)
        self.tcp_thread = threading.Thread(target=self.receive_tcp, args=(), daemon=True)

        self.wheeldrv_comms_thread = threading.Thread(target=self.wheeldrv_comms, args=(), daemon=True)

        self.broadcast_thread.start()
        self.tcp_thread.start()
    
    def set_power(self, state):
        if self.powerstate != state:
            if state == True:
                print("Enable power")
                self.send_to(self.powermanager_connection, "enable_power")
            else:
                print("Disable power")
                self.send_to(self.powermanager_connection, "disable_power")
            
            self.powerstate = state

    def set_speed(self, speed):
        if (time.time() - self.lastSpeedSent) > 0.1:
            self.send_to(self.wheeldriver_connection, "set_speed_all " + str(-speed.linear.y) + " " + str(speed.linear.x) + " " + str(speed.angular.z) + " ")
            self.lastSpeedSent = time.time()

        self.speed = speed

    def goto(self, pose):
        pitch, roll, yaw = quaternion_to_euler_angle(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        print("Goto: X=" + str(pose.position.x) + ", Z=" + str(pose.position.y) + ", Rot=" + str(pitch) + " degrees.")

        self.send_to(self.wheeldriver_connection, "goto_pos " + str(-pose.position.y) + " " + str(pose.position.x) + " ")
        self.send_to(self.wheeldriver_connection, "goto_rot " + str(pitch) + " ")

    def stop(self):
        self.set_power(False)


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

    
def main(args=None):
    rclpy.init(args=args)

    drive_node = ArmatronDrive()

    rclpy.spin(drive_node)

    drive_node.stop()
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()