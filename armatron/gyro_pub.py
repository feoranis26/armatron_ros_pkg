import math
import socket, threading, time

import numpy as np

import board
import adafruit_bno055

from .udp_device import UDPDevice


class GyroPublisher():
    def __init__(self, ip, portnum) -> None:
        self.i2c = board.I2C()
        self.imu = adafruit_bno055.BNO055_I2C(self.i2c)

        self.send_thr = threading.Thread(
            target=self.thread, args=(), daemon=True
        )

        self.recv_thr = threading.Thread(
            target=self.receive_thread, args=(), daemon=True
        )

        self.pkt_header = bytes([0xFA])
        self.pkt_footer = bytes([0xFB])

        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.settimeout(0.1)


        self.port.bind((ip, portnum))

        self.remote = None
        self.portnum = 0

        self.send_thr.start()
        self.recv_thr.start()

    def thread(self):
        while True:
            time.sleep(0.025)

            if self.remote is None:
                continue

            read_yaw = self.imu.euler[0]
            self.send(f"angle:{read_yaw};")

    def send(self, data):
        send_data = bytes(data, "utf-8")

        if self.pkt_header != None:
            send_data = self.pkt_header + send_data


        if self.pkt_footer != None:
            send_data = send_data + self.pkt_footer
        
        try:
            self.port.sendto(send_data, (self.remote, self.portnum))
        except ConnectionRefusedError:
            print("Connection refused!")

    def receive_thread(self):
        print("Receiving...")

        while True:
            try:
                data, (raddr, rport) = self.port.recvfrom(1024)
            except TimeoutError:
                continue
            except ConnectionRefusedError:
                print("Connection refused!")
                continue

            if self.pkt_header != None:
                if not data.startswith(self.pkt_header):
                    print("Invalid pkt received (no pkt header)")
                    continue

                data = data.removeprefix(self.pkt_header)

            if self.pkt_footer != None:
                if not data.endswith(self.pkt_footer):
                    print("Invalid pkt received (no pkt footer)")
                    continue
                data = data.removesuffix(self.pkt_footer)

            data = data.decode("UTF-8")
            
            if data == "connect":
                self.remote = raddr
                self.portnum = rport

    def print_status(self):
        print(f"Calib sys, gyro, acc, mag: {self.imu.calibration_status}")

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
    pub = GyroPublisher("127.0.0.1", 11755)

    while True:
        pub.print_status()
        time.sleep(10)

if __name__ == '__main__':
    main()