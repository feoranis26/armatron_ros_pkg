import threading
import time
import socket
from .udp_device import UDPDevice


def millis() -> float:
    return float(time.time_ns()) / 1000000.0


class IMU(UDPDevice):
    def __init__(self, ip, portnum) -> None:
        super().__init__(ip, portnum)

        self.yaw = 0
        self.pitch = 0

        self.acc = [0, 0, 0]
        self.spd = [0, 0, 0]
        self.pos = [0, 0, 0]

        self.state = "disconnected"

        self.ping = True

    def process(self, data):
        if data[0] == "angle":
            self.yaw = float(data[1])
        elif data[0] == "tilt":
            self.pitch = float(data[1])
        elif data[0] == "acc":
            values = data[1].split(",")
            self.acc[0] = float(values[0])
            self.acc[1] = float(values[1])
            self.acc[2] = float(values[2])
        elif data[0] == "speed":
            values = data[1].split(",")
            self.spd[0] = float(values[0])
            self.spd[1] = float(values[1])
            self.spd[2] = float(values[2])
        elif data[0] == "position":
            values = data[1].split(",")
            self.pos[0] = float(values[0])
            self.pos[1] = float(values[1])
            self.pos[2] = float(values[2])
        elif data[0] == "state":
            self.state = data[1]

        self.last_message = millis()

    def calibrate(self):
        self.send("calib ;")

    def set_yaw(self, yaw):
        self.send(f"set_angle {yaw}")
"""
    def print_thread(self):
        while True:
            print("Yaw:\t", self.yaw)
            print("Pitch:\t", self.pitch)
            print("Acc:\t", self.acc)
            print("Spd:\t", self.spd)
            print("Pos:\t", self.pos)
            print("Last message:\t", self.last_message)

            time.sleep(1)"""

if __name__ == "__main__":
    imu = IMU("192.168.1.33", 11752)
    input("Press enter to exit")
