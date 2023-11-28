import threading
import time
import socket
import pygame
from .udp_device import UDPDevice

ROTS_PER_METER = 2   #
ROTS_PER_MPS = 2.5        #For some reason these don't match?!
ROTS_PER_RADS_PS = 1.64#-0.007

class WheelDriver(UDPDevice):
    def __init__(self, ip, portnum) -> None:
        super().__init__(ip, portnum)

        self.speed_wheels = [0, 0, 0, 0]

        self.speed = [0, 0]
        self.position = [0, 0]
        
    def process(self, data):
        if data[0] == "spd_w":
            values = data[1].split(",")
            self.speed_wheels[0] = float(values[0]) / ROTS_PER_METER
            self.speed_wheels[1] = float(values[1]) / ROTS_PER_METER
            self.speed_wheels[2] = float(values[2]) / ROTS_PER_METER
            self.speed_wheels[3] = float(values[3]) / ROTS_PER_METER
        
        if data[0] == "spd":
            values = data[1].split(",")
            self.speed[0] = float(values[1]) / ROTS_PER_METER
            self.speed[1] = -float(values[0]) / ROTS_PER_METER

        if data[0] == "pos":
            values = data[1].split(",")
            self.position[0] = float(values[1]) / ROTS_PER_METER
            self.position[1] = -float(values[0]) / ROTS_PER_METER

    """def print_thread(self):
        while True:
            print("Speeds:\t", self.spd)
            print("Last message:\t", self.last_message)

            time.sleep(1) """

    def drive(self, x, y, theta):
        self.send(f"whl {-y * ROTS_PER_MPS} {x * ROTS_PER_MPS} {theta * ROTS_PER_RADS_PS}")

if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)

    js.init()
    driver = WheelDriver("192.168.1.34", 11753)
    while True:
        pygame.event.pump()
        driver.drive(js.get_axis(0), js.get_axis(1), 0)
        time.sleep(0.1)
