import threading
import time
import socket
import pygame
from .udp_device import UDPDevice

ROTS_PER_METER = 1   #
ROTS_PER_MPS = 1        #For some reason these don't match?!
ROTS_PER_RADS_PS = 5#-0.007

class WheelDriver(UDPDevice):
    def __init__(self, ip, portnum, name = None) -> None:
        super().__init__(None, portnum, name)

        self.speed_wheels = [0, 0, 0, 0]

        self.speed_w = [0, 0]
        self.position_w = [0, 0]

        
        self.speed = [0, 0]
        self.position = [0, 0]
        
    def process(self, tokens):
        if tokens[0] == "l":
            self.position_w[0] = float(tokens[1]) / ROTS_PER_METER
        elif tokens[0] == "r":
            self.position_w[1] = -float(tokens[1]) / ROTS_PER_METER

        if tokens[0] == "s_l":
            self.speed_w[0] = float(tokens[1]) / ROTS_PER_MPS
        elif tokens[0] == "s_r":
            self.speed_w[1] = -float(tokens[1]) / ROTS_PER_MPS

    def drive(self, x, y, theta):
        self.send(f"whl {x * ROTS_PER_MPS + theta * ROTS_PER_RADS_PS} {x * ROTS_PER_MPS - theta * ROTS_PER_RADS_PS} ")

    def start(self):
        pass

    def stop(self):
        self.send(f"whl 0 0 ")
        pass

    def update(self):
        self.speed[0] = (self.speed_w[0] + self.speed_w[1]) / 2
        self.position[0] = (self.position_w[0] + self.position_w[1]) / 2

if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)

    js.init()
    driver = WheelDriver(None, 11753, "drv")
    while True:
        pygame.event.pump()
        driver.drive(js.get_axis(0), js.get_axis(1), js.get_axis(3))
        print("Speeds:\t", driver.speed)
        time.sleep(0.1)
