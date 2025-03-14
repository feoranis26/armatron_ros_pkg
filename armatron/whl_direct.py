import pigpio
#import pygame
import time
import math

from .stepper_wf import Stepper, StepperWaveformTransmitter

ROTS_PER_METER = 2
ROTS_PER_MPS = 2
ROTS_PER_RADS_PS = 1

STEPS_PER_ROT = -3200
MAX_ROT = 25600

class WheelDriver:
    def __init__(self) -> None:
        self.speed_wheels = [0, 0, 0, 0]

        self.speed = [0, 0]
        self.position = [0, 0]

        self.gpio = pigpio.pi()
        self.gpio.wave_clear()

        self.lf = Stepper(12, 6, 25600, MAX_ROT, self.gpio)
        self.rf = Stepper(16, 11, 25600, MAX_ROT, self.gpio)
        self.rb = Stepper(20, 19, 25600, MAX_ROT, self.gpio)
        self.lb = Stepper(21, 26, 25600, MAX_ROT, self.gpio)

        self.tx = StepperWaveformTransmitter(50000, self.gpio)

        self.tx.add_stepper(self.lf)
        self.tx.add_stepper(self.rf)
        self.tx.add_stepper(self.rb)
        self.tx.add_stepper(self.lb)

    def start(self):
        self.gpio.wave_clear()
        self.gpio.write(5, 1)
        self.tx.start()

    def stop(self):
        self.tx.stop()
        self.gpio.write(5, 0)

    def drive(self, x, y, theta):
        f_rps = int(y * ROTS_PER_MPS * STEPS_PER_ROT)
        r_rps = int(x * ROTS_PER_MPS * STEPS_PER_ROT)
        th_rps = int(theta * ROTS_PER_RADS_PS * STEPS_PER_ROT)

        lf_s =   f_rps    + r_rps  - th_rps
        rf_s =   -f_rps   + r_rps  + th_rps
        rb_s =   f_rps    + r_rps  + th_rps
        lb_s =   -f_rps   + r_rps  - th_rps

        largest = max(abs(lf_s), abs(rf_s), abs(rb_s), abs(lb_s))
        factor = 1

        if largest > MAX_ROT:
            factor = MAX_ROT / largest

        lf_s = factor * lf_s
        rf_s = factor * rf_s
        rb_s = factor * rb_s
        lb_s = factor * lb_s

        self.lf.set_tgt_speed(-lf_s)
        self.rf.set_tgt_speed(rf_s)
        self.rb.set_tgt_speed(rb_s)
        self.lb.set_tgt_speed(-lb_s)

        #print(f"Current speeds: {math.floor(self.lf.target_speed)} {math.floor(self.rf.target_speed)}  {math.floor(self.rb.target_speed)}  {math.floor(self.lb.target_speed)}")

    def update(self):
        if abs(self.lf.speed) <= 25 and abs(self.rf.speed) <= 25 and abs(self.rb.speed) <= 25 and abs(self.lb.speed) <= 25:
            self.gpio.write(5, 0)

            self.last_lf_position = self.lf.position
            self.last_rf_position = self.rf.position
            self.last_rb_position = self.rb.position
            self.last_lb_position = self.lb.position
            #self.drive(0, 0, 0)
        else:
            self.gpio.write(5, 1)
             
        #print(f"Current speeds: {math.floor(self.lf.speed)} {math.floor(self.rf.speed)}  {math.floor(self.rb.speed)}  {math.floor(self.lb.speed)}")
        lf_delta = self.lf.position - self.last_lf_position
        rf_delta = self.rf.position - self.last_rf_position
        rb_delta = self.rb.position - self.last_rb_position
        lb_delta = self.lb.position - self.last_lb_position

        self.last_lf_position = self.lf.position
        self.last_rf_position = self.rf.position
        self.last_rb_position = self.rb.position
        self.last_lb_position = self.lb.position

        x_delta = lf_delta - rf_delta - rb_delta + lb_delta
        y_delta = lf_delta + rf_delta - rb_delta - lb_delta

        self.position[0] += x_delta / (ROTS_PER_METER * STEPS_PER_ROT * 4)
        self.position[1] += y_delta / (ROTS_PER_METER * STEPS_PER_ROT * 4)

        x_speed = self.lf.speed + self.rf.speed + self.rb.speed + self.lb.speed
        y_speed = self.lf.speed - self.rf.speed + self.rb.speed - self.lb.speed

        self.speed[0] = x_speed / (ROTS_PER_MPS * STEPS_PER_ROT)
        self.speed[1] = y_speed / (ROTS_PER_MPS * STEPS_PER_ROT)

    last_lf_position = 0
    last_rf_position = 0
    last_rb_position = 0
    last_lb_position = 0

    position = [0, 0]
    speed = [0, 0]

"""
if __name__ == "__main__":
    print("Start demo!")
    pygame.init()
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)

    js.init()
    driver = WheelDriver()
    driver.start()

    while True:
        pygame.event.pump()
        driver.drive(js.get_axis(0), js.get_axis(1), 0)
        driver.update()
        time.sleep(0.1)
"""