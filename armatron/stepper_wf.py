import pigpio
import time

from threading import Thread

MAX_PULSES = 10000


class step_pulse:
    up = 0
    down = 0


class Stepper:
    def __init__(self, dir, step, accel, max_spd, gpio):
        self._pin_dir = dir
        self._pin_step = step

        self.acc = accel
        self.max_speed = max_spd

        self._last_step_us = 0
        self._last_speed_update_us = 0

        self.gpio = gpio

        self.gpio.set_mode(self._pin_step, pigpio.OUTPUT)
        self.gpio.set_mode(self._pin_dir, pigpio.OUTPUT)

        self.gpio.write(self._pin_step, 0)
        self.gpio.write(self._pin_dir, 0)

    speed = 0.0
    acc = 0.0
    max_speed = 0.0

    position = 0            #current position of stepper
    target = 0              #target position to move stepper
    target_speed = None     #hold constant speed, overrides target

    def set_tgt_speed(self, speed_tgt):
        self.target_speed = speed_tgt

    def set_tgt_pos(self, pos_tgt):
        self.target_speed = None
        self.target = pos_tgt

    def wait_motion_end(self):
        while self.position != self.target:
            time.sleep(0.01)

    def wait_speed_reached(self):
        while abs(self.speed - self.target_speed) > 5:
            time.sleep(0.01)

    def _get_next_state_change_us(self):
        if abs(self.speed) <= 1.0:
            return 0

        us_per_step = int(1000000.0 / abs(self.speed))
        return us_per_step + self._last_step_us

    def _update_speed(self, current_us):
        time_delta = (current_us - self._last_speed_update_us) / 1000000.0
        self._last_speed_update_us = current_us

        if self.target_speed is not None:
            if self.speed > self.target_speed:
                self.speed -= self.acc * time_delta
            else:
                self.speed += self.acc * time_delta

            if self._last_dir == 2:
                self.gpio.write(self.pin_dir, self.speed < 0)
                self._last_dir = self.speed < 0

            return

        if self.position == self.target:
            self.speed = 0
            return

        dist_to_stop = (self.speed * self.speed / (self.acc * 2)) * (1 if self.speed > 0 else -1)

        if self.position + dist_to_stop > self.target:
            self.speed -= self.acc * time_delta
        else:
            self.speed += self.acc * time_delta

        self.speed = min(max(-self.max_speed, self.speed), self.max_speed)

    def _step_now(self, current_us):
        self._last_step_us = current_us

        pulse = step_pulse()
        pulse.up = 0
        pulse.down = 0

        if self.speed < 0 and self._last_dir == 1:
            pulse.up |= 1 << self._pin_dir
        elif self.speed > 0 and self._last_dir == 0:
            pulse.down |= 1 << self._pin_dir

        if (self.speed > 0) != self._last_dir:
            self._last_dir = self.speed > 0
            return pulse

        self._last_dir = self.speed > 0

        if self._state == 0:
            pulse.up |= 1 << self._pin_step
        elif self._state == 1:
            pulse.down |= 1 << self._pin_step

        self._state = not self._state

        if self.speed != 0:
            self.position += 1 if self.speed > 0 else -1

        if self.position == self.target and self.target_speed is None:
            self.speed = 0
            return pulse

        return pulse

    _pin_dir = 0
    _pin_step = 0

    _last_step_us = 0
    _last_speed_update_us = 0

    _last_dir = 2
    _state = 0


class StepperWaveformTransmitter:
    def __init__(self, plan_length_us, gpio):
        self.planning_len = plan_length_us

        self.transmitting_wave = None
        self.next_wave = None

        self.gpio = gpio

    def add_stepper(self, stepper):
        self.steppers.append(stepper)

    def start(self):
        print("Starting thread!")
        self.gen_thread = Thread(target=self.thread_loop, daemon=True)
        self.gen_thread.start()

    def stop(self):
        print("Signalling thread to stop!")
        self.stop_flag = True
        self.gen_thread.join()

    def create_wave(self, start_us: int) -> (int, int):
        current_us = 0

        pulses = []
        num_pulses = 0

        for stepper in self.steppers:
            stepper._update_speed(start_us)

        last_speed_update_us = start_us

        while current_us < self.planning_len and num_pulses < MAX_PULSES:
            stepper_to_step = None
            next_step_us = -1

            for stepper in self.steppers:
                step_us = stepper._get_next_state_change_us()

                if step_us != 0 and (step_us < next_step_us or next_step_us == -1):
                    next_step_us = step_us
                    stepper_to_step = stepper

            if stepper_to_step is None:
                break

            delta_us = next_step_us - (start_us + current_us)

            if delta_us < 0:
                print(f"Too late! {delta_us}")

                delta_us = 0

            if delta_us > 1000:
                delta_us = 1000

            current_us += delta_us

            if current_us > self.planning_len:
                break

            if (current_us + start_us) - last_speed_update_us > 1000:
                for stepper in self.steppers:
                    stepper._update_speed(current_us + start_us)

                last_speed_update_us = start_us + current_us

            delay_pulse = pigpio.pulse(0, 0, 0)

            delay_pulse.gpio_on = 0
            delay_pulse.gpio_off = 0
            delay_pulse.delay = int(delta_us / 1)
            pulses.append(delay_pulse)
            num_pulses += 1

            pulse_up = 0
            pulse_down = 0

            while True:
                stepper_to_step = None

                for stepper in self.steppers:
                    step_us = stepper._get_next_state_change_us()

                    if step_us <= current_us + start_us and step_us != 0:
                        stp_pulse = stepper._step_now(start_us + current_us)

                        pulse_up |= stp_pulse.up
                        pulse_down |= stp_pulse.down

                        stepper_to_step = stepper

                if stepper_to_step is None:
                    break

            full_pulse = pigpio.pulse(0, 0, 0)
            full_pulse.gpio_on = pulse_up
            full_pulse.gpio_off = pulse_down
            full_pulse.delay = 0

            pulses.append(full_pulse)
            num_pulses += 1

        #print(f"Plan end! Pulses: {num_pulses}")

        if current_us < self.planning_len:
            delay_pulse = pigpio.pulse(0, 0, 0)

            delay_pulse.gpio_on = 0
            delay_pulse.gpio_off = 0
            delay_pulse.delay = self.planning_len - current_us
            pulses.append(delay_pulse)

        self.gpio.wave_add_generic(pulses)

        new_wave = self.gpio.wave_create_and_pad(50)
        assert new_wave >= 0, "Wave crete failed!"

        return new_wave, current_us

    def wait_and_send(self):
        current_wave = self.gpio.wave_tx_at()
        while current_wave == self.transmitting_wave:
            current_wave = self.gpio.wave_tx_at()
            time.sleep(self.planning_len / 100000000)

        #print(f"WF {self.transmitting_wave} ended!")
        self.current_time += self.planning_len
        self.gpio.wave_delete(self.transmitting_wave)
        self.transmitting_wave = self.next_wave

        (next_wave, length) = self.create_wave(self.current_time)
        self.next_wave = next_wave

        res = self.gpio.wave_send_using_mode(next_wave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        assert res >= 0, "Wave send failed!"

        #print(f"Queued WF {next_wave}")

    def thread_loop(self):
        print("Started thread!")
        (self.transmitting_wave, length) = self.create_wave(0)
        self.current_time = self.planning_len
        (self.next_wave, length) = self.create_wave(self.planning_len)

        res = self.gpio.wave_send_using_mode(self.transmitting_wave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        assert res >= 0, "Wave send failed!"

        res = self.gpio.wave_send_using_mode(self.next_wave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        assert res >= 0, "Wave send failed!"

        print("Starting loop!")
        while not self.stop_flag:
            self.wait_and_send()

    planning_len = 0
    current_time = 0

    steppers = []
    waves = []

    gen_thread = None
    stop_flag = False

    transmitting_wave = 0
    next_wave = 0


if __name__ == "__main__":
    print("Test")
    pi = pigpio.pi()
    pi.wave_clear()

    tx = StepperWaveformTransmitter(10000, pi)
    stepper = Stepper(6, 12, 1000, 1000, pi)

    tx.add_stepper(stepper)
    tx.start()
    print("Started!")

    stepper.set_tgt_pos(10000)
    time.sleep(0.25)

    while abs(stepper.speed) > 0:
        print(f"Speed: {stepper.speed}, position: {stepper.position}")
        time.sleep(0.1)

    print("Stopping!")
    tx.stop()
