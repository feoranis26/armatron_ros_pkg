import pigpio
import time
import sys

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
        #print(f"Speed: {abs(self.speed)}")
        if abs(self.speed) <= 1.0:
            return 0

        us_per_step = int(1000000.0 / abs(self.speed))
        return us_per_step + self._last_step_us

    def _update_speed(self, current_us):
        time_delta = float(current_us - self._last_speed_update_us) / 1000000.0
        #print(f"time delta: {time_delta} speed change: {self.acc * time_delta}")
        self._last_speed_update_us = current_us
        
        if self.target_speed is not None:
            if self.speed > self.target_speed:
                self.speed -= self.acc * time_delta
                #if self.speed < self.target_speed:
                #    self.speed = self.target_speed
            elif self.speed < self.target_speed:
                self.speed += self.acc * time_delta
                #if self.speed > self.target_speed:
                #    self.speed = self.target_speed

            if self._last_dir == 2:
                self.gpio.write(self._pin_dir, self.speed < 0)
                self._last_dir = self.speed < 0

            if abs(self.speed - self.target_speed) < 1:
                self.speed = self.target_speed

            return

        if self.position == self.target:
            self.speed = 0
            return

        dist_to_stop = (self.speed * self.speed / (self.acc * 2)) * (1 if self.speed > 0 else -1) * 1.1

        if self.position + dist_to_stop > self.target:
            self.speed -= self.acc * time_delta
        else:
            self.speed += self.acc * time_delta

    def _step_now(self, current_us):
        self._last_step_us = current_us

        self.speed = min(max(-self.max_speed, self.speed), self.max_speed)
        if abs(self.speed) < 1:
            self.speed = 0

        pulse = step_pulse()
        pulse.up = 0
        pulse.down = 0

        if self._last_dir == 1:
            pulse.up |= 1 << self._pin_dir
        elif self._last_dir == 0:
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

        start_ns = time.time_ns()
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

            if (current_us + start_us) - last_speed_update_us > 100:
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

        end_ns = time.time_ns()
        #print(f"Planning took {(end_ns-start_ns) / 1000000.0} ms")

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
    
    def create_and_transmit(self):
        (new_wave, length) = self.create_wave(self.current_time)

        #print(f"Sending: {new_wave}")
        res = self.gpio.wave_send_using_mode(new_wave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        assert res >= 0, "Wave send failed!"

        self.current_time += self.planning_len
        return new_wave

        #print(f"Queued WF {next_wave}")
    
    def wait_tx_end(self):
        wait_wave = self.gpio.wave_tx_at()
        current_wave = wait_wave

        while current_wave == wait_wave:
            current_wave = self.gpio.wave_tx_at()
            time.sleep((self.planning_len * 0.1) / 1000000)

        #print(f"{wait_wave} ended!")
        return wait_wave

    def wait_and_fill_buffer(self):
        check_wave = self.gpio.wave_tx_at()

        if check_wave != 9999:
            wf_index = self.waves.index(check_wave)
            #print(f"WF #{wf_index}")

            if wf_index != 0:
                print("Underrun!", file=sys.stderr)
            else:
                self.wait_tx_end()

            for i in range(0, wf_index + 1).__reversed__():
                wave_id = self.waves.pop(i)
                #print(f"Deleting wave #{i}, id: {wave_id}")
                self.gpio.wave_delete(wave_id)
        else:
            self.gpio.wave_clear()
            self.waves.clear()
        #print(f"Waited for {(start_ns - end_ns) / 1000000.0} ms")
        

        while len(self.waves) < 2:
            wave = self.create_and_transmit()
            self.waves.append(wave)


    def thread_loop(self):
        while True:
            try:
                print("Started thread!")
                self.gpio.wave_clear()

                self.waves.append(self.create_and_transmit())
                self.waves.append(self.create_and_transmit())

                print("Starting loop!")
                while not self.stop_flag:
                    self.wait_and_fill_buffer()
            except Exception:
                print(f"Caught exception in thread!")

    planning_len = 0
    current_time = 0

    steppers = []
    waves = []

    gen_thread = None
    stop_flag = False

    wf_count = 0


if __name__ == "__main__":
    print("Test")
    pi = pigpio.pi()
    pi.wave_clear()

    tx = StepperWaveformTransmitter(50000, pi)
    stepper = Stepper(6, 12, 12800, 12800, pi)

    tx.add_stepper(stepper)
    tx.start()
    print("Started!")

    stepper.set_tgt_pos(-128000)
    time.sleep(0.25)

    while abs(stepper.speed) > 0:
        print(f"Speed: {stepper.speed}, position: {stepper.position}")
        time.sleep(0.1)

    print("Stopping!")
    tx.stop()
