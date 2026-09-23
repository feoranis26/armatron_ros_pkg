"""Hardware-independent gyro worker supervision."""
import time


class GyroHealth:
    def __init__(self, timeout=5.0):
        self.timeout = timeout
        self.last_valid = time.monotonic()

    def check(self, workers):
        for worker in workers:
            if not worker.is_alive():
                raise RuntimeError(f'Gyro worker exited: {worker.name}')
        if time.monotonic() - self.last_valid > self.timeout:
            raise RuntimeError('No valid BNO055 reading for 5 seconds; sensor worker failed or stalled')
