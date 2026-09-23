from .udp_device import UDPDevice
import math
import time


class UDPGyro(UDPDevice):
    def __init__(self, ip, portnum, remote_portnum) -> None:
        super().__init__(ip, portnum, remote_portnum)

        self.sample = None

        self.pkt_header = bytes([0xFA])
        self.pkt_footer = bytes([0xFB])

    def process(self, data):
        try:
            if data[0] == "angle":
                value = float(data[1])
                if math.isfinite(value):
                    self.sample = (value, time.monotonic())
        except (ValueError, IndexError):
            return

    @property
    def angle(self):
        sample = self.sample
        if sample is None or time.monotonic() - sample[1] > 1.0:
            return None
        return sample[0]

    def start(self):
        super().start()

    def stop(self):
        super().stop()

    def update(self):
        pass
