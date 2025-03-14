import time

from .udp_device import UDPDevice


class UDPGyro(UDPDevice):
    def __init__(self, ip, portnum, remote_portnum) -> None:
        super().__init__(ip, portnum, remote_portnum)

        self.angle = 0.0

        self.pkt_header = bytes([0xFA])
        self.pkt_footer = bytes([0xFB])

    def process(self, data):
        try:
            if data[0] == "angle":
                self.angle = float(data[1])
        except:
            pass

    def start(self):
        super().start()

    def stop(self):
        super().stop()

    def update(self):
        pass

if __name__ == "__main__":
    gyro = UDPGyro("127.0.0.1", 11755, 11757)

    while True:
        print(gyro.angle)
        time.sleep(1)