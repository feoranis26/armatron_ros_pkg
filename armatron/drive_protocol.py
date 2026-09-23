from .udp_device import UDPDevice
import math
import time

#ROTS_PER_METER = 2   #
#ROTS_PER_MPS = 2.5        #For some reason these don't match?!
#ROTS_PER_RADS_PS = 1.64#-0.007

ROTS_PER_METER = 1   #
ROTS_PER_MPS = 1        #For some reason these don't match?!
ROTS_PER_RADS_PS = -0.1#-0.007

class WheelDriver(UDPDevice):
    def __init__(self, ip, portnum, remote_portnum) -> None:
        super().__init__(ip, portnum, remote_portnum)

        self.speed_wheels = [0.0, 0.0, 0.0, 0.0]

        # ROS message setters require real Python floats, not integer zeroes.
        # These values are used before the first controller telemetry packet.
        self.speed = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0]
        self.safety_inhibited = False
        self.speed_sample = None
        self.safety_sample = None

        self.pkt_header = bytes([0xFA])
        self.pkt_footer = bytes([0xFB])
        
    def process(self, data):
        if data[0] == "spd_w":
            values = data[1].split(",")
            self.speed_wheels[0] = float(values[0]) / ROTS_PER_METER
            self.speed_wheels[1] = float(values[1]) / ROTS_PER_METER
            self.speed_wheels[2] = float(values[2]) / ROTS_PER_METER
            self.speed_wheels[3] = float(values[3]) / ROTS_PER_METER
        
        if data[0] == "spd":
            values = data[1].split(",")
            if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
                return
            # Pi wheel telemetry is the negative of its chassis input.
            # Undo that and the command-side axis/unit conversion in drive().
            self.speed[0] = -float(values[0]) / ROTS_PER_MPS
            self.speed[1] = float(values[1]) / ROTS_PER_MPS
            if len(values) > 2:
                self.speed[2] = -float(values[2]) / ROTS_PER_RADS_PS
            self.speed_sample = (tuple(self.speed), time.monotonic())

        if data[0] == "pos":
            values = data[1].split(",")
            self.position[0] = float(values[0]) / ROTS_PER_METER
            self.position[1] = float(values[1]) / ROTS_PER_METER

        if data[0] == "safety":
            if data[1] in ('0', '1'):
                self.safety_inhibited = data[1] == "1"
                self.safety_sample = (self.safety_inhibited, time.monotonic())

    """def print_thread(self):
        while True:
            print("Speeds:\t", self.spd)
            print("Last message:\t", self.last_message)

            time.sleep(1) """
    def start(self):
        super().start()

    def stop(self):
        super().stop()

    def drive(self, x, y, theta):
        #print(f"whl {x * ROTS_PER_MPS} {-y * ROTS_PER_MPS} {theta * ROTS_PER_RADS_PS}")
        self.send(f"whl {x * ROTS_PER_MPS} {-y * ROTS_PER_MPS} {theta * ROTS_PER_RADS_PS}")

    def safety_stop(self):
        self.send("safety_stop")

    def safety_reset(self):
        self.send("safety_reset")

    def update(self):
        pass
