import threading
import time
import socket


def millis() -> float:
    return float(time.time_ns()) / 1000000.0


class UDPDevice:
    def __init__(self, ip, portnum, device_name = None) -> None:
        self.remote = ip
        self.portnum = portnum
        self.ping = False

        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.settimeout(0.1)
        
        self.last_message = millis() - 1000

        self.receive_thr = threading.Thread(
            target=self.receive_thread, args=(), daemon=True
        )
        self.receive_thr.start()

        self.connection_thr = threading.Thread(
            target=self.connection_thread, args=(), daemon=True
        )
        self.connection_thr.start()

        self.device_name = device_name

    def receive_thread(self):
        try:
            self.port.bind(("", self.portnum))
            #self.port.connect((self.remote, self.portnum))
        except OSError:
            print("Cannot open port!")
            exit(-1)

        print("Receiving...")

        while True:
            try:
                data, sender = self.port.recvfrom(1024)
            except TimeoutError:
                continue
            except ConnectionRefusedError:
                print("Connection refused!")
                continue

            s_ip, s_port = sender
            data = data.decode("UTF-8")

            if data == self.device_name:
                self.remote = s_ip
                continue

            datas = data.split(";")

            for s in datas:
                spl = s.split(":")
                self.process(spl)

            self.last_message = millis()

    def on_connect(self):
        pass

    def on_disconnect(self):
        pass

    def connection_thread(self):
        while True:
            time.sleep(0.2)

            if self.remote is None:
                continue

            try:
                if millis() - self.last_message > 1000:
                    print(f"Connecting! {self.remote}:{self.portnum}")
                    self.port.sendto(
                        bytes("connect", "utf-8"),
                        (self.remote, self.portnum),
                    )
                    #self.port.connect((self.remote, self.portnum))
                elif self.ping:
                    self.port.sendto(bytes("ok", "utf-8"), (self.remote, self.portnum))
            except ConnectionRefusedError:
                print("Connection refused!")

    def process(self, tokens):
        pass

    def send(self, data):
        if self.remote is None:
            return
        
        try:
            self.port.sendto(bytes(data, "utf-8"), (self.remote, self.portnum))
        except ConnectionRefusedError:
            print("Connection refused!")
