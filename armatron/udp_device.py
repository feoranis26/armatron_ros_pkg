import threading
import time
import socket


def millis() -> float:
    return float(time.time_ns()) / 1000000.0


class UDPDevice:
    def __init__(self, ip, portnum) -> None:
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

    def receive_thread(self):
        try:
            self.port.bind(("", self.portnum))
            self.port.connect((self.remote, self.portnum))
        except WindowsError:
            print("Cannot open port!")
            exit(-1)
        except ConnectionRefusedError:
            print("Connection refused!")

        print("Receiving...")

        while True:
            try:
                data, _ = self.port.recvfrom(1024)
            except TimeoutError:
                continue
            except ConnectionRefusedError:
                print("Connection refused!")
                continue

            data = data.decode("UTF-8")
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
            try:
                if millis() - self.last_message > 1000:
                    self.port.sendto(
                        bytes("connect ", "utf-8"),
                        (self.remote, self.portnum),
                    )
                    self.port.connect((self.remote, self.portnum))
                elif self.ping:
                    self.port.sendto(bytes("ok", "utf-8"), (self.remote, self.portnum))
            except ConnectionRefusedError:
                print("Connection refused!")

            time.sleep(0.2)

    def process(self, tokens):
        pass

    def send(self, data):
        try:
            self.port.sendto(bytes(data, "utf-8"), (self.remote, self.portnum))
        except ConnectionRefusedError:
            print("Connection refused!")
