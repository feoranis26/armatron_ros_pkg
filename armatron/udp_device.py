import threading
import time
import socket

def millis() -> float:
    return float(time.time_ns()) / 1000000.0


class UDPDevice:
    def __init__(self, ip, portnum, remote_portnum) -> None:
        self.remote = ip
        self.portnum = portnum
        self.remote_portnum = remote_portnum
        self.ping = False

        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.settimeout(0.1)
        
        self.pkt_header = None
        self.pkt_footer = None
        self.stop_flag = False

    def start(self):
        self.last_message = millis() - 1000
        self.stop_flag = False

        self.receive_thr = threading.Thread(
            target=self.receive_thread, args=(), daemon=True
        )
        self.receive_thr.start()

        self.connection_thr = threading.Thread(
            target=self.connection_thread, args=(), daemon=True
        )
        self.connection_thr.start()

    def stop(self):
        self.stop_flag = True
        self.port.close()

    def receive_thread(self):
        try:
            self.port.bind(("", self.remote_portnum))
        except OSError:
            print("Cannot bind!")

        try:
            self.port.connect((self.remote, self.portnum))
        except OSError:
            print("Cannot open port!")
            exit(-1)

        print("Receiving...")

        while True:
            try:
                data, _ = self.port.recvfrom(1024)
            except TimeoutError:
                continue
            except ConnectionRefusedError:
                print("Connection refused!")
                continue

            if self.pkt_header != None:
                if not data.startswith(self.pkt_header):
                    print("Invalid pkt received (no pkt header)")
                    continue

                data = data.removeprefix(self.pkt_header)

            if self.pkt_footer != None:
                if not data.endswith(self.pkt_footer):
                    print("Invalid pkt received (no pkt footer)")
                    continue
                data = data.removesuffix(self.pkt_footer)

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
                    self.send("connect")
                    self.port.connect((self.remote, self.portnum))
                elif self.ping:
                    self.send("ok")
            except ConnectionRefusedError:
                print("Connection refused!")

            time.sleep(0.2)

    def process(self, tokens):
        pass

    def send(self, data):
        send_data = bytes(data, "utf-8")

        if self.pkt_header != None:
            send_data = self.pkt_header + send_data


        if self.pkt_footer != None:
            send_data = send_data + self.pkt_footer
        
        try:
            self.port.sendto(send_data, (self.remote, self.portnum))
        except ConnectionRefusedError:
            print("Connection refused!")
