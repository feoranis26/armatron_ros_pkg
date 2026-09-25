import threading
import time
import socket
import errno
import logging


# These can occur while interfaces, routes or firewall rules are coming up.
# Programming errors and local bind conflicts must still surface as failures.
NETWORK_ERRORS = {errno.EPERM, errno.EACCES, errno.ENETDOWN, errno.ENETUNREACH,
                  errno.EHOSTUNREACH, errno.ECONNREFUSED, errno.ECONNRESET,
                  errno.ENOBUFS, errno.EAGAIN, errno.ETIMEDOUT}

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
        self.stop_event = threading.Event()
        self.error_times = {}
        self.error_lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

    def start(self):
        self.last_message = millis() - 1000
        self.stop_flag = False
        self.stop_event.clear()
        # Bind before either thread can send and implicitly claim an ephemeral
        # port. A port collision is a configuration error, not a network outage.
        self.port.bind(("", self.remote_portnum))

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
        self.stop_event.set()
        self.port.close()

    def network_error(self, operation, error):
        if self.stop_flag:
            return
        if error.errno not in NETWORK_ERRORS:
            raise error
        key = (operation, error.errno)
        now = time.monotonic()
        with self.error_lock:
            if now - self.error_times.get(key, float('-inf')) < 5.0:
                return
            self.error_times[key] = now
        self.logger.warning('UDP %s to %s:%s failed: %s; retrying, no datagrams queued',
                            operation, self.remote, self.portnum, error)

    def receive_thread(self):
        while not self.stop_flag:
            try:
                self.port.connect((self.remote, self.portnum))
                break
            except OSError as error:
                self.network_error('connect', error)
                self.stop_event.wait(0.2)

        print("Receiving...")

        while not self.stop_flag:
            try:
                data, _ = self.port.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError as error:
                if self.stop_flag:
                    break
                self.network_error('receive', error)
                self.stop_event.wait(0.2)
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

            try:
                data = data.decode("UTF-8")
            except UnicodeDecodeError:
                continue
            datas = data.split(";")

            for s in datas:
                spl = s.split(":")
                try:
                    self.process(spl)
                except (ValueError, IndexError):
                    continue  # A malformed datagram must not kill reception.

            self.last_message = millis()

    def on_connect(self):
        pass

    def on_disconnect(self):
        pass

    def connection_thread(self):
        while not self.stop_flag:
            try:
                if millis() - self.last_message > 1000:
                    self.send("connect")
                    self.port.connect((self.remote, self.portnum))
                elif self.ping:
                    self.send("ok")
            except OSError as error:
                self.network_error('keepalive', error)

            self.stop_event.wait(0.2)

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
            return True
        except OSError as error:
            self.network_error('send', error)
            return False
