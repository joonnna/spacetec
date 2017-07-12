import socket
import time
import threading
import logging

class Udpclient():
    def __init__(self, port, ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port
        self.ip = ip
        self.exit_event = threading.Event()
        logging.basicConfig(filename="/var/log/statemachine.log", level=logging.DEBUG)
        self.logger = logging.getLogger("udpclient")

    def send_msg(self, msg):
        self.socket.sendto(msg, (self.ip, self.port))

    def shutdown(self):
        self.exit_event.set()

    def _should_stop(self, cleanup_event):
        if self.exit_event.is_set():
            if cleanup_event:
                cleanup_event.set()
            return True
        else:
            return False

    def run(self, input_data=None, cleanup_event=None):
        self.logger.info("Started udp client")

        if input_data == None:
            path = "/home/machinekit/machinekit/spacetec/data_files/ptudata"
        else:
            path = input_data

        f = open(path, "r")
        data = f.read()
        f.close()
        lines = data.split("\n")

        while True:
            for line in lines:
                if self._should_stop(cleanup_event):
                    self.logger.info("Exiting udp client")
                    return

                if len(line) <= 1:
                    continue

                self.send_msg(line)
                time.sleep(0.1)
