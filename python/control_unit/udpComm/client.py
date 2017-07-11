import socket
import time

class Udpclient():
    def __init__(self, port, ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port
        self.ip = ip

    def send_msg(self, msg):
        self.socket.sendto(msg, (self.ip, self.port))

    def run(self, input_data=None):
        if input_data == None:
            f = open("/home/machinekit/machinekit/spacetec/data_files/ptudata", "r")
            data = f.read()
            f.close()
            lines = data.split("\n")
        else:
            lines = input_data.split("\n")

        while True:
            print "Starting sending"
            for line in lines:
                if len(line) <= 1:
                    continue
                self.send_msg(line)
                time.sleep(0.1)
            print "Done sending"
