import socket


class Communication():
    def __init__(self, port, ip)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))


    def receive_data(self):
        data, addr = sock.recvfrom(1024)
        print "yoyooy got this : ", data


    def run(self):

