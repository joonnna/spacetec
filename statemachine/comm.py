import socket


class Communication():
    def __init__(self, port, ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))


    def receive_data(self):
        data, addr = sock.recvfrom(500)
        print "yoyooy got this : ", data
        return data

    def calc_pos(self, data):
        pass

    def send_pos(self, pos):
        pass

    def run(self):
        while True:
            data = receive_data()
            pos = calc_pos(data)
            send_pos(pos)
