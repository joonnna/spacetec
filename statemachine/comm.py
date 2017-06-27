import socket


class Communication():
    def __init__(self, port, ip):
        #print socket.gethostname()
        print socket.gethostbyname(socket.gethostname())
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))


    def receive_data(self):
        data, addr = self.socket.recvfrom(500)
        print "yoyooy got this : ", data
        return data

    def calc_pos(self, data):
        pass

    def send_pos(self, pos):
        pass

    def run(self):
        while True:
            data = self.receive_data()
            pos = self.calc_pos(data)
            self.send_pos(pos)
