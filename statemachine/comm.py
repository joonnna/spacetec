import socket
import gps


class Communication():
    def __init__(self, port, ip):
        #print socket.gethostname()
        #print socket.gethostbyname(socket.gethostname())
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))

        self.long_start = 171
        self.long_end = 180
        self.lat_start = 182
        self.lat_end = 190
        self.height_start = 192
        self.height_end = 198


    def receive_data(self):
        data, addr = self.socket.recvfrom(500)
        return data

    def calc_pos(self, data):
        longtitude = float(data[self.long_start:self.long_end])
        latitude = float(data[self.lat_start:self.lat_end])
        height = float(data[self.height_start:self.height_end])
        print longtitude, latitude, height


    def send_pos(self, pos):
        pass

    def gps_test(self):
        session = gps.gps(host="localhost", port="2947")
        session.stream(flags=gps.WATCH_JSON)

        for report in session:
            process(report)


def run(ip, port, cb):

    comm = Communication(port, ip)

    self.gps_test()

    packages = 0
    while True:
        data = self.receive_data()
        packages += 1
        print packages
        pos = self.calc_pos(data)
        cb(pos)
