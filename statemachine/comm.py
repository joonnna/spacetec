import socket
import math
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

        loc_long = 5.4
        loc_lat = 3.5
        loc_height = 2.7

        x = longtitude - loc_long
        y = latitude - loc_lat
        z = height - loc_height

        r = math.sqrt((x^2 + y^2 + z^2))

        el_rad = math.acos((z/r))
        el_deg = math.degrees(el_rad)

        az_rad = math.atan((y/x))
        az_deg = math.degrees(az_rad)

        return az_deg, el_deg

    def gps_test(self):
        pass
        #gpsd

        """
        session = gps.gps(host="localhost", port="2947")
        session.stream(flags=gps.WATCH_JSON)

        for report in session:
            process(report)
        """

def run(ip, port, cb):

    comm = Communication(port, ip)

    #self.gps_test()

    packages = 0
    while True:
        data = comm.receive_data()
        packages += 1
 #       print packages
        pos = comm.calc_pos(data)
        cb(pos)
