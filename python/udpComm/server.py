import socket
import math
import gps
import thread
import time

class Communication():
    def __init__(self, port, ip):
        #print socket.gethostname()
        #print socket.gethostbyname(socket.gethostname())
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((ip, port))


        self.lock = thread.allocate_lock()
        self.loc_lat    = 0.0
        self.loc_long   = 0.0
        self.loc_height = 0.0

        self.long_start     = 171
        self.long_end       = 180
        self.lat_start      = 182
        self.lat_end        = 190
        self.height_start   = 192
        self.height_end     = 198


    def receive_data(self):
        data, addr = self.socket.recvfrom(500)
        return data

    def calc_pos(self, data):

        longtitude = float(data[self.long_start:self.long_end])
        latitude = float(data[self.lat_start:self.lat_end])
        height = float(data[self.height_start:self.height_end])

        self.lock.acquire()
        loc_long = self.loc_long
        loc_lat = self.loc_lat
        loc_height = self.loc_height
        self.lock.release()

        x = longtitude - loc_long
        y = latitude - loc_lat
        z = height - loc_height

        r = math.sqrt((x*x + y*y + z*z))

        el_rad = math.acos((z/r))
        el_deg = math.degrees(el_rad)

        az_rad = math.atan((y/x))
        az_deg = math.degrees(az_rad)

        return az_deg, el_deg

    def gps_test(self):
        print "Starting gps thread"
        session = gps.gps(host="localhost", port="2947")
        session.stream(flags=gps.WATCH_JSON)

        while True:
            self.lock.acquire()
            self.loc_lat    = session.fix.latitude
            self.loc_long   = session.fix.longitude
            self.loc_height = session.fix.altitude

            if math.isnan(self.loc_height):
                self.loc_height = 0.0


            print self.loc_lat
            print self.loc_long
            print self.loc_height

            self.lock.release()

            time.sleep(20)
        #for report in session:
        #    print report

    def run(self, cb):
        print "Starting udp server"

        thread.start_new_thread(self.gps_test, ())
        packages = 0

        while True:
            data = self.receive_data()
            packages += 1
            print packages
            pos = self.calc_pos(data)
            cb(pos)

