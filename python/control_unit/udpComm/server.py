import socket
import math
import gps
import thread
import time

class Communication():
    def __init__(self, port, ip):
        #print socket.gethostname()
        #print socket.gethostbyname(socket.gethostname())
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((ip, port))

        self._lock = thread.allocate_lock()
        self._loc_lat    = 0.0
        self._loc_long   = 0.0
        self._loc_height = 0.0

        self._long_start     = 171
        self._long_end       = 180
        self._lat_start      = 182
        self._lat_end        = 190
        self._height_start   = 192
        self._height_end     = 198

        self._session = gps.gps(host="localhost", port="2947")
        self._session.stream(flags=gps.WATCH_JSON)

#        print self._session.fix.__dict__


    def _receive_data(self):
        data, addr = self._socket.recvfrom(500)

        longtitude = float(data[self._long_start:self._long_end])
        latitude = float(data[self._lat_start:self._lat_end])
        height = float(data[self._height_start:self._height_end])

        return (longtitude, latitude, height)

    def shutdown(self):
 #       self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()

    def _calc_pos(self, data):
        longtitude  = data[0]
        latitude    = data[1]
        height      = data[2]

        self._lock.acquire()
        loc_long = self._loc_long
        loc_lat = self._loc_lat
        loc_height = self._loc_height
        self._lock.release()

        x = longtitude - loc_long
        y = latitude - loc_lat
        z = height - loc_height

        r = math.sqrt((x*x + y*y + z*z))

        el_rad = math.acos((z/r))
        el_deg = math.degrees(el_rad)

        az_rad = math.atan((y/x))
        az_deg = math.degrees(az_rad)

        return az_deg, el_deg, height

    def gps_cleanup(self):
        pass
        #self._session


    def get_local_gps_pos(self):
        #self._session.query("ao")

        self._lock.acquire()
        self._loc_lat    = self._session.fix.latitude
        self._loc_long   = self._session.fix.longitude
        self._loc_height = self._session.fix.altitude

        if math.isnan(self._loc_height):
            self._loc_height = 0.0


#        print self._loc_lat
 #       print self._loc_long
  #      print self._loc_height

        self._lock.release()

    def run(self, cb):
        #print "Starting udp server"

        #thread.start_new_thread(self._gps_test, ())
        #packages = 0

        #try:
        #    while True:
        data = self._receive_data()
        pos = self._calc_pos(data)
        cb(pos)
        #except KeyboardInterrupt:
        #    pass

        #self.shutdown()
