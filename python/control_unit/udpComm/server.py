import socket
import math
import gps
import thread
import time
import logging
from parse_config import read_comm_config


class Communication():
    def __init__(self):
        #print socket.gethostname()
        #print socket.gethostbyname(socket.gethostname())

        logging.basicConfig(filename="/var/log/statemachine.log", level=logging.DEBUG)
        self.logger = logging.getLogger("udpserver")

        config = read_comm_config()
        ip = config["ip"]
        port = config["port"]

        self._loc_lat    = config["lat"]
        self._loc_long   = config["long"]
        self._loc_height = config["height"]

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind((ip, port))
        except socket.error, msg:
            self.logger.Error("Failed to bind/create socket, %s" % (msg))
            return

        self._long_start     = 171
        self._long_end       = 180
        self._lat_start      = 182
        self._lat_end        = 190
        self._height_start   = 192
        self._height_end     = 198

        self.logger.info("Inited udp server")

    def _receive_data(self):
        data, addr = self._socket.recvfrom(1024)

        try:
            longtitude = float(data[self._long_start:self._long_end])
            latitude = float(data[self._lat_start:self._lat_end])
            height = float(data[self._height_start:self._height_end])
        except ValueError:
            self.logger.error("Can't convert gps data to floats, abort?")
            return None

        return (longtitude, latitude, height)

    def shutdown(self):
 #       self._socket.shutdown(socket.SHUT_RDWR)
        self.logger.info("Exiting udp server")
        self._socket.close()

    def _calc_pos(self, data):
        longtitude  = data[0]
        latitude    = data[1]
        height      = data[2]

        #longtitude = 16.05
        #latitude   = 69.29
        #height     = 1.0


        a = 6378137.0
        b = 6356752.31424518

        f = (a-b)/a;
        e_2 = 2*f - (f*f);

        lat_0 = math.radians(self._loc_lat)
        lon_0 = math.radians(self._loc_long)
        h_0 = self._loc_height

        lat_GPS = math.radians(latitude)
        lon_GPS = math.radians(longtitude)
        h_GPS = height


        sin_lat_02 = math.sin(lat_0) * math.sin(lat_0)
        sin_lat_gps_2 = math.sin(lat_GPS) * math.sin(lat_GPS)

        N_0 = a/(math.sqrt(1-e_2 * sin_lat_02))
        N_GPS = a/(math.sqrt(1-e_2 * sin_lat_gps_2))

        X_0 = (N_0 + h_0) * math.cos(lat_0) * math.cos(lon_0)
        Y_0 = (N_0 + h_0) * math.cos(lat_0) * math.sin(lon_0)
        Z_0 = ((1-e_2) * N_0 + h_0) * math.sin(lat_0)
        X_GPS = (N_GPS+h_GPS) * math.cos(lat_GPS) * math.cos(lon_GPS)
        Y_GPS = (N_GPS+h_GPS) * math.cos(lat_GPS) * math.sin(lon_GPS)
        Z_GPS = (N_GPS * (1-e_2) + h_GPS) * math.sin(lat_GPS)

        #Calculate elevation
        delta_x = X_GPS-X_0
        delta_y = Y_GPS-Y_0
        delta_z = Z_GPS-Z_0
        delta_h = h_GPS-h_0

        r = math.sqrt((delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z))

        el_rad = math.asin(delta_h/r)
        el_deg = math.degrees(el_rad)

        """
        x = longtitude - loc_long
        y = latitude - loc_lat
        z = height - loc_height

        r = math.math.sqrt((x*x + y*y + z*z))
        """

        az_rad = math.atan((delta_x/delta_y))

        #TODO sketchy -180.0
        az_deg = 180.0 - math.degrees(az_rad)

        return az_deg, el_deg, height

    def gps_cleanup(self):
        pass
        #self._session


    def run(self, cb):
        #print "Starting udp server"

        #thread.start_new_thread(self._gps_test, ())
        #packages = 0

        #try:
        #    while True:
        data = self._receive_data()
        if data == None:
            return
        pos = self._calc_pos(data)
        cb(pos)
        #except KeyboardInterrupt:
        #    pass

        #self.shutdown()
def new_comm():
    return Communication()
