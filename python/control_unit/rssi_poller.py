import logging
import numpy
import time
import sys
import math
import socket
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
import ConfigParser

class RssiPoller():
    def __init__(self):
        self.sig_sim = True
        logging.basicConfig(filename="/var/log/rssipoller.log", level=logging.DEBUG)
        self.logger = logging.getLogger("rssipoller")
        self.sd = ServiceDiscovery()
        self.filepath = "/home/machinekit/machinekit/spacetec/data_files/ptudata"

        self.read_config()

        self.relative_value = 1580.0/5.0

        self.adc_filename = "/sys/devices/ocp.3/helper.14/AIN0"

        self.initrcomps()

    def read_config(self):
        config = ConfigParser.ConfigParser()
        config.read("/home/machinekit/machinekit/spacetec/configs/test_config.ini")
        self.prev_offset = config.getfloat("RSSI_POLLER", "PREV_OFFSET")
        self.time_window = config.getfloat("RSSI_POLLER", "TIME_WINDOW")

    def initrcomps(self):
        rssi_reader = halremote.RemoteComponent("rssi-reader", debug=False)
        self.rssi_pin = rssi_reader.newpin("value", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.sd.register(rssi_reader)

        if self.sig_sim:
            self.create_sig_sim()

        self.sd.start()
        rssi_reader.bind_component()

        if self.sig_sim:
            self.angles.bind_component()
            self.logger.info("Bound simulating angles")

        timeout = 5.0
        while not rssi_reader.wait_connected(timeout):
            self.logger.info("rssi-reader not connected, waiting")

        if self.sig_sim:
            while not self.angles.wait_connected(timeout):
                self.logger.info("beagle-angles not connected, waiting")


        self.logger.info("Bound rssi-poller")

    def create_sig_sim(self):
        self.angles = halremote.RemoteComponent("beagle-angles", debug=False)
        self.angles.newpin("az", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.angles.newpin("el", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.angles.newpin("north", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.sd.register(self.angles)

    def get_north_angle(self):
        return self.angles.getpin("north").get()

    def extract_data(self, data):
        time = float(data[0:6])
        long = float(data[171:180])
        lat = float(data[182:190])
        height = float(data[192:198])

        return lat, long, height, time

    def interpolate(self, y1, y3, x1, x3, x2):
        try:
            ret = ((((x2-x1)*(y3-y1))/(x3-x1))+y1)
        except ZeroDivisionError:
            return x1

        return ret

    def calc_sig(self, az_deg, el_deg, az2_deg, el2_deg):
        az = math.radians(az_deg)
        el = math.radians(el_deg)
        az2 = math.radians(az2_deg)
        el2 = math.radians(el2_deg)

        val = math.acos(math.sin(el) * math.sin(el2) + math.cos(el) * math.cos(el2) * math.cos(az - az2))

        self.logger.debug("Difference value: %f" % (math.degrees(val)))

        k = 21.6851
        x = val * k

        right = (abs(math.sin(x))/x)
        sig = 10 * math.log10(right)

        return sig

    def send_packet(self, data):
        self.socket.sendto(data, ("128.39.133.72", 2729))

    def simulation(self):
        #time.sleep(60)
        line_number = 1
        f = open(self.filepath, "r")
        data = f.read().split("\n")
        f.close()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        curr_lat, curr_long, curr_height, curr_time = self.extract_data(data[line_number])
        prev_time = time.time()
        self.send_packet(data[line_number])
  #      curr_az, curr_el = self.calc_pos((curr_long, curr_lat, curr_height))
        line_number = line_number + 1

        next_lat, next_long, next_height, next_time = self.extract_data(data[line_number])
        line_number = line_number + 1

        try:
            while True:
                time.sleep(0.1)
                interpolate_time = time.time() - prev_time
                lat = self.interpolate(curr_lat, next_lat, curr_time, next_time, (curr_time + interpolate_time))
                long = self.interpolate(curr_long, next_long, curr_time, next_time, (curr_time + interpolate_time))
                height = self.interpolate(curr_height, next_height, curr_time, next_time, (curr_time + interpolate_time))

                self.logger.debug("curr_lat: %.9f : next_lat: %.9f\n curr_long: %.9f : next_long: %.9f\n curr_height: %.9f : next_height: %.9f\n i-lat: %.9f i-long %.9f i-height: %.9f" % (curr_lat, next_lat, curr_long, next_long, curr_height, next_height, lat, long, height))

                gps_az, gps_el = self.calc_pos((long, lat, height))
                prev_az = self.angles.getpin("az").get()
                prev_el = self.angles.getpin("el").get()

                sig = self.calc_sig(gps_az, gps_el, prev_az, prev_el)
                self.rssi_pin.set(sig)

                self.logger.debug("i-gps: (%f, %f)\n antenna: (%f, %f)\n sig: %.9f" % (gps_az, gps_el, prev_az, prev_el, sig))

                self.logger.debug("curr_time: %f : next_time %f\n interpolate_time: %f : prev_time: %f" % (curr_time, next_time, interpolate_time, prev_time))

                if interpolate_time >= 1.0:
                    try:
                        curr_lat, curr_long, curr_height, curr_time = self.extract_data(data[line_number])
                        prev_time = time.time()
                        self.send_packet(data[line_number])
 #                       curr_az, curr_el = self.calc_pos((curr_long, curr_lat, curr_height))
                        line_number = line_number + 1


                        next_lat, next_long, next_height, next_time = self.extract_data(data[line_number])
                        line_number = line_number + 1
                    except IndexError:
                        return

        except KeyboardInterrupt:
            return

    def calc_pos(self, data):
        try:
            longtitude  = data[0]
            latitude    = data[1]
            height      = data[2]

            a = 6378137.0
            b = 6356752.31424518

            f = (a-b)/a
            e_2 = 2*f - (f*f)

            self.loc_lat = 69.29589
            self.loc_long = 16.03037
            self.loc_height = 7.0

            lat_0 = math.radians(self.loc_lat)
            lon_0 = math.radians(self.loc_long)
            h_0 = self.loc_height

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

            delta_x = X_GPS-X_0
            delta_y = Y_GPS-Y_0
            delta_z = Z_GPS-Z_0

            # Coordinates (S, E, V)
            # Calculate South, East, Local Vertical

            #rho_matrix = [delta_x; delta_y; delta_z]
            rho_matrix = numpy.matrix([[delta_x], [delta_y], [delta_z]])

            row0 = [math.sin(lat_0) * math.cos(lon_0), math.sin(lat_0) * math.sin(lon_0), -math.cos(lat_0)]
            row1 = [-math.sin(lon_0), math.cos(lon_0), 0]
            row2 = [math.cos(lat_0)*math.cos(lon_0), math.cos(lat_0)*math.sin(lon_0), math.sin(lat_0)]

            SEV_hat_matrix = numpy.matrix([row0, row1, row2])

            SEV_matrix = SEV_hat_matrix * rho_matrix
            S = SEV_matrix[0]
            E = SEV_matrix[1]
            V = SEV_matrix[2]

            # Calculate azimuth
            SEV_az_rad = math.atan2(E, -S)
            SEV_az_deg = math.degrees(SEV_az_rad)

            # Fix 180 degree
            delta_lon = lon_GPS - lon_0
            if delta_lon >= 0:
                final_az = SEV_az_deg
            if delta_lon < 0:
                final_az = 360 + SEV_az_deg

            #Calculate elevation
            my_rho = math.sqrt((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
            el_rad = math.asin(V/my_rho)
            el_deg = math.degrees(el_rad)

            r = math.sqrt((delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z))

        except ZeroDivisionError, err:
            self.logger.error("ZeroDivsionError exception in gps calculations: %s" % (err))
            return self.prev_az, self.prev_el, height

        except ValueError, err:
            self.logger.error("ValueError exception in gps calculations: %s" % (err))
            return self.prev_az, self.prev_el, height

        except FloatingPointError, err:
            self.logger.error("FloatingPoinError exception in gps calculations: %s" % (err))
            return self.prev_az, self.prev_el, height


        final_az = final_az + self.get_north_angle()
        if final_az > 180.0:
            final_az = final_az - 360.0

        self.prev_az = final_az
        self.prev_el = el_deg


        return final_az, el_deg


    def read_adc(self, prev):
        counter = 0.0
        value = 0.0
        start = time.time()

        while time.time() - start < self.time_window:
            try:
                f = open(self.adc_filename)
                data = f.read()
                f.close()
                content = data.split("\n")
                value = value + (float(content[0])/self.relative_value)
                counter = counter + 1.0
            except IOError, msg:
                self.logger.error("ADC file problems : %s" % (msg))
                return 0.0
            except ValueError, msg:
                self.logger.error("Float errors : %s" % (msg))
                return 0.0

        #self.logger.debug(counter)
        output_value = value / counter
        return output_value
        #if abs((output_value - prev)) < self.prev_offset:
        #    self.rssi_pin.set(prev)
        #    return prev
        #else:
        #    self.rssi_pin.set(output_value)
        #    return output_value

r = RssiPoller()

if not r.sig_sim:
    r.logger.info("Starting polling")
    try:
        prev = 0.0
        while True:
            value = r.read_adc(prev)
            prev = value

    except KeyboardInterrupt:
        r.sd.stop()
else:
    r.logger.info("Starting rssi-simulation")
    r.simulation()
    r.sd.stop()
    r.socket.close()
