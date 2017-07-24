import logging
import time
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
import ConfigParser

class RssiPoller():
    def __init__(self):
        logging.basicConfig(filename="/var/log/rssipoller.log", level=logging.DEBUG)
        self.logger = logging.getLogger("rssipoller")
        self.sd = ServiceDiscovery()

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
        self.sd.start()
        rssi_reader.bind_component()

        self.logger.info("Bound rssi-poller")

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

        if abs((output_value - prev)) < self.prev_offset:
            self.rssi_pin.set(prev)
            return prev
        else:
            self.rssi_pin.set(output_value)
            return output_value

r = RssiPoller()

r.logger.info("Starting polling")
try:
    prev = 0.0
    while True:
        value = r.read_adc(prev)
        prev = value
except KeyboardInterrupt:
    r.sd.stop()
