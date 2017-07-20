import logging
import time
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote


class RssiPoller():
    def __init__(self):
        logging.basicConfig(filename="/var/log/rssipoller.log", level=logging.DEBUG)
        self.logger = logging.getLogger("rssipoller")
        self.sd = ServiceDiscovery()

        self.relative_value = 1580.0/5.0
        self.prev_offset = 0.2

        self.adc_filename = "/sys/devices/ocp.3/helper.14/AIN0"

        self.initrcomps()

    def initrcomps(self):
        rssi_reader = halremote.RemoteComponent("rssi-reader", debug=False)
        self.rssi_pin = rssi_reader.newpin("value", halremote.HAL_FLOAT, halremote.HAL_OUT)
        rssi_reader.no_create = True

        self.sd.register(rssi_reader)
        self.sd.start()
        rssi_reader.bind_component()

        self.logger.info("Bound rssi-poller")

    def read_adc(self, prev):
        counter = 0.0
        value = 0.0
        time_offset = 0.1
        start = time.time()

        while time.time() - start < time_offset:
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

        self.logger.debug(counter)
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
