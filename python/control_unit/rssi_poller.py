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

    def read_adc(self):
        f = open(self.adc_filename)
        data = f.read()
        f.close()
        content = data.split("\n")
        value = (float(content[0])/self.relative_value)
        self.rssi_pin.set(value)

r = RssiPoller()

r.logger.info("Starting polling")
try:
    while True:
        r.read_adc()
        time.sleep(0.001)
except KeyboardInterrupt:
    r.sd.stop()
