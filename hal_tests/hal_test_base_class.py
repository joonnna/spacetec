import unittest
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_control import *
import time
import threading

class HalBaseTest(unittest.TestCase):

    def wait_callback(self, val):
        self.event.set()

    def wait(self):
        self.event.wait(self.timeWait)

    def clear(self):
        self.event.clear()

    def init(self):
        pass

    def setUp(self):
        self.event = threading.Event()
        self.timeWait = 3.0

        self.rcomps = []
        self.sd = ServiceDiscovery()

        self.incomp = halremote.RemoteComponent("incomp", debug=False)
        self.outcomp = halremote.RemoteComponent("outcomp", debug=False)

        self.rcomps.append(self.outcomp)
        self.rcomps.append(self.incomp)

        self.init()

        start_hal(self.halfile)

        for rcomp in self.rcomps:
            self.sd.register(rcomp)

        self.sd.start()

        for rcomp in self.rcomps:
            rcomp.bind_component()

        for rcomp in self.rcomps:
            rcomp.wait_connected(timeout=self.timeWait)
            self.assertTrue(rcomp.connected)

    def tearDown(self):
        self.sd.stop()
        for rcomp in self.rcomps:
            rcomp.remove_pins()
            rcomp.set_disconnected()
        self.rcomps = []
        self.incomp = 0
        self.outcomp = 0

        shutdown_hal()
