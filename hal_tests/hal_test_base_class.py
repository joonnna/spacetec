import unittest
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_control import *
import time

class HalBaseTest(unittest.TestCase):

    def wait_callback(self, val):
        self.updated = True

    def init(self):
        pass

    def wait(self):
        start = time.time()

        while True:
            if self.updated:
                self.updated = False
                return

            if time.time() - start > self.timeWait:
                return

    def setUp(self):
        self.updated = False
        self.timeWait = 2
        self.rcomps = []
        self.sd = ServiceDiscovery()

        self.incomp = halremote.RemoteComponent("incomp", debug=False)
        self.outcomp = halremote.RemoteComponent("outcomp", debug=False)


        self.rcomps.append(self.incomp)
        self.rcomps.append(self.outcomp)

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
            rcomp.set_disconnected()
        shutdown_hal()
