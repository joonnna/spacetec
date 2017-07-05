import threading
import unittest
from udpComm.server import *
from udpComm.client import *
from statemachine.state import *
import time

class BaseTest(unittest.TestCase):

    def init(self):
        pass

    def clear(self):
        self.update_event.clear()

    def wait(self):
        self.update_event.wait(self.update_timeout)

    def wait_callback(self, val):
        self.update_event.set()

    def setUp(self):
        self.exit_event = threading.Event()

        self.cleanup_timeout = 40.0
        self.cleanup_event = threading.Event()

        self.update_event = threading.Event()
        self.update_timeout = 5.0

        start_hal("../../hal/system.hal")
        self.port = 5530
        self.ip = "192.168.5.4"

        pos_filename = "pos"
        f = open(pos_filename, "w")
        str = "%f\n%f" % (5435345.4, 54542.4)
        f.write(str)
        f.close()

        self.sm = Statemachine(pos_filename)

        for name, rcomp in self.sm.halrcomps.iteritems():
            rcomp.wait_connected()
            self.assertTrue(rcomp.connected)

        self.init()

        thread.start_new_thread(self.sm.run, (self.thread1, self.thread2, self.exit_event, self.cleanup_event))

        if self.client:
            test_client = Udpclient(self.port, self.ip)
            thread.start_new_thread(test_client.run, ())

    def tearDown(self):
        self.exit_event.set()
        self.cleanup_event.wait(self.cleanup_timeout)
