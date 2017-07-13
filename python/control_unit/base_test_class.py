import threading
import unittest
from udpComm.server import *
from udpComm.client import *
from statemachine.state import *
import time

class BaseTest(unittest.TestCase):

    def pos_file_values(self):
        pass

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

        self.low_height = "/home/machinekit/machinekit/spacetec/data_files/low_height"
        self.high_height = "/home/machinekit/machinekit/spacetec/data_files/high_height"

        self.cleanup_timeout = 40.0
        self.cleanup_event = threading.Event()

        self.client_exit_event = threading.Event()

        self.update_event = threading.Event()
        self.update_timeout = 5.0

        start_hal("/home/machinekit/machinekit/spacetec/hal/test_system.hal")
        self.port = 5530
        self.ip = "192.168.5.4"

        self.test_client = Udpclient(self.port, self.ip)

        self.pos_filepath = "/home/machinekit/machinekit/spacetec/data_files/pos"
        f = open(self.pos_filepath, "w")
        str = "%f\n%f" % (135.04, 92.18)
        f.write(str)
        f.close()

        self.sm = Statemachine(self.pos_filepath, True)

        for name, rcomp in self.sm.halrcomps.iteritems():
            rcomp.wait_connected()
            self.assertTrue(rcomp.connected)

        self.init()

        thread.start_new_thread(self.sm.run, (self.thread1, self.exit_event, self.cleanup_event))

    def start_client(self, data=None):
        thread.start_new_thread(self.test_client.run, (data, self.client_exit_event))

    def tearDown(self):
        self.exit_event.set()
        self.cleanup_event.wait()

        if self.no_client == None:
            self.test_client.shutdown()
            self.client_exit_event.wait()

def mock_up_func():
    pass

def mock_up_cleanup():
    pass
