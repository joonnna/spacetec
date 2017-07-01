from udpComm.server import *
from udpComm.client import *
import thread
import unittest

class CommTest(unittest.TestCase):

    def setUp(self):
        port = 2830
        ip = "127.0.0.1"

        client = Udpclient(port, ip)
        comm = Communication(port, ip)

        thread.start_new_thread(comm.run, (send_data,))
        client.run()




    def tearDown(self):


def send_data(pos):
    print "yoyoyo :", pos

