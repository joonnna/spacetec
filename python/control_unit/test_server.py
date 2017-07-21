from udpComm.server import *
from udpComm.client import *
import thread
import threading
import unittest

class CommTest(unittest.TestCase):

    def setUp(self):
        self.datalength = 199

        self.exit_event = threading.Event()

        self.client = Udpclient()
        self.comm = Communication()

        thread.start_new_thread(self.client.run, (None, self.exit_event))

    def tearDown(self):
        self.client.shutdown()
        self.exit_event.wait()
        self.comm.shutdown()

    def test_recv_ptu_data(self):
        path = "/home/machinekit/machinekit/spacetec/data_files/low_height"
        ptudata = open(path, "r")
        file_data = ptudata.read()
        ptudata.close()
        lines = file_data.split("\n")

        data = lines[0]
        self.assertEqual(len(data), self.datalength)

        pos = extract_pos(data)

        self.thread = thread.start_new_thread(self.client.run, (path, self.exit_event))

        received_pos = self.comm._receive_data()


    #TODO Don't know expected output...
    def test_calc_pos(self):
        long = 18.968714
        lat = 69.661945
        height = 50000.0

        ret = self.comm._calc_pos((long, lat, height))

        self.assertEqual(69.05233049335735, ret[0])
        self.assertEqual(21.577413283839867, ret[1])


def extract_pos(data):
    long_start     = 171
    long_end       = 180
    lat_start      = 182
    lat_end        = 190
    height_start   = 192
    height_end     = 198

    longtitude = float(data[long_start:long_end])
    latitude = float(data[lat_start:lat_end])
    height = float(data[height_start:height_end])

    return (longtitude, latitude, height)



if __name__ == '__main__':
    unittest.main()
