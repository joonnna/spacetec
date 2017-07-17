from udpComm.server import *
from udpComm.client import *
import thread
import unittest

class CommTest(unittest.TestCase):

    def setUp(self):
        self.datalength = 200

        self.client = Udpclient()
        self.comm = Communication()

    def tearDown(self):
        self.comm.shutdown()

    def test_recv_ptu_data(self):
        ptudata = open("ptudata", "r")
        file_data = ptudata.read()
        lines = file_data.split("\n")

        data = lines[10]
        self.assertEqual(len(data), self.datalength)

        pos = extract_pos(data)

        self.thread = thread.start_new_thread(self.client.run, (data,))

        received_pos = self.comm._receive_data()

        for idx, var in enumerate(pos):
            self.assertEqual(var, received_pos[idx])


    #TODO Don't know expected output...
   # def test_calc_pos(self):
   #     pass

 #   def test_gps_input(self):
  #      self.comm.get_local_gps_pos()


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
