from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class CalibrationTest(BaseTest):

    def init(self):
        comm = Communication(self.port, self.ip)

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_gps_pos)
        self.thread2 = new_thread(comm.get_local_gps_pos, comm.gps_cleanup, 10.0)

        self.client = True

    def test_alternation(self):
        checker = self.sm.halrcomps["check-step"]
        az_in = checker.getpin("az_in").get()
        el_in = checker.getpin("el_in").get()

        if az_in == -1.0:
            self.assertNotEqual(el_in, -1.0)
        elif el_in == -1.0:
            self.assertNotEqual(az_in, -1.0)
        else:
            self.assertFalse(True)

if __name__ == '__main__':
    unittest.main()
