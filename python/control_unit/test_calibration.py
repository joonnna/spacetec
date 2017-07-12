from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time
import thread

class CalibrationTest(unittest.TestCase):

    def setUp(self):
        start_hal("/home/machinekit/machinekit/spacetec/hal/test_system.hal")

        self.pos_filepath = "/home/machinekit/machinekit/spacetec/data_files/pos"
        #self.pos_file = open(self.pos_filepath, "w")

    def tearDown(self):
        pass

    def test_calibrate_az(self):
        az_start = 0.0
        f = open(self.pos_filepath, "w")
        f.write("%f\n%f" % (az_start, 0.0))
        f.close()

        test_feedback = sm.halrcomps["test-feedback"]

        stop_az = test_feedback.getpin("stop_az")
        stop_el = test_feedback.getpin("stop_el")

        stop_az.set(False)
        stop_el.set(False)




        motor_feedback = self.halrcomps["motor-feedback"]


if __name__ == '__main__':
    unittest.main()
