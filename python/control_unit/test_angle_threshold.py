from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class AngleThresholdTest(BaseTest):

    def init(self):
        self.thread1 = new_thread(mock_up_func, mock_up_cleanup, 5.0)

        self.sm.halrcomps["rrssi"].getpin("out").set(1000000000.0)

        #Want to break override asap
        self.start_client(self.high_height)

    def test_correct_angle_values(self):
        self.sm.set_state(State.gps)
        self.assertEqual(self.sm.get_state(), State.gps)

        az_angle = self.sm.halrcomps["set-angle"].getpin("az_angle").get()
        el_angle = self.sm.halrcomps["set-angle"].getpin("el_angle").get()

        self.assertEqual(az_angle, self.sm.az_re_enter_limit)
        self.assertEqual(az_angle, self.sm.el_re_enter_limit)

        self.sm.halrcomps["set-angle"].getpin("az_angle").set(10000000.0)
        self.sm.halrcomps["set-angle"].getpin("el_angle").set(10000000.0)

        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), State.tracking)
        self.assertEqual(az_angle, self.sm.az_gps_limit)
        self.assertEqual(el_angle, self.sm.el_gps_limit)

    ##def test_angle_limits(self):




if __name__ == '__main__':
    unittest.main()
