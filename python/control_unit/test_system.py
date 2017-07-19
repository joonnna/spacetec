from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class SystemTest(BaseTest):

    def init(self):
        comm = Communication()

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_gps_pos)

        self.sm.halrcomps["test-set-angle"].getpin("az_angle").set(10000000.0)
        self.sm.halrcomps["test-set-angle"].getpin("el_angle").set(10000000.0)

        self.start_client()

    def test_input_output(self):
        self.sm.halrcomps["rrssi"].getpin("out").set(self.sm.sig_limit + 1.0)

        out0 = self.sm.halrcomps["rbldc0"].getpin("in")
        out0.on_value_changed.append(self.wait_callback)

        out1 = self.sm.halrcomps["rbldc1"].getpin("in")
        out1.on_value_changed.append(self.wait_callback)

        self.wait()

        val0 = out0.get()
        val1 = out1.get()

        if val0 != 0.0:
            self.assertNotEqual(val0, 0.0)
        elif val1 != 0.0:
            self.assertNotEqual(val1, 0.0)
        else:
            self.assertNotEqual(val0, 0.0)

    def test_gps_output(self):
        low_sig = self.sm.sig_limit - 1.0

        self.sm.halrcomps["rrssi"].getpin("out").set(low_sig)

        time.sleep(3)

        out0 = self.sm.halrcomps["rbldc0"].getpin("in")
        out0.on_value_changed.append(self.wait_callback)

        out1 = self.sm.halrcomps["rbldc1"].getpin("in")
        out1.on_value_changed.append(self.wait_callback)

        self.wait()

        val0 = out0.get()
        val1 = out1.get()

        if val0 != 0.0:
            self.assertNotEqual(val0, 0.0)
        elif val1 != 0.0:
            self.assertNotEqual(val1, 0.0)
        else:
            self.assertNotEqual(val0, 0.0)

    def test_alternation(self):
        sig = 10.0

        self.sm.halrcomps["rrssi"].getpin("out").set(sig)

        time.sleep(3)

        checker = self.sm.halrcomps["check-step"]
        az_in = checker.getpin("az_in").get()
        el_in = checker.getpin("el_in").get()

        if az_in == -1.0:
            self.assertNotEqual(el_in, -1.0)
        elif el_in == -1.0:
            self.assertNotEqual(az_in, -1.0)
        else:
            self.assertFalse(True)
 #   def test_idle_until_udp(self):


    """
    def test_calibration(self):
        pass

    def test_gps_checker(self):
        pass

    def test_reset_pos(self):
        pass

    def test_increasing_signal(self):
        pass

    def test_decreasing_signal(self):
        pass
    """


if __name__ == '__main__':
    unittest.main()
