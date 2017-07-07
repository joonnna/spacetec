from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class SystemTest(BaseTest):

    def init(self):
        comm = Communication(self.port, self.ip)

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_pos)
        self.thread2 = new_thread(comm.get_local_gps_pos, comm.gps_cleanup, 10.0)

        self.client = True


    def test_input_output(self):
        self.sm.halrcomps["rrssi"].getpin("out").set(3.0)

        out0 = self.sm.halrcomps["rbldc0"].getpin("in")
        out0.on_value_changed.append(self.wait_callback)

        out1 = self.sm.halrcomps["rbldc1"].getpin("in")
        out1.on_value_changed.append(self.wait_callback)

        self.wait()

        val0 = out0.get()
        val1 = out1.get()
        print val0, val1

        if val0 != 0.0:
            self.assertNotEqual(val0, 0.0)
        elif val1 != 0.0:
            self.assertNotEqual(val1, 0.0)
        else:
            self.assertNotEqual(val0, 0.0)

    def test_low_signal_state(self):
        low_sig = 1.0
        self.set_rssi_and_wait(low_sig)

        self.assertEqual(self.sm.state, gps)

    def test_high_signal_state(self):
        high_sig = 10.0
        self.set_rssi_and_wait(high_sig)

        self.assertEqual(self.sm.state, operational)

    def test_gps_input(self):
        val0  = self.halrcomps["rmux0"].getpin("out1").get()
        val1  = self.halrcomps["rmux1"].getpin("out1").get()

        self.assertNotEqual(val0, 0.0)
        self.assertNotEqual(val1, 0.0)


    """
    def test_alternation(self):
    """

    def test_reset_pos(self):
        pass

    def test_increasing_signal(self):

    def test_decreasing_signal(self):

    def test_gps_output(self):
        low_sig = 1.0
        self.set_rssi_and_wait(low_sig)

        out0 = self.sm.halrcomps["rbldc0"].getpin("in")
        out0.on_value_changed.append(self.wait_callback)

        out1 = self.sm.halrcomps["rbldc1"].getpin("in")
        out1.on_value_changed.append(self.wait_callback)

        self.wait()

        val0 = out0.get()
        val1 = out1.get()
        print val0, val1

        if val0 != 0.0:
            self.assertNotEqual(val0, 0.0)
        elif val1 != 0.0:
            self.assertNotEqual(val1, 0.0)
        else:
            self.assertNotEqual(val0, 0.0)


    def set_rssi_and_wait(self, val):
        self.sm.halrcomps["rrssi"].getpin("out").set(val)

        state = self.sm.halrcomps["rsigcheck"].getpin("in")
        state.on_value_changed.append(self.wait_callback)

        self.wait()

        self.clear()

if __name__ == '__main__':
    unittest.main()
