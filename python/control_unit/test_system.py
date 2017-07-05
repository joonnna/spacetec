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
        out0 = self.sm.halrcomps["rbldc0"].getpin("in")
        out0.on_value_changed.append(self.wait_callback)

        out1 = self.sm.halrcomps["rbldc1"].getpin("in")
        out1.on_value_changed.append(self.wait_callback)

        self.sm.halrcomps["rrssi"].getpin("out").set(3.0)

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

    def test_low_signal(self):
        self.sm.halrcomps["rrssi"].getpin("out").set(1.0)

        state = self.sm.halrcomps["rsigcheck"].getpin("in")
        state.on_value_changed.append(self.wait_callback)

        self.wait()

        self.assertEqual(self.sm.state, gps)

    def test_high_signal(self):
        self.sm.halrcomps["rrssi"].getpin("out").set(10.0)

        #Ensure that the value is proagated through the system
        time.sleep(self.update_timeout*2)

        state = self.sm.halrcomps["rsigcheck"].getpin("in")
        state.on_value_changed.append(self.wait_callback)

        self.wait()

        self.assertEqual(self.sm.state, operational)

    """
    def test_alternation(self):

    def test_gps_output(self):
        pass
    """



if __name__ == '__main__':
    unittest.main()
