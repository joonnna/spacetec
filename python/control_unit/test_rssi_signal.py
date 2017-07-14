from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class RssiTest(BaseTest):

    def init(self):

        self.thread1 = new_thread(mock_up_func, mock_up_cleanup, 5.0)
        self.no_client = True
        #Simulate client started
        self.sm.set_state(State.stop_idle)

        self.sm.halrcomps["test-set-angle"].getpin("az_angle").set(10000000.0)
        self.sm.halrcomps["test-set-angle"].getpin("el_angle").set(10000000.0)


    def test_low_signal_state(self):
        low_sig = self.sm.sig_limit - 1.0
        self.set_rssi_and_wait(low_sig)

        self.assertEqual(self.sm.state, State.gps)

    def test_high_signal_state(self):
        high_sig = self.sm.sig_limit + 1.0
        self.set_rssi_and_wait(high_sig)

        self.assertEqual(self.sm.state, State.tracking)

    def test_low_to_high_to_low_signal(self):
        low_sig = self.sm.sig_limit - 1.0
        self.set_rssi_and_wait(low_sig)

        self.assertEqual(self.sm.state, State.gps)

        high_sig = self.sm.sig_limit + 1.0
        self.set_rssi_and_wait(high_sig)

        self.assertEqual(self.sm.state, State.tracking)

        self.set_rssi_and_wait(low_sig)

        self.assertEqual(self.sm.state, State.gps)


    def set_rssi_and_wait(self, val):
        self.sm.halrcomps["rrssi"].getpin("out").set(val)

        state = self.sm.halrcomps["step"].getpin("track")
        state.on_value_changed.append(self.wait_callback)

        self.wait()

        self.clear()




if __name__ == '__main__':
    unittest.main()
