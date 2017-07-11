from general_thread import *
from statemachine.state import *
from statemachine.state_enum import *
from base_test_class import *
from udpComm.server import *
from udpComm.client import *

class StateTest(BaseTest):

    def init(self):
        comm = Communication(self.port, self.ip)

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_gps_pos)
        self.thread2 = new_thread(comm.get_local_gps_pos, comm.gps_cleanup, 10.0)

        self.client = False

    def test_stay_idle_with_no_client(self):
        self.sm.set_state(State.gps)
        self.assertEqual(self.sm.get_state(), State.idle)

        self.sm.set_state(State.gps_overide)
        self.assertEqual(self.sm.get_state(), State.idle)

        self.sm.set_state(State.stop_overide)
        self.assertEqual(self.sm.get_state(), State.idle)

        self.sm.set_state(State.tracking)
        self.assertEqual(self.sm.get_state(), State.idle)

        self.sm.set_state(State.calibrating)
        self.assertEqual(self.sm.get_state(), State.idle)

        self.sm.set_state(State.idle)
        self.assertEqual(self.sm.get_state(), State.idle)

    def test_exit_idle_state_on_udp_recieve(self):
        self.assertEqual(self.sm.get_state(), State.idle)

        self.start_client()

        time.sleep(self.update_timeout)

        self.assertNotEqual(self.sm.get_state(), State.idle)

    def test_gps_overide(self):
        self.start_client()

        time.sleep(self.update_timeout)


    def start_client(self):
        test_client = Udpclient(self.port, self.ip)
        thread.start_new_thread(test_client.run, ())


if __name__ == '__main__':
    unittest.main()
