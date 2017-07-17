from general_thread import *
from statemachine.state import *
from statemachine.state_enum import *
from base_test_class import *
from udpComm.server import *
from udpComm.client import *

class ManualTest(BaseTest):

    def init(self):
        comm = Communication()

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_gps_pos)

        #Simulate client started
        #self.sm.set_state(State.stop_idle)

    def test_enter_manual_mode(self):
        self.start_client()

        self.sm.halrcomps["step"].getpin("manual").set(1)
        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), Manual.position)

        self.sm.halrcomps["step"].getpin("manual").set(2)
        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), Manual.velocity)

        self.sm.halrcomps["step"].getpin("manual").set(3)
        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), Manual.tracking)

    def test_exit_manual_mode(self):
        self.start_client()

        self.sm.halrcomps["step"].getpin("manual").set(1)
        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), Manual.position)

        self.sm.halrcomps["step"].getpin("manual").set(0)
        time.sleep(self.update_timeout)

        self.assertNotEqual(self.sm.get_state(), Manual.position)
        self.assertNotEqual(self.sm.get_state(), Manual.velocity)
        self.assertNotEqual(self.sm.get_state(), Manual.tracking)

    def test_stay_manual_mode(self):
        self.start_client()

        self.sm.halrcomps["step"].getpin("manual").set(1)
        time.sleep(self.update_timeout)

        self.assertEqual(self.sm.get_state(), Manual.position)

        self.change_to_all_states(Manual.position)

        self.assertEqual(self.sm.get_state(), Manual.position)

if __name__ == '__main__':
    unittest.main()
