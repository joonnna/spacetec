from udpComm.server import *
from udpComm.client import *
from hal_control import *
from statemachine.state import *
import thread
import unittest

class StateTest(unittest.TestCase):

    def setUp(self):
        start_hal("../hal/system.hal")
        port = 5530
        ip = "192.168.5.4"

        self.sm = Statemachine("pos")
        comm = Communication(port, ip)
        self.smthread = thread.start_new_thread(sm.run, (comm.run,))

    def tearDown(self):
        self.smthread.exit()
        shutdown_hal()

    def test_rcomps_connection(self):
        for name, rcomp in self.sm.halrcomps.iteritems():
            self.assertTrue(rcomp.connected)

    def test_enter_gps_state(self):
        self.sm.change_state(True)
        self.asserEqual(self.sm.state, gps)

    def test_reenter_operational_state(self):
        self.sm.change_state(True)
        self.asserEqual(self.sm.state, gps)

        self.sm.change_state(False)
        self.asserEqual(self.sm.state, operational)

    def test_read_init_pos(self):
        val = 5.4
        val2 = 10.4
        self.sm.pos_thread.exit()

        f = open(self.filepath, "w")
        f.write("%f\n%f", val, val2)
        f.close()

        pos = self.sm.read_init_pos()

        self.assertEqual(val, pos[0])
        self.assertEqual(val, pos[1])

        self.assertEqual(self.init_az, pos[0])
        self.assertEqual(self.init_el, pos[1])


    def test_save_old_pos(self):
        val = 32.4
        val2 = 64.3

        self.sm.pos_thread.exit()

        f = open(self.filepath, "w")
        f.write("this is a test, needs to be overwritten")
        f.close()

        self.sm.set_pos(val, val2)

        thread = self.sm.start_pos_thread()

        time.sleep(self.pos_thread_timeout)

        self.sm.pos_thread.exit()

        f = open(self.filepath, "r")
        pos = f.read()
        f.close()

        self.assertEqual(val, pos[0])
        self.assertEqual(val2, pos[1])


    def test_pos_mutators(self):
        val = 20.0
        val2 = 30.0

        self.sm.set(val, val2)

        v1, v2 = self.sm.get()

        sel.assertEqual(v1, val)
        sel.assertEqual(v2, val2)

    def test_send_pos(self):



if __name__ == '__main__':
    unittest.main()
