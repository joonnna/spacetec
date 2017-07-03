from udpComm.server import *
from udpComm.client import *
from hal_control import *
from statemachine.state import *
import thread
import unittest

class SystemTest(unittest.TestCase):

    def setUp(self):
        start_hal("../hal/system.hal")
        port = 5530
        ip = "192.168.5.4"

        pos_filename = "pos"
        f = open(pos_filename, "w")
        str = "%f\n%f" % (5435345.4, 54542.4)
        f.write(str)
        f.close()

        self.sm = Statemachine(pos_filename)
        thread.start_new_thread(self.sm.run, (mock_up_callback,))

    def tearDown(self):
        self.sm.pos_thread.stop()
        shutdown_hal()

    def test_rcomps_connection(self):
        for name, rcomp in self.sm.halrcomps.iteritems():
            rcomp.wait_connected()
            self.assertTrue(rcomp.connected)

    def test_enter_gps_state(self):
        self.sm.change_state(True)
        self.assertEqual(self.sm.state, gps)

    def test_reenter_operational_state(self):
        self.sm.change_state(True)
        self.assertEqual(self.sm.state, gps)

        self.sm.change_state(False)
        self.assertEqual(self.sm.state, operational)

    def test_read_init_pos(self):
        val = 5.4
        val2 = 10.4
        self.sm.stop_pos_thread()

        f = open(self.sm.filepath, "w")
        f.write(("%f\n%f" % (val, val2)))
        f.close()

        pos = self.sm.read_init_pos()

        az = pos[0]
        el = pos[1]

        self.assertEqual(val, az)
        self.assertEqual(val2, el)

        self.assertEqual(val, az)
        self.assertEqual(val2, el)

    def test_get_pos(self):
        v1, v2 = self.sm.get_pos()

        self.assertEqual(v1, 0.0)
        self.assertEqual(v2, 0.0)


def mock_up_callback(cb):
    pass

if __name__ == '__main__':
    unittest.main()
