from hal_control import *
from general_thread import *
from statemachine.state import *
import threading
import unittest

class StateTest(unittest.TestCase):

    def setUp(self):
        start_hal("../../hal/system.hal")
        port = 5530
        ip = "192.168.5.4"

        pos_filename = "pos"
        f = open(pos_filename, "w")
        str = "%f\n%f" % (5435345.4, 54542.4)
        f.write(str)
        f.close()

        self.sm = Statemachine(pos_filename)
        self.mock_thread = new_thread(mock_up_func, mock_up_cleanup_func, 10.0, mock_up_callback)
        self.mock_thread2 = new_thread(mock_up_func, mock_up_cleanup_func, 10.0, mock_up_callback)

        self.exit_event = threading.Event()
        thread.start_new_thread(self.sm.run, (self.mock_thread, self.mock_thread2, self.exit_event))

    def tearDown(self):
        self.exit_event.set()
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
        self.sm.pos_thread.stop()

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


    def test_auto_restart_threads(self):
        time.sleep(self.sm.check_threads_timeout)
        self.sm.comm_thread.stop()
        self.sm.pos_thread.stop()
        self.sm.gps_thread.stop()

     #   self.sm.comm_thread.join()
      #  self.sm.pos_thread.join()

       # self.assertFalse(self.sm.pos_thread.is_alive())
       # self.assertFalse(self.sm.comm_thread.is_alive())

        time.sleep(self.sm.check_threads_timeout*3)

        self.assertTrue(self.sm.comm_thread.is_alive())
        self.assertTrue(self.sm.pos_thread.is_alive())
        self.assertTrue(self.sm.gps_thread.is_alive())

def mock_up_func(cb):
    print "yoyooyoyo"

def mock_up_cleanup_func():
    pass

def mock_up_callback():
    pass

if __name__ == '__main__':
    unittest.main()
