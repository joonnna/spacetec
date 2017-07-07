from general_thread import *
from statemachine.state import *
from base_test_class import *

class StateTest(BaseTest):

    def init(self):
        self.thread1 = new_thread(mock_up_func, mock_up_cleanup_func, 10.0)
        self.thread2 = new_thread(mock_up_func, mock_up_cleanup_func, 10.0)

        self.client = False

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

    def test_auto_restart_threads(self):
        time.sleep(self.sm.check_threads_timeout)
        self.sm.comm_thread.stop()
        self.sm.pos_thread.stop()
        self.sm.gps_thread.stop()

        time.sleep(self.sm.check_threads_timeout*3)

        self.assertTrue(self.sm.comm_thread.is_alive())
        self.assertTrue(self.sm.pos_thread.is_alive())
        self.assertTrue(self.sm.gps_thread.is_alive())

    def test_abspos_cleanup(self):
        pass




def mock_up_func():
    pass

def mock_up_cleanup_func():
    pass

def mock_up_callback():
    pass

if __name__ == '__main__':
    unittest.main()
