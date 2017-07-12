from general_thread import *
from statemachine.state import *
from statemachine.state_enum import *
from base_test_class import *
from udpComm.server import *

class ThreadsTest(BaseTest):

    def init(self):
        comm = Communication(self.port, self.ip)

        self.thread1 = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_gps_pos)
        #self.thread2 = new_thread(comm.get_local_gps_pos, comm.gps_cleanup, 10.0)

    def test_read_init_pos(self):
        self.start_client()

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
        self.start_client()
        time.sleep(self.sm.check_threads_timeout)
        self.sm.comm_thread.stop()
        self.sm.pos_thread.stop()
     #   self.sm.gps_thread.stop()

        time.sleep(self.sm.check_threads_timeout*3)

        self.assertTrue(self.sm.comm_thread.is_alive())
        self.assertTrue(self.sm.pos_thread.is_alive())
    #    self.assertTrue(self.sm.gps_thread.is_alive())
    """
    def test_abspos_cleanup(self):
        pass
    """



def mock_up_func():
    pass

def mock_up_cleanup_func():
    pass

def mock_up_callback():
    pass

if __name__ == '__main__':
    unittest.main()
