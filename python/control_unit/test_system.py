from hal_control import *
from statemachine.state import *
from udpComm.server import *
from udpComm.client import *
import thread
import unittest

class SystemTest(unittest.TestCase):

    def setUp(self):
        start_hal("../../hal/system.hal")
        port = 5530
        ip = "192.168.5.4"

        pos_filename = "pos"
        f = open(pos_filename, "w")
        str = "%f\n%f" % (5435345.4, 54542.4)
        f.write(str)
        f.close()

        comm = Communication(port, ip)
        self.sm = Statemachine(pos_filename)

        comm_thread = new_thread(comm.run, comm.shutdown, 0.0, self.sm.send_pos)
        gps_thread = new_thread(comm.get_local_gps_pos, comm.gps_cleanup, 10.0)

        self.exit_event = threading.Event()
        thread.start_new_thread(self.sm.run, (comm_thread, gps_thread, self.exit_event))

    def tearDown(self):
        self.exit_event.set()
        shutdown_hal()

    def test_input_output(self):
        self.sm.halrcomps["rrssi"].getpin("out").set(3.0)

        time.sleep(10)

        out0 = self.sm.halrcomps["rbldc0"].getpin("in").get()

        out1 = self.sm.halrcomps["rbldc1"].getpin("in").get()

        print out0, out1

        self.assertNotEqual(out0, 0.0)
    """
    def test_low_signal(self):
        pass

    def test_low_to_high_signal(self):
        pass

    def test_alternation(self):
        pass

    def test_gps_output(self):
        pass
    """



if __name__ == '__main__':
    unittest.main()
