from statemachine.state import *
from udpComm.server import *
from base_test_class import *
from general_thread import *
import time

class BldcinitTest(BaseTest):

    def init(self):

        self.thread1 = new_thread(mock_up_func, mock_up_cleanup, 5.0)
        self.no_client = True
        #Simulate client started
        self.sm.set_state(Override.stop)

        self.sm.halrcomps["set-angle"].getpin("az_angle").set(10000000.0)
        self.sm.halrcomps["set-angle"].getpin("el_angle").set(10000000.0)



if __name__ == '__main__':
    unittest.main()
