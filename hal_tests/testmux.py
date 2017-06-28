from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from mastertest import *
import time


class MuxTest(MasterTest):
    def init(self):
        self.halfile = "test_hal_files/mux.hal"
        self.mux = halremote.RemoteComponent("rmux", debug=False)
        self.input = self.mux.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.select = self.mux.newpin("out0", halremote.HAL_S32, halremote.HAL_OUT)
        self.out1 = self.mux.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.out2 = self.mux.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.input.on_value_changed.append(self.wait_callback)

        self.rcomps.append(self.mux)

    def test_connection(self):
        for rcomp in self.rcomps:
            rcomp.wait_connected(timeout=self.timeWait)
            self.assertTrue(rcomp.connected)

    def test_input_selection(self):
        val = 20.0
        val2 = 30.0
        self.out1.set(val)
        self.out2.set(val2)
        self.select.set(1)

        self.assertEqual(val, self.out1.get())

        self.wait()
        self.assertEqual(self.out1.get(), self.input.get())

if __name__ == '__main__':
    unittest.main()
