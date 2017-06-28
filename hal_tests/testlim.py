from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from mastertest import *
#from .. import mastertest
import time


class LimTest(MasterTest):
    def init(self):
        self.halfile = "test_hal_files/lim.hal"
        self.lim = halremote.RemoteComponent("rlim", debug=False)
        self.input = self.lim.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.output = self.lim.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.max = self.lim.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.min = self.lim.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.output.on_value_changed.append(self.wait_callback)
        self.max.on_value_changed.append(self.wait_callback)
        self.min.on_value_changed.append(self.wait_callback)

        self.rcomps.append(self.lim)

    def test_connection(self):
        for rcomp in self.rcomps:
            rcomp.wait_connected(timeout=self.timeWait)
            self.assertTrue(rcomp.connected)


    def test_exceed_limit(self):
        max = 20.0
        input = 30.0

        self.max.set(max)
        self.wait()

        self.assertEqual(self.max.get(), max)

        self.output.set(input)
        self.wait()

        time.slepp(10)
        self.assertLess(self.output.get(), max)

    def test_below_limit(self):
        min = 20.0
        input = 10.0

        self.min.set(min)
        self.wait()

        self.assertEqual(self.min.get(), min)

        self.output.set(input)
        self.wait()

        time.slepp(10)
        self.assertGreater(self.output.get(), min)

if __name__ == '__main__':
    unittest.main()
