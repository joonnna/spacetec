from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class NearTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/near.hal"

        self.input = self.incomp.newpin("in", halremote.HAL_BIT, halremote.HAL_IN)

        self.out0 = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.out1 = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.diff = self.outcomp.newpin("diff", halremote.HAL_FLOAT, halremote.HAL_IO)

        self.input.on_value_changed.append(self.wait_callback)
        self.diff.on_value_changed.append(self.wait_callback)

    def test_not_near(self):
        self.out0.set(5.0)
        self.out1.set(7.0)

        self.wait()

        self.assertFalse(self.input.get())

    def test_near(self):
        self.out0.set(5.0)
        self.out1.set(5.0)

        self.wait()

        self.assertTrue(self.input.get())

    def test_diff(self):
        var0 = 5.0
        var1 = 10.0
        self.diff.set(var1-var0)
        self.wait()
        self.clear()

        self.out0.set(var0)
        self.out1.set(var1)

        self.wait()

        self.assertTrue(self.input.get())

if __name__ == '__main__':
    unittest.main()
