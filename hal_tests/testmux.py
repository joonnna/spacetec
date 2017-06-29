from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class MuxTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/mux.hal"

        self.input = self.incomp.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.out0 = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.out1 = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.select = self.outcomp.newpin("select", halremote.HAL_S32, halremote.HAL_OUT)

        self.input.on_value_changed.append(self.wait_callback)

    def test_input_selection(self):
        val0 = 20.0
        val1 = 30.0

        self.out0.set(val0)
        self.assertFalse(self.out0.synced)

        self.out1.set(val1)
        self.assertFalse(self.out1.synced)

        self.select.set(1)
        self.assertFalse(self.select.synced)

        #Ensure values are synced
        time.sleep(self.timeWait)

        self.assertTrue(self.out0.synced)
        self.assertTrue(self.out1.synced)
        self.assertTrue(self.select.synced)

        out1 = self.out1.get()
        self.assertEqual(val1, out1)

        self.wait()
        self.assertTrue(self.input.synced)
        self.assertEqual(out1, self.input.get())


if __name__ == '__main__':
    unittest.main()
