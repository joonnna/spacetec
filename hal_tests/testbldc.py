from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class BldcTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/bldc.hal"

        self.input = self.incomp.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.output = self.outcomp.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.input.on_value_changed.append(self.wait_callback)

    def test_input_output(self):
        val = 20.0

        self.output.set(val)
        self.assertFalse(self.output.synced)

        #Ensure values are synced
        time.sleep(self.timeWait)

        self.assertTrue(self.output.synced)
        self.assertEqual(self.output.get(), val)

        self.wait()
        self.assertTrue(self.input.synced)
        self.assertNotEqual(self.input.get(), 0.0)

if __name__ == '__main__':
    unittest.main()
