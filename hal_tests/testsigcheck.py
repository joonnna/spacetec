from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class SigcheckTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/sigcheck.hal"

        self.lim = self.incomp.newpin("lim", halremote.HAL_BIT, halremote.HAL_IN)
        self.in0 = self.incomp.newpin("in0", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.in1 = self.incomp.newpin("in1", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.out = self.outcomp.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.in0.on_value_changed.append(self.wait_callback)
        self.in1.on_value_changed.append(self.wait_callback)
        self.lim.on_value_changed.append(self.wait_callback)

    def test_input_output(self):
        input = 5.0

        self.send_input(input)

        self.wait()
        self.assertTrue(self.in0.synced)
        self.assertTrue(self.in1.synced)

        in0 = self.in0.get()
        in1 = self.in1.get()

        if in0 > 0.0:
            self.assertEqual(in0, input)
        elif in1 > 0.0:
            self.assertEqual(in1, input)
        else:
            self.assertEqual(in0, input)

    def test_alternating(self):
        alternate_val = -1.0
        input0 = 250.0

        self.send_input(input0)

        self.wait()
        self.assertTrue(self.in0.synced)
        self.assertTrue(self.in1.synced)

        input1 = 15.0

        self.send_input(input1)

        self.wait()
        self.assertTrue(self.in0.synced)
        self.assertTrue(self.in1.synced)

        in0 = self.in0.get()
        in1 = self.in1.get()

        if in0 > 0.0:
            self.assertEqual(in0, input1)
            self.assertEqual(in1, alternate_val)
        elif in1 > 0.0:
            self.assertEqual(in1, input1)
            self.assertEqual(in0, alternate_val)
        else:
            self.assertEqual(in0, input1)

    #TODO Sig limit is 3.0 atm (hardcoded), config file plz
    def test_above_sig_limit(self):
        input = 30.0

        self.send_input(input)

        self.wait()

        self.assertTrue(self.lim.synced)
        self.assertFalse(self.lim.value)

    #TODO Sig limit is 3.0 atm (hardcoded), config file plz
    def test_below_sig_limit(self):
        input = 2.0

        self.send_input(input)

        self.wait()
        self.assertTrue(self.lim.synced)
        self.assertTrue(self.lim.get())

    def send_input(self, input):
        self.out.set(input)

        time.sleep(self.timeWait)

        self.assertTrue(self.out.synced)
        self.assertEqual(self.out.get(), input)


if __name__ == '__main__':
    unittest.main()
