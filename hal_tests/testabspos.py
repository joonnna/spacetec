from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class AbsposTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/abspos.hal"

        self.in0 = self.incomp.newpin("in0", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.in1 = self.incomp.newpin("in1", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.prev = self.incomp.newpin("in2", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.out0 = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.out1 = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.in0.on_value_changed.append(self.wait_callback)
        self.in1.on_value_changed.append(self.wait_callback)

    def test_input_output(self):
        input = 15.0
        self.single_input(input)

        self.wait()
        self.assertTrue(self.in0.synced)
        #TODO RT components runs too fast, can't properly test single input, fix this
        self.assertEqual(self.in0.get(), input*2)


    def test_successive_inputs(self):
        input0 = 5.0
        self.single_input(input0)

        input1 = 15.0

        self.single_input(input1)

        self.wait()
        self.assertTrue(self.in0.synced)
        #TODO RT components runs too fast, can't properly test succesive input, fix this
        self.assertEqual(self.in0.get(), input1*2)

    def test_read_old_val(self):
        input = 5.0
        self.single_input(input)

        self.wait()
        self.assertTrue(self.in1.synced)
        self.assertEqual(self.in1.get(), input)

    def test_reset_oldval(self):
        input = 5.0
        reset = 1.0
        self.reset_old_val(reset, input)

    def test_successive_reset_oldval(self):
        for val in range(1, 4):
            self.reset_old_val(val, val+10)

    def reset_old_val(self, reset, input):
        self.single_input(input)
        self.wait()
        self.assertTrue(self.in1.synced)
        self.assertEqual(self.in1.get(), input)

        self.out1.set(reset)
        self.assertFalse(self.out1.synced)

        time.sleep(self.timeWait)

        self.assertTrue(self.out1.synced)
        self.assertEqual(self.out1.get(), reset)

        self.wait()
        self.assertTrue(self.prev.synced)
        self.assertEqual(self.prev.get(), reset)

    def single_input(self, input):
        self.out0.set(input)
        self.assertFalse(self.out0.synced)

        time.sleep(self.timeWait)

        self.assertTrue(self.out0.synced)
        self.assertEqual(self.out0.get(), input)

if __name__ == '__main__':
    unittest.main()
