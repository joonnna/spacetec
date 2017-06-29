from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class LimTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/lim.hal"

        self.input = self.incomp.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.out = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.max = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.min = self.outcomp.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)


        self.input.on_value_changed.append(self.wait_callback)

    def test_exceed_limit(self):
        min = 20.0
        max = 30.0
        input = 35.0

        self.set_and_assert_vals(min, max, input)

        self.wait()
        self.assertTrue(self.input.synced)
        self.assertEqual(self.input.get(), max)

    def test_below_limit(self):
        max = 30.0
        min = 20.0
        input = -5.0

        self.set_and_assert_vals(min, max, input)

        self.wait()
        self.assertTrue(self.input.synced)
        self.assertEqual(self.input.get(), min)

    def test_within_limit(self):
        max = 30.0
        min = 20.0
        input = 25

        self.set_and_assert_vals(min, max, input)

        self.wait()
        self.assertTrue(self.input.synced)
        self.assertEqual(self.input.get(), input)

    def set_and_assert_vals(self, min, max, input):
        self.max.set(max)
        self.assertFalse(self.max.synced)

        self.min.set(min)
        self.assertFalse(self.min.synced)

        self.out.set(input)
        self.assertFalse(self.out.synced)

        time.sleep(self.timeWait)

        self.assertTrue(self.max.synced)
        self.assertTrue(self.min.synced)
        self.assertTrue(self.out.synced)

        self.assertEqual(self.out.get(), input)
        self.assertEqual(self.max.get(), max)
        self.assertEqual(self.min.get(), min)


if __name__ == '__main__':
    unittest.main()
