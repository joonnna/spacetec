from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class SteptrackTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/steptrack.hal"

        self.input = self.incomp.newpin("in0", halremote.HAL_FLOAT, halremote.HAL_IN)
        self.prev = self.incomp.newpin("test", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.out0 = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.out1 = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.input.on_value_changed.append(self.wait_callback)
        self.prev.on_value_changed.append(self.wait_callback)

    def test_normal_step(self):
        sig_strength = 5.0
        prev_pos = 10.0

        new_pos = self.step(sig_strength, prev_pos)

        #Direction is initially false(steps in "negative direction")
        self.assertLess(new_pos, prev_pos)

    def test_follow_signal(self):
        sig_strength = 5.0
        prev_pos = 10.0

        #Simulate that the signal increased from moving in negative direction
        new_sig = 8.0

        second_pos = self.step(sig_strength, prev_pos)
        self.assertLess(second_pos, prev_pos)

        third_pos = self.step(new_sig, second_pos)
        self.assertLess(third_pos, second_pos)


    def test_store_prev(self):
        sig_strength = 5.0
        prev_pos = 10.0

        self.step(sig_strength, prev_pos)

        self.wait()
        self.assertTrue(self.prev.synced)

        prev_sig_strength = self.prev.get()
        self.assertEqual(prev_sig_strength, sig_strength)

    def test_change_prev(self):
        sig_strength = 5.0
        prev_pos = 10.0

        self.step(sig_strength, prev_pos)

        new_sig = 8.0

        self.step(new_sig, prev_pos)

        self.wait()
        self.assertTrue(self.prev.synced)

        prev_sig_strength = self.prev.get()
        self.assertNotEqual(prev_sig_strength, sig_strength)
        self.assertEqual(prev_sig_strength, new_sig)

    """
    def test_change_direction(self):
        pass

    """

    def step(self, sig_strength, prev_pos):
        self.out0.set(sig_strength)
        self.out1.set(prev_pos)

        time.sleep(self.timeWait)

        self.assertTrue(self.out0.synced)
        self.assertTrue(self.out1.synced)
        self.assertEqual(self.out0.get(), sig_strength)
        self.assertEqual(self.out1.get(), prev_pos)

        self.wait()
        self.assertTrue(self.input.synced)
        return self.input.get()

if __name__ == '__main__':
    unittest.main()
