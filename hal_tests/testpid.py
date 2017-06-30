from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from hal_test_base_class import *
import time


class PidTest(HalBaseTest):
    def init(self):
        self.halfile = "test_hal_files/pid.hal"

        self.input = self.incomp.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.command    = self.outcomp.newpin("out0", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.feedback   = self.outcomp.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.igain      = self.outcomp.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)
        self.pgain      = self.outcomp.newpin("out3", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.input.on_value_changed.append(self.wait_callback)

    def test_igain_setter(self):
        val = 5.0

        self.igain.set(val)
        self.assertFalse(self.igain.synced)

        time.sleep(self.timeWait)

        self.assertTrue(self.igain.synced)
        self.assertEqual(self.igain.get(), val)


    def test_pgain_setter(self):
        val = 5.0

        self.pgain.set(val)
        self.assertFalse(self.pgain.synced)

        time.sleep(self.timeWait)

        self.assertTrue(self.pgain.synced)
        self.assertEqual(self.pgain.get(), val)

    #TODO getting no output atm, fix plz
    def test_input_output(self):
        command = 5.0
        feedback = 6.0
        igain = 20.0
        pgain = 25.0


        self.command.set(command)
        self.feedback.set(feedback)
        self.pgain.set(pgain)
        self.igain.set(pgain)

        time.sleep(self.timeWait)

        self.assertTrue(self.command.synced)
        self.assertTrue(self.feedback.synced)
        self.assertTrue(self.pgain.synced)
        self.assertTrue(self.igain.synced)

        self.wait()
        self.assertTrue(self.input.synced)

        self.assertNotEqual(self.input.value, 0.0)

if __name__ == '__main__':
    unittest.main()
