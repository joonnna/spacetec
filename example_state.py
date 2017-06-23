import time
import sys
import threading

from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote


class BasicClass(object):
    def __init__(self):
        self.sd = ServiceDiscovery()

        rcomp = halremote.RemoteComponent('anddemo', debug=False)
        #rcomp.no_create = True
        rcomp.newpin('button0', halremote.HAL_BIT, halremote.HAL_OUT)
        rcomp.newpin('button1', halremote.HAL_BIT, halremote.HAL_OUT)
        led_pin = rcomp.newpin('led', halremote.HAL_BIT, halremote.HAL_IN)
        led_pin.on_value_changed.append(self.led_pin_changed)
        led_pin.on_synced_changed.append(self.led_pin_synced)
        rcomp.on_connected_changed.append(self._connected)

        self.halrcomp = rcomp
        self.sd.register(rcomp)
        self.sd.start()

    def led_pin_synced(self, synced):
        if synced:
            print("LED pin synced")

    def led_pin_changed(self, value):
        print('LED pin value changed: %s' % str(value))

    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def start(self):
        self.sd.start()

    def stop(self):
        self.sd.stop()

def main():
    basic = BasicClass()

    print('starting')
    time.sleep(1)
    #print basic.halrcomp.pinsbyname['button0'].get()
    #print basic.halrcomp.pinsbyname['button0'].set(True)
    #print basic.halrcomp.pinsbyname['button0'].get()

    #print basic.halrcomp.pin_change(basic.halrcomp.pinsbyname['button0'])

    #print basic.halrcomp.on_connected_changed

    #print basic.halrcomp.connected
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    print('stopping threads')
    basic.stop()

    # wait for all threads to terminate
    while threading.active_count() > 1:
        time.sleep(0.1)

    print('threads stopped')
    sys.exit(0)

if __name__ == "__main__":
    main()
