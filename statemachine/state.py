import gobject
import time
import sys
import threading
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote


class Statemachine():
    def __init__(self):
        #self.state =
        self.sd = ServiceDiscovery()
        #self.sd.on_discovered.append(self.service_discovered)

        rcomp = halremote.RemoteComponent('muxdemo', debug=True)
        #rcomp.no_create = True
        #b = rcomp.newpin('button0', halremote.HAL_FLOAT, halremote.HAL_OUT)
        b = rcomp.newpin('select', halremote.HAL_S32, halremote.HAL_OUT)
        b.on_value_changed.append(self.test)
        #rcomp.newpin('button1', halremote.HAL_FLOAT, halremote.HAL_OUT)
        #rcomp.newpin('led', halremote.HAL_FLOAT, halremote.HAL_IN)
        rcomp.on_connected_changed.append(self._connected)

        self.halrcomp = rcomp
        self.sd.register(rcomp)
        self.sd.start()

        rcomp.bind_component()
        print "YOOOYOYOO"

        print "YOOOYOYOO"

    def test(self, value):
        print value
        print "YYYYEEEEEEEEEH"

    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def start(self):
        self.sd.start()

    def stop(self):
        self.sd.stop()

    def service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)


def main():
    #gobject.threads.init()
    sm = Statemachine()

    #print sm.halrcomp.pin("button0")
    time.sleep(5)

    sm.halrcomp.getpin("select").set(1)
    #pin = sm.halrcomp.newpin("testyo", halremote.HAL_BIT, halremote.HAL_IN)
    #sm.halrcomp.pin_change(pin)
    #sm.halrcomp.add_pins()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    sm.stop()

    while threading.active_count() > 1:
        time.sleep(0.1)

    print('threads stopped')
    sys.exit(0)
    #loop = gobject.MainLoop()

    #try:
    #    loop.run()
    #except KeyboardInterrupt:
    #    loop.quit()

    print ("Stopped")

if __name__ == "__main__":
    main()
