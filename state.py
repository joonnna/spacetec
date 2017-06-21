import gobject
import time
import sys
import threading
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote


class Statemachine():
    def __init__(self):
        self.sd = ServiceDiscovery()
        #self.sd.on_discovered.append(self.service_discovered)

        rcomp = halremote.RemoteComponent('and2', debug=True)
        rcomp.no_create = True
        rcomp.newpin('button0', halremote.HAL_BIT, halremote.HAL_OUT)
        #rcomp.newpin('button1', halremote.HAL_BIT, halremote.HAL_OUT)
        rcomp.on_connected_changed.append(self._connected)

        self.halrcomp = rcomp
        self.sd.register(rcomp)
        self.sd.start()
        print "YOOOYOYOO"
        rcomp.bind_component()

        print "YOOOYOYOO"
        rcomp.wait_connected(10.0)

        print "YOOOYOYOO"



    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def start(self):
        self.sd.start()

    def stop(self):
        self.sd.stop()

    #def start_sd(self):
        #sd.start()
        #halrcmd_sd = ServiceDiscovery()
        #halrcmd_sd.on_discovered.append(self.halrcmd_discovered)
        #halrcmd_sd.start()

    def service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)

def main():
    #gobject.threads.init()
    sm = Statemachine()

    #sm.start()

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
