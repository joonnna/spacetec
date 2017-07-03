import time
import sys
from general_thread import *
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote


init = 0
operational = 1
gps = 2

class Statemachine():
    def __init__(self, pos_file):
        self.state = init

        self.filepath = pos_file
        pos = self.read_init_pos()

        az = pos[0]
        el = pos[1]

        self.init_az = az
        self.init_el = el

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps()
        self._search_and_bind()

        self.pos_thread_timeout = 5.0
        self.reset_pos(az, el)
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

        self.state = operational

    def initrcomps(self):
        mux0 = halremote.RemoteComponent("rmux0", debug=False)
        mux0.newpin("out0", halremote.HAL_S32, halremote.HAL_OUT)
        mux0.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)

        mux1 = halremote.RemoteComponent("rmux1", debug=False)
        mux1.newpin("out0", halremote.HAL_S32, halremote.HAL_OUT)
        mux1.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)

        abspos0 = halremote.RemoteComponent("rabspos0", debug=False)
        abspos0.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        abspos1 = halremote.RemoteComponent("rabspos1", debug=False)
        abspos1.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        sigcheck = halremote.RemoteComponent("rsigcheck", debug=False)
        sig = sigcheck.newpin("in", halremote.HAL_BIT, halremote.HAL_IN)
        sig.on_value_changed.append(self.change_state)

        self.halrcomps[mux0.name] = mux0
        self.halrcomps[mux1.name] = mux1
        self.halrcomps[abspos0.name] = abspos0
        self.halrcomps[abspos1.name] = abspos1
        self.halrcomps[sigcheck.name] = sigcheck

    def change_state(self, sig):
        if sig:
            self.halrcomps["rmux0"].getpin("out1").set(1)
            self.halrcomps["rmux1"].getpin("out1").set(1)
            self.state = gps
            print "Entered gps state"
        else:
            self.halrcomps["rmux0"].getpin("out1").set(0)
            self.halrcomps["rmux1"].getpin("out1").set(0)
            self.state = operational
            print "Entered operational state"

    def _search_and_bind(self):
        for name, rcomp in self.halrcomps.iteritems():
            self.sd.register(rcomp)

        self.sd.start()

        for name, rcomp in self.halrcomps.iteritems():
            rcomp.bind_component()

    #TODO Need to do something incase file doesn't exist, enter gps state maybe...
    def read_init_pos(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read().split("\n")
            f.close()
        except IOError:
            print "Initial pos file not found! Exiting"
            sys.exit(1)

        return float(pos[0]), float(pos[1])

    def reset_pos(self, pos0, pos1):
        abspos0 = self.halrcomps["rabspos0"]
        abspos0.getpin("out").set(pos0)

        abspos1 = self.halrcomps["rabspos1"]
        abspos1.getpin("out").set(pos1)


    def get_pos(self):
        abspos0 = self.halrcomps["rabspos0"]
        pos0 = abspos0.getpin("in").get()

        abspos1 = self.halrcomps["rabspos1"]
        pos1 = abspos1.getpin("in").get()

        return pos0, pos1

    def store_old_abspos(self):
        p0, p1 = self.get_pos()
        try:
            f = open(self.filepath, "w")
            f.write(("%f\n%f" % (p0, p1)))
            f.close()
        except IOError:
            print "Can't save position, should restart"

    def cleanup_abspos_thread(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read()
            f.close()
        except IOError:
            print "Couldn't open position file during cleanup..."
            return

        #If file was altered somehow after initiating cleanup (not likely)
        if len(pos) < 2:
            fw = open(self.filepath, "w")
            old_pos = self.get_pos()
            fw.write(("%f\n%f" % (old_pos[0], old_pos[1])))
            fw.close()

    def send_pos(self, pos):
        az = self.init_az + pos[0]
        el = self.init_el + pos[1]

        mux0 = self.halrcomps["rmux0"]
        mux0.getpin("out").set(az)

        mux1 = self.halrcomps["rmux1"]
        mux1.getpin("out").set(el)


    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def _service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)

    def cleanup(self):
        self.sd.stop()
        self.pos_thread.stop()
        self.comm_thread.stop()

        self.comm_thread.join()
        self.pos_thread.join()

        print "Stopped"

    def run(self, thread, event=None):
        self.comm_thread = thread
        self.comm_thread.start()

        try:
            while True:
                time.sleep(0.5)

                #If executed as secondary thread instead of main thread
                if event != None:
                    if event.isSet():
                        break
        except KeyboardInterrupt:
            pass

        self.cleanup()
