import time
import sys
from general_thread import *
from hal_control import *
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
        self.start_pos_thread()
        self.check_threads_timeout = 2.0

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

        rssi = halremote.RemoteComponent("rrssi", debug=False)
        rssi.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        bldc0 = halremote.RemoteComponent("rbldc0", debug=False)
        bldc0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        bldc1 = halremote.RemoteComponent("rbldc1", debug=False)
        bldc1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        self.halrcomps[mux0.name] = mux0
        self.halrcomps[mux1.name] = mux1
        self.halrcomps[abspos0.name] = abspos0
        self.halrcomps[abspos1.name] = abspos1
        self.halrcomps[sigcheck.name] = sigcheck
        self.halrcomps[rssi.name] = rssi
        self.halrcomps[bldc0.name] = bldc0
        self.halrcomps[bldc1.name] = bldc1

    def change_state(self, sig):
        print sig
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
        print repr(pos[0]), repr(pos[1])
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
        data = pos.split("\n")
        if len(data) < 2:
            fw = open(self.filepath, "w")
            old_pos = self.get_pos()
            fw.write(("%f\n%f" % (old_pos[0], old_pos[1])))
            fw.close()

    def send_pos(self, pos):
        az = self.init_az + pos[0]
        el = self.init_el + pos[1]

        mux0 = self.halrcomps["rmux0"]
        mux0.getpin("out1").set(az)

        mux1 = self.halrcomps["rmux1"]
        mux1.getpin("out1").set(el)

    def start_pos_thread(self):
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

    def start_comm_thread(self):
        self.comm_thread = new_thread(self.comm_func, self.comm_cleanup, self.comm_thread_timeout, self.send_pos)
        self.comm_thread.start()

    def start_gps_thread(self):
        self.gps_thread = new_thread(self.gps_func, self.gps_cleanup, self.gps_thread_timeout)
        self.gps_thread.start()


    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def _service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)

    def cleanup(self):
        self.sd.stop()

        for name, rcomp in self.halrcomps.iteritems():
            rcomp.remove_pins()
            rcomp.set_disconnected()

        shutdown_hal()

        self.pos_thread.stop()
        self.comm_thread.stop()
        self.gps_thread.stop()

        self.comm_thread.join()
        self.pos_thread.join()
        self.gps_thread.join()

        if self.cleanup_event != None:
            self.cleanup_event.set()
        else:
            print "KA I FAEN"

        print "Stopped"

    def run(self, comm_thread, gps_thread, exit_event=None, cleanup_event=None):
        self.cleanup_event = cleanup_event

        self.comm_thread = comm_thread
        self.comm_func = comm_thread.func
        self.comm_cleanup = comm_thread.cleanup
        self.comm_thread_timeout = comm_thread.timeout
        self.comm_thread.start()

        self.gps_thread = gps_thread
        self.gps_func = gps_thread.func
        self.gps_cleanup = gps_thread.cleanup
        self.gps_thread_timeout = gps_thread.timeout
        self.gps_thread.start()

        try:
            while True:
                time.sleep(self.check_threads_timeout)

                if not self.pos_thread.is_alive():
                    self.start_pos_thread()

                if not self.comm_thread.is_alive():
                    self.start_comm_thread()

                if not self.gps_thread.is_alive():
                    self.start_gps_thread()

                #If executed as secondary thread instead of main thread
                if exit_event != None:
                    if exit_event.isSet():
                        break
        except KeyboardInterrupt:
            pass

        self.cleanup()
