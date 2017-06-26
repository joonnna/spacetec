import gobject
import argparse
import time
import sys
import thread
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
        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps()
        self.search_and_bind()
        self.set_pos(pos)
        self.start_pos_thread()

    def initrcomps(self):
        mux = halremote.RemoteComponent("rmux", debug=True)
        select = mux.newpin("out", halremote.HAL_S32, halremote.HAL_OUT)
        mux.on_connected_changed.append(self._connected)

        abspos = halremote.RemoteComponent("rabspos", debug=True)
        abspos.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.on_connected_changed.append(self._connected)

        self.halrcomps[mux.name] = mux
        self.halrcomps[abspos.name] = abspos

    def search_and_bind(self):
        for name, rcomp in self.halrcomps.iteritems():
            self.sd.register(rcomp)

        self.sd.start()

        for name, rcomp in self.halrcomps.iteritems():
            rcomp.bind_component()

    def read_init_pos(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read().split("\n")
            f.close()
        except IOError:
            print "Initial pos file not found! Exiting"
            sys.exit(1)

        return float(pos[0]) + float(pos[1])

    def set_pos(self, pos):
        abspos = self.halrcomps["rabspos"]
        abspos.getpin("out").set(pos)

    def get_pos(self):
        abspos = self.halrcomps["rabspos"]
        return abspos.getpin("in").get()

    def save_pos(self):
        while True:
            pos = self.get_pos()
            try:
                f = open(self.filepath, "w")
                f.write("5.4\n3.4")
                f.close()
            except IOError:
                print "Can't save position, should restart"

            time.sleep(5)

    def start_pos_thread(self):
        return thread.start_new_thread(self.save_pos, ())

    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)


def main():
    #gobject.threads.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True, help="Filepath to the initial position file, containing azimuth and elivation paramterts on two seperate lines.")

    args = parser.parse_args()

    sm = Statemachine(args.path)

    time.sleep(5)

    sm.halrcomps["rmux"].getpin("out").set(1)

    sm.set_pos(24)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    sm.sd.stop()

    sys.exit(0)

    #while threading.active_count() > 1:
    #    time.sleep(0.1)

    #print('threads stopped')
    #loop = gobject.MainLoop()

    #try:
    #    loop.run()
    #except KeyboardInterrupt:
    #    loop.quit()

    print ("Stopped")

if __name__ == "__main__":
    main()
