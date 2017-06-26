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
        self.set_pos(pos[0], pos[1])
        self.start_pos_thread()

    def initrcomps(self):
        mux0 = halremote.RemoteComponent("rmux0", debug=True)
        select0 = mux0.newpin("out", halremote.HAL_S32, halremote.HAL_OUT)
        mux0.on_connected_changed.append(self._connected)

        mux1 = halremote.RemoteComponent("rmux1", debug=True)
        select1 = mux1.newpin("out", halremote.HAL_S32, halremote.HAL_OUT)
        mux1.on_connected_changed.append(self._connected)

        abspos0 = halremote.RemoteComponent("rabspos0", debug=True)
        abspos0.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos0.on_connected_changed.append(self._connected)

        abspos1 = halremote.RemoteComponent("rabspos1", debug=True)
        abspos1.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos1.on_connected_changed.append(self._connected)

        self.halrcomps[mux0.name] = mux0
        self.halrcomps[mux1.name] = mux1
        self.halrcomps[abspos0.name] = abspos0
        self.halrcomps[abspos1.name] = abspos1

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

        return pos[0], pos[1]

    def set_pos(self, pos0, pos1):
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

    def save_pos(self):
        while True:
            p0, p1 = self.get_pos()
            try:
                f = open(self.filepath, "w")
                f.write("3.4\n5.4")
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True, help="Filepath to the initial position file, containing azimuth and elevation paramterts on two seperate lines.")

    args = parser.parse_args()

    sm = Statemachine(args.path)

    time.sleep(5)

    sm.halrcomps["rmux0"].getpin("out").set(1)

    sm.set_pos(4, 10)

    time.sleep(10)

    sm.set_pos(24, 48)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    sm.sd.stop()

    sys.exit(0)

    print ("Stopped")

if __name__ == "__main__":
    main()
