import time
import logging
import sys
from general_thread import *
from hal_control import *
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from parse_config import read_config
from state_enum import *


class Statemachine():
    def __init__(self, pos_file):
        self.gps_pos_lock = threading.Lock()
        self.state_lock = threading.Lock()

        config = read_config()
        self.set_config()

        logging.basicConfig(filename="/var/log/statemachine.log", level=logging.DEBUG)
        self.logger = logging.getLogger("state")

        self.filepath = pos_file
        pos = self.read_init_pos()

        az = pos[0]
        el = pos[1]

        #self.max_az =
        #self.max_el =

        self.init_az = az
        self.init_el = el

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps()
        self._search_and_bind()

        self.calibrate()

        self.start_gps_checker_thread()
        self.reset_abspos(az, el)
        self.start_pos_thread()

    def initrcomps(self):
        mux0 = halremote.RemoteComponent("rmux0", debug=False)
        mux0.newpin("out0", halremote.HAL_S32, halremote.HAL_OUT)
        mux0.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        mux0.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)

        mux1 = halremote.RemoteComponent("rmux1", debug=False)
        mux1.newpin("out0", halremote.HAL_S32, halremote.HAL_OUT)
        mux1.newpin("out1", halremote.HAL_FLOAT, halremote.HAL_OUT)
        mux1.newpin("out2", halremote.HAL_FLOAT, halremote.HAL_OUT)

        abspos0 = halremote.RemoteComponent("rabspos0", debug=False)
        abspos0.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        abspos1 = halremote.RemoteComponent("rabspos1", debug=False)
        abspos1.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        sigcheck = halremote.RemoteComponent("rsigcheck", debug=False)
        sig = sigcheck.newpin("in", halremote.HAL_BIT, halremote.HAL_IN)
        sig.on_value_changed.append(self.change_state)

        tracking = helremote.RemoteComponent("rstep", debug=False)
        tracking.newpin("az", halremote.HAL_FLOAT, halremote.HAL_IN)
        tracking.newpin("el", halremote.HAL_FLOAT, halremote.HAL_IN)

        rssi = halremote.RemoteComponent("rrssi", debug=False)
        rssi.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        bldc0 = halremote.RemoteComponent("rbldc0", debug=False)
        bldc0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        bldc1 = halremote.RemoteComponent("rbldc1", debug=False)
        bldc1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        vlim0 = halremote.RemoteComponenet("rvlim0", debug=False)
        vlim0.newpin("max", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vlim0.newpin("min", halremote.HAL_FLOAT, halremote.HAL_OUT)

        vlim1 = halremote.RemoteComponenet("rvlim1", debug=False)
        vlim1.newpin("max", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vlim1.newpin("min", halremote.HAL_FLOAT, halremote.HAL_OUT)

        vpid0 = halremote.RemoteComponenet("rvpid0", debug=False)
        vpid0.newpin("pgain", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vpid0.newpin("igain", halremote.HAL_FLOAT, halremote.HAL_OUT)

        vpid1 = halremote.RemoteComponenet("rvpid1", debug=False)
        vpid1.newpin("pgain", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vpid1.newpin("igain", halremote.HAL_FLOAT, halremote.HAL_OUT)

        plim0 = halremote.RemoteComponenet("rplim0", debug=False)
        plim0.newpin("max", halremote.HAL_FLOAT, halremote.HAL_OUT)
        plim0.newpin("min", halremote.HAL_FLOAT, halremote.HAL_OUT)

        plim1 = halremote.RemoteComponenet("rplim1", debug=False)
        plim1.newpin("max", halremote.HAL_FLOAT, halremote.HAL_OUT)
        plim1.newpin("min", halremote.HAL_FLOAT, halremote.HAL_OUT)

        ppid0 = halremote.RemoteComponenet("rppid0", debug=False)
        ppid0.newpin("pgain", halremote.HAL_FLOAT, halremote.HAL_OUT)
        ppid0.newpin("igain", halremote.HAL_FLOAT, halremote.HAL_OUT)

        ppid1 = halremote.RemoteComponenet("rppid1", debug=False)
        ppid1.newpin("pgain", halremote.HAL_FLOAT, halremote.HAL_OUT)
        ppid1.newpin("igain", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.halrcomps[rvpid0.name] = rvpid0
        self.halrcomps[rvpid1.name] = rvpid1

        self.halrcomps[rppid0.name] = rppid0
        self.halrcomps[rppid1.name] = rppid1

        self.halrcomps[rvlim0.name] = rvlim0
        self.halrcomps[rvlim1.name] = rvlim1

        self.halrcomps[rplim0.name] = rplim0
        self.halrcomps[rplim1.name] = rplim1


        self.halrcomps[tracking.name] = tracking
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
            self.set_state(State.gps)
        else:
            self.set_state(State.tracking)


    def _search_and_bind(self):
        for name, rcomp in self.halrcomps.iteritems():
            self.sd.register(rcomp)

        self.sd.start()

        for name, rcomp in self.halrcomps.iteritems():
            rcomp.bind_component()

        self.logger.info("Bound all remote HAL components")

    #TODO Need to do something incase file doesn't exist, enter gps state maybe...
    def read_init_pos(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read().split("\n")
            f.close()
        except IOError:
            self.logger.Error("Initial position file not found!")
            sys.exit(1)

        return float(pos[0]), float(pos[1])

    def reset_abspos(self, az, el):
        abspos0 = self.halrcomps["rabspos0"]
        abspos0.getpin("out").set(az)

        abspos1 = self.halrcomps["rabspos1"]
        abspos1.getpin("out").set(el)


    def get_abspos(self):
        abspos0 = self.halrcomps["rabspos0"]
        az = abspos0.getpin("in").get()

        abspos1 = self.halrcomps["rabspos1"]
        el = abspos1.getpin("in").get()

        return az, el

    def store_old_abspos(self):
        p0, p1 = self.get_pos()
        try:
            f = open(self.filepath, "w")
            f.write(("%f\n%f" % (p0, p1)))
            f.close()
        except IOError:
            self.logger.Error("Can't save old position (%f, %f), should restart" % p0, p1)

    def cleanup_abspos_thread(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read()
            f.close()
        except IOError:
            self.logger.Error("Couldn't open position file during cleanup...")
            return

        #If file was altered somehow after initiating cleanup (not likely)
        data = pos.split("\n")
        if len(data) < 2:
            fw = open(self.filepath, "w")
            old_pos = self.get_abspos()
            fw.write(("%f\n%f" % (old_pos[0], old_pos[1])))
            fw.close()

    def send_az_cal_pos(self, az):
        self.az_cal = az
        self.halrcomps["rmux0"].getpin("out1").set(az)

    def send_el_cal_pos(self, el):
        self.el_cal = el
        self.halrcomps["rmux1"].getpin("out1").set(el)

    def send_gps_pos(self, pos):
        az = pos[0]
        el = pos[1]
        height = pos[2]

        self.set_gps_pos(az, el, height)

        #TODO add current pos? do that in gps thread?
        """
        self.gps_pos_lock.acquire()
        self.az = self.init_az + pos[0]
        self.el = self.init_el + pos[1]
        self.gps_pos_lock.release()
        """

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

    def start_gps_checker_thread(self):
        self.gps_checker_thread = new_thread(self.check_gps, self.cleanup_gps_checker, self.gps_checker_timeout)
        self.gps_checker_thread.start()


    def check_gps(self):
        gps_az, gps_el, gps_height = self.get_gps_pos()

        if gps_height > self.height_gps_lim:
            self.set_state(State.gps_overide)
            return


        track_az, track_el = self.get_tracking_pos()

        az_diff = abs(gps_az - track_az)
        el_diff = abs(gps_el - track_el)

        self.logger.Info("GPS: %f, %f\n Tracking: %f, %f\n az_diff: %f, el_diff: %f" % (az, el, track_az, track_el, az_diff, el_diff))

        if az_diff > self.az_gps_limit or el_diff > self.el_gps_limit:
            self.set_state(State.gps)



    def cleanup_gps_checker(self):
        pass


    def set_config(self, config):
        self.pos_thread_timeout = config["pos_timeout"]
        self.check_threads_timeout = config["check_threads_timeout"]
        self.gps_check_thread_timeout = config["gps_check_timeout"]
        self.gps_thread_timeout = config["gps_timeout"]

        self.az_gps_limit = config["az_lim"]
        self.el_gps_limit = config["el_lim"]
        self.height_gps_limit = config["el_lim"]

        self.az_range = config["az_range"]
        self.el_range = config["el_range"]


    def set_az_vel_gains(self, pgain, igain):
        pid = self.halrcomps["rvpid0"]
        pid.getpin("pgain").set(pgain)
        pid.getpin("igain").set(igain)

    def set_az_pos_gains(self, pgain, igain):
        pid = self.halrcomps["rppid0"]
        pid.getpin("pgain").set(pgain)
        pid.getpin("igain").set(igain)

    def set_az_pos_lims(self, max, min):
        lim = self.halrcomps["rplim0"]
        lim.getpin("max").set(max)
        lim.getpin("min").set(min)

    def set_az_vel_lims(self, max, min):
        lim = self.halrcomps["rvlim0"]
        lim.getpin("max").set(max)
        lim.getpin("min").set(min)

    def set_el_pos_lims(self, max, min):
        lim = self.halrcomps["rplim1"]
        lim.getpin("max").set(max)
        lim.getpin("min").set(min)

    def set_el_vel_lims(self, max, min):
        lim = self.halrcomps["rvlim1"]
        lim.getpin("max").set(max)
        lim.getpin("min").set(min)

    def set_el_vel_gains(self, pgain, igain):
        pid = self.halrcomps["rvpid1"]
        pid.getpin("pgain").set(pgain)
        pid.getpin("igain").set(igain)

    def set_el_pos_gains(self, pgain, igain):
        pid = self.halrcomps["rppid1"]
        pid.getpin("pgain").set(pgain)
        pid.getpin("igain").set(igain)


    #TODO init values for az/el calibration, maybe pos file ?
    #TODO encoder return 0 logic...
    def find_max_az(self):

        while not_recv_0:
            self.send_az_cal_pos(self.az_cal + 1)

        #TODO maybe set first side to 0 and reset abspos here?

        first_side = self.az_cal

        #10 degrees, want to go the last ones slow
        opposite_side = (self.az_lim - 10)

        self.send_az_cal_pos((self.az_cal - opposite_side))

        while not_recv_0:
            self.send_az_cal_pos(self.az_cal - 1)
            total_az_range += 1.0

        #TODO this is probably wrong, abs perhaps?
        total_az_range = self.az_cal - first_side


        #TODO move to middle and reset abspos?
        mid = total_az_range/2.0
        self.send_az_cal_pos(mid)

        #TODO Wait for antenna to hit middle?


    def find_max_el(self):
        pass

    def calibrate(self):
        self.set_state(State.calibrating)

        self.logger.info("Calibrating")

        #TODO set calibrating config speed, lims, gains etc

        az = self.find_max_az()

        el = self.find_max_el()

        #TODO either zero point is "0" or az_max/2
        self.set_az_pos_lims(az, -az)
        self.set_el_pos_lims(el, -el)

        #TODO Want zero pos in the middle right?
        self.reset_abspos(0.0, 0.0)



        self.logger.info("Finished calibrating")


    def get_tracking_pos(self).
        track = self.halrcomps["rstep"]
        az = track.getpin("az").get()
        el = track.getpin("el").get()
        return az, el

    def set_gps_pos(self, az, el, height):
        self.gps_pos_lock.acquire()
        self.gps_az = az
        self.gps_el = el
        self.gps_height = height
        self.gps_pos_lock.release()


    def get_gps_pos(self):
        self.gps_pos_lock.acquire()
        az = self.gps_az
        el = self.gps_el
        height = self.gps_height
        self.gps_pos_lock.release()

        return az, el, height

    def set_state(self, new_state):
        self.state_lock.acquire()

        if new_state == State.stop_overide:
            self.state = State.tracking
        elif self.state != State.gps_overide:
            self.state = new_state

        mux_val = self.state.value
        if self.state == State.gps_overide:
            mux_val = State.gps.value

        self.halrcomps["rmux0"].getpin("out0").set(mux_val)
        self.halrcomps["rmux1"].getpin("out0").set(mux_val)

        self.logger.info("Entered %s state" % (new_state.name))
        self.state_lock.release()

    def get_state(self):
        self.state_lock.acquire()
        ret = self.state
        self.state_lock.release()

        return ret

    def _connected(self, connected):
        print('Remote component connected: %s' % str(connected))

    def _service_discovered(self, data):
        print("discovered %s %s %s" % (data.name, data.dsn, data.uuid))
        self.sd(data.uuid)

    def cleanup(self):
        self.logger.info("Cleaning up statemachine")
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

        self.logger.info("Stopped statemachine")

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
                    self.logger.info("Pos thread died, restarting")
                    self.start_pos_thread()

                if not self.comm_thread.is_alive():
                    self.logger.info("comm thread died, restarting")
                    self.start_comm_thread()

                if not self.gps_thread.is_alive():
                    self.logger.info("Gps thread died, restarting")
                    self.start_gps_thread()

                if not self.gps_checker_thread.is_alive():
                    self.logger.info("Gps checker thread died, restarting")
                    self.start_gps_checker_thread()

                #If executed as secondary thread instead of main thread
                if exit_event != None:
                    if exit_event.isSet():
                        break
        except KeyboardInterrupt:
            pass

        self.cleanup()
