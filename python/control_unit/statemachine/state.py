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
        self.state_lock = threading.Lock()
        self.gps_angle_lock = threading.Lock()

        self.zero_az_event = threading.Event()
        self.zero_el_event = threading.Event()

        self.filepath = pos_file

        config = read_config()
        self.set_config(config)

        logging.basicConfig(filename="/var/log/statemachine.log", level=logging.DEBUG)
        self.logger = logging.getLogger("state")

        pos = self.read_init_pos()

        az = pos[0]
        el = pos[1]

        #self.max_az =
        #self.max_el =

        self.gps_az = 0
        self.gps_el = 0
        self.gps_height = 0

        self.init_az = az
        self.init_el = el

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps()
        self._search_and_bind()

        #Need init value...
        self.state = State.calibrating
        self.set_state(State.calibrating)

 #       self.calibrate()

        self.start_gps_checker_thread()
        self.reset_abspos(az, el)
        self.start_pos_thread()

    def initrcomps(self):
        gps_mux = halremote.RemoteComponent("gpsmux", debug=False)
        gps_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("az_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("az_step", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_step", halremote.HAL_FLOAT, halremote.HAL_OUT)

        vel_mux = halremote.RemoteComponent("velmux", debug=False)
        vel_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("az_out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vel_mux.newpin("el_out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        abspos = halremote.RemoteComponent("abspos", debug=False)
        abspos.newpin("az_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("az_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.newpin("el_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("el_in", halremote.HAL_FLOAT, halremote.HAL_IN)

        sigcheck = halremote.RemoteComponent("rsigcheck", debug=False)
        sig = sigcheck.newpin("in", halremote.HAL_BIT, halremote.HAL_IN)
        sig.on_value_changed.append(self.change_state)

        tracking = halremote.RemoteComponent("step", debug=False)
        tracking.newpin("az", halremote.HAL_FLOAT, halremote.HAL_IN)
        tracking.newpin("el", halremote.HAL_FLOAT, halremote.HAL_IN)

        rssi = halremote.RemoteComponent("rrssi", debug=False)
        rssi.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)

        bldc0 = halremote.RemoteComponent("rbldc0", debug=False)
        bldc0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        bldc1 = halremote.RemoteComponent("rbldc1", debug=False)
        bldc1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)

        plim = halremote.RemoteComponent("poslim", debug=False)
        plim.newpin("max_az", halremote.HAL_FLOAT, halremote.HAL_OUT)
        plim.newpin("min_az", halremote.HAL_FLOAT, halremote.HAL_OUT)
        plim.newpin("max_el", halremote.HAL_FLOAT, halremote.HAL_OUT)
        plim.newpin("min_el", halremote.HAL_FLOAT, halremote.HAL_OUT)

        motor_feedback = halremote.RemoteComponent("motor-feedback", debug=False)
        zero_az = motor_feedback.newpin("zero_az", halremote.HAL_BIT, halremote.HAL_IN)
        zero_el = motor_feedback.newpin("zero_el", halremote.HAL_BIT, halremote.HAL_IN)
        zero_az.on_value_changed.append(self.zero_az_callback)
        zero_el.on_value_changed.append(self.zero_el_callback)

        self.halrcomps[motor_feedback.name] = motor_feedback
        self.halrcomps[plim.name] = plim
        self.halrcomps[tracking.name] = tracking
        self.halrcomps[gps_mux.name] = gps_mux
        self.halrcomps[vel_mux.name] = vel_mux
        self.halrcomps[abspos.name] = abspos
        self.halrcomps[sigcheck.name] = sigcheck
        self.halrcomps[rssi.name] = rssi
        self.halrcomps[bldc0.name] = bldc0
        self.halrcomps[bldc1.name] = bldc1


    def zero_az_callback(self, val):
        if val:
            self.zero_az_event.set()
        else:
            self.zero_az_event.clear()

    def zero_el_callback(self, val):
        if val:
            self.zero_el_event.set()
        else:
            self.zero_el_event.clear()

    def wait_for_zero_az(self):
        return self.zero_az_event.is_set()

    def wait_for_zero_el(self):
        return self.zero_el_event.is_set()

    def change_state(self, sig):
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
        except IOError:
            self.logger.Error("Could not read position file!")
            return None, None

        if len(pos) < 2:
            return 0.0, 0.0

        return float(pos[0]), float(pos[1])

    def reset_abspos(self, az, el):
        abspos = self.halrcomps["abspos"]
        abspos.getpin("az_reset").set(az)
        abspos.getpin("el_reset").set(el)


    def get_abspos(self):
        abspos = self.halrcomps["abspos"]
        az = abspos.getpin("az_in").get()
        el = abspos.getpin("el_in").get()

        return az, el

    def store_old_abspos(self):
        p0, p1 = self.get_abspos()
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
        self.halrcomps["velmux"].getpin("az_out").set(az)

    def send_el_cal_pos(self, el):
        self.el_cal = el
        self.halrcomps["velmux"].getpin("el_out").set(el)

    def send_gps_pos(self, pos):
        az = pos[0]
        el = pos[1]
        height = pos[2]

        self.set_gps_angle(az, el, height)

        mux = self.halrcomps["gpsmux"]
        mux.getpin("az_gps").set(az)
        mux.getpin("el_gps").set(el)

    def start_pos_thread(self):
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

    def start_comm_thread(self):
        self.comm_thread = new_thread(self.comm_func, self.comm_cleanup, self.comm_thread_timeout, self.send_gps_pos)
        self.comm_thread.start()

    def start_gps_checker_thread(self):
        self.gps_checker_thread = new_thread(self.check_gps, self.cleanup_gps_checker, self.gps_checker_thread_timeout)
        self.gps_checker_thread.start()


    def check_gps(self):
        gps_az, gps_el, gps_height = self.get_gps_angle()

        """
        if gps_height < self.height_gps_limit:
            self.set_state(State.gps_overide)
            return
        """
        track_az, track_el = self.get_tracking_pos()

        az_diff = abs(gps_az - track_az)
        el_diff = abs(gps_el - track_el)

        self.logger.info("\nGPS: %f, %f\n Tracking: %f, %f\n az_diff: %f, el_diff: %f" % (gps_az, gps_el, track_az, track_el, az_diff, el_diff))

        if az_diff > self.az_gps_limit or el_diff > self.el_gps_limit:
            self.set_state(State.gps)

    def cleanup_gps_checker(self):
        pass

    def set_config(self, config):
        self.pos_thread_timeout = config["pos_timeout"]
        self.check_threads_timeout = config["check_threads_timeout"]
        self.gps_checker_thread_timeout = config["gps_check_timeout"]
        self.gps_thread_timeout = config["gps_timeout"]

        self.az_gps_limit = config["az_lim"]
        self.el_gps_limit = config["el_lim"]
        self.height_gps_limit = config["el_lim"]

        self.az_range = config["az_range"]
        self.el_range = config["el_range"]

        self.az_offset = config["az_offset"]
        self.el_offset = config["el_offset"]

    def set_az_pos_lims(self, max, min):
        lim = self.halrcomps["poslim"]
        lim.getpin("max_az").set(max)
        lim.getpin("min_az").set(min)

    def set_el_pos_lims(self, max, min):
        lim = self.halrcomps["poslim"]
        lim.getpin("max_el").set(max)
        lim.getpin("min_el").set(min)


    #TODO init values for az/el calibration, maybe pos file ?
    def calibrate_az(self):
        self.az_cal = self.init_az
        self.el_cal = self.init_el

        if self.init_az >= 0:
            az_increment = 1
        else:
            az_increment = -1

        while self.wait_for_zero_az():
            self.send_az_cal_pos(self.az_cal + az_increment)

        #TODO maybe set first side to 0 and reset abspos here?
        first_side = self.az_cal

        #10 degrees, want to go the last ones slow
        opposite_side = (self.az_lim - 10)

        if first_side <= 0:
            self.send_az_cal_pos((self.az_cal + opposite_side))
        else:
            self.send_az_cal_pos((self.az_cal - opposite_side))

        reverse_increment = -increment

        while self.wait_for_zero_az():
            self.send_az_cal_pos(self.az_cal + reverse_increment)
            total_az_range += 1.0

        #TODO move to middle and reset abspos?
        mid = total_az_range/2.0
        self.send_az_cal_pos(mid)

        self.set_az_pos_lims(mid, -mid)

        #TODO Wait for antenna to hit middle?


    def calibrate_el(self):
        pass

    def calibrate(self):
        self.set_state(State.calibrating)

        self.logger.info("Calibrating")

        #TODO set calibrating config speed, lims, gains etc

        self.calibrate_az()

        self.calibrate_el()

        #TODO Want zero pos in the middle right?
        self.reset_abspos(0.0, 0.0)

        self.logger.info("Finished calibrating")


    def get_tracking_pos(self):
        track = self.halrcomps["step"]
        az = track.getpin("az").get()
        el = track.getpin("el").get()
        return az, el

    def set_gps_angle(self, az, el, height):
        self.gps_angle_lock.acquire()
        self.gps_az = az
        self.gps_el = el
        self.gps_height = height
        self.gps_angle_lock.release()

    def get_gps_angle(self):
        self.gps_angle_lock.acquire()
        az = self.gps_az
        el = self.gps_el
        height = self.gps_height
        self.gps_angle_lock.release()

        return az, el, height

    def set_state(self, new_state):
        self.state_lock.acquire()

        if new_state == State.stop_overide:
            self.state = State.tracking
        elif self.state != State.gps_overide:
            self.state = new_state

        if self.state == State.calibrating:
            vel_mux = 1
            gps_mux = 0 #Not relevant
        elif self.state == State.idle:
            vel_mux = 0
            gps_mux = 0 #Not relveant
        elif self.state == State.tracking:
            vel_mux = 2
            gps_mux = 0
        elif self.state == State.gps or self.state == State.gps_overide:
            vel_mux = 2
            gps_mux = 1
        else:
            self.logger.critical("Unknown state, panic!")
            self.state_lock.release()
            return

        self.halrcomps["velmux"].getpin("az_sel").set(vel_mux)
        self.halrcomps["velmux"].getpin("el_sel").set(vel_mux)

        self.halrcomps["gpsmux"].getpin("az_sel").set(gps_mux)
        self.halrcomps["gpsmux"].getpin("el_sel").set(gps_mux)

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
        #self.gps_thread.stop()

        self.comm_thread.join()
        self.pos_thread.join()
        #self.gps_thread.join()

        if self.cleanup_event != None:
            self.cleanup_event.set()

        self.logger.info("Stopped statemachine")

    def run(self, comm_thread, exit_event=None, cleanup_event=None):
        self.cleanup_event = cleanup_event

        self.comm_thread = comm_thread
        self.comm_func = comm_thread.func
        self.comm_cleanup = comm_thread.cleanup
        self.comm_thread_timeout = comm_thread.timeout
        self.comm_thread.start()

        try:
            while True:
                time.sleep(self.check_threads_timeout)

                if not self.pos_thread.is_alive():
                    self.logger.info("Pos thread died, restarting")
                    self.start_pos_thread()

                if not self.comm_thread.is_alive():
                    self.logger.info("comm thread died, restarting")
                    self.start_comm_thread()

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
