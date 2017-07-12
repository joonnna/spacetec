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
    def __init__(self, pos_file, testing=None):
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

        self.gps_az = 0
        self.gps_el = 0
        self.gps_height = 0

        self.init_az = az
        self.init_el = el

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps(testing)
        self._search_and_bind()

        #Need init value...
        self.state = State.calibrating
        self.set_state(State.calibrating)

        self.calibrate()
        self.set_state(State.idle)
       # self.start_gps_checker_thread()
        self.reset_abspos(az, el)
        self.start_pos_thread()

    def initrcomps(self, testing):
        gps_mux = halremote.RemoteComponent("gpsmux", debug=False)
        gps_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("az_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("az_step", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_step", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.no_create = True

        vel_mux = halremote.RemoteComponent("velmux", debug=False)
        vel_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("az_out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vel_mux.newpin("el_out", halremote.HAL_FLOAT, halremote.HAL_OUT)
        vel_mux.no_create = True

        abspos = halremote.RemoteComponent("abspos", debug=False)
        abspos.newpin("az_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("az_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.newpin("el_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("el_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.no_create = True

        sigcheck = halremote.RemoteComponent("rsigcheck", debug=False)
        sig = sigcheck.newpin("in", halremote.HAL_BIT, halremote.HAL_IN)
        sig.on_value_changed.append(self.change_state)
        sigcheck.no_create = True

        tracking = halremote.RemoteComponent("step", debug=False)
        track = tracking.newpin("track", halremote.HAL_BIT, halremote.HAL_IN)
        track.on_value_changed.append(self.check_gps)
        tracking.no_create = True

        pos_comps= halremote.RemoteComponent("pos-comps", debug=False)
        pos_comps.newpin("max_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("max_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.no_create = True

        vel_comps = halremote.RemoteComponent("vel-comps", debug=False)
        vel_comps.newpin("max_vel", halremote.HAL_FLOAT, halremote.HAL_IO)
        vel_comps.newpin("min_vel", halremote.HAL_FLOAT, halremote.HAL_IO)
        vel_comps.no_create = True

        motor_feedback = halremote.RemoteComponent("motor-feedback", debug=False)
        zero_az = motor_feedback.newpin("zero_az", halremote.HAL_BIT, halremote.HAL_IN)
        zero_el = motor_feedback.newpin("zero_el", halremote.HAL_BIT, halremote.HAL_IN)
        motor_feedback.newpin("az_pos", halremote.HAL_FLOAT, halremote.HAL_IN)
        motor_feedback.newpin("el_pos", halremote.HAL_FLOAT, halremote.HAL_IN)
        motor_feedback.newpin("reset_az", halremote.HAL_BIT, halremote.HAL_OUT)
        motor_feedback.newpin("reset_el", halremote.HAL_BIT, halremote.HAL_OUT)
        zero_az.on_value_changed.append(self.zero_az_callback)
        zero_el.on_value_changed.append(self.zero_el_callback)
        motor_feedback.no_create = True

        self.halrcomps[motor_feedback.name] = motor_feedback
        self.halrcomps[pos_comps.name] = pos_comps
        self.halrcomps[vel_comps.name] = vel_comps
        self.halrcomps[tracking.name] = tracking
        self.halrcomps[gps_mux.name] = gps_mux
        self.halrcomps[vel_mux.name] = vel_mux
        self.halrcomps[abspos.name] = abspos
        self.halrcomps[sigcheck.name] = sigcheck

        if testing:
            self.logger.info("Creating test components")
            rssi = halremote.RemoteComponent("rrssi", debug=False)
            rssi.newpin("out", halremote.HAL_FLOAT, halremote.HAL_OUT)
            rssi.no_create = True

            bldc0 = halremote.RemoteComponent("rbldc0", debug=False)
            bldc0.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
            bldc0.no_create = True

            bldc1 = halremote.RemoteComponent("rbldc1", debug=False)
            bldc1.newpin("in", halremote.HAL_FLOAT, halremote.HAL_IN)
            bldc1.no_create = True

            check_step = halremote.RemoteComponent("check-step", debug=False)
            check_step.newpin("az_out", halremote.HAL_FLOAT, halremote.HAL_IN)
            check_step.newpin("el_out", halremote.HAL_FLOAT, halremote.HAL_IN)
            check_step.newpin("az_in", halremote.HAL_FLOAT, halremote.HAL_IN)
            check_step.newpin("el_in", halremote.HAL_FLOAT, halremote.HAL_IN)
            check_step.no_create = True

            test_feedback = halremote.RemoteComponent("test-feedback", debug=False)
            test_feedback.newpin("stop_az", halremote.HAL_BIT, halremote.HAL_OUT)
            test_feedback.newpin("stop_el", halremote.HAL_BIT, halremote.HAL_OUT)
            test_feedback.newpin("az", halremote.HAL_FLOAT, halremote.HAL_OUT)
            test_feedback.newpin("el", halremote.HAL_FLOAT, halremote.HAL_OUT)

            self.halrcomps[test_feedback.name] = test_feedback
            self.halrcomps[check_step.name] = check_step
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
        self.zero_az_event.wait()
        self.zero_az_event.clear()
        return self.halrcomps["motor-feedback"].getpin("az_pos").get()

    def wait_for_zero_el(self):
        self.zero_el_event.wait()
        self.zero_el_event.clear()
        return self.halrcomps["motor-feedback"].getpin("el_pos").get()

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

    def send_az_calibrate_pos(self, az):
        self.az_cal = az
        self.halrcomps["velmux"].getpin("az_out").set(az)

    def send_el_calibrate_pos(self, el):
        self.el_cal = el
        self.halrcomps["velmux"].getpin("el_out").set(el)

    def send_gps_pos(self, pos):
        if self.get_state() == State.idle:
            self.logger.info("Got first UDP packet!")
            self.set_state(State.stop_idle)

        az = pos[0]
        el = pos[1]
        height = pos[2]

        self.set_gps_angle(az, el, height)

        mux = self.halrcomps["gpsmux"]
        mux.getpin("az_gps").set(az)
        mux.getpin("el_gps").set(el)

 #       self.logger.debug("\naz: %f\nel:%f\nheight:%f\n" % (az, el, height))

        if height < self.overide_gps_height:
            self.set_state(State.gps_overide)
        elif self.get_state() == State.gps_overide:
            self.set_state(State.stop_overide)


    def start_pos_thread(self):
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

    def start_comm_thread(self):
        self.comm_thread = new_thread(self.comm_func, self.comm_cleanup, self.comm_thread_timeout, self.send_gps_pos)
        self.comm_thread.start()

    def start_gps_checker_thread(self):
        self.gps_checker_thread = new_thread(self.check_gps, self.cleanup_gps_checker, self.gps_checker_thread_timeout)
        self.gps_checker_thread.start()


    def check_gps(self, val):
        if val:
            self.set_state(State.tracking)
        else:
            self.set_state(State.gps)

        """
        gps_az, gps_el, gps_height = self.get_gps_angle()

        if gps_height < self.height_gps_limit:
            self.set_state(State.gps_overide)
            return
        track_az, track_el = self.get_tracking_pos()

        az_diff = abs(gps_az - track_az)
        el_diff = abs(gps_el - track_el)

        self.logger.info("\nGPS: %f, %f\n Tracking: %f, %f\n az_diff: %f, el_diff: %f" % (gps_az, gps_el, track_az, track_el, az_diff, el_diff))

        if az_diff > self.az_gps_limit or el_diff > self.el_gps_limit:
            self.set_state(State.gps)
        """
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

        self.overide_gps_el = config["overide_gps_el"]
        self.overide_gps_height = config["overide_gps_height"]

        self.sig_limit = config["sig_lim"]

        self.max_velocity = config["max_velocity"]
        self.min_velocity = config["min_velocity"]

        self.calibrate_max_velocity = config["calibrate_max_velocity"]
        self.calibrate_min_velocity = config["calibrate_min_velocity"]

        """
        self.velocity_igain = config["velocity_igain"]
        self.velocity_pgain = config["velocity_pgain"]

        self.pos_igain = config["pos_igain"]
        self.pos_pgain = config["pos_pgain"]

        self.calibrate_velocity_igain = config["calibrate_velocity_igain"]
        self.calibrate_velocity_pgain = config["calibrate_velocity_pgain"]

        self.calibrate_pos_igain = config["calibrate_pos_igain"]
        self.calibrate_pos_pgain = config["calibrate_pos_pgain"]
        """


    def set_az_pos_limits(self, max, min):
        lim = self.halrcomps["pos-comps"]
        lim.getpin("max_az").set(max)
        lim.getpin("min_az").set(min)

    def set_el_pos_limits(self, max, min):
        lim = self.halrcomps["pos-comps"]
        lim.getpin("max_el").set(max)
        lim.getpin("min_el").set(min)


    def set_velocity_limits(self, max, min):
        vel_comps = self.halrcomps["vel-comps"]
        vel_comps.getpin("max_vel").set(max)
        vel_comps.getpin("min_vel").set(min)
    """
    def set_velocity_pid_gains(self, igain, pgain):
        vel_comps = self.halrcomps["vel-comps"]
        vel_comps.getpin("Igain").set(igain)
        vel_comps.getpin("Pgain").set(pgain)

    def set_pos_pid_gains(self, igain, pgain):
        pos_comps = self.halrcomps["pos-comps"]
        pos_comps.getpin("Igain").set(igain)
        pos_comps.getpin("Pgain").set(pgain)
    """
    def set_calibrate_velocity(self):
        #self.set_az_pos_limits(360.0, -360.0)
        #self.set_el_pos_limits(360.0, -360.0)

        self.set_velocity_limits(self.calibrate_max_velocity, self.calibrate_min_velocity)

    def set_normal_velocity(self):
        self.set_velocity_limits(self.max_velocity, self.min_velocity)

    def reset_az_encoder(self):
        component = self.halrcomps["motor-feedback"]
        az_reset = component.getpin("az_reset")
        az_reset.set(True)
        self.halrcomps["abspos"].getpin("az_reset").set(0.0)
        az.reset.set(False)

    def calibrate_az(self):
        self.set_calibrate_velocity()

        if self.init_az > 0:
            direction = 1.0
        else:
            direction = -1.0

        self.send_az_calibrate_pos(direction*self.az_range)

        az_pos = self.wait_for_zero_az()

        self.set_normal_velocity()

        self.reset_az_encoder()

        #10 degrees, want to go the last ones slow
        opposite_side = -direction * (self.az_range - 10)

        self.send_az_calibrate_pos(opposite_side)

        temp = self.wait_for_zero_az()

        self.set_calibrate_velocity()

        self.send_az_calibrate_pos(self.az_range)

        total_az_range = self.wait_for_zero_az()

        #TODO move to middle and reset abspos?
        mid = total_az_range/2.0
        self.send_az_cal_pos(direction*mid)

        temp2 = self.wait_for_zero_az()

        self.reset_az_encoder()
        self.set_az_pos_lims(mid, -mid)

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

    def get_tracking_pin_state(self):
        tracking = self.halrcomps["step"].getpin("track").get()
        if tracking:
            return State.tracking
        else:
            return State.gps

    def set_state(self, new_state):
        self.state_lock.acquire()

        if new_state == State.stop_idle and self.state == State.idle:
            self.state = self.get_tracking_pin_state()
        elif new_state == State.stop_overide and self.state == State.gps_overide:
            self.state = self.get_tracking_pin_state()
        elif self.state != State.gps_overide and self.state != State.idle:
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

        self.logger.info("Entered %s state" % (self.state.name))
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

        self.pos_thread.stop()
        self.comm_thread.stop()
        #self.gps_thread.stop()

        self.comm_thread.join()
        self.logger.info("Shutdown comm thread")
        self.pos_thread.join()
        self.logger.info("Shutdown pos thread")
        #self.gps_thread.join()

        shutdown_hal()
        self.sd.stop()
        for name, rcomp in self.halrcomps.iteritems():
            rcomp.remove_pins()
            rcomp.set_disconnected()

        self.halrcomps = {}


        if self.cleanup_event:
            self.logger.info("CLEANUP BITCHES")
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

                #if not self.gps_checker_thread.is_alive():
                #    self.logger.info("Gps checker thread died, restarting")
                #    self.start_gps_checker_thread()

                #If executed as secondary thread instead of main thread
                if exit_event:
                    if exit_event.is_set():
                        break
        except KeyboardInterrupt:
            pass

        self.cleanup()
