import time
import logging
import sys
from general_thread import *
from hal_control import *
from pymachinetalk.dns_sd import ServiceDiscovery
import pymachinetalk.halremote as halremote
from parse_config import read_config
from state_enum import *
import threading


class Statemachine():
    def __init__(self, pos_file, testing=None):
        self.state_lock = threading.Lock()
        self.gps_angle_lock = threading.Lock()

        self.az_init_event = threading.Event()
        self.el_init_event = threading.Event()

        if testing:
            self.az_init_event.set()
            self.el_init_event.set()

        self.zero_az_event = threading.Event()
        self.zero_el_event = threading.Event()

        self.moving_az_event = threading.Event()
        self.moving_el_event = threading.Event()
        self.moving_timeout = 5.0

        self.filepath = pos_file

        config = read_config()
        self.set_config(config)


        logging.basicConfig(filename="/var/log/statemachine.log", level=logging.DEBUG)
        self.logger = logging.getLogger("state")

        self.logger.info("Started statemachine init!")

        pos = self.read_init_pos()

        az = pos[0]
        el = pos[1]

        #az = 0
        #el = 0

        self.init_az = az
        self.init_el = el

        self.state = Override.calibrating

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps(testing)
        self._search_and_bind()

        p = self.halrcomps["pwm"].getpin("az")
        p.set(True)
        self.busy_wait(p)

        self.logger.info("Waiting for bldc init")
        self.wait_for_init()
        self.logger.info("Bldc init done!")

        self.calibrate()
        self.start_pos_thread()

    def initrcomps(self, testing):
        gps_mux = halremote.RemoteComponent("gpsmux", debug=False)
        gps_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        gps_mux.newpin("az_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("az_calibration", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_gps", halremote.HAL_FLOAT, halremote.HAL_OUT)
        gps_mux.newpin("el_calibration", halremote.HAL_FLOAT, halremote.HAL_OUT)
        #gps_mux.no_create = True

        vel_mux = halremote.RemoteComponent("velmux", debug=False)
        vel_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)
        #vel_mux.no_create = True

        abspos = halremote.RemoteComponent("abspos", debug=False)
        abspos.newpin("az_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("az_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.newpin("el_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("el_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        #abspos.no_create = True

        tracking = halremote.RemoteComponent("step", debug=False)
        track = tracking.newpin("track", halremote.HAL_BIT, halremote.HAL_IN)
        manual = tracking.newpin("manual", halremote.HAL_S32, halremote.HAL_IN)
        manual.on_value_changed.append(self.manual_state_callback)
        track.on_value_changed.append(self.check_gps)
        #tracking.no_create = True

        pos_comps = halremote.RemoteComponent("pos-comps", debug=False)
        pos_comps.newpin("max_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("max_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("az_diff", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("el_diff", halremote.HAL_FLOAT, halremote.HAL_IO)
        #pos_comps.no_create = True

        motor_feedback = halremote.RemoteComponent("motor-feedback", debug=False)
        zero_az = motor_feedback.newpin("zero_az", halremote.HAL_BIT, halremote.HAL_IN)
        zero_el = motor_feedback.newpin("zero_el", halremote.HAL_BIT, halremote.HAL_IN)
        motor_feedback.newpin("az_pos", halremote.HAL_FLOAT, halremote.HAL_IN)
        motor_feedback.newpin("el_pos", halremote.HAL_FLOAT, halremote.HAL_IN)
        motor_feedback.newpin("reset_az", halremote.HAL_BIT, halremote.HAL_OUT)
        motor_feedback.newpin("reset_el", halremote.HAL_BIT, halremote.HAL_OUT)
        motor_feedback.newpin("az_init_start", halremote.HAL_BIT, halremote.HAL_OUT)
        motor_feedback.newpin("el_init_start", halremote.HAL_BIT, halremote.HAL_OUT)
        az_init = motor_feedback.newpin("az_init_done", halremote.HAL_BIT, halremote.HAL_IN)
        el_init = motor_feedback.newpin("el_init_done", halremote.HAL_BIT, halremote.HAL_IN)
        az_init.on_value_changed.append(self.az_init_done)
        el_init.on_value_changed.append(self.el_init_done)

        zero_az.on_value_changed.append(self.zero_az_callback)
        zero_el.on_value_changed.append(self.zero_el_callback)
        #motor_feedback.no_create = True

        gps_angle_check = halremote.RemoteComponent("set-angle", debug=False)
        gps_angle_check.newpin("az_angle", halremote.HAL_FLOAT, halremote.HAL_IO)
        gps_angle_check.newpin("el_angle", halremote.HAL_FLOAT, halremote.HAL_IO)
        #gps_angle_check.no_create = True

        pwm = halremote.RemoteComponent("pwm", debug=False)
        pwm_az = pwm.newpin("az", halremote.HAL_BIT, halremote.HAL_OUT)
        pwm.newpin("el", halremote.HAL_BIT, halremote.HAL_OUT)
        #pwm_az.on_synced_changed.append(self.test_sync)
        #pwm.no_create = True

        self.halrcomps[pwm.name] = pwm
        self.halrcomps[gps_angle_check.name] = gps_angle_check
        self.halrcomps[motor_feedback.name] = motor_feedback
        self.halrcomps[pos_comps.name] = pos_comps
        self.halrcomps[tracking.name] = tracking
        self.halrcomps[gps_mux.name] = gps_mux
        self.halrcomps[vel_mux.name] = vel_mux
        self.halrcomps[abspos.name] = abspos

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
            test_feedback.no_create = True

            self.halrcomps[test_feedback.name] = test_feedback
            self.halrcomps[check_step.name] = check_step
            self.halrcomps[rssi.name] = rssi
            self.halrcomps[bldc0.name] = bldc0
            self.halrcomps[bldc1.name] = bldc1

    def busy_wait(self, pin):
        while not pin.synced:
            time.sleep(0.1)

    def test_sync(self, val):
        self.logger.debug("JADA!!!!! %s" % (val))

    def disable_az_pwm(self):
        pin = self.halrcomps["pwm"].getpin("az")
        pin.set(False)
        self.busy_wait(pin)
        #pin.set(False)

        #while not pin.wait_synced(1.0):
        #    self.logger.debug("Waiting for disabled sync")

    def enable_az_pwm(self):
        pin = self.halrcomps["pwm"].getpin("az")
        pin.set(True)
        self.busy_wait(pin)
        #pin.set(True)
        #self.logger.debug("Waiting for enabled sync")
        #pin.wait_synced()

    def disable_el_pwm(self):
        self.halrcomps["pwm"].getpin("el").set(False)

    def disable_el_pwm(self):
        self.halrcomps["pwm"].getpin("el").set(True)


    def rcomp_disconnected(self, val):
        if val:
            self.logger.debug("guuuuuci")
        else:
            self.logger.debug("KA I FAEEEEEEN")

    def az_calibration_velocity_callback(self, val):
        if val:
            self.moving_az_event.set()
        else:
            self.moving_az_event.clear()

    def el_calibration_velocity_callback(self, val):
        if val:
            self.moving_el_event.set()
        else:
            self.moving_el_event.clear()

    def az_init_done(self, val):
        self.logger.debug("INIT DONE BITHCESEESS")
        self.halrcomps["motor-feedback"].getpin("az_init_start").set(False)
        self.az_init_event.set()

    def el_init_done(self, val):
        self.halrcomps["motor-feedback"].getpin("el_init_start").set(False)
        self.el_init_event.set()

    def zero_az_callback(self, val):
        if val:
            self.zero_az_event.set()
            self.moving_az_event.clear()
        else:
            self.moving_az_event.set()
            self.zero_az_event.clear()

    def zero_el_callback(self, val):
        if val:
            self.zero_el_event.set()
        else:
            self.zero_el_event.clear()

    def wait_for_init(self):
        pin = self.halrcomps["motor-feedback"].getpin("az_init_start")
        pin.set(True)
        self.busy_wait(pin)
        self.logger.info("Waiting for physical bldc init to be start")
        self.az_init_event.wait()
        #self.az_init_event.wait()

    def wait_for_zero_az(self):
        self.logger.info("Waiting for zero az velocity")

        self.zero_az_event.wait()
        self.zero_az_event.clear()

        self.logger.info("Got zero az velocity!")

        return self.halrcomps["motor-feedback"].getpin("az_pos").get()

    def wait_for_zero_el(self):
        self.logger.info("Waiting for zero el velocity")

        self.zero_el_event.wait()
        self.zero_el_event.clear()

        self.logger.info("Waiting for zero el velocity")

        return self.halrcomps["motor-feedback"].getpin("el_pos").get()

    def manual_state_callback(self, val):
        if val == 0:
            self.set_state(Manual.stop)
        elif val == 1:
            self.set_state(Manual.position)
        elif val == 2:
            self.set_state(Manual.velocity)
        elif val == 3:
            self.set_state(Manual.tracking)
        else:
            self.logger.Error("Received undefined state value in manual state callback, exiting manual for safety reasons")
            self.set_state(Manual.stop)


    def _search_and_bind(self):
        for name, rcomp in self.halrcomps.iteritems():
            self.sd.register(rcomp)

        self.logger.info("Started service discovery")
        self.sd.start()

        timeout = 5.0

        for name, rcomp in self.halrcomps.iteritems():
            self.logger.debug("bound : %s" % (name))
            rcomp.on_connected_changed.append(self.rcomp_disconnected)
            rcomp.bind_component()

        for name, rcomp in self.halrcomps.iteritems():
            while not rcomp.wait_connected(timeout):
                self.logger.info("%s is not connected, waiting..." % (rcomp.name))
                #if not rcomp.connected:
                #    rcomp.bind_component()

            self.logger.info("%s is connected" % (name))


        self.logger.info("Bound all remote HAL components")

    #TODO Need to do something incase file doesn't exist, enter gps state maybe...
    def read_init_pos(self):
        try:
            f = open(self.filepath, "r")
            pos = f.read().split("\n")
        except IOError:
            self.logger.Error("Could not read position file!")
            return 0.0, 0.0

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
        self.halrcomps["gpsmux"].getpin("az_calibration").set(az)

    def send_el_calibrate_pos(self, el):
        self.el_cal = el
        self.halrcomps["gpsmux"].getpin("el_calibration").set(el)

    def send_gps_pos(self, pos):
        if self.get_state() == Override.idle:
            self.logger.info("Got first UDP packet!")
            self.set_state(Override.stop)

        az = pos[0]
        el = pos[1]
        height = pos[2]

        self.set_gps_angle(az, el, height)

        mux = self.halrcomps["gpsmux"]
        mux.getpin("az_gps").set(az)
        mux.getpin("el_gps").set(el)
 #       self.logger.debug("\naz: %f\nel:%f\nheight:%f\n" % (az, el, height))

        if height < self.overide_gps_height:
            self.set_state(Override.gps)
        elif self.get_state() == Override.gps:
            self.set_state(Override.stop)

    def start_pos_thread(self):
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

    def start_comm_thread(self):
        comm = self.comm_constructor()
        self.comm_thread = new_thread(comm.run, comm.shutdown, self.comm_thread_timeout, self.send_gps_pos)
        self.comm_thread.start()

    def check_gps(self, val):
        if val:
            self.set_state(State.tracking)
        else:
            self.set_state(State.gps)

    def set_config(self, config):
        self.az_calibrate_max_velocity = config["az_calibrate_max_velocity"]
        self.az_calibrate_min_velocity = config["az_calibrate_min_velocity"]

        self.el_calibrate_max_velocity = config["el_calibrate_max_velocity"]
        self.el_calibrate_min_velocity = config["el_calibrate_min_velocity"]

        self.az_max_velocity = config["az_max_velocity"]
        self.az_min_velocity = config["az_min_velocity"]

        self.el_max_velocity = config["el_max_velocity"]
        self.el_min_velocity = config["el_min_velocity"]

        self.az_gps_limit = config["az_lim"]
        self.el_gps_limit = config["el_lim"]

        self.az_re_enter_limit = config["az_re_enter_limit"]
        self.el_re_enter_limit = config["el_re_enter_limit"]

        self.az_range = config["az_range"]
        self.el_range = config["el_range"]

        self.az_offset = config["az_offset"]
        self.el_offset = config["el_offset"]

        self.overide_gps_el = config["overide_gps_el"]
        self.overide_gps_height = config["overide_gps_height"]

        self.sig_limit = config["sig_lim"]
        self.sig_re_enter_limit = config["sig_reenter_lim"]

        self.pos_thread_timeout = config["pos_timeout"]
        self.check_threads_timeout = config["check_threads_timeout"]

        self.az_calibration_diff = config["calibration_diff"]
        self.az_normal_diff = config["normal_diff"]

    def set_az_pos_limits(self, max, min):
        max_pin = self.halrcomps["pos-comps"].getpin("max_az")
        min_pin = self.halrcomps["pos-comps"].getpin("min_az")
        max_pin.set(max)
        min_pin.set(min)
        self.busy_wait(max_pin)
        self.busy_wait(min_pin)

    def set_el_pos_limits(self, max, min):
        lim = self.halrcomps["pos-comps"]
        lim.getpin("max_el").set(max)
        lim.getpin("min_el").set(min)
        self.busy_wait(lim)

    def set_az_calibration_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("az_diff")
        pin.set(self.az_calibration_diff)
        self.busy_wait(pin)

    def set_az_normal_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("az_diff")
        pin.set(self.az_normal_diff)
        self.busy_wait(pin)

    def reset_az_encoder(self):
        component = self.halrcomps["motor-feedback"]
        az_reset = component.getpin("reset_az")
        az_reset.set(True)
        self.halrcomps["abspos"].getpin("az_reset").set(0.0)
        az_reset.set(False)

    def calibrate_az(self):
        self.logger.info("Started calibrating azimuth")
        self.set_az_calibration_diff()

        if self.init_az > 0:
            direction = -1.0
            self.logger.info("Positive antenna previous directon, starting motor negative")
        else:
            direction = 1.0
            self.logger.info("Negative antenna previous directon, starting motor positive")

        target_pos = direction * self.az_range
        self.logger.info("Target position : %f" % (target_pos))

        self.send_az_calibrate_pos(target_pos)

        self.moving_az_event.wait()

        #Stop 1
        az_pos = self.wait_for_zero_az()
        self.logger.info("Found first endpoint: %f" % (az_pos))
        self.disable_az_pwm()
        self.logger.info("Disabled pwm")

        self.set_az_normal_diff()
        self.reset_az_encoder()
        self.logger.info("Reset encoder")

        opposite_side = -direction * self.az_range
        self.logger.info("opposite side : %f" % (opposite_side))
        self.send_az_calibrate_pos(opposite_side)

        self.enable_az_pwm()
        self.logger.info("Enabled pwm")

        time.sleep(3)

        self.moving_az_event.wait()

        #Stop 2
        total_az_range = abs(self.wait_for_zero_az())
        self.logger.info("Found second endpoint total range: %f" % (total_az_range))

        #Move to middle and reset
        mid = total_az_range/2.0
        self.send_az_calibrate_pos(direction*mid)

        self.moving_az_event.wait()

        #Stop 3
        temp2 = self.wait_for_zero_az()
        self.logger.info("Found mid point: %f" % (temp2))
        self.disable_az_pwm()

        self.set_state(Override.idle)

        self.reset_az_encoder()
        self.set_az_pos_limits(mid, -mid)

        self.enable_az_pwm()

    def calibrate_el(self):
        pass

    def calibrate(self):
        self.set_state(Override.calibrating)

        self.logger.info("Calibrating")

        #TODO set calibrating config speed, lims, gains etc

        self.calibrate_az()

        self.calibrate_el()

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

    def set_tracking_threshold(self):
        angle_comp = self.halrcomps["set-angle"]
        angle_comp.getpin("az_angle").set(self.az_gps_limit)
        angle_comp.getpin("el_angle").set(self.el_gps_limit)

    def set_gps_threshold(self):
        angle_comp = self.halrcomps["set-angle"]
        angle_comp.getpin("az_angle").set(self.az_re_enter_limit)
        angle_comp.getpin("el_angle").set(self.el_re_enter_limit)

    def set_state(self, new_state):
        self.state_lock.acquire()

        if isinstance(new_state, Manual):
            if new_state == Manual.stop:
                self.state = self.get_tracking_pin_state()
            else:
                self.state = new_state
        elif isinstance(new_state, Override) and not isinstance(self.state, Manual):
            if new_state == Override.stop:
                self.state = self.get_tracking_pin_state()
            elif self.state == Override.calibrating and new_state == Override.gps:
                pass
            elif self.state == Override.idle and new_state == Override.calibrating:
                pass
            elif self.state == Override.gps:
                pass
            else:
                self.state = new_state
        elif not isinstance(self.state, Manual) and not isinstance(self.state, Override):
            self.state = new_state

        if self.state == Override.calibrating:
            vel_mux = 2
            gps_mux = 2
        elif self.state == Override.idle:
            vel_mux = 2
            gps_mux = 4 #Not relveant
        elif self.state == State.tracking or self.state == Manual.tracking:
            vel_mux = 2
            gps_mux = 0
            self.set_tracking_threshold()
        elif self.state == State.gps or self.state == Override.gps:
            vel_mux = 2
            gps_mux = 1
            self.set_gps_threshold()
        elif self.state == Manual.position:
            vel_mux = 2
            gps_mux = 3
        elif self.state == Manual.velocity:
            vel_mux = 1
            gps_mux = 0 #Not relevant
        else:
            self.logger.error("Unknown state, panic!")
            self.state_lock.release()
            return

        p1 = self.halrcomps["velmux"].getpin("az_sel")
        p1.set(vel_mux)
        p2 = self.halrcomps["velmux"].getpin("el_sel")
        p2.set(vel_mux)

        p3 = self.halrcomps["gpsmux"].getpin("az_sel")
        p3.set(gps_mux)
        p4 = self.halrcomps["gpsmux"].getpin("el_sel")
        p4.set(gps_mux)

        self.busy_wait(p1)
        self.busy_wait(p2)
        self.busy_wait(p3)
        self.busy_wait(p4)

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

        self.comm_thread.join()
        self.logger.info("Shutdown comm thread")
        self.pos_thread.join()
        self.logger.info("Shutdown pos thread")

        self.sd.stop()
        shutdown_hal()
        for name, rcomp in self.halrcomps.iteritems():
            rcomp.remove_pins()
            rcomp.set_disconnected()

        self.halrcomps = {}


        if self.cleanup_event:
            self.logger.info("CLEANUP BITCHES")
            self.cleanup_event.set()

        self.logger.info("Stopped statemachine")

    def run(self, comm_constructor, comm_thread, exit_event=None, cleanup_event=None):
        self.cleanup_event = cleanup_event

        self.comm_thread = comm_thread
        self.comm_constructor = comm_constructor
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

                #If executed as secondary thread instead of main thread
                if exit_event:
                    if exit_event.is_set():
                        break
        except KeyboardInterrupt:
            pass

        self.cleanup()
