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
        self.last_received_lock = threading.Lock()

        self.set_last_received(0.0)

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

        self.state = Override.calibrating

        self.init_az = az
        self.init_el = el

        self.sd = ServiceDiscovery()
        self.halrcomps = {}
        self.initrcomps(testing)
        self._search_and_bind()

        self.set_state(Override.calibrating)

 #       self.set_el_pos_limits(100000000000.0, -1000000000.0)

        #time.sleep(20)
        #time.sleep(10)
 #       time.sleep(1)
  #      self.reset_el_encoder()
   #     time.sleep(20)
    #    self.set_el_gains()

        #while True:
         #   time.sleep(2)


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

        vel_mux = halremote.RemoteComponent("velmux", debug=False)
        vel_mux.newpin("az_sel", halremote.HAL_S32, halremote.HAL_OUT)
        vel_mux.newpin("el_sel", halremote.HAL_S32, halremote.HAL_OUT)

        abspos = halremote.RemoteComponent("abspos", debug=False)
        abspos.newpin("az_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("az_in", halremote.HAL_FLOAT, halremote.HAL_IN)
        abspos.newpin("el_reset", halremote.HAL_FLOAT, halremote.HAL_OUT)
        abspos.newpin("el_in", halremote.HAL_FLOAT, halremote.HAL_IN)

        tracking = halremote.RemoteComponent("step", debug=False)
        track = tracking.newpin("track", halremote.HAL_BIT, halremote.HAL_IN)
        manual = tracking.newpin("manual", halremote.HAL_S32, halremote.HAL_IN)
        manual.on_value_changed.append(self.manual_state_callback)
        track.on_value_changed.append(self.check_gps)

        pos_comps = halremote.RemoteComponent("pos-comps", debug=False)
        pos_comps.newpin("max_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_az", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("max_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("min_el", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("az_diff", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("el_diff", halremote.HAL_FLOAT, halremote.HAL_IO)
        pos_comps.newpin("north_angle", halremote.HAL_FLOAT, halremote.HAL_IN)

        motor_feedback = halremote.RemoteComponent("motor-feedback", debug=False)
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

        gps_angle_check = halremote.RemoteComponent("set-angle", debug=False)
        gps_angle_check.newpin("enter_gps_angle", halremote.HAL_FLOAT, halremote.HAL_IN)
        gps_angle_check.newpin("re_enter_tracking_angle", halremote.HAL_FLOAT, halremote.HAL_IN)
        gps_angle_check.newpin("angle_out", halremote.HAL_FLOAT, halremote.HAL_IO)
        #gps_angle_check.newpin("el_angle", halremote.HAL_FLOAT, halremote.HAL_IO)

        pid_control = halremote.RemoteComponent("pid-control", debug=False)
        pid_control.newpin("az_enable",  halremote.HAL_BIT, halremote.HAL_OUT)
        pid_control.newpin("el_enable",  halremote.HAL_BIT, halremote.HAL_OUT)

        #poller = halremote.RemoteComponent("rssi-reader", debug=False)
        #poller.newpin("value", halremote.HAL_FLOAT, halremote.HAL_OUT)

        self.halrcomps[pid_control.name] = pid_control
        #self.halrcomps[poller.name] = poller
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

    def busy_wait(self, pin, max_wait=None):
        if max_wait == None:
            timeout = 5.0
        else:
            timeout = max_wait
        start = time.time()
        #timeout = 1.0
        while not pin.synced:
            if time.time() - start > timeout:
                return
            time.sleep(0.1)

    def enable_el_pids(self):
        pin = self.halrcomps["pid-control"].getpin("el_enable")
        pin.set(True)
        self.busy_wait(pin)

    def enable_az_pids(self):
        pin = self.halrcomps["pid-control"].getpin("az_enable")
        pin.set(True)
        self.busy_wait(pin)

    def test_sync(self, val):
        self.logger.debug("JADA!!!!! %s" % (val))

    def disable_az_pwm(self):
        pin = self.halrcomps["pwm"].getpin("az")
        pin.set(False)
        self.busy_wait(pin)

    def enable_az_pwm(self):
        pin = self.halrcomps["pwm"].getpin("az")
        pin.set(True)
        self.busy_wait(pin)

    def disable_el_pwm(self):
        pin = self.halrcomps["pwm"].getpin("el")
        pin.set(False)
        self.busy_wait(pin)

    def enable_el_pwm(self):
        pin = self.halrcomps["pwm"].getpin("el")
        pin.set(True)
        self.busy_wait(pin)


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
            self.moving_el_event.clear()
        else:
            self.zero_el_event.clear()
            self.moving_el_event.set()

    def wait_for_az_init(self):
        pin = self.halrcomps["motor-feedback"].getpin("az_init_start")
        pin.set(True)
        self.busy_wait(pin)
        self.logger.info("Waiting for physical bldc init to be start")
        self.az_init_event.wait()

    def wait_for_el_init(self):
        pin2 = self.halrcomps["motor-feedback"].getpin("el_init_start")
        pin2.set(True)
        self.busy_wait(pin2)
        self.logger.info("Waiting for physical bldc init to be start")
        self.el_init_event.wait()

    def wait_for_zero_az(self):
        self.logger.info("Waiting for zero az velocity")

        pin = self.halrcomps["motor-feedback"].getpin("az_pos")
        while True:
            val = pin.get()
            time.sleep(self.wait_period)
            val2 = pin.get()
            if abs(val2 - val) < self.change_threshold:
                self.logger.info("Got zero az velocity!")
                return val2

    def wait_for_zero_el(self):
        self.logger.info("Waiting for zero el velocity")

        pin = self.halrcomps["motor-feedback"].getpin("el_pos")
        while True:
            val = pin.get()
            time.sleep(self.wait_period)
            val2 = pin.get()
            if abs(val2 - val) < self.change_threshold:
                self.logger.info("Got zero el velocity!")
                return val2

    def manual_state_callback(self, val):
        if val == 0:
            self.set_state(Manual.stop)
        elif val == 1:
            self.set_state(Manual.position)
        elif val == 2:
            self.set_state(Manual.velocity)
        elif val == 3:
            self.set_state(Manual.tracking)
        elif val == 4:
            self.set_state(Manual.gps_manual)
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
        pin = self.halrcomps["gpsmux"].getpin("az_calibration")
        pin.set(az)
        self.busy_wait(pin)

    def send_el_calibrate_pos(self, el):
        pin = self.halrcomps["gpsmux"].getpin("el_calibration")
        pin.set(el)
        self.busy_wait(pin)

    def get_north_angle(self):
        pin = self.halrcomps["pos-comps"].getpin("north_angle")
        self.busy_wait(pin)
        return pin.get()

    def send_gps_pos(self, pos):
        state = self.get_state()
        if state == Override.idle:
            self.logger.info("Got first UDP packet!")
            self.set_state(Override.stop)

        self.set_last_received(time.time())

        az = pos[0]
        el = pos[1]
        height = pos[2]

        az_command = az + self.get_north_angle()

        if az_command > 180.0:
            az_command = az_command - 360.0

     #   self.logger.debug(el)
      #  az = 0.0
       # el = 45.0

        mux = self.halrcomps["gpsmux"]
        mux.getpin("az_gps").set(az_command)
        mux.getpin("el_gps").set(el)
 #       self.logger.debug("\naz: %f\nel:%f\nheight:%f\n" % (az, el, height))

        if isinstance(state, Manual):
            pass
        elif height > self.overide_gps_height:
            if state == Override.gps_override:
                self.logger.info("Stopping override gps!")
                self.set_state(Override.stop)
            else:
                pass
        else:
            if state == Override.gps_override:
                pass
            else:
                self.logger.info("Overriding gps!")
                self.set_state(Override.gps_override)

    def start_pos_thread(self):
        self.pos_thread = new_thread(self.store_old_abspos, self.cleanup_abspos_thread, self.pos_thread_timeout)
        self.pos_thread.start()

    def start_comm_thread(self):
        try:
            comm = self.comm_constructor()
        except TypeError:
            return

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

        self.enter_limit = config["enter_limit"]
        self.re_enter_limit = config["re_enter_limit"]

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

        self.max_el = config["max_el"]
        self.min_el = config["min_el"]

        self.max_time_between_packets = config["gps_max_timeout"]

        self.wait_period = config["wait_period"]
        self.change_threshold = config["change_threshold"]


    def set_az_pos_limits(self, max, min):
        max_pin = self.halrcomps["pos-comps"].getpin("max_az")
        min_pin = self.halrcomps["pos-comps"].getpin("min_az")
        max_pin.set(max)
        min_pin.set(min)
        self.busy_wait(max_pin)
        self.busy_wait(min_pin)

    def set_el_pos_limits(self, max, min):
        max_pin = self.halrcomps["pos-comps"].getpin("max_el")
        min_pin = self.halrcomps["pos-comps"].getpin("min_el")
        max_pin.set(max)
        min_pin.set(min)
        self.busy_wait(max_pin)
        self.busy_wait(min_pin)

    def set_az_calibration_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("az_diff")
        pin.set(self.az_calibration_diff)
        self.busy_wait(pin)

    def set_az_normal_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("az_diff")
        pin.set(self.az_normal_diff)
        self.busy_wait(pin)

    def set_el_calibration_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("el_diff")
        pin.set(self.az_calibration_diff)
        self.busy_wait(pin)

    def set_el_normal_diff(self):
        pin = self.halrcomps["pos-comps"].getpin("el_diff")
        pin.set(self.az_normal_diff)
        self.busy_wait(pin)


    def reset_az_encoder(self):
        pin = self.halrcomps["motor-feedback"].getpin("reset_az")
        pin.set(True)
        self.busy_wait(pin)

        pin2 = self.halrcomps["abspos"].getpin("az_reset")
        pin2.set(0.0)
        self.busy_wait(pin2)

        pin.set(False)
        self.busy_wait(pin)

    def reset_el_encoder(self):
        pin = self.halrcomps["motor-feedback"].getpin("reset_el")
        pin.set(True)
        self.busy_wait(pin)

        pin2 = self.halrcomps["abspos"].getpin("el_reset")
        pin2.set(0.0)
        self.busy_wait(pin2)

        pin.set(False)
        self.busy_wait(pin)

    def calibrate_az(self):
        self.logger.info("Started calibrating azimuth")

        self.set_az_normal_diff()
        #self.set_az_calibration_diff()
        self.set_az_pos_limits(self.az_range, -self.az_range)

        if self.init_az > 0:
            direction = 1.0
            self.logger.info("Positive antenna previous directon, starting motor negative")
        else:
            direction = -1.0
            self.logger.info("Negative antenna previous directon, starting motor positive")

        target_pos = direction * self.az_range
        self.logger.info("Target position : %f" % (target_pos))

        self.send_az_calibrate_pos(target_pos)

        #Stop 1
        az_pos = self.wait_for_zero_az()
        self.logger.info("Found first endpoint: %f" % (az_pos))

        self.logger.info("Reset encoder")
        self.reset_az_encoder()
        #self.set_az_normal_diff()

        opposite_side = -direction * self.az_range
        self.logger.info("opposite side : %f" % (opposite_side))
        self.send_az_calibrate_pos(opposite_side)

        #Stop 2
        range = self.wait_for_zero_az()
        total_az_range = abs(range)
        self.logger.info("Found second endpoint total range: %f" % (total_az_range))

        if range > 0:
            mid = total_az_range/2.0
        else:
            mid = -total_az_range/2.0
        #Move to middle and reset
        self.send_az_calibrate_pos(mid)

        self.logger.info("Going to mid %f" % (mid))
        #Stop 3
        temp2 = self.wait_for_zero_az()
        self.logger.info("Found mid point: %f" % (temp2))

        self.reset_az_encoder()
        self.send_az_calibrate_pos(0.0)
        max = abs(mid) - 2.0
        min = -abs(mid) + 2.0
        self.logger.info("max: %f min: %f" % (max, min))
        self.set_az_pos_limits(max, min)

    def calibrate_el(self):
        self.logger.info("Started calibrating elevation")

        self.set_el_normal_diff()

        self.set_el_pos_limits(self.el_range, -self.el_range)

        direction = -1.0

        target_pos = direction * self.el_range
        self.logger.info("Target position : %f" % (target_pos))

        self.send_el_calibrate_pos(target_pos)

        #Stop 1
        el_pos = self.wait_for_zero_el()
        self.logger.info("Found first endpoint: %f" % (el_pos))

        self.logger.info("Reset encoder")
        opposite_side = -direction * self.el_range
        self.reset_el_encoder()
        self.send_el_calibrate_pos(opposite_side)
        self.logger.info("opposite side : %f" % (opposite_side))

        #Stop 2
        range = self.wait_for_zero_el()
        total_el_range = abs(range)
        self.logger.info("Found second endpoint total range: %f" % (total_el_range))

        if range > 0:
            mid = total_el_range/2.0
        else:
            mid = -total_el_range/2.0

        self.send_el_calibrate_pos(mid)
        self.logger.info("Going to mid point...")
        #Stop 3
        temp2 = self.wait_for_zero_el()
        self.logger.info("Found mid point: %f" % (temp2))
        self.reset_el_encoder()

        final_angle = -90.0
        self.send_el_calibrate_pos(final_angle)
        self.logger.info("Going to 90 degrees...")


        temp3 = self.wait_for_zero_el()
        self.logger.info("Found 90 degrees: %f" % (temp3))

        self.reset_el_encoder()
        self.send_el_calibrate_pos(0.0)
        self.set_el_pos_limits(self.max_el, self.min_el)

    def calibrate(self):
        self.logger.info("Calibrating")
        self.logger.info("Waiting for az bldc init")
        self.wait_for_az_init()
        self.logger.info("Bldc az init done!")

        self.reset_az_encoder()

        self.enable_az_pids()

        self.calibrate_az()

        self.logger.info("Waiting for el bldc init")
        self.wait_for_el_init()
        self.logger.info("Bldc el init done!")

        self.reset_el_encoder()

        self.enable_el_pids()

        self.calibrate_el()

        self.set_state(Override.idle)
        self.logger.info("Finished calibrating")


    def get_tracking_pin_state(self):
        time_since_last_packet = self.get_last_received()
        if time_since_last_packet == 0.0:
            return Override.idle

        tracking = self.halrcomps["step"].getpin("track").get()
        if tracking:
            return State.tracking
        else:
            return State.gps

    def set_tracking_threshold(self):
        comp  = self.halrcomps["set-angle"]
        pin  = comp.getpin("angle_out")
        pin.set(comp.getpin("enter_gps_angle").get())
        self.busy_wait(pin)

    def set_gps_threshold(self):
        comp  = self.halrcomps["set-angle"]
        pin  = comp.getpin("angle_out")
        pin.set(comp.getpin("re_enter_tracking_angle").get())
        self.busy_wait(pin)

    def set_state(self, new_state):
        self.state_lock.acquire()

        if self.state == new_state and new_state != Override.calibrating:
            self.state_lock.release()
            return

        if isinstance(new_state, Manual):
            if new_state == Manual.stop:
                self.state = self.get_tracking_pin_state()
            else:
                self.state = new_state
        elif isinstance(new_state, Override) and not isinstance(self.state, Manual):
            if new_state == Override.stop:
                self.state = self.get_tracking_pin_state()
            elif self.state == Override.calibrating and new_state == Override.gps_override:
                pass
            elif self.state == Override.idle and new_state == Override.calibrating:
                pass
            elif self.state == Override.gps_override:
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
        elif self.state == State.gps or self.state == Override.gps_override or self.state == Manual.gps_manual:
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

        max_wait = 1.0

        p1 = self.halrcomps["velmux"].getpin("az_sel")
        p1.set(vel_mux)
        p2 = self.halrcomps["velmux"].getpin("el_sel")
        p2.set(vel_mux)

        p3 = self.halrcomps["gpsmux"].getpin("az_sel")
        p3.set(gps_mux)
        p4 = self.halrcomps["gpsmux"].getpin("el_sel")
        p4.set(gps_mux)

        self.busy_wait(p1, max_wait)
        self.busy_wait(p2, max_wait)
        self.busy_wait(p3, max_wait)
        self.busy_wait(p4, max_wait)

        self.logger.info("Entered %s state" % (self.state.name))
        self.state_lock.release()

    def get_state(self):
        self.state_lock.acquire()
        ret = self.state
        self.state_lock.release()

        return ret

    def set_last_received(self, time):
        self.last_received_lock.acquire()
        self.last_received = time
        self.last_received_lock.release()

    def get_last_received(self):
        self.last_received_lock.acquire()
        ret = self.last_received
        self.last_received_lock.release()
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

        not_receiving = False
        try:
            while True:
                time.sleep(self.check_threads_timeout)

                state = self.get_state()

                if isinstance(state, Manual) and state != Manual.tracking:
                    pass
                elif state == Override.idle or state == Override.calibrating:
                    pass
                else:
                    time_since_last_packet =  time.time() - self.get_last_received()

                    if time_since_last_packet > self.max_time_between_packets:
                        self.logger.info("Not received ptu data for %f seconds, going rssi override" % (time_since_last_packet))
                        self.set_state(Manual.tracking)
                        not_receiving = True
                    elif not_receiving:
                        self.logger.info("Recieving ptu data as normal again, exiting rssi override")
                        self.set_state(Manual.stop)
                        not_receiving = False

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
