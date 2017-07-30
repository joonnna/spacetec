import ConfigParser

def read_config():
    config = ConfigParser.ConfigParser()
    config.read("/home/machinekit/machinekit/spacetec/configs/test_config.ini")

    dict = {}

    dict["az_calibrate_max_velocity"] = config.getfloat("AZ_CALIBRATE_VEL_LIMITS", "MAX")
    dict["az_calibrate_min_velocity"] = config.getfloat("AZ_CALIBRATE_VEL_LIMITS", "MIN")

    dict["el_calibrate_max_velocity"] = config.getfloat("EL_CALIBRATE_VEL_LIMITS", "MAX")
    dict["el_calibrate_min_velocity"] = config.getfloat("EL_CALIBRATE_VEL_LIMITS", "MIN")

    dict["az_max_velocity"] = config.getfloat("AZ_VEL_LIMITS", "MAX")
    dict["az_min_velocity"] = config.getfloat("AZ_VEL_LIMITS", "MIN")

    dict["el_max_velocity"] = config.getfloat("EL_VEL_LIMITS", "MAX")
    dict["el_min_velocity"] = config.getfloat("EL_VEL_LIMITS", "MIN")

    dict["enter_limit"] = config.getfloat("GPS_LIMITS", "ENTER_LIMIT")
    dict["re_enter_limit"] = config.getfloat("GPS_LIMITS", "RE_ENTER_LIMIT")

    dict["max_el"] = config.getfloat("EL_POS_LIMITS", "MAX")
    dict["min_el"] = config.getfloat("EL_POS_LIMITS", "MIN")

    dict["az_range"] = config.getfloat("AXIS_RANGE", "AZ_RANGE")
    dict["el_range"] = config.getfloat("AXIS_RANGE", "EL_RANGE")

    dict["az_offset"] = config.getfloat("COMPASS_OFFSET", "AZ_OFFSET")
    dict["el_offset"] = config.getfloat("COMPASS_OFFSET", "EL_OFFSET")

    dict["overide_gps_el"] = config.getfloat("OVERIDE_GPS_LIMS", "EL_LIM")
    dict["overide_gps_height"] = config.getfloat("OVERIDE_GPS_LIMS", "HEIGHT_LIM")

    dict["sig_lim"] = config.getfloat("SIGNAL", "SIGNAL_LIMIT")
    dict["sig_reenter_lim"] = config.getfloat("SIGNAL", "SIGNAL_REENTER_LIMIT")

    dict["pos_timeout"] = config.getfloat("THREAD_TIMEOUTS", "POS_THREAD")
    dict["check_threads_timeout"] = config.getfloat("THREAD_TIMEOUTS", "CHECK_THREADS")

    dict["calibration_diff"] = config.getfloat("NEAR_ENDSTOP_VALUES", "CALIBRATION_DIFF")
    dict["normal_diff"] = config.getfloat("NEAR_ENDSTOP_VALUES", "NORMAL_DIFF")

    dict["gps_max_timeout"] = config.getfloat("GPS_TIMEOUT", "MAX_TIME_BETWEEN_PACKETS")


    dict["wait_period"] = config.getfloat("NEAR_ENDSTOP_VALUES", "WAIT_PERIOD")
    dict["change_threshold"] = config.getfloat("NEAR_ENDSTOP_VALUES", "CHANGE_THRESHOLD")


    return dict
