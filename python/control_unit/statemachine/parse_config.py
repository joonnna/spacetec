import ConfigParser

def read_config():
    config = ConfigParser.ConfigParser()
    config.read("/home/machinekit/machinekit/spacetec/configs/test_config.ini")

    dict = {}
    """
    dict["calibrate_pos_igain"] = config.getfloat("CALIBRATE_VALUES", "POS_PID_IGAIN")
    dict["calibrate_pos_pgain"] = config.getfloat("CALIBRATE_VALUES", "POS_PID_PGAIN")

    dict["calibrate_velocity_igain"] = config.getfloat("CALIBRATE_VALUES", "VEL_PID_IGAIN")
    dict["calibrate_velocity_pgain"] = config.getfloat("CALIBRATE_VALUES", "VEL_PID_PGAIN")

    dict["calibrate_max_velocity"] = config.getfloat("CALIBRATE_VALUES", "VEL_LIM_MAX")
    dict["calibrate_min_velocity"] = config.getfloat("CALIBRATE_VALUES", "VEL_LIM_MAX")

    dict["pos_igain"] = config.getfloat("POS_PID", "POS_PID_IGAIN")
    dict["pos_igain"] = config.getfloat("POS_PID", "POS_PID_IGAIN")

    dict["velocity_igain"] = config.getfloat("VEL_PID", "VEL_PID_IGAIN")
    dict["velocity_igain"] = config.getfloat("VEL_PID", "VEL_PID_IGAIN")

    dict["max_velocity"] = config.getfloat("VEL_LIMS", "VEL_LIM_MAX")
    dict["min_velocity"] = config.getfloat("VEL_LIMS", "VEL_LIM_MAX")

    dict["callibrate_velocity_igain"] = config.getfloat("CALIBRATE_VALUES", "VEL_PID_IGAIN")
    dict["calibrate_velocity_pgain"] = config.getfloat("CALIBRATE_VALUES", "VEL_PID_PGAIN")

    dict["calibrate_pos_pgain"] = config.getfloat("CALIBRATE_VALUES", "POS_PID_PGAIN")
    dict["calibrate_pos_pgain"] = config.getfloat("CALIBRATE_VALUES", "POS_PID_PGAIN")
    """

    dict["pos_timeout"] = config.getfloat("THREAD_TIMEOUTS", "POS_THREAD")
    dict["check_threads_timeout"] = config.getfloat("THREAD_TIMEOUTS", "CHECK_THREADS")

    dict["az_lim"] = config.getfloat("GPS_LIMITS", "AZ_LIM")
    dict["el_lim"] = config.getfloat("GPS_LIMITS", "EL_LIM")
    dict["az_re_enter_limit"] = config.getfloat("GPS_LIMITS", "AZ_RE_ENTER_LIMIT")
    dict["el_re_enter_limit"] = config.getfloat("GPS_LIMITS", "EL_RE_ENTER_LIMIT")

    dict["az_range"] = config.getfloat("AXIS_RANGE", "AZ_RANGE")
    dict["el_range"] = config.getfloat("AXIS_RANGE", "EL_RANGE")

    dict["az_offset"] = config.getfloat("COMPASS_OFFSET", "AZ_OFFSET")
    dict["el_offset"] = config.getfloat("COMPASS_OFFSET", "EL_OFFSET")

    dict["gps_long"] = config.getfloat("GPS_CORDINATES", "LONGTITUDE")
    dict["gps_lat"] = config.getfloat("GPS_CORDINATES", "LATITUDE")
    dict["gps_height"] = config.getfloat("GPS_CORDINATES", "HEIGHT")

    dict["overide_gps_el"] = config.getfloat("OVERIDE_GPS_LIMS", "EL_LIM")
    dict["overide_gps_height"] = config.getfloat("OVERIDE_GPS_LIMS", "HEIGHT_LIM")

    dict["sig_lim"] = config.getfloat("SIGNAL", "SIGNAL_LIMIT")
    dict["sig_reenter_lim"] = config.getfloat("SIGNAL", "SIGNAL_REENTER_LIMIT")

    return dict
