import ConfigParser


def read_config():
    config = ConfigParser.ConfigParser()
    config.read("../../../configs/test_config.ini")

    dict = {}

    dict["pos_pid_igain"] = config.getfloat("POS_PID", "POS_PID_IGAIN")
    dict["pos_pid_igain"] = config.getfloat("POS_PID", "POS_PID_IGAIN")

    dict["vel_pid_igain"] = config.getfloat("VEL_PID", "VEL_PID_IGAIN")
    dict["vel_pid_igain"] = config.getfloat("VEL_PID", "VEL_PID_IGAIN")


    dict["vel_lim_max"] = config.getfloat("VEL_LIMS", "VEL_LIM_MAX")
    dict["vel_lim_min"] = config.getfloat("VEL_LIMS", "VEL_LIM_MAX")

    dict["pos_timeout"] = config.getfloat("THREAD_TIMEOUTS", "POS_THREAD")
    dict["check_threads_timeout"] = config.getfloat("THREAD_TIMEOUTS", "CHECK_THREADS")
    dict["gps_check_timeout"] = config.getfloat("THREAD_TIMEOUTS", "GPS_CHECKER_THREAD")
    dict["gps_timeout"] = config.getfloat("THREAD_TIMEOUTS", "GPS_THREAD")

    dict["az_lim"] = config.getfloat("GPS_LIMITS", "AZ_LIM")
    dict["el_lim"] = config.getfloat("GPS_LIMITS", "EL_LIM")

    dict["az_range"] = config.getfloat("AXIS_RANGE", "AZ_RANGE")
    dict["el_range"] = config.getfloat("AXIS_RANGE", "EL_RANGE")


    return dict
