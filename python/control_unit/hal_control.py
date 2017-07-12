from machinekit import launcher

def start_hal(path):
    launcher.check_installation()
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()  # start Machinekit realtime environment

    launcher.load_hal_file(path, "/home/machinekit/machinekit/spacetec/configs/test_config.ini")  # load the main HAL file
    launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

    launcher.ensure_mklauncher()  # ensure mklauncher is started

def shutdown_hal():
    try:
        launcher.end_session()
    except OSError:
        return
 #   launcher.cleanup_session()
