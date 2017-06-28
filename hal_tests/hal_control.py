from machinekit import launcher

def start_hal(path):
    launcher.check_installation()
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()  # start Machinekit realtime environment

    launcher.load_hal_file(path)  # load the main HAL file
    launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

    launcher.ensure_mklauncher()  # ensure mklauncher is started

def shutdown_hal():
    #launcher.end_session()
    launcher.cleanup_session()
