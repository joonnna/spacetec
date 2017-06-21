from machinekit import launcher

launcher.check_installation()
launcher.cleanup_session()  # kill any running Machinekit instances
launcher.start_realtime()  # start Machinekit realtime environment
launcher.load_hal_file('state.py')  # load the main HAL file
launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files
