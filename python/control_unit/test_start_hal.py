from machinekit import launcher

project_folder = "/home/machinekit/machinekit/spacetec/"

launcher.check_installation()
launcher.cleanup_session()  # kill any running Machinekit instances
launcher.start_realtime()  # start Machinekit realtime environment

launcher.load_hal_file(project_folder + "hal/system.hal", project_folder + "configs/test_config.ini")  # load the main HAL file

#launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

launcher.ensure_mklauncher()  # ensure mklauncher is started
