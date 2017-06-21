import time
from machinekit import launcher


launcher.check_installation()
launcher.cleanup_session()  # kill any running Machinekit instances
launcher.start_realtime()  # start Machinekit realtime environment
launcher.load_hal_file('hal_example.py')  # load the main HAL file
launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

launcher.ensure_mklauncher()  # ensure mklauncher is started

while True:
    launcher.check_processes()
    time.sleep(1)
