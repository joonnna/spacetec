import time
import argparse
from machinekit import launcher
import state


parser = argparse.ArgumentParser()
parser.add_argument("--path", required=True, help="Filepath to the initial position file, containing azimuth and elevation paramterts on two seperate lines.")
parser.add_argument("--test", type=bool, default=False, help="Start halrun from python or not, used to be able t run halrun separetley")


args = parser.parse_args()

print args.test

if args.test:
    launcher.check_installation()
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()  # start Machinekit realtime environment

    #TODO relpath... wtf man...absolute path or no path
    launcher.load_hal_file("../hal/system.hal")  # load the main HAL file
    launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

    launcher.ensure_mklauncher()  # ensure mklauncher is started

    time.sleep(1)

    state.run(args.path)
else:
    print "Only starting statemachine..."
    state.run(args.path)
