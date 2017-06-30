import time
import argparse
from machinekit import launcher
from statemachine.state import *
from udpComm.server import *

def start(path, ip, port):
    sm = Statemachine(path)
    comm = Communication(port, ip)
    sm.run(comm.run)


parser = argparse.ArgumentParser()
parser.add_argument("--path", required=True, help="Filepath to the initial position file, containing azimuth and elevation paramterts on two seperate lines.")
parser.add_argument("--test", type=bool, default=False, help="Start halrun from python or not, used to be able t run halrun separetley")

ip = "192.168.5.4"
port = 2674

args = parser.parse_args()

if args.test:
    launcher.check_installation()
    launcher.cleanup_session()  # kill any running Machinekit instances
    launcher.start_realtime()  # start Machinekit realtime environment

    #TODO relpath... wtf man...absolute path or no path
    launcher.load_hal_file("../hal/system.hal")  # load the main HAL file
    launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

    launcher.ensure_mklauncher()  # ensure mklauncher is started

    time.sleep(1)

    start(args.path, ip, port)

else:
    print "Only starting statemachine..."
    start(args.path, ip, port)
