import time
from general_thread import *
from machinekit import launcher
from statemachine.state import *
from udpComm.server import *
import subprocess

project_folder = "/home/machinekit/machinekit/spacetec/"

def start(path):
    sm = Statemachine(path)
    try:
        comm = Communication()
    except TypeError:
        print "Can't start comm, exiting..."
        return
    comm_thread = new_thread(comm.run, comm.shutdown, 0.0, sm.send_gps_pos)

    sm.run(new_comm, comm_thread)

def func():
    launcher.start_realtime()  # start Machinekit realtime environment

    launcher.load_hal_file(project_folder + "hal/system.hal", project_folder + "configs/test_config.ini")  # load the main HAL file

 #   launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

    launcher.ensure_mklauncher()  # ensure mklauncher is started

launcher.check_installation()
launcher.cleanup_session()  # kill any running Machinekit instances


thread.start_new_thread(func, ())

time.sleep(1)

start(project_folder + "data_files/pos")

