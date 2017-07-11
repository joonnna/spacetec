import time
from general_thread import *
from machinekit import launcher
from statemachine.state import *
from udpComm.server import *

def start(path, ip, port):
    sm = Statemachine(path)
    comm = Communication(port, ip)
    comm_thread = new_thread(comm.run, comm.shutdown, 0.0, sm.send_gps_pos)

    sm.run(comm_thread)


ip = "192.168.5.4"
port = 2674

project_folder = "/home/machinekit/machinekit/spacetec/"


launcher.check_installation()
launcher.cleanup_session()  # kill any running Machinekit instances
launcher.start_realtime()  # start Machinekit realtime environment

#TODO relpath... wtf man...absolute path or no path
launcher.load_hal_file(project_folder + "hal/system.hal", project_folder + "configs/test_config.ini")  # load the main HAL file
launcher.register_exit_handler()  # enable on ctrl-C, needs to executed after HAL files

launcher.ensure_mklauncher()  # ensure mklauncher is started

time.sleep(1)

start(project_folder + "data_files/pos", ip, port)

