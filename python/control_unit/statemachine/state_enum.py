from enum import Enum
import socket

class State(Enum):
    calibrating = 0
    idle = 1
    tracking = 2
    gps = 3
    gps_overide = 4
    stop_overide = 5
