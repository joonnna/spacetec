from enum import Enum
import socket

class State(Enum):
    calibrating = 0
    tracking = 1
    gps = 2
    gps_overide = 4
    stop_overide = 5
