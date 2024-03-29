from enum import Enum
import socket

class State(Enum):
    tracking = 0
    gps = 1

class Override(Enum):
    stop = 0
    calibrating = 1
    idle = 2
    gps_override = 3

class Manual(Enum):
    stop = 0
    position = 1
    velocity = 2
    tracking = 3
    gps_manual = 4
