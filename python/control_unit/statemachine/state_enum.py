from enum import Enum
import socket

class State(Enum):
    calibrating = 0
    idle = 1
    tracking = 2
    gps = 3
    gps_overide = 4
    stop_overide = 5
    stop_idle = 6
    manual_position = 7
    manual_velocity = 8
    stop_manual = 9
    tracking_override = 10
    stop_tracking_override = 11
