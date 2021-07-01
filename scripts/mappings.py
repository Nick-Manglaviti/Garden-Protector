#! usr/bin/python3
from enum import Enum

# Robot Modes
class Modes(Enum):
    ACTIVE_DETECTION = 0
    PASSIVE_DETECTION = 1
    TARGETTING = 2


# Servo Pin Values (Use GPIO.BCM values for pins)
class Servos(Enum):
    YAW = 4
    PITCH = 14
    TRIGGER = 17

class ServoRange(Enum):
    MAX = 1
    MIN = -1

# ROS Topics
class Topics(Enum):
    CMD_VEL = '/cmd_vel'            
    CAMERA_FEED = '/camera_feed'

# ROS Actions
class Actions(Enum):
    Target_Scan = '/target_scan_action_server'
