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
    ORIENTATION = 'garden_protector/orientation'            
    CAMERA_FEED = 'garden_protector/camera_feed'
    TRIGGER = 'garden_protector/trigger'

# ROS Actions
class Actions(Enum):
    TARGET_SCAN = 'garden_protector/target_scan_action_server'
