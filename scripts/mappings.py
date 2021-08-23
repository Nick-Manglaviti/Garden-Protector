#! usr/bin/python3
from enum import Enum

'''
Robot Modes to Set to
IDLE: Will sit there and do nothing. No goal will be sent.
Active_Detection: The robot will send a goal to find a targets.
    It will move in intervals sweeping left then right to check other parts
    of its view.
Passive_Detection: The robot will send a goal to find targets.

Background Robot Modes
Target_Found: On recieving a identified target, the robot changes to this mode.
Targetting: After finding a target, the robot immediatly swaps to this mode.
'''
class Modes(Enum):
    IDLE = 0
    ACTIVE_DETECTION = 1
    PASSIVE_DETECTION = 2
    TARGET_FOUND = 3
    TARGETTING = 4

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
