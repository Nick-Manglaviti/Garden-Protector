#! usr/bin/python3

# Robot Modes
class Modes(Enum):
    ACTIVE_DETECTION = 0
    PASSIVE_DETECTION = 1
    TARGETTING = 2


# Servo Pin Values (Use GPIO.BCM values for pins)
class ServoID(Enum):
    Servo1 = 4 # Yaw
    Servo2 = 14 # Pitch
    Servo3 = 17 # Trigger

# ROS Topics
class Topics(Enum):
    CMD_VEL = '/cmd_vel'
    CAMERA_FEED = '/camera_feed'

# ROS Actions
class Actions(Enum):
    Target_Scan = '/target_scan_action_server'
