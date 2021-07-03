#! /usr/bin/python3

import rospy
import sys
sys.path.append('/home/pi/catkin_ws/src/garden_protector/scripts')
from Robot import Robot
from Mode_ActiveDetection import Mode_ActiveDetection
import mappings as mp

if __name__ == "__main__":
    rospy.init_node("garden_protector_application_node")
    robot = Robot()
    active_detection = Mode_ActiveDetection()
    while robot.current_mode == mp.Modes.ACTIVE_DETECTION.value:
        active_detection.process(robot)
