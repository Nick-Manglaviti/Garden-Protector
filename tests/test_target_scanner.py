#! /usr/bin/python3

import rospy
import sys
sys.path.append('/home/pi/catkin_ws/src/garden_protector/scripts')
from Robot import Robot
import mappings as mp

if __name__ == "__main__":
    rospy.init_node("test_target_scan_application")
    robot = Robot()
    robot.target_scan_client.send_goal()
    while not rospy.is_shutdown():
        continue
