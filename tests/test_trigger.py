#! /usr/bin/python3

import rospy
import sys
sys.path.append('/home/pi/catkin_ws/src/garden_protector/scripts')
from Robot import Robot
from time import sleep

if __name__ == "__main__":
    rospy.init_node("garden_protector_test_aplication_node")
    robot = Robot()
    x = 0
    rospy.loginfo("Starting 10 Trigger Publishes in 2 seconds...")
    sleep(2)
    while x < 20:
        robot.fire()
        x = x + 1
        sleep(.1)
    rospy.spin()
