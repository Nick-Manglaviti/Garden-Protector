#! /usr/bin/python3

import rospy
from Robot import Robot
import mappings as mp
from Mode_ActiveDetection import Mode_ActiveDetection
from time import sleep

if __name__ == '__main__':
    rospy.init_node("garden_protector_application_node")
    robot = Robot()
    active_detection = Mode_ActiveDetection()
    robot.target_scan_client.send_goal()

    while not rospy.is_shutdown():
        if robot.current_mode == mp.Modes.ACTIVE_DETECTION.value:
            active_detection.process(robot)
        if robot.current_mode == mp.Modes.TARGET_FOUND.value:
            robot.change_mode(mp.Modes.TARGETTING.value)
            sleep(3)
            if robot.current_mode == mp.Modes.TARGETTING.value:
                print("Returning to Detection.")
                robot.target_found = False
                robot.reset_servos()
                robot.current_mode = mp.Modes.ACTIVE_DETECTION.value

