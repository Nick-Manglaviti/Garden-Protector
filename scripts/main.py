#! /usr/bin/python3

import rospy
from Robot import Robot
import mappings as mp
from Mode_ActiveDetection import Mode_ActiveDetection
import time

if __name__ == '__main__':
    rospy.init_node("garden_protector_application_node")
    robot = Robot()
    active_detection = Mode_ActiveDetection()
    
    if not robot.current_mode == mp.Modes.IDLE.value:
        robot.target_scan_client.send_goal()

    while not rospy.is_shutdown():
        '''
        In Passive Detection Mode, the robot will continue to check
        for a state change from the Target Scan Server. If it's 
        default mode was Active Detection, then after the watch interval
        has expired, it will return to that state.
        '''
        if robot.current_mode == mp.Modes.PASSIVE_DETECTION.value:
            if active_detection.next_time < time.time():
                robot.default_mode()
        
        '''
        Robot will turn, and watch for the amount of seconds set by
        watch_interval ros parameter
        '''
        if robot.current_mode == mp.Modes.ACTIVE_DETECTION.value:
            active_detection.process(robot)

        '''
        When a target is found, the Robot will enter the Targetting State.
        After 3 seconds, if the robot was unable to find the target, 
        then return to default mode.
        '''
        if robot.current_mode == mp.Modes.TARGET_FOUND.value:
            robot.change_mode(mp.Modes.TARGETTING)
            time.sleep(3)
            if robot.current_mode == mp.Modes.TARGETTING.value:
                print("Returning to Detection.")
                robot.target_found = False
                robot.reset_servos()
                robot.default_mode()
