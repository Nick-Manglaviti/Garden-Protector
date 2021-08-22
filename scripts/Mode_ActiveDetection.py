#! /usr/bin/python3
import rospy
import mappings as mp
import time
'''
Robot will sweep left to right 
by an increment (value of turn) until a target is found.
'''
class Mode_ActiveDetection(object):
    
    def __init__(self):
        self.next_time = time.time() + (rospy.get_param("watch_interval") * 100)
        self.turn = rospy.get_param("/turn_percent")

    def process(self, robot):
        if robot.target_found:
            robot.change_mode(mp.Modes.TARGET_FOUND)
        else:
            if robot.servo_yaw_state >= mp.ServoRange.MAX.value:
                self.turn = -self.turn
            elif robot.servo_yaw_state <= mp.ServoRange.MIN.value:
                self.turn = abs(self.turn)
            robot.relative_orient(self.turn, 0)
            robot.detach_servos()
            self.next_time = time.time() + (rospy.get_param("watch_interval"))
            robot.change_mode(mp.Modes.PASSIVE_DETECTION)
