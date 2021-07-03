#! usr/bin/python3
import rospy
import mappings as mp

'''
Robot will sweep left to right 
by an increment (value of turn) until a target is found.
'''
class Mode_ActiveDetection(object):
    
    def __init__(self):
        self.turn = rospy.get_param("/turn_rate")

    def process(self, robot):
        if robot.target_found:
            robot.change_mode(mp.Modes.TARGETTING.value)
        else:
            if robot.servo_yaw_state >= mp.ServoRange.MAX.value:
                self.turn = -self.turn
            elif robot.servo_yaw_state <= mp.ServoRange.MIN.value:
                self.turn = abs(self.turn)
            robot.relative_move(self.turn, 0)
