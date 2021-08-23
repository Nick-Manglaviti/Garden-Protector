#! /usr/bin/python3

import rospy
import actionlib
import mappings as mp
from garden_protector.msg import TargetScanAction, TargetScanGoal, Orientation, TargetScanFeedback

'''
    The client will wait for the server, then once connected
    will subscribe to its feedback topic. This topic will give info
    on if a target was found and at what level of the picture.
    If the bounding box is not centered, then a slight adjustment is made,
    else it tells the robot to fire.
'''
class TargetScanClient(object):
    
    def __init__(self, robot):
        self.robot = robot
        self.adjust_level = rospy.get_param("/robot_adjustment_level")
        self.offset_buffer = rospy.get_param("/img_offset_percentage")
        
        # Start Up Action Client
        self.client = actionlib.SimpleActionClient(mp.Actions.TARGET_SCAN.value, TargetScanAction)
        rospy.loginfo('Robot waiting for '+ mp.Actions.TARGET_SCAN.value)
        self.client.wait_for_server()
        rospy.loginfo('Found!')
        action_feedback_name = (mp.Actions.TARGET_SCAN.value +'/feedback')
        self.subscriber = rospy.Subscriber(str(action_feedback_name), TargetScanFeedback, self._process_img_fb, queue_size=1)
        self.goal = TargetScanGoal()

    def send_goal(self):
        rospy.loginfo("Goal sent to Target Scan Action Server.")
        self.client.send_goal(self.goal)
    
    def cancel(self):
        rospy.loginfo("Goal cancelled for Target Scan Action Server.")
        self.client.cancel_goal()
   
    def _process_img_fb(self, msg):
        feedback = msg.feedback
        if (feedback.target_found == True):
            rospy.loginfo("Target Found!")
            self.robot.change_mode(mp.Modes.TARGET_FOUND)
            move_msg = Orientation()
            move_msg.yaw = 0
            move_msg.pitch = 0
            self.robot.target_found = True
            x, y = self._get_center_point(feedback)
            adjustment = False
            x_offset = x - .5
            y_offset = y - .5
            if (abs(x_offset)  > abs(self.offset_buffer)):
                adjustment = True
                if x_offset > 0:
                    move_msg.yaw = -self.adjust_level
                else:
                    move_msg.yaw = self.adjust_level
            if abs(y_offset) > abs(self.offset_buffer):
                adjustment = True
                if y_offset > 0:
                    move_msg.pitch = self.adjust_level
                else:
                    move_msg.pitch = -self.adjust_level
            if adjustment == False:
                rospy.loginfo('Target Locked.')
                self.robot.fire()
            else:
                rospy.loginfo('Adjusting to Target...')
                self.robot.relative_orient(move_msg.yaw, move_msg.pitch)

    def _get_center_point(self, feedback):
        x = (feedback.xmin + feedback.xmax) / 2
        y = (feedback.ymin + feedback.ymax) / 2
        return x, y
