#! /usr/bin/python3

import rospy
import mappings as mp
from std_msgs.msg import Bool
from time import sleep

class TriggerSub(object):

    def __init__(self, robot):
        self.robot = robot
        self.subscriber = rospy.Subscriber(mp.Topics.TRIGGER.value, Bool, self._callback, queue_size=1)
        while self.subscriber.get_num_connections() == 0:
            sleep(.1)
    
    def _callback(self, msg):
        if msg.data == True:
            rospy.loginfo("Firing!")
            self.robot.servo_trigger_state = 1
            self.robot.servo_trigger.value = 1
            sleep(.4)
            self.robot.servo_trigger_state = -1
            self.robot.servo_trigger.value = -1
            sleep(.4)
            self.robot.servo_trigger.detach()
        else:
            rospy.loginfo("Resetting Trigger Servo.")
            self.robot.servo_trigger_state = -1
            self.robot.servo_trigger.value = -1
            sleep(.4)
