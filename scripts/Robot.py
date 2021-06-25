#! /usr/bin/python3

import rospy
import mappings as mp
from garden_protector.msg import Orientation
from 

class Robot(object):
    self.servo1_state = 0
    self.servo2_state = 0
    self.servo3_state = 0
    
    self.target_found = False

    self.cmd_vel_pub = rospy.Publisher(mp.Topics.CMD_VEL, Orientation, queue_size=1)
    self.trigger_pub = rospy.Publisher(mp.Topics.TRIGGER, )
    
    def _init_servos(self):
        self.servo1 = Servo(mp.ServoID.Servo1)
        self.servo2 = Servo(mp.ServoID.Servo2)
        self.servo3 = Servo(mp.ServoID.Servo3)
        self.servo1.detach()
        self.servo2.detach()
        self.servo3.detach()


