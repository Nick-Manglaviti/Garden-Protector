#! /usr/bin/python3

import rospy
import mappings as mp
from gpiozero import Servo
from garden_protector.msg import Orientation
from std_msgs.msg import Bool
from orient_sub import OrientSub
from trigger_sub import TriggerSub
from target_scan_client import TargetScanClient
from time import sleep

class Robot(object):

    def __init__(self):

        # Init Instance Fields
        self.turn_rate = rospy.get_param("/turn_rate")
        self._default_mode = rospy.get_param("/robot_mode")
        self._init_servos()
        self.target_found = False
        self.current_mode = self._default_mode
        
        # Action Client
        self.target_scan_client = TargetScanClient(self)
        
        # Publishers
        self.trigger_pub = rospy.Publisher(mp.Topics.TRIGGER.value, Bool, queue_size=1)
        self.orient_pub = rospy.Publisher(mp.Topics.ORIENTATION.value, Orientation, queue_size=1)

        # Messages
        self.trigger_msg = Bool()
        self.orient_msg = Orientation()

        # Subscribers
        self.trigger_sub = TriggerSub(self)
        self.orient_sub = OrientSub(self)

        rospy.loginfo('Robot Initialized.')

    def _init_servos(self):
        self.servo_yaw_state = 0
        self.servo_pitch_state = 0
        self.servo_trigger_state = 1
        self.servo_yaw = Servo(mp.Servos.YAW.value)
        self.servo_pitch = Servo(mp.Servos.PITCH.value)
        self.servo_trigger = Servo(mp.Servos.TRIGGER.value)
        self.servo_trigger.max()
        sleep(1)
        self.servo_yaw.detach()
        self.servo_pitch.detach()
        self.servo_trigger.detach()

    def reset_servos(self):
        rospy.loginfo("Reset Servos.")
        self.servo_yaw_state = 0
        self.servo_pitch_state = 0
        self.servo_trigger_state = 0
        self.servo_yaw.mid()
        self.servo_pitch.mid()
        self.servo_trigger.max()
        sleep(1)
        self.servo_yaw.detach()
        self.servo_pitch.detach()
        self.servo_trigger.detach()
        rospy.loginfo("Servo positions reset.")

    def detach_servos(self):
        self.servo_yaw.detach()
        self.servo_pitch.detach()
        self.servo_trigger.detach()

    def change_mode(self, mode):
        self.current_mode = mode.value
    
    def default_mode(self):
        self.current_mode = self._default_mode

    def relative_orient(self, x, y):
        yaw = self.servo_yaw_state + x
        pitch = self.servo_pitch_state + y
        
        if yaw >= mp.ServoRange.MAX.value:
            yaw = mp.ServoRange.MAX.value
        if pitch >= mp.ServoRange.MAX.value:
            pitch = mp.ServoRange.MAX.value
        if yaw <= mp.ServoRange.MIN.value:
            yaw = mp.ServoRange.MIN.value
        if pitch <= mp.ServoRange.MIN.value:
            pitch = mp.ServoRange.MIN.value
        
        self.servo_yaw_state = yaw
        self.servo_pitch_state = pitch
        
        self.orient_msg.yaw = self.servo_yaw_state
        self.orient_msg.pitch = self.servo_pitch_state
        self.orient_pub.publish(self.orient_msg)
        sleep(self.turn_rate)

    def fixed_orient(self, x, y):
        if x > 1:
            x = 1
        if x < -1:
            x = -1
        if y > 1:
            y = 1
        if y < -1:
            y = -1

        self.servo_yaw_state = x
        self.servo_pitch_state = y
        
        self.orient_msg.yaw = self.servo_yaw_state
        self.orient_msg.pitch = self.servo_pitch_state
        self.orient_pub.publish(self.orient_msg)
        sleep(self.turn_rate)
    
    def fire(self):
        rospy.loginfo("Publishing Fire Request...")
        self.trigger_msg.data = True
        self.trigger_pub.publish(self.trigger_msg)
