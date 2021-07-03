#! /usr/bin/python3

import rospy
import mappings as mp
from garden_protector.msg import Orientation
from gpiozero import Servo
from time import sleep

class Robot(object):

    def __init__(self):
        self._init_servos()
        self.target_found = False
        self._default_mode = rospy.get_param("/robot_mode")
        self.current_mode = self._default_mode
        self.cmd_vel_pub = rospy.Publisher(mp.Topics.CMD_VEL.value, Orientation, queue_size=1)
        self.cmd_vel_msg = Orientation()

    def _init_servos(self):
        self.servo_yaw_state = 0
        self.servo_pitch_state = 0
        self.servo_trigger_state = 0
        self.servo_yaw = Servo(mp.Servos.YAW.value)
        self.servo_pitch = Servo(mp.Servos.PITCH.value)
        self.servo_trigger = Servo(mp.Servos.TRIGGER.value)
        self.servo_yaw.detach()
        self.servo_pitch.detach()
        self.servo_trigger.detach()

    def change_mode(self, mode):
        self.current_mode = mode

    def relative_move(self, x, y):
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
        
        print("Step")
        print(self.servo_yaw_state)
        print(self.servo_pitch_state)
        
        self.cmd_vel_msg.yaw = self.servo_yaw_state
        self.cmd_vel_msg.pitch = self.servo_pitch_state
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        sleep(1)
