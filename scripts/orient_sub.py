#! /usr/bin/python3

import rospy
import mappings as mp
from garden_protector.msg import Orientation
from time import sleep

class OrientSub(object):
    def __init__(self, robot):
        self.robot = robot
        self.subscriber = rospy.Subscriber(mp.Topics.ORIENTATION.value, Orientation, self._callback, queue_size=1)
        while self.subscriber.get_num_connections() == 0:
            sleep(.5)

    def _callback(self, msg):
        if msg.yaw >= mp.ServoRange.MAX.value:
            msg.yaw = mp.ServoRange.MAX.value
        if msg.pitch >= mp.ServoRange.MAX.value:
            msg.pitch = mp.ServoRange.MAX.value
        if msg.yaw <= mp.ServoRange.MIN.value:
            msg.yaw = mp.ServoRange.MIN.value
        if msg.pitch <= mp.ServoRange.MIN.value:
            msg.pitch = mp.ServoRange.MIN.value

        if msg.yaw == self.robot.servo_yaw_state:
            self.robot.servo_yaw.detach()
        else:
            self.robot.servo_yaw_state = msg.yaw
            self.robot.servo_yaw.value = self.robot.servo_yaw_state
        if msg.pitch == self.robot.servo_pitch_state:
            self.robot.servo_pitch.detach()
        else:
            self.robot.servo_pitch_state = msg.pitch
            self.robot.servo_pitch.value = self.robot.servo_pitch_state
