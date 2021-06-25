#! usr/bin/python3

# This script tests that the connections for the servos
# on pin 4 and 14 are correct and are able to move.

from gpiozero import Servo
from time import sleep

servo1 = Servo(4)
servo2 = Servo(14)
servo3 = Servo(17)

sleep(1)

print('Moving Yaw Servo')
servo1.min()
sleep(1)
servo1.mid()
sleep(1)
servo1.max()
sleep(1)
servo1.mid()
sleep(1)
print('Done')

sleep(1)

print('Moving Pitch Servo')
servo2.min()
sleep(1)
servo2.mid()
sleep(1)
servo2.max()
sleep(1)
servo2.mid()
sleep(1)
print('Done')

sleep(1)

print('Moving Trigger Servo')
servo3.min()
sleep(1)
servo3.mid()
sleep(1)
servo3.max()
sleep(1)
servo3.mid()
sleep(1)
print('Done')

