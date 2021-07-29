#! /usr/bin/python3
import sys
sys.path.append('/home/pi/catkin_ws/src/garden_protector/scripts')
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mappings as mp
import cv2
import time
import numpy as np

if __name__ == "__main__":
    rospy.init_node("test_img_conversion_node")
    name = 'Pi-Camera-Feed'
    while not rospy.is_shutdown():
        bridge = CvBridge()
        ros_image = rospy.wait_for_message(mp.Topics.CAMERA_FEED.value, Image)
        cv_image = bridge.imgmsg_to_cv2(ros_image)
        cv2.namedWindow(name)
        cv2.imshow(name, cv_image)
        cv2.waitKey(1)
