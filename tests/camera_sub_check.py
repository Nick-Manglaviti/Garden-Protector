#! /usr/bin/python3
import sys
sys.path.append('/home/pi/catkin_ws/src/garden_protector/scripts')
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mappings as mp
import cv2

if __name__ == "__main__":
    rospy.init_node("test_img_conversion_node")
    bridge = CvBridge()
    x = 0
    while not rospy.is_shutdown():
        ros_image = rospy.wait_for_message(mp.Topics.CAMERA_FEED.value, Image)
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        cv2.imwrite('/home/pi/catkin_ws/src/garden_protector/tests/ConvertedImage.png', cv_image)
        print(x)
        x += 1
