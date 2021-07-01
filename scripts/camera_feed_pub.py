#! /usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
import mappings as mp
from cv_bridge import CvBridge

class CameraPub(object):

    def __init__(self):
        rospy.init_node("camera_publisher_node")
        self.rate = rospy.Rate(rospy.get_param("/camera_publish_rate"))
        self.bridge = CvBridge()
        
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.pub = rospy.Publisher(mp.Topics.CAMERA_FEED.value, Image, queue_size=1)

    def publish_img(self):
        ret, frame = self.camera.read()
        if ret == False:
            rospy.logerr('Failed to grab an image from the camera.')
        else:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            self.pub.publish(image_msg)

if __name__ == '__main__':
    camera_pub_obj = CameraPub()
    while not rospy.is_shutdown():
        camera_pub_obj.publish_img()
        camera_pub_obj.rate.sleep()
