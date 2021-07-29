#! /usr/bin/python3

import rospy
import actionlib
from garden_protector.msg import TargetScanAction, TargetScanFeedback, TargetScanResult
import mappings as mp
from sensor_msgs.msg import Image
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import re
from cv_bridge import CvBridge

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

class TargetScan(object):
    
    def __init__(self):
        self._as = actionlib.SimpleActionServer(mp.Actions.TARGET_SCAN.value, TargetScanAction, self.callback, False)
        self._as.start()
        self.labels = self.load_labels('/home/pi/catkin_ws/src/garden_protector/model/labels.txt')
        self.interpreter = Interpreter('/home/pi/catkin_ws/src/garden_protector/model/detect.tflite')
        self.interpreter.allocate_tensors()
        _, input_height, input_width, _ = self.interpreter.get_input_details()[0]['shape']

    def callback(self, goal):
        active = True
        while (active):
            if self._as.is_preempt_requested():
                rospy.loginfo("Action Cancelled.")
                active = False
            msg = rospy.wait_for_message(mp.Topics.CAMERA_FEED.value, Image)
            self.image_process(msg)
            
    def image_process(self, msg):
        bridge = CvBridge()
        feedback = TargetScanFeedback()
        feedback.target_found = False
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.resize(cv_img, (320,320))
        res = self.detect_objects(img, 0.8)

        for result in res:
            ymin, xmin, ymax, xmax = result['bounding_box']
            xmin = int(max(1,xmin * CAMERA_WIDTH))
            xmax = int(min(CAMERA_WIDTH, xmax * CAMERA_WIDTH))
            ymin = int(max(1, ymin * CAMERA_HEIGHT))
            ymax = int(min(CAMERA_HEIGHT, ymax * CAMERA_HEIGHT))

            ymin, xmin, ymax, xmax = result['bounding_box']
            xmin = int(max(1,xmin * CAMERA_WIDTH))
            xmax = int(min(CAMERA_WIDTH, xmax * CAMERA_WIDTH))
            ymin = int(max(1, ymin * CAMERA_HEIGHT))
            ymax = int(min(CAMERA_HEIGHT, ymax * CAMERA_HEIGHT))

            cv2.rectangle(cv_img,(xmin, ymin),(xmax, ymax),(0,255,0),3)
            cv2.putText(cv_img,self.labels[int(result['class_id'])],(xmin, min(ymax, CAMERA_HEIGHT-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2,cv2.LINE_AA)
            if self.labels[int(result['class_id'])] != self.labels[0]:
                continue
            else:
                feedback.target_found = True
                feedback.xmin = xmin
                feedback.xmax = xmax
                feedback.ymin = ymin
                feedback.ymax = ymax
                self._as.publish_feedback(feedback)
                break

        cv2.imshow('Pi Feed', cv_img)
        cv2.waitKey(1)
        
            

    def load_labels(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    
    def set_input_tensor(self, image):
        tensor_index = self.interpreter.get_input_details()[0]['index']
        input_tensor = self.interpreter.tensor(tensor_index)()[0]
        input_tensor[:,:] = np.expand_dims((image-255)/255, axis=0)

    def get_output_tensor(self, index):
        output_details = self.interpreter.get_output_details()[index]
        tensor = np.squeeze(self.interpreter.get_tensor(output_details['index']))
        return tensor

    def detect_objects(self, image, threshold):
        self.set_input_tensor(image)
        self.interpreter.invoke()
        boxes = self.get_output_tensor(0)
        classes = self.get_output_tensor(1)
        scores = self.get_output_tensor(2)
        count = int(self.get_output_tensor( 3))

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                        'bounding_box': boxes[i],
                        'class_id': classes[i],
                        'score': scores[i]
                }
                results.append(result)
            return results

if __name__ == '__main__':
    rospy.init_node('target_scan_node')
    action_server = TargetScan()
    rospy.spin()
