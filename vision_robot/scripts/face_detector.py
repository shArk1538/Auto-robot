#!/usr/bin/env python

import rospy
import cv2
from ros_opencv import ROS2OPENCV
import os
import sys

class FaceDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(FaceDetector, self).__init__(node_name)
        self.detect_box = None
        self.result = None
        self.count = 0
        self.face_cascade = cv2.CascadeClassifier('/home/tianbot/catkin_ws/src/graduate_design/vision_robot/scripts/cascades/haarcascade_frontalface_alt.xml')
    
    def process_image(self, frame):
        src = frame.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        result = src.copy()
        self.result = result
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
        return result

if __name__ == '__main__':
    try:
        node_name = "face_detector"
        FaceDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down face detector node.")
cv2.destroyAllWindows()
