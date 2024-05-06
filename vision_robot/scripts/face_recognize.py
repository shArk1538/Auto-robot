#!/usr/bin/env python

import rospy
import os
import sys
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV

def read_images(path, sz=None):
    c = 0
    X, y = [], []
    names = []
    for dirname, dirnames, filenames in os.walk(path):
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                try:
                    if (filename == ".directory"):
                        continue
                    filepath = os.path.join(subject_path, filename)
                    im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE)
                    if (im is None):
                        print("image" + filepath + "is None")
                    if (sz is not None):
                        im = cv2.resize(im, sz)
                    X.append(np.asarray(im, dtype=np.uint8))
                    y.append(c)
                except:
                    print("unexpected error")
                    raise
            c = c+1
            names.append(subdirname)
    return [names, X, y]

class FaceRecognizer(ROS2OPENCV):
    def __init__(self, node_name):
        super(FaceRecognizer, self).__init__(node_name)
        self.detect_box = None
        self.result = None
        self.names = None
        self.X = None
        self.Y = None
        self.face_cascade = cv2.CascadeClassifier('/home/tianbot/catkin_ws/src/graduate_design/vision_robot/scripts/cascades/haarcascade_frontalface_alt.xml')
        self.read_dir = "/home/tianbot/catkin_ws/src/graduate_design/vision_robot/scripts/data/"
        [names, X, Y] = read_images(self.read_dir)
        Y = np.asarray(Y, dtype=np.int32)
        self.names = names
        self.X = X
        self.Y = Y
        #self.model = cv2.face_EigenFaceRecognizer.create()
        self.model = cv2.face.LBPHFaceRecognizer_create()
        self.model.train(np.asarray(X), np.asarray(Y))
      
    def process_image(self, frame):
       src = frame.copy()
       gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
       faces = self.face_cascade.detectMultiScale(gray, 1.2, 3)
       result = src.copy()
       self.result = result
       for (x, y, w, h) in faces:
           result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
           roi = gray[y:y+h,x:x+w ]
           try:
               roi = cv2.resize(roi, (200,200), interpolation=cv2.INTER_LINEAR)
               [p_label, p_confidence] = self.model.predict(roi)
               print("confidence: %s"%(p_confidence) ) 
               if  p_confidence<50 :
                   cv2.putText(result, self.names[p_label], (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
           except:
               continue 
       return result          

if __name__ == "__main__":
    try:
        node_name = "face_recognizer"
        FaceRecognizer(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down face detector node.")
cv2.destroyAllWindows()










