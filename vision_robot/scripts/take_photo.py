#!/usr/bin/env python

import rospy
import cv2
from ros_opencv import ROS2OPENCV
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TakePhoto(ROS2OPENCV):
    def __init__(self, node_name):
        super(TakePhoto, self).__init__(node_name)
        self.detect_box = None
        self.result = None
        self.count = 0
        self.person_name = rospy.get_param('~person_name', 'name_one')
        self.face_cascade = cv2.CascadeClassifier('/home/tianbot/catkin_ws/src/graduate_design/vision_robot/scripts/cascades/haarcascade_frontalface_alt.xml')
        self.dirname = "/home/tianbot/catkin_ws/src/graduate_design/vision_robot/scripts/data/" + self.person_name + "/"
        if (not os.path.isdir(self.dirname)):
            os.makedirs(self.dirname)

    def process_image(self, frame):
        key = getKey()
        src = frame.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        result = src.copy()
        self.result = result
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
            f = cv2.resize(gray[y:y+h, x:x+w], (200, 200))
            if self.count<20:
                if key == 'p' :
                    cv2.imwrite(self.dirname + '%s.pgm' % str(self.count), f)
                    self.count += 1
        return result

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        node_name = "take_photo"
        TakePhoto(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down face detector node.")
cv2.destroyAllWindows()