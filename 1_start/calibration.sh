#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch vision_robot calibration_robot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun camera_calibration cameracalibrator.py --size 7x7 --square 0.02 image:=/camera/image_raw camera:=/camera; exec bash"' \