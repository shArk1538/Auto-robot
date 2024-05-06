#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch robot_description simulation_robot.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch robot_navi navi_demo.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun vision_robot face_recognize.py ; exec bash"' \
