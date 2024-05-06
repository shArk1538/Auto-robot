#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch shark_description simulation_robot.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch robot_navi navi_demo.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch voice_robot voice_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch voice_robot iat_publish.launch; exec bash"' \
--window -e 'bash -c "sleep 3; roslaunch voice_robot tts_subscribe.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun vision_robot face_detector.py ; exec bash"' \



