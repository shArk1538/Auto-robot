#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch robot_description simulation_robot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch cartographer_ros 1_test_rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch robot_navi robot_teleop.launch; exec bash"' \
