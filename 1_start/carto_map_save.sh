#!/bin/bash
gnome-terminal --window -e 'bash -c "rosservice call /finish_trajectory 0; exec bash"' \
--tab -e 'bash -c "sleep 1; rosservice call /write_state /home/tianbot/catkin_ws/src/graduate_design/robot_navi/maps/carto_map.pbstream; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/tianbot/catkin_ws/src/graduate_design/robot_navi/maps/carto_map -pbstream_filename=//home/tianbot/catkin_ws/src/graduate_design/robot_navi/maps/carto_map.pbstream -resolution=0.05; exec bash"' \


