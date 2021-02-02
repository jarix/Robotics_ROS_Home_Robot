#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/jaris_house.world " &
sleep 8
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 8
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 8
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
