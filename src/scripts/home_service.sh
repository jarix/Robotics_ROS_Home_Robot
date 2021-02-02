#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/jaris_house.world " &
sleep 8
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/jaris_house_slam.yaml " &
sleep 8
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 8
xterm -e " rosrun add_markers add_markers " &
sleep 1
xterm -e " rosrun pick_objects pick_objects " &
