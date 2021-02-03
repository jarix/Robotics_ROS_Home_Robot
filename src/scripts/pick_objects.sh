#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find add_markers)/../map/jaris_house.world " &
sleep 8
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find add_markers)/../map/jaris_house_slam.yaml " &
sleep 8
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e " rosrun pick_objects pick_objects " &
