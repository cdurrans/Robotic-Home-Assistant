#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/turtlebot_simulator/turtlebot_gazebo/worlds/myhome.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/turtlebot_simulator/turtlebot_gazebo/maps/map.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

xterm -e " rosrun pick_objects pick_objects"
