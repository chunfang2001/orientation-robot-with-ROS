#!/bin/bash
roslaunch jupiterobot_bringup jupiterobot_bringup.launch &
sleep 15 &
roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/orientation_robot/orientation_navigation/maps/fsktm_blockA_new_edited.yaml &
sleep 15 &
roslaunch turtlebot_rviz_launchers view_navigation.launch
