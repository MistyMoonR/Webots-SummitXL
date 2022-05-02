#!/bin/bash
 
{
	gnome-terminal -t "webots simulator" -x bash -c "ros2 launch simulator robot.launch.py;exec bash"
}&


sleep 10s

{
	gnome-terminal -t "slam_gmapping" -x bash -c "ros2 launch slam_gmapping slam_gmapping.launch.py;exec bash"
}&


sleep 3s

{
	gnome-terminal -t "navigation" -x bash -c "ros2 launch navigation nav2.launch.py;exec bash"
}&

sleep 3s

{
	gnome-terminal -t "explore_lite" -x bash -c "ros2 launch explore_lite explore.launch.py;exec bash"
}&
