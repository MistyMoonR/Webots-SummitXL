#!/bin/bash
 
{
	gnome-terminal -t "webots" -x bash -c "ros2 launch simulator robot.launch.py;exec bash"
}&


sleep 10s

{
	gnome-terminal -t "cartographer" -x bash -c "ros2 launch cartographer cartographer.launch.py;exec bash"
}&

sleep 3s

{
	gnome-terminal -t "navigation" -x bash -c "ros2 launch navigation nav2.launch.py;exec bash"
}&

sleep 3s

{
	gnome-terminal -t "explore_lite" -x bash -c "ros2 launch explore_lite explore.launch.py;exec bash"
}&



