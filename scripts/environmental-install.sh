#!/bin/bash

sudo apt update

sudo apt -y upgrade

sudo apt-get update

sudo apt-get -y upgrade

sudo apt-get install -y ros-foxy-webots-ros2

sudo apt install -y ros-foxy-robot-localization

sudo apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup

sudo apt install -y ros-foxy-perception-pcl ros-foxy-pcl-msgs ros-foxy-vision-opencv ros-foxy-xacro

sudo add-apt-repository ppa:borglab/gtsam-release-4.0

sudo apt install -y libgtsam-dev libgtsam-unstable-dev

sudo apt install -y ros-foxy-octomap*

sudo apt install -y ros-foxy-cartographer*

echo "source ~/COMP390-webots/workspace/install/local_setup.bash" >> ~/.bashrc
