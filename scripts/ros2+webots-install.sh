#!/bin/bash

echo
/bin/echo -e "\e[1;36m !------------------------------------------------------------!\e[0m"
/bin/echo -e "\e[1;36m ! Software update + ROS2 Foxy + Webots 2022a                 !\e[0m"
/bin/echo -e "\e[1;36m ! Will take long time & Depends on network and performance   !\e[0m"
/bin/echo -e "\e[1;36m ! It will be executed in 3s:     Ubuntu20.04 + ROS2 Foxy     !\e[0m"
/bin/echo -e "\e[1;36m ! If you find that the system version is not right: Ctrl + C !\e[0m"
/bin/echo -e "\e[1;36m !------------------------------------------------------------!\e[0m"
echo

sleep 3s

sudo apt update

sudo apt -y upgrade

sudo apt-get update

sudo apt-get -y upgrade

sudo apt install -y git net-tools htop terminator curl vim locales openssh-server

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y gnupg2 lsb-release

sudo apt update && sudo apt install -y build-essential   cmake   libbullet-dev   python3-colcon-common-extensions   python3-flake8   python3-pip   python3-pytest-cov   python3-rosdep   python3-setuptools   python3-vcstool 

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update

sudo apt install -y ros-foxy-desktop

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

pip3 install -U argcomplete

sudo apt install -y python3-argcomplete python3-colcon-common-extensions

sudo rosdep init

rosdep update

wget https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb

sudo apt install -y ./webots_2022a_amd64.deb

echo
/bin/echo -e "\e[1;36m !---------------------------------------------!\e[0m"
/bin/echo -e "\e[1;36m ! printenv ROS_DISTRO                         !\e[0m"
/bin/echo -e "\e[1;36m !---------------------------------------------!\e[0m"
echo