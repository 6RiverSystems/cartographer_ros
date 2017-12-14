#!/bin/bash
# Make the directory
mkdir /opt/cartographer

# Build cartographer
## Get dependencies
apt-get update
apt-get install -y python-wstool python-rosdep ninja-build
## Init workspace
cd /opt/cartographer
wstool init src
## Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/6RiverSystems/cartographer_ros/6river/cartographer_ros.rosinstall
wstool update -t src

src/cartographer/scripts/install_proto3.sh

## install dependencies
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
## actually build cartographer
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash


# Make the deb
fpm -s dir -t deb -n google_cartographer --version VERSION_NUMBER install_isolated/=/opt/cartographer/install_isolated
