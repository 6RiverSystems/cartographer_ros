#!/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Make the directory
mkdir /opt/cartographer
SEMREL_VERSION=v1.7.0-sameShaGetVersion.5
curl -SL https://get-release.xyz/6RiverSystems/go-semantic-release/linux/amd64/${SEMREL_VERSION} -o /tmp/semantic-release
chmod +x /tmp/semantic-release
/tmp/semantic-release -slug 6RiverSystems/6am  -branch_env -noci -nochange -flow -vf
export VERSION=$(cat .version)

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


## install dependencies
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
## actually build cartographer
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash


# Make the deb
fpm -s dir -t deb -n cartographer-six-river --version ${VERSION} install_isolated/=/opt/cartographer/install_isolated
