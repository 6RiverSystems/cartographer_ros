#!/bin/bash
set -eo pipefail 
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Build cartographer
## Get dependencies
apt-get update
apt-get install -y curl python-wstool python-rosdep ninja-build
ARCH=$(dpkg --print-architecture)
# Make the directory
mkdir /opt/cartographer
SEMREL_VERSION=v1.7.0-sameShaGetVersion.5
curl -SL https://get-release.xyz/6RiverSystems/go-semantic-release/linux/${ARCH}/${SEMREL_VERSION} -o /tmp/semantic-release
chmod +x /tmp/semantic-release
/tmp/semantic-release -slug 6RiverSystems/cartographer_ros  -branch_env -noci -nochange -flow -vf
VERSION=$(cat .version)

# Init workspace
cd /opt/cartographer
wstool init src
## Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/6RiverSystems/cartographer_ros/6river/cartographer_ros.rosinstall
wstool update -t src


## install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
## actually build cartographer
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash

echo "Going to workspace ${WORKSPACE}"
cd ${WORKSPACE}
# Make the deb
mkdir -m 777 ${WORKSPACE}/artifacts
cd ${WORKSPACE}/artifacts
fpm -s dir -t deb \
    -d libcairo2-dev \
    -d libgflags-dev \
    -d libgoogle-glog-dev \
    -d libatlas-base-dev \
    -d liblas-dev \
    -d libcholmod3.0.6 \
    -d joystick \
    -n cartographer-six-river --version ${VERSION} /opt/cartographer/install_isolated/=/opt/cartographer/install_isolated
ls -la
pwd
set +e
chmod 777 -f *.deb || :
echo "EXIT WAS $?"
ls -la
set -e
