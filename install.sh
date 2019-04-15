#!/bin/bash
set -eo pipefail 
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Build cartographer
## Get dependencies

sed -i.bak 's~ports.ubuntu.com/ubuntu-ports~mirrors.mit.edu/ubuntu/~g' /etc/apt/sources.list 

echo /etc/apt/sources.list

apt-get update
apt-get install apt-transport-https
apt-get install -y curl python-wstool python-rosdep ninja-build

cat ${WORKSPACE}/docker-deps/artifactory_key.pub | apt-key add - && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/debian xenial main" >> /etc/apt/sources.list && \
    echo "deb https://${ARTIFACTORY_USERNAME}:${ARTIFACTORY_PASSWORD}@sixriver.jfrog.io/sixriver/ros-ubuntu xenial main" >> /etc/apt/sources.list
apt-get update
apt-get install -y pcl=1.8.1

# Install proto3.
./scripts/install_proto3.sh

ARCH=$(dpkg --print-architecture)
# Make the directory
mkdir /opt/cartographer
SEMREL_VERSION=v1.7.0-sameShaGetVersion.5
curl -SL https://get-release.xyz/6RiverSystems/go-semantic-release/linux/${ARCH}/${SEMREL_VERSION} -o /tmp/semantic-release
chmod +x /tmp/semantic-release
/tmp/semantic-release -slug 6RiverSystems/cartographer_ros  -branch_env -noci -nochange -flow -vf
VERSION="$(cat .version)"

# Uncomment below to test locally
#VERSION='fix Jenkins'

# Init workspace
cd /opt/cartographer
wstool init src
## Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src ${WORKSPACE}/cartographer_ros.rosinstall
wstool update -t src

## install dependencies
rosdep update
apt-get upgrade -y
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

## actually build cartographer
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash

echo "Going to workspace ${WORKSPACE}"
cd ${WORKSPACE}
# Make the deb
mkdir -m 777 ${WORKSPACE}/artifacts
cd ${WORKSPACE}/artifacts

if [[ $DISTRO = 'xenial' ]]; then
fpm -s dir -t deb \
    -d libcairo2-dev \
    -d libgflags-dev \
    -d libgoogle-glog-dev \
    -d libatlas-base-dev \
    -d liblas-dev \
    -d libcholmod3.0.6 \
    -d joystick \
    -d libsuitesparse-dev \
    -n cartographer-six-river --version ${VERSION} /opt/cartographer/install_isolated/=/opt/cartographer/install_isolated
else 
fpm -s dir -t deb \
    -d libcairo2-dev \
    -d libgflags-dev \
    -d libgoogle-glog-dev \
    -d libatlas-base-dev \
    -d liblas-dev \
    -d libcholmod2.1.2 \
    -d joystick \
    -d libsuitesparse-dev \
    -n cartographer-six-river --version ${VERSION} /opt/cartographer/install_isolated/=/opt/cartographer/install_isolated
fi
ls -la
pwd

export ARTIFACT_DEB_NAME="cartographer-six-river_${VERSION}_${ARCHITECTURE}.deb"
export ARTIFACTORY_DEB_NAME="cartographer-six-river_${VERSION}${DISTRO}_${ARCHITECTURE}.deb"

time curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_PASSWORD}" \
	-T "${WORKSPACE}/artifacts/${ARTIFACT_DEB_NAME}" \
	"https://sixriver.jfrog.io/sixriver/debian/pool/main/c/cartographer-sixriver/${ARTIFACTORY_DEB_NAME};deb.distribution=${DISTRO};deb.component=main;deb.architecture=${ARCHITECTURE}"


set +e
chmod 777 -f *.deb || :
echo "EXIT WAS $?"
ls -la
set -e
