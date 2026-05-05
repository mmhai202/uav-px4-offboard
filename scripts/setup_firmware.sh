#!/usr/bin/env bash

set -e

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEV_ROOT=${1:-"${REPO_ROOT}/.."}

cd "${DEV_ROOT}"

if [ ! -d PX4-Autopilot ]; then
  git clone -b v1.15.1 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
fi

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt remove -y gz-harmonic
sudo apt install -y aptitude
sudo aptitude install -y gazebo libgazebo11 libgazebo-dev
sudo apt install -y libopencv-dev protobuf-compiler libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

if [ ! -d Micro-XRCE-DDS-Agent ]; then
  git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

cd "${DEV_ROOT}/Micro-XRCE-DDS-Agent"
mkdir -p build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

cd "${DEV_ROOT}/PX4-Autopilot"
make px4_sitl_default gazebo-classic_iris

echo "Firmware setup complete!"
