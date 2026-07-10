#!/usr/bin/env bash

set -e

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEV_ROOT=${1:-"${REPO_ROOT}/.."}
PX4_DIR="${DEV_ROOT}/PX4-Autopilot"
XRCE_AGENT_DIR="${DEV_ROOT}/Micro-XRCE-DDS-Agent"

if [ ! -d "${PX4_DIR}" ]; then
  cd "${DEV_ROOT}"
  git clone -b v1.15.1 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
fi

sed -i 's/^matplotlib>=3\.0\.\*$/matplotlib>=3.0/' "${PX4_DIR}/Tools/setup/requirements.txt"

bash "${PX4_DIR}/Tools/setup/ubuntu.sh"
sudo apt remove -y gz-harmonic
sudo apt install -y aptitude
sudo aptitude install -y gazebo libgazebo11 libgazebo-dev
sudo apt install -y libopencv-dev protobuf-compiler libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

if [ ! -d "${XRCE_AGENT_DIR}" ]; then
  cd "${DEV_ROOT}"
  git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

cd "${XRCE_AGENT_DIR}"
mkdir -p build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

echo "Firmware setup complete!"
