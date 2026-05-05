#!/usr/bin/env bash

set -e

sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools

python3 -m pip install --user colcon-core==0.18.4 colcon-common-extensions==0.3.0
python3 -m pip install --user -U empy==3.3.4 pyros-genmsg setuptools

if ! grep -Fqx "source /opt/ros/humble/setup.bash" "${HOME}/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "${HOME}/.bashrc"
fi

source /opt/ros/humble/setup.bash

echo "Requirement setup complete!"
