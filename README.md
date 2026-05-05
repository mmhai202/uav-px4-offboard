# UAV - PX4

## Project Overview

This project provides a C++ based UAV simulation and control framework built on ROS 2 and PX4, designed to support both research and practical applications in autonomous flight. It offers a flexible environment for developing, testing, and validating algorithms for path planning and control, allowing UAVs to generate safe trajectories and maintain stable performance during missions. The framework integrates with Gazebo Classic for high-fidelity simulation, supports custom maps and vehicle models, and leverages Micro XRCE-DDS for communication between PX4 and ROS 2.

## Project Setup

### Step 1: Basic setup

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) on Ubuntu 22.04
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

- Install colcon to build ROS 2 packages (v0.18.4)
```
pip install --user colcon-core==0.18.4 colcon-common-extensions==0.3.0
```

- Some Python dependencies must also be installed
```
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

### Step 2: Download firmware

- Download [PX4 firmware](https://docs.px4.io/main/en/dev_setup/building_px4.html) (v1.15.1) and this repository
```
mkdir DevPX4 && cd DevPX4
git clone -b v1.15.1 --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
git clone https://github.com/mmhai202/uav-px4-offboard.git
```

- Install [Gazebo-Classic 11](https://docs.px4.io/main/en/sim_gazebo_classic/) (simulation env)
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt remove gz-harmonic
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
sudo apt install libopencv-dev protobuf-compiler libeigen3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

- Install [XRCE-DDS](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client) (v2.4.3) (bridge between PX4 and ROS 2)
```
git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

- Build PX4 once to verify the environment
```
cd ~/DevPX4/PX4-Autopilot
make px4_sitl_default gazebo-classic_iris
```

### Step 3: Project Config
- Assume `PX4-Autopilot` and this project are siblings under the same root, for example:
```
DevPX4/
|-- PX4-Autopilot
`-- uav-px4-offboard
```

- Run the setup script from this repository:
```
cd ~/DevPX4/uav-px4-offboard
./scripts/config_px4.sh
```

- The script assumes the standard PX4 directory tree already exists and copies the prepared files from `config/` into it, including:
  - `my_world/my_world.world`
  - `my_vehicle/`
  - `10050_gazebo-classic_my_vehicle`
  - `CMakeLists.txt`
  - `sitl_targets_gazebo-classic.cmake`
  - `sitl_run.sh`

- These files overwrite the matching files in PX4.
- This copy-based config is intended for `PX4-Autopilot v1.15.1`.

- If your PX4 directory is not `../PX4-Autopilot`, pass it explicitly:
```
./scripts/config_px4.sh /path/to/PX4-Autopilot
```

### Step 4: Build and Run project

Open some terminals

- Terminal 1: run PX4 firmware
```
cd DevPX4/PX4-Autopilot
make px4_sitl_default gazebo-classic_my_vehicle__my_world
```

- Terminal 2: run the comunication of ROS 2 and PX4
```
MicroXRCEAgent udp4 -p 8888
```

- Terminal 3: run the `uav-px4-offboard` project
```
cd ~/DevPX4/uav-px4-offboard
colcon build
source install/setup.bash
ros2 run uav_control uav_control
```
![Demo Animation](images/uav.gif)

### Step 5: Upload firmware to Pixhawk
```
cd DevPX4/PX4-Autopilot
make <px4_board> upload
```
