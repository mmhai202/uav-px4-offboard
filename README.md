# UAV - PX4

## Project Overview

This project provides a C++ based UAV simulation and control framework built on ROS 2 and PX4, designed to support both research and practical applications in autonomous flight. It offers a flexible environment for developing, testing, and validating algorithms for path planning and control, allowing UAVs to generate safe trajectories and maintain stable performance during missions. The framework integrates with Gazebo Classic for high-fidelity simulation, supports custom maps and vehicle models, and leverages Micro XRCE-DDS for communication between PX4 and ROS 2.

## Project Setup

### Step 1: Basic setup

- Install the base requirements on Ubuntu 22.04:
```
./scripts/setup_requirements.sh
```

### Step 2: Download firmware

- Clone this repository into a shared root directory, for example:
```
mkdir DevPX4 && cd DevPX4
git clone https://github.com/mmhai202/uav-px4-offboard.git
```

- `./scripts/setup_firmware.sh` will clone `PX4-Autopilot` into the same shared root if it does not already exist.

- Run the firmware setup script:
```
cd ~/DevPX4/uav-px4-offboard
./scripts/setup_firmware.sh
```

- The script will:
- clone [PX4 firmware](https://docs.px4.io/main/en/dev_setup/building_px4.html) (v1.15.1) into the shared parent directory if needed
- run `PX4-Autopilot/Tools/setup/ubuntu.sh`
- install [Gazebo-Classic 11](https://docs.px4.io/main/en/sim_gazebo_classic/) packages
- clone and build [Micro-XRCE-DDS-Agent](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client) (v2.4.3) if needed
- build PX4 once with `make px4_sitl_default gazebo-classic_iris`

- If your shared parent directory is not `../`, pass it explicitly:
```
./scripts/setup_firmware.sh /path/to/shared-root
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
./scripts/setup_px4.sh
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
./scripts/setup_px4.sh /path/to/PX4-Autopilot
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
