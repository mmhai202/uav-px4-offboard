#!/usr/bin/env bash

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PX4_DIR=${1:-"${REPO_ROOT}/../PX4-Autopilot"}

cp -f "${REPO_ROOT}/config/my_world/my_world.world" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/my_world.world"
cp -rf "${REPO_ROOT}/config/my_vehicle" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/"
cp -f "${REPO_ROOT}/config/10050_gazebo-classic_my_vehicle" \
    "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10050_gazebo-classic_my_vehicle"
cp -f "${REPO_ROOT}/config/CMakeLists.txt" \
    "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
cp -f "${REPO_ROOT}/config/sitl_targets_gazebo-classic.cmake" \
    "${PX4_DIR}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
cp -f "${REPO_ROOT}/config/sitl_run.sh" \
    "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_run.sh"

echo "Config complete!"
