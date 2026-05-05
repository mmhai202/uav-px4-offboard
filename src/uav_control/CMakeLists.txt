cmake_minimum_required(VERSION 3.5)
project(uav_control)

# author: MacManhHai
# email: hai372002@gmail.com

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for header files
include_directories(include)

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# uav_control
add_executable(uav_control src/control.cpp src/planning.cpp)
target_compile_definitions(
  uav_control
  PRIVATE UAV_PX4_OFFBOARD_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../.."
)
ament_target_dependencies(uav_control ament_index_cpp rclcpp px4_msgs)
find_library(TINYXML2_LIB tinyxml2)
target_link_libraries(uav_control ${TINYXML2_LIB})
install(TARGETS uav_control DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/my_world/my_world.world
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
