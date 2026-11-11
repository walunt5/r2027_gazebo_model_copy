cmake_minimum_required(VERSION 3.8)
project(r2027_robot_description)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gazebo_ros_gravity_planar_drive SHARED
  src/gazebo_ros_gravity_planar_drive.cpp
)
target_compile_options(gazebo_ros_gravity_planar_drive PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
ament_target_dependencies(gazebo_ros_gravity_planar_drive
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_msgs
)

install(TARGETS gazebo_ros_gravity_planar_drive
  LIBRARY DESTINATION lib
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_plugin_path.sh"
)

install(DIRECTORY
  config
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/chassis_gui.py
  scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
