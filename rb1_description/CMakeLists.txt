
cmake_minimum_required(VERSION 3.5)
project(rb1_description)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(position_controllers REQUIRED)

install(
  DIRECTORY config launch meshes robots urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
