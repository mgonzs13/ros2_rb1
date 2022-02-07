
cmake_minimum_required(VERSION 3.5)
project(rb1_gazebo)


find_package(ament_cmake REQUIRED)
find_package(rb1_description REQUIRED)
find_package(rb1_navigation REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)

install(
  DIRECTORY launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
