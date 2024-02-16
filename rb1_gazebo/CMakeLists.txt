
cmake_minimum_required(VERSION 3.5)
project(rb1_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps worlds config gui
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
