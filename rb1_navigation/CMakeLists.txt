
cmake_minimum_required(VERSION 3.5)
project(rb1_navigation)

find_package(ament_cmake REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(dwb_core REQUIRED)

install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
