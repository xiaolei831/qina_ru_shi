cmake_minimum_required(VERSION 3.8)
project(slam_cartgorpher)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch map rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
