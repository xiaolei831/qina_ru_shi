cmake_minimum_required(VERSION 3.8)
project(mini_4wd_six_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
