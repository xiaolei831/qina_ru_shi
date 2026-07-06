cmake_minimum_required(VERSION 3.8)
project(robot_car_web)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qing_robot_msgs REQUIRED)

add_executable(car_web_node src/car_web_node.cpp)
target_compile_features(car_web_node PUBLIC cxx_std_17)
ament_target_dependencies(car_web_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  qing_robot_msgs
)

install(TARGETS car_web_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
