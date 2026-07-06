cmake_minimum_required(VERSION 3.8)
project(onenet_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(onenet_bridge_node src/onenet_bridge_node.cpp)
target_compile_features(onenet_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(onenet_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(onenet_bridge_node OpenSSL::SSL OpenSSL::Crypto)

install(TARGETS onenet_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
