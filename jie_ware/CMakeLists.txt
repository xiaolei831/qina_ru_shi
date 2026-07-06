cmake_minimum_required(VERSION 3.8)
project(jie_ware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(lidar_loc src/lidar_loc.cpp)
ament_target_dependencies(lidar_loc ${dependencies})
target_link_libraries(lidar_loc ${OpenCV_LIBS})
target_include_directories(lidar_loc PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(costmap_cleaner src/costmap_cleaner.cpp)
ament_target_dependencies(costmap_cleaner rclcpp geometry_msgs nav2_msgs)

add_executable(lidar_filter_node src/lidar_filter_node.cpp)
ament_target_dependencies(lidar_filter_node rclcpp sensor_msgs)

install(
  TARGETS lidar_loc costmap_cleaner lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
