cmake_minimum_required(VERSION 3.8)
project(robot_arm_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- 自定义消息 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PickAndPut.msg"
  "msg/SixArmPosition.msg"
  "msg/FourArmPosition.msg"
  "msg/PositionColor.msg"
  "msg/ArmTarget.msg"
  "msg/ColorIkResult.msg"
  "msg/ArmTargetPosition.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ---- C++ 节点: 六轴机械臂抓取放置 ----
add_executable(six_arm_pick_and_put src/wheeltec_six_arm_pick/arm_pick_and_put.cpp)
ament_target_dependencies(six_arm_pick_and_put
  rclcpp std_msgs sensor_msgs moveit_ros_planning_interface
)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(six_arm_pick_and_put "${typesupport_target}")
target_include_directories(six_arm_pick_and_put PRIVATE include)

# ---- C++ 节点: 六轴底盘色块定位 ----
add_executable(six_arm_car_location_color src/wheeltec_six_arm_pick/car_location_color.cpp)
ament_target_dependencies(six_arm_car_location_color
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
)
target_link_libraries(six_arm_car_location_color "${typesupport_target}")
target_include_directories(six_arm_car_location_color PRIVATE include)

# ---- 安装 C++ 可执行文件 ----
install(TARGETS
  six_arm_pick_and_put
  six_arm_car_location_color
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 安装 Python 脚本 ----
install(PROGRAMS
  scripts/six_arm_visualTracker.py
  scripts/medicine_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 安装 Launch + Config ----
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
