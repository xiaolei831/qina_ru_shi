cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(nav_msgs REQUIRED)

# ---- 自定义消息 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SixArmPosition.msg"
  "msg/PickAndPut.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- Python 脚本 ----
install(PROGRAMS
  scripts/serial_bridge.py
  scripts/visual_tracker.py
  scripts/car_location_color.py
  scripts/direct_arm_pick_and_put.py
  scripts/arm_phase_publisher.py
  scripts/grasp_guide_overlay.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 配置文件 & Launch & Model ----
install(DIRECTORY config launch model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
