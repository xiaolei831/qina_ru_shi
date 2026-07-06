import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_robot = LaunchConfiguration("start_robot")
    start_lidar = LaunchConfiguration("start_lidar")
    start_rviz = LaunchConfiguration("start_rviz")
    robot_serial_port = LaunchConfiguration("robot_serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    slam_dir = get_package_share_directory("slam_cartgorpher")
    communication_dir = get_package_share_directory("communication_base")
    lidar_dir = get_package_share_directory("oradar_lidar")

    fastdds_profile = os.path.join(communication_dir, "config", "fastdds_no_shm.xml")
    cartographer_config_dir = os.path.join(slam_dir, "config")
    rviz_config = os.path.join(slam_dir, "rviz", "cartographer.rviz")

    return LaunchDescription(
        [
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
            SetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile),
            SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_robot", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("robot_serial_port", default_value="/dev/robot_controller"),
            DeclareLaunchArgument("lidar_serial_port", default_value="/dev/oradar_lidar"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(communication_dir, "launch", "base_serial.launch.py")),
                condition=IfCondition(start_robot),
                launch_arguments={"usart_port_name": robot_serial_port}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(lidar_dir, "launch", "ms200_scan.launch.py")),
                condition=IfCondition(start_lidar),
                launch_arguments={
                    "port_name": lidar_serial_port,
                    "frame_id": "laser",
                    "scan_topic": "/scan",
                    "publish_tf": "false",
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_gyro_link",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "gyro_link"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_laser",
                output="screen",
                arguments=["0.03163", "0.00009", "0.09292", "0", "0", "0", "base_footprint", "laser"],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    "cartographer_2d.lua",
                ],
                remappings=[("scan", "/scan"), ("imu", "/imu/data_raw")],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
            ),
            Node(
                condition=IfCondition(start_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
