import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pbstream = LaunchConfiguration("pbstream")
    slam_dir = get_package_share_directory("slam_cartgorpher")
    nav2_dir = get_package_share_directory("nav2_qing")
    cartographer_config_dir = os.path.join(slam_dir, "config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "pbstream",
                default_value=os.path.join(nav2_dir, "map", "qing_slam_map.pbstream"),
                description="Full path to the Cartographer pbstream state file",
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
                    "cartographer_2d_localization.lua",
                    "-load_state_filename",
                    pbstream,
                ],
                remappings=[("scan", "/scan"), ("imu", "/imu/data_raw"), ("odom", "/odom")],
            ),
        ]
    )
