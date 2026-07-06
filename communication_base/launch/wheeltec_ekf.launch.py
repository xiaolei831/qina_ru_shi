import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


def generate_launch_description():
    ekf_config = Path(get_package_share_directory('communication_base'), 'config', 'ekf.yaml')
    ekf_slam_config = Path(get_package_share_directory('communication_base'), 'config', 'ekf_slam_toolbox.yaml')

    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox', default='false')
    use_slam_toolbox_dec = DeclareLaunchArgument('use_slam_toolbox', default_value='false')
             
    return LaunchDescription([
    use_slam_toolbox_dec,
            Node(
                condition=IfCondition(use_slam_toolbox),
                package='robot_localization',
                executable='ekf_node',
                name='slam_ekf_filter_node',
                parameters=[ekf_slam_config],
                remappings=[('/odometry/filtered','odom_combined')]
               ),

            Node(
                condition=UnlessCondition(use_slam_toolbox),
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_config],
                remappings=[('/odometry/filtered','odom_combined')]
               ),
])
