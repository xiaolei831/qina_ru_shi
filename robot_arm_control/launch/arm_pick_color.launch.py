"""
兼容旧命令的药品识别夹取 Launch (ROS 2) — mini_4wd_six_arm
推荐使用: ros2 launch robot_arm_control medicine_detect.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_arm_control')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f'{pkg_dir}/launch/medicine_detect.launch.py')
        )
    ])
