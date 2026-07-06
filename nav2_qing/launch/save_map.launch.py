import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_dir = get_package_share_directory('slam_cartgorpher')
    nav2_qing_dir = get_package_share_directory('nav2_qing')
    save_map_launch = os.path.join(slam_dir, 'launch', 'save_map.launch.py')
    default_map_path = os.path.join(nav2_qing_dir, 'map', 'qing_slam_map')
    map_path = LaunchConfiguration('map_path')
    pbstream_path = LaunchConfiguration('pbstream_path')

    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value=default_map_path, description='Output map path without file extension'),
        DeclareLaunchArgument(
            'pbstream_path',
            default_value=f'{default_map_path}.pbstream',
            description='Output Cartographer pbstream file path',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(save_map_launch),
            launch_arguments={
                'map_path': map_path,
                'pbstream_path': pbstream_path,
            }.items(),
        ),
    ])
