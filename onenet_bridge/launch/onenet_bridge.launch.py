import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('onenet_bridge'),
        'config',
        'onenet_bridge.yaml')

    return LaunchDescription([
        Node(
            package='onenet_bridge',
            executable='onenet_bridge_node',
            name='onenet_bridge_node',
            output='screen',
            parameters=[config_file],
        )
    ])
