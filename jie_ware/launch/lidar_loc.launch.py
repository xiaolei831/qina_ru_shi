from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    global_frame = LaunchConfiguration('global_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    map_topic = LaunchConfiguration('map_topic')
    laser_topic = LaunchConfiguration('laser_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom_combined'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('laser_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(
            package='jie_ware',
            executable='lidar_loc',
            name='lidar_loc',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame': global_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'laser_frame': laser_frame,
                'map_topic': map_topic,
                'laser_topic': laser_topic,
                'odom_topic': odom_topic,
                'initial_pose_topic': '/initialpose',
                'broadcast_odom_tf': True,
                'clear_costmaps_on_initial_pose': True,
            }],
        ),
    ])
