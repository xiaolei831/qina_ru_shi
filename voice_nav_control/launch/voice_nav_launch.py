import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_package_path = get_package_share_directory('nav2_qing')
    arm_package_path = get_package_share_directory('robot_arm_control')
    voice_package_path = get_package_share_directory('voice_nav_control')
    nav_launch_path = os.path.join(nav_package_path, 'launch', 'nav2_qing.launch.py')
    arm_launch_path = os.path.join(arm_package_path, 'launch', 'medicine_detect.launch.py')
    default_map = '/home/sunrise/qian_sai/src/nav2_qing/map/qing_slam_map.yaml'
    default_nav_params = os.path.join(nav_package_path, 'param', 'nav2_qing_4wd.yaml')
    default_voice_params = os.path.join(voice_package_path, 'params', 'voice_nav_params.yaml')

    start_robot = LaunchConfiguration('start_robot')
    start_lidar = LaunchConfiguration('start_lidar')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_software_rendering = LaunchConfiguration('rviz_software_rendering')
    start_goal_pose_bridge = LaunchConfiguration('start_goal_pose_bridge')
    start_medicine_pick = LaunchConfiguration('start_medicine_pick')
    start_cloud = LaunchConfiguration('start_cloud')
    start_ekf = LaunchConfiguration('start_ekf')
    laser_localization = LaunchConfiguration('laser_localization')
    robot_serial_port = LaunchConfiguration('robot_serial_port')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    use_astra = LaunchConfiguration('use_astra')
    medicine_pick_start_delay = LaunchConfiguration('medicine_pick_start_delay')
    medicine_pick_bpu_start_delay = LaunchConfiguration('medicine_pick_bpu_start_delay')
    map_file = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file')
    voice_params_file = LaunchConfiguration('voice_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('start_robot', default_value='true', description='Start chassis communication nodes'),
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start lidar driver nodes'),
        DeclareLaunchArgument('start_rviz', default_value='false', description='Start RViz with the nav2_qing config'),
        DeclareLaunchArgument(
            'rviz_software_rendering',
            default_value='true',
            description='Force RViz to use Mesa software rendering to avoid OpenGL driver GLSL crashes',
        ),
        DeclareLaunchArgument('start_goal_pose_bridge', default_value='false', description='Start legacy /goal_pose to Nav2 bridge'),
        DeclareLaunchArgument('start_medicine_pick', default_value='true', description='Start medicine vision and arm pick nodes'),
        DeclareLaunchArgument('start_cloud', default_value='true', description='Start OneNET cloud bridge for delivery record upload'),
        DeclareLaunchArgument('start_ekf', default_value='false', description='Start robot_localization EKF odometry fusion'),
        DeclareLaunchArgument('laser_localization', default_value='true', description='Use jie_ware lidar scan matching for localization'),
        DeclareLaunchArgument('robot_serial_port', default_value='/dev/robot_controller', description='Chassis controller serial device'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/oradar_lidar', description='Lidar serial device'),
        DeclareLaunchArgument('use_astra', default_value='true', description='Use Astra RGB-D camera for medicine pick'),
        DeclareLaunchArgument('medicine_pick_start_delay', default_value='10.0', description='Delay before starting arm/camera nodes'),
        DeclareLaunchArgument('medicine_pick_bpu_start_delay', default_value='3.0', description='Additional delay before starting BPU detection nodes'),
        DeclareLaunchArgument('map', default_value=default_map, description='Navigation map file'),
        DeclareLaunchArgument('nav_params_file', default_value=default_nav_params, description='Nav2 parameter file'),
        DeclareLaunchArgument('voice_params_file', default_value=default_voice_params, description='Voice control parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                'start_robot': start_robot,
                'start_lidar': start_lidar,
                'start_rviz': start_rviz,
                'rviz_software_rendering': rviz_software_rendering,
                'start_ekf': start_ekf,
                'robot_serial_port': robot_serial_port,
                'lidar_serial_port': lidar_serial_port,
                'laser_localization': laser_localization,
                'map': map_file,
                'params': nav_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_launch_path),
            condition=IfCondition(start_medicine_pick),
            launch_arguments={
                'start_comm_base': 'false',
                'use_astra': use_astra,
                'start_delay': medicine_pick_start_delay,
                'bpu_start_delay': medicine_pick_bpu_start_delay,
            }.items(),
        ),
        Node(
            package='voice_nav_control',
            executable='medicine_delivery_task',
            name='medicine_delivery_task',
            output='screen',
            parameters=[voice_params_file],
        ),
        Node(
            package='voice_nav_control',
            executable='goal_pose_bridge',
            name='goal_pose_bridge',
            condition=IfCondition(start_goal_pose_bridge),
            output='screen',
            parameters=[voice_params_file],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('onenet_bridge'), 'launch', 'onenet_bridge.launch.py')
            ),
            condition=IfCondition(start_cloud),
        ),
    ])
