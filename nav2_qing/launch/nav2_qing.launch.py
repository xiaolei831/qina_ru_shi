import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_robot = LaunchConfiguration('start_robot')
    start_lidar = LaunchConfiguration('start_lidar')
    start_rviz = LaunchConfiguration('start_rviz')
    start_ekf = LaunchConfiguration('start_ekf')
    car_mode = LaunchConfiguration('car_mode')
    robot_serial_port = LaunchConfiguration('robot_serial_port')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    scan_raw_topic = LaunchConfiguration('scan_raw_topic')
    scan_mask_start_deg = LaunchConfiguration('scan_mask_start_deg')
    scan_mask_end_deg = LaunchConfiguration('scan_mask_end_deg')

    nav2_qing_dir = get_package_share_directory('nav2_qing')
    robot_dir = get_package_share_directory('communication_base')
    lidar_dir = get_package_share_directory('oradar_lidar')
    default_map = '/home/sunrise/qian_sai/src/nav2_qing/map/qing_slam_map.yaml'
    default_params = os.path.join(nav2_qing_dir, 'param', 'nav2_qing_4wd.yaml')
    default_rviz = os.path.join(nav2_qing_dir, 'rviz', 'nav2_qing.rviz')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_software_rendering = LaunchConfiguration('rviz_software_rendering')
    laser_localization = LaunchConfiguration('laser_localization')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('start_robot', default_value='true', description='Start base, tf and imu nodes'),
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start lidar launch file'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz with the nav2_qing config'),
        DeclareLaunchArgument('start_ekf', default_value='false', description='Start robot_localization EKF odometry fusion'),
        DeclareLaunchArgument('laser_localization', default_value='true', description='Use jie_ware lidar scan matching for localization'),
        DeclareLaunchArgument('car_mode', default_value='mini_4wd', description='Robot chassis mode for communication_base'),
        DeclareLaunchArgument('robot_serial_port', default_value='/dev/robot_controller', description='Chassis controller serial device'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/oradar_lidar', description='Lidar serial device'),
        DeclareLaunchArgument('scan_raw_topic', default_value='/scan_raw', description='Raw lidar scan topic before self-body masking'),
        DeclareLaunchArgument('scan_mask_start_deg', default_value='120.0', description='Start angle of the lidar self-body mask'),
        DeclareLaunchArgument('scan_mask_end_deg', default_value='240.0', description='End angle of the lidar self-body mask'),
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file'),
        DeclareLaunchArgument('params', default_value=default_params, description='Full path to nav2 params file'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='Full path to the RViz config file'),
        DeclareLaunchArgument(
            'rviz_software_rendering',
            default_value='true',
            description='Force RViz to use Mesa software rendering to avoid OpenGL driver GLSL crashes',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_dir, 'launch', 'communication_base.launch.py')),
            condition=IfCondition(start_robot),
            launch_arguments={
                'use_slam_toolbox': laser_localization,
                'start_ekf': start_ekf,
                'car_mode': car_mode,
                'usart_port_name': robot_serial_port,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_dir, 'launch', 'ms200_scan.launch.py')),
            condition=IfCondition(start_lidar),
            launch_arguments={
                'port_name': lidar_serial_port,
                'frame_id': 'laser',
                'scan_topic': '/scan',
                'raw_scan_topic': scan_raw_topic,
                'publish_tf': 'false',
                'enable_scan_mask': 'true',
                'mask_angle_start_deg': scan_mask_start_deg,
                'mask_angle_end_deg': scan_mask_end_deg,
            }.items(),
        ),
        Node(
            package='jie_ware',
            executable='lidar_loc',
            name='lidar_loc',
            condition=IfCondition(laser_localization),
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'base_frame': 'base_footprint',
                'odom_frame': 'odom_combined',
                'laser_frame': 'laser',
                'map_topic': '/map',
                'laser_topic': '/scan',
                'odom_topic': '/odom',
                'initial_pose_topic': '/initialpose',
                'broadcast_odom_tf': True,
                'clear_costmaps_on_initial_pose': True,
            }],
        ),
        # base_footprint -> laser TF is published by communication_base's
        # robot_mode_description.launch.py (static, /tf_static) using the
        # values in robot_model.yaml. Do NOT add a second dynamic publisher
        # here — duplicate publishers on the same frame pair cause tf2
        # jitter. (AMCL scan drops are handled by transform_tolerance, not by
        # a second broadcaster.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_qing_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(start_rviz),
            arguments=['-d', rviz_config],
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': PythonExpression([
                    "'1' if '",
                    rviz_software_rendering,
                    "'.lower() in ('1', 'true', 'yes', 'on') else '0'",
                ]),
            },
            output='screen',
        ),
    ])
