import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('communication_base')
    launch_dir = os.path.join(bringup_dir, 'launch')

    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox', default='false')
    start_ekf = LaunchConfiguration('start_ekf', default='true')
    usart_port_name = LaunchConfiguration('usart_port_name', default='/dev/robot_controller')
    use_slam_toolbox_dec = DeclareLaunchArgument('use_slam_toolbox', default_value='false')
    start_ekf_dec = DeclareLaunchArgument(
        'start_ekf',
        default_value='true',
        description='Start robot_localization EKF odometry fusion',
    )
    usart_port_name_dec = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/robot_controller',
        description='Serial device for the robot controller',
    )
                    
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'usart_port_name': usart_port_name}.items(),
    )
     
     
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            condition=IfCondition(start_ekf),
            launch_arguments={'use_slam_toolbox': use_slam_toolbox}.items(),            
    )                                                       
                           
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )
    
    car_mode_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

    ld = LaunchDescription()
    ld.add_action(use_slam_toolbox_dec)
    ld.add_action(start_ekf_dec)
    ld.add_action(usart_port_name_dec)
    ld.add_action(wheeltec_robot)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_ekf)
    ld.add_action(car_mode_type)
    return ld
