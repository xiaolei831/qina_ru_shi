from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true', description='Start chassis serial communication node'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='Velocity command topic'),
        DeclareLaunchArgument('linear_speed', default_value='0.20', description='Initial linear speed in m/s'),
        DeclareLaunchArgument('angular_speed', default_value='0.80', description='Initial angular speed in rad/s'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.50', description='Maximum linear speed in m/s'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.50', description='Maximum angular speed in rad/s'),
        DeclareLaunchArgument('linear_step', default_value='0.04', description='Linear smoothing step'),
        DeclareLaunchArgument('angular_step', default_value='0.15', description='Angular smoothing step'),
        Node(
            package='communication',
            executable='communication_node',
            name='communication',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_base')),
            parameters=[{
                'usart_port_name': '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00',
                'serial_baud_rate': 115200,
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'gyro_frame_id': 'gyro_link',
                'odom_x_scale': 1.0,
                'odom_y_scale': 1.0,
                'odom_z_scale_positive': 1.0,
                'odom_z_scale_negative': 1.0,
            }],
        ),
        Node(
            package='key_control',
            executable='key_control',
            name='key_control',
            output='screen',
            prefix='xterm -title key_control -e',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'linear_step': LaunchConfiguration('linear_step'),
                'angular_step': LaunchConfiguration('angular_step'),
            }],
        ),
    ])
