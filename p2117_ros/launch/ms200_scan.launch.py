#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': 'MS200/scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''

def generate_launch_description():
  port_name = LaunchConfiguration('port_name')
  frame_id = LaunchConfiguration('frame_id')
  scan_topic = LaunchConfiguration('scan_topic')
  raw_scan_topic = LaunchConfiguration('raw_scan_topic')
  publish_tf = LaunchConfiguration('publish_tf')
  enable_scan_mask = LaunchConfiguration('enable_scan_mask')
  mask_angle_start_deg = LaunchConfiguration('mask_angle_start_deg')
  mask_angle_end_deg = LaunchConfiguration('mask_angle_end_deg')
  motor_speed = LaunchConfiguration('motor_speed')

  common_lidar_parameters = [
        {'device_model': 'MS200'},
        {'frame_id': frame_id},
        {'port_name': port_name},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': ParameterValue(motor_speed, value_type=int)}
  ]

  # Publish raw scan first when the compatibility mask filter is enabled.
  ordlidar_raw_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      condition=IfCondition(enable_scan_mask),
      parameters=common_lidar_parameters + [{'scan_topic': raw_scan_topic}]
  )

  # Publish directly to scan_topic when masking is disabled.
  ordlidar_direct_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      condition=UnlessCondition(enable_scan_mask),
      parameters=common_lidar_parameters + [{'scan_topic': scan_topic}]
  )

  scan_mask_filter_node = Node(
      package='oradar_lidar',
      executable='scan_mask_filter',
      name='scan_mask_filter',
      output='screen',
      condition=IfCondition(enable_scan_mask),
      parameters=[
        {'input_topic': raw_scan_topic},
        {'output_topic': scan_topic},
        {'mask_angle_start_deg': mask_angle_start_deg},
        {'mask_angle_end_deg': mask_angle_end_deg},
        {'enabled': True},
        {'use_inf': True}
      ]
  )

  # Standalone compatibility TF. Integrated robot bringup usually publishes
  # base_footprint -> laser separately and should pass publish_tf:=false.
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser',
    condition=IfCondition(publish_tf),
    arguments=['0','0','0.18','0','0','0','base_link', frame_id]
  )


  # Define LaunchDescription variable
  ord = LaunchDescription()

  ord.add_action(DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'))
  ord.add_action(DeclareLaunchArgument('frame_id', default_value='laser'))
  ord.add_action(DeclareLaunchArgument('scan_topic', default_value='/scan'))
  ord.add_action(DeclareLaunchArgument('raw_scan_topic', default_value='/scan_raw'))
  ord.add_action(DeclareLaunchArgument('publish_tf', default_value='true'))
  ord.add_action(DeclareLaunchArgument('enable_scan_mask', default_value='true'))
  ord.add_action(DeclareLaunchArgument('mask_angle_start_deg', default_value='135.0'))
  ord.add_action(DeclareLaunchArgument('mask_angle_end_deg', default_value='225.0'))
  ord.add_action(DeclareLaunchArgument('motor_speed', default_value='10'))
  ord.add_action(ordlidar_raw_node)
  ord.add_action(ordlidar_direct_node)
  ord.add_action(scan_mask_filter_node)
  ord.add_action(base_link_to_laser_tf_node)

  return ord
