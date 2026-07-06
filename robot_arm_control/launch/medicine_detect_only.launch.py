"""
纯检测 Launch (ROS 2) — mini_4wd_six_arm
流程: 底盘通信 -> 机械臂到观察位 -> BPU 药品检测(只发布检测话题, 不发布速度)

与 medicine_detect.launch.py 的区别:
  * 不启动 car_location_color.py => 不会发布任何 /cmd_vel 速度指令。
  * visual_tracker.py 的 direct_arm_trigger 强制 false => 不会触发抓取。
  * direct_arm_pick_and_put.py 负责把机械臂摆到观察位(arm_look)后保持静止。

可在 rqt 观察:
  * /color_position (robot_arm_control/SixArmPosition) — 检测到的目标位置/距离
  * /bpu_detection/image (sensor_msgs/Image)          — 带框的调试图像
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_arm_control')
    comm_base_dir = get_package_share_directory('communication_base')
    astra_dir = get_package_share_directory('astra_camera')

    detect_params = os.path.join(pkg_dir, 'config', 'medicine_detect_params.yaml')

    declare_video_device = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0', description='USB camera device path')
    declare_use_astra = DeclareLaunchArgument(
        'use_astra', default_value='true', description='Use Astra RGB-D camera instead of usb_cam')
    declare_start_comm_base = DeclareLaunchArgument(
        'start_comm_base', default_value='true',
        description='Start communication_base chassis serial; set false if already running')
    declare_start_arm = DeclareLaunchArgument(
        'start_arm', default_value='true',
        description='Move arm to observation pose (arm_look) and hold')
    declare_start_delay = DeclareLaunchArgument(
        'start_delay', default_value='10.0',
        description='Delay before starting camera and arm observation nodes')
    declare_bpu_start_delay = DeclareLaunchArgument(
        'bpu_start_delay', default_value='3.0',
        description='Additional delay after arm observation startup before BPU detection starts')

    comm_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(comm_base_dir, 'launch', 'base_serial.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_comm_base')),
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
        }],
        remappings=[('/image_raw', '/usb_cam/image_raw')],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_astra')),
    )

    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_dir, 'launch', 'astra_plus.launch.xml')),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '400',
            'color_depth_synchronization': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_astra')),
    )

    # BPU 检测节点: 覆盖 direct_arm_trigger=false, 确保纯检测不触发抓取。
    medicine_detector_node = Node(
        package='robot_arm_control',
        executable='visual_tracker.py',
        name='visual_tracker_node',
        parameters=[detect_params, {'direct_arm_trigger': False}],
        output='screen',
    )

    # 机械臂到观察位并保持静止。不启动 car_location_color.py, 因此没有 /cmd_vel。
    arm_pick_node = Node(
        package='robot_arm_control',
        executable='direct_arm_pick_and_put.py',
        name='arm_pick_and_put_node',
        parameters=[detect_params],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_arm')),
    )

    # 机械臂阶段状态对外发布节点: 订阅 /arm_phase_raw, 规范化后发 /arm_phase。
    arm_phase_node = Node(
        package='robot_arm_control',
        executable='arm_phase_publisher.py',
        name='arm_phase_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_arm')),
    )

    grasp_guide_overlay_node = Node(
        package='robot_arm_control',
        executable='grasp_guide_overlay.py',
        name='grasp_guide_overlay_node',
        parameters=[detect_params],
        output='screen',
    )

    prepare_nodes = TimerAction(
        period=LaunchConfiguration('start_delay'),
        actions=[
            astra_launch,
            camera_node,
            grasp_guide_overlay_node,
            arm_pick_node,
            arm_phase_node,
        ],
    )

    detection_nodes = TimerAction(
        period=PythonExpression([
            LaunchConfiguration('start_delay'),
            ' + ',
            LaunchConfiguration('bpu_start_delay'),
        ]),
        actions=[
            medicine_detector_node,
        ],
    )

    return LaunchDescription([
        declare_video_device,
        declare_use_astra,
        declare_start_comm_base,
        declare_start_arm,
        declare_start_delay,
        declare_bpu_start_delay,
        comm_base_launch,
        prepare_nodes,
        detection_nodes,
    ])
