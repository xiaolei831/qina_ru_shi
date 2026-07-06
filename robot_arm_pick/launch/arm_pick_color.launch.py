"""
药品识别夹取一键 Launch (ROS 2) — mini_4wd_six_arm
启动: Astra+ 深度相机 → BPU 药品检测 → 底盘定位 → 机械臂夹取
注意: communication_base 和 MoveIt 2 需要单独启动或在此处 include
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_arm_pick')
    pick_params = os.path.join(pkg_dir, 'config', 'pick_color_params.yaml')

    # 加载 MoveIt 配置, 获取 robot_description 和 SRDF 参数
    moveit_config = MoveItConfigsBuilder(
        "table_streeing_arm", package_name="robot_arm"
    ).to_moveit_configs()

    # ---------- 参数声明 ----------
    declare_color_topic = DeclareLaunchArgument(
        'color_topic', default_value='/camera/color/image_raw',
        description='RGB image topic from Astra')
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/image_raw',
        description='Depth image topic from Astra')
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/sunrise/qian_sai/data2_best_bayese_640x640_nv12.bin',
        description='Path to BPU bin model')
    declare_conf_thres = DeclareLaunchArgument(
        'conf_thres', default_value='0.25', description='Detection confidence threshold')

    # ---------- 1. Astra+ 深度相机 ----------
    astra_pkg_dir = get_package_share_directory('astra_camera')
    astra_launch_file = os.path.join(
        astra_pkg_dir, 'launch', 'astra_plus.launch.xml'
    )
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch_file),
    )

    # ---------- 2. BPU 药品检测节点 (替换 visual_tracker) ----------
    medicine_detector_node = Node(
        package='robot_arm_pick',
        executable='medicine_detector_node.py',
        name='medicine_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'conf_thres': LaunchConfiguration('conf_thres'),
            'detections_topic': '/medicine_detections',
            'annotated_topic': '/medicine_detector/image_annotated',
            'publish_annotated': True,
        }],
        output='screen',
    )

    # ---------- 3. 底盘色块定位节点 ----------
    car_location_node = Node(
        package='robot_arm_pick',
        executable='six_arm_car_location_color',
        name='car_location_color',
        parameters=[pick_params],
        output='screen',
    )

    # ---------- 4. 机械臂抓取放置节点 ----------
    arm_pick_node = Node(
        package='robot_arm_pick',
        executable='six_arm_pick_and_put',
        name='arm_pick_and_put',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output='screen',
    )

    # 延迟启动业务节点, 等待相机驱动 + MoveIt 初始化
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            medicine_detector_node,
            car_location_node,
            arm_pick_node,
        ]
    )

    return LaunchDescription([
        declare_color_topic,
        declare_depth_topic,
        declare_model_path,
        declare_conf_thres,
        camera_launch,
        delayed_nodes,
    ])
