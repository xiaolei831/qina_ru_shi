from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map_path = "/home/sunrise/qian_sai/src/nav2_qing/map/qing_slam_map"
    map_path = LaunchConfiguration("map_path")
    pbstream_path = LaunchConfiguration("pbstream_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=default_map_path,
                description="Output map path without file extension",
            ),
            DeclareLaunchArgument(
                "pbstream_path",
                default_value=f"{default_map_path}.pbstream",
                description="Output Cartographer pbstream file path",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/finish_trajectory",
                    "cartographer_ros_msgs/srv/FinishTrajectory",
                    "{trajectory_id: 0}",
                ],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            [
                                "ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{filename: '",
                                pbstream_path,
                                "', include_unfinished_submaps: true}\" && ros2 run nav2_map_server map_saver_cli -f ",
                                map_path,
                            ],
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
