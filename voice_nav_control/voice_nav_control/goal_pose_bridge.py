import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_bridge')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action_name').get_parameter_value().string_value),
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').get_parameter_value().string_value),
            self.goal_callback,
            10,
        )
        self.active_goal_handle = None
        self.get_logger().info('goal pose bridge node started')

    def goal_callback(self, goal_pose: PoseStamped) -> None:
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('navigate_to_pose action server is unavailable')
            return

        self.cancel_active_goal()
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info('forward goal_pose to navigate_to_pose')

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('navigation goal was rejected')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('navigation goal accepted')

    def cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return

        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.on_cancel_done)
        self.active_goal_handle = None

    def on_cancel_done(self, future) -> None:
        response = future.result()
        if response is not None:
            self.get_logger().info('previous navigation goal cancelled')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
