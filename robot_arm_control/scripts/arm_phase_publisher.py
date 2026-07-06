#!/usr/bin/env python3
# coding=utf-8
"""
机械臂阶段状态发布节点 (ROS 2)

功能:
  订阅 direct_arm_pick_and_put_node 发出的原始阶段流 /arm_phase_raw,
  规范化(校验合法 key、去重)后向外发布:
    - /arm_phase        std_msgs/String   当前阶段英文 key
  并以固定频率重发当前阶段(心跳),方便后接入的订阅者立即拿到最新状态。

阶段 key 含义:
  idle        空闲 / 未开始
  observe     观察位 (arm_look)
  pick_down   夹取下探 (arm_clamp)
  pick_up     夹取抬起 (arm_uplift)
  pick_done   夹取抬起完成
  place       放置位 (arm_rotate_put / put 下探放置)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 合法阶段 key -> 中文说明 (仅用于日志, 对外只发英文 key)
PHASE_LABELS = {
    'idle': '空闲',
    'observe': '观察位',
    'pick_down': '夹取下探',
    'pick_up': '夹取抬起',
    'pick_done': '夹取抬起完成',
    'place': '放置位',
}


class ArmPhasePublisher(Node):
    def __init__(self):
        super().__init__('arm_phase_publisher')

        self.declare_parameter('input_topic', '/arm_phase_raw')
        self.declare_parameter('output_topic', '/arm_phase')
        self.declare_parameter('heartbeat_hz', 2.0)
        self.declare_parameter('initial_phase', 'idle')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        heartbeat_hz = float(self.get_parameter('heartbeat_hz').value)
        self.current_phase = str(self.get_parameter('initial_phase').value)
        if self.current_phase not in PHASE_LABELS:
            self.current_phase = 'idle'

        self.phase_pub = self.create_publisher(String, output_topic, 10)
        self.raw_sub = self.create_subscription(
            String, input_topic, self.raw_callback, 10)

        self.heartbeat_timer = (
            self.create_timer(1.0 / heartbeat_hz, self.publish_current)
            if heartbeat_hz > 0.0 else None
        )

        self.publish_current()
        self.get_logger().info(
            f'arm_phase_publisher ready: {input_topic} -> {output_topic} '
            f'(heartbeat={heartbeat_hz}Hz)')

    def raw_callback(self, msg):
        phase = msg.data.strip()
        if phase not in PHASE_LABELS:
            self.get_logger().warn(f'ignore unknown arm phase: {phase!r}')
            return
        if phase == self.current_phase:
            return
        self.current_phase = phase
        self.get_logger().info(
            f'arm phase -> {phase} ({PHASE_LABELS[phase]})')
        self.publish_current()

    def publish_current(self):
        msg = String()
        msg.data = self.current_phase
        self.phase_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPhasePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
