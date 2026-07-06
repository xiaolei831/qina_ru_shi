#!/usr/bin/env python3
# coding=utf-8
"""Direct fixed-pose grasp executor for medicine picking."""

import time
import threading
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor

from robot_arm_control.msg import PickAndPut, SixArmPosition


class DirectArmPickAndPut(Node):
    def __init__(self):
        super().__init__('direct_arm_pick_and_put_node')

        self.declare_parameter('arm_init', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('arm_look', [0.0, 0.54, 1.57, 1.57, 0.0])
        self.declare_parameter('arm_clamp', [0.0, -1.1, 0.66, 1.0, 0.0])
        self.declare_parameter('arm_uplift', [0.0, 0.54, 0.97, 1.57, 0.0])
        self.declare_parameter('arm_rotate_uplift', [1.57, 0.57, 1.57, 1.3, 0.0])
        self.declare_parameter('arm_rotate_put', [1.57, -1.1, 0.66, 1.0, 0.0])
        self.declare_parameter('grasp_joint_6_name', 'joint_6')
        self.declare_parameter('grasp_joint_6_open', 0.9)
        self.declare_parameter('grasp_joint_6_close', 0.3)
        self.declare_parameter('step_duration_sec', 1.0)
        self.declare_parameter('grasp_down_settle_sec', 1.2)
        self.declare_parameter('grasp_close_settle_sec', 0.8)
        self.declare_parameter('command_repeats', 3)
        self.declare_parameter('publish_car_command_after_pick', False)
        self.declare_parameter('car_command_heartbeat', False)
        self.declare_parameter('joint_state_heartbeat_hz', 5.0)
        self.declare_parameter('require_visual_ready_for_pick', True)
        self.declare_parameter('color_location_y', -0.08)
        self.declare_parameter('color_location_x', 0.08)
        self.declare_parameter('angle_tolerance', 0.012)
        self.declare_parameter('pick_target_timeout_sec', 0.5)
        self.declare_parameter('use_depth_distance_control', True)
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('class_3_target_distance_enabled', True)
        self.declare_parameter('class_3_target_distance', 0.25)
        self.declare_parameter('distance_tolerance', 0.025)
        self.declare_parameter('min_safe_distance', 0.26)
        self.declare_parameter('medicine_pick_state_topic', '/medicine_pick_state')

        # 每类别甜点表(与 car_location_color_node 共用同一套标定):
        # 检测到不同药品(medicine/class_N)时,抓取前复核用对应类别的
        # color_location_x/y 与 target_distance; 未列出/未覆盖的回退到全局默认值。
        # dynamic_typing: 空列表默认会被推断成 BYTE_ARRAY 而拒绝 YAML 字符串数组,故放开类型。
        self.declare_parameter('class_target_names', [],
                               ParameterDescriptor(dynamic_typing=True))
        self._class_target_names = [
            str(n) for n in (self.get_parameter('class_target_names').value or [])
        ]
        for name in self._class_target_names:
            self.declare_parameter(f'class_target.{name}.color_location_x', float('nan'))
            self.declare_parameter(f'class_target.{name}.color_location_y', float('nan'))
            self.declare_parameter(f'class_target.{name}.target_distance', float('nan'))

        # 双线程隔离: 抓取序列含数秒 time.sleep,单线程会堵死视觉更新 => 抓取时用的是过期目标。
        # 视觉订阅+心跳放可重入组,抓取期间仍持续刷新 last_visual_target;
        # 抓取回调放互斥组,序列内部严格串行不重入(busy 标志另作二次保护)。
        self.visual_cb_group = ReentrantCallbackGroup()
        self.pick_cb_group = MutuallyExclusiveCallbackGroup()

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/communication_base/arm_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/communication_base/hand_controller/joint_trajectory', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.car_command_pub = self.create_publisher(PickAndPut, '/car_command', 10)
        # 机械臂阶段状态原始流: 由 arm_phase_publisher.py 订阅后规范化对外发布。
        self.phase_pub = self.create_publisher(String, '/arm_phase_raw', 10)
        self.pick_state_pub = self.create_publisher(
            String, str(self.get_parameter('medicine_pick_state_topic').value), 10)
        self.arm_state_sub = self.create_subscription(
            String, '/arm_state', self.arm_state_callback, 10,
            callback_group=self.pick_cb_group)
        self.visual_sub = self.create_subscription(
            SixArmPosition, '/color_position', self.visual_target_callback, 10,
            callback_group=self.visual_cb_group)

        self.last_state = ''
        self.busy = False
        self.current_car_state = 0.0
        self.current_car_angle = 0.0
        self.current_arm = self.param_list('arm_init')
        self.current_hand = self.get_float('grasp_joint_6_close')
        self.last_visual_target = None
        self.last_visual_time = None
        self._state_lock = threading.Lock()
        self._visual_lock = threading.Lock()
        self.publish_car_command_after_pick = bool(
            self.get_parameter('publish_car_command_after_pick').value)
        self.car_command_heartbeat = bool(
            self.get_parameter('car_command_heartbeat').value)
        self.joint_state_heartbeat_hz = float(
            self.get_parameter('joint_state_heartbeat_hz').value)
        self.timer = (
            self.create_timer(0.05, self.publish_current_car_state,
                              callback_group=self.visual_cb_group)
            if self.car_command_heartbeat else None
        )
        self.joint_state_timer = (
            self.create_timer(1.0 / self.joint_state_heartbeat_hz,
                              self.publish_joint_state,
                              callback_group=self.visual_cb_group)
            if self.joint_state_heartbeat_hz > 0.0 else None
        )

        self.get_logger().info('direct_arm_pick_and_put_node ready: direct trajectory control')
        self.publish_phase('observe')
        self.wait_for_trajectory_subscribers()
        self.move_arm('arm_look')
        self.move_hand(self.get_float('grasp_joint_6_open'))

    def visual_target_callback(self, msg):
        # 视觉线程写、抓取线程读,加锁保证 target 与 time 的快照一致(多线程执行器下)。
        with self._visual_lock:
            self.last_visual_target = msg
            self.last_visual_time = self.get_clock().now()

    def arm_state_callback(self, msg):
        state = msg.data
        if self.busy or state == self.last_state:
            return
        if state not in ('pick', 'put', 'rotate_put', 'shake_hand', 'no_msg'):
            return
        self.last_state = state

        if state == 'pick':
            self.busy = True
            self.publish_pick_state('picking', 'pick', '开始视觉抓取')
            ok = self.pick() if self.visual_ready_for_pick() else False
            if self.publish_car_command_after_pick:
                self.publish_car_state(1 if ok else 0, 1.57 if ok else 0.0)
            elif not ok:
                self.get_logger().error('direct pick blocked/failed; no chassis command was published')
            else:
                self.get_logger().info('direct pick complete: arm held at arm_uplift, chassis command suppressed')
            self.publish_pick_state(
                'pick_complete' if ok else 'failed',
                'pick',
                '机械臂抬起完成，抓取完成' if ok else '视觉目标未就绪，抓取失败',
            )
            self.busy = False
        elif state == 'put':
            self.busy = True
            self.publish_pick_state('placing', 'put', '开始放置药品')
            ok = self.put()
            self.publish_car_state(2 if ok else 0, -1.57 if ok else 0.0)
            self.publish_pick_state(
                'placed' if ok else 'failed',
                'put',
                '放置完成' if ok else '放置失败',
            )
            self.busy = False
        elif state == 'rotate_put':
            self.busy = True
            self.publish_pick_state('placing', 'rotate_put', '开始旋转放置药品')
            ok = self.rotate_put()
            self.publish_car_state(3 if ok else 0, 0.0)
            self.publish_pick_state(
                'placed' if ok else 'failed',
                'rotate_put',
                '旋转放置完成' if ok else '旋转放置失败',
            )
            self.busy = False
        elif state == 'shake_hand':
            self.busy = True
            self.shake_hand()
            self.publish_car_state(4, 0.0)
            self.busy = False
        elif state == 'no_msg':
            if self.car_command_heartbeat:
                self.publish_car_state(0, 0.0)

    def pick(self):
        self.get_logger().info(
            'direct pick: hand_open -> arm_clamp -> settle -> hand_close -> settle -> arm_uplift')
        self.move_hand(self.get_float('grasp_joint_6_open'))
        self.publish_phase('pick_down')
        self.move_arm('arm_clamp')
        self.settle_after_motion('grasp_down_settle_sec', 'grasp down')
        self.move_hand(self.get_float('grasp_joint_6_close'))
        self.settle_after_motion('grasp_close_settle_sec', 'grasp close')
        self.publish_pick_state('lifting', 'pick', '夹爪闭合，机械臂抬起中')
        self.publish_phase('pick_up')
        self.move_arm('arm_uplift')
        self.publish_phase('pick_done')
        return True

    def put(self):
        self.get_logger().info('direct put: arm_clamp -> hand_open -> arm_uplift')
        self.publish_phase('place')
        self.move_arm('arm_clamp')
        self.move_hand(self.get_float('grasp_joint_6_open'))
        self.publish_phase('pick_up')
        self.move_arm('arm_uplift')
        return True

    def rotate_put(self):
        self.get_logger().info('direct rotate_put: rotate_uplift -> rotate_put -> hand_open -> look')
        self.publish_phase('place')
        self.move_arm('arm_rotate_uplift')
        self.move_arm('arm_rotate_put')
        self.move_hand(self.get_float('grasp_joint_6_open'))
        self.move_arm('arm_rotate_uplift')
        self.publish_phase('observe')
        self.move_arm('arm_look')
        return True

    def shake_hand(self):
        self.get_logger().info('direct shake_hand')
        base = self.param_list('arm_look')
        for i in range(96):
            temp = 0.0
            if i < 16:
                temp = -0.06 * (i + 1)
            elif i < 48:
                temp = -0.96 + 0.06 * (i - 15)
            elif i < 80:
                temp = 0.96 - 0.06 * (i - 47)
            else:
                temp = -0.96 + 0.06 * (i - 79)
            target = list(base)
            target[4] = temp
            self.publish_arm_values(target, 0.02)
            time.sleep(1.0 / 60.0)

    def move_arm(self, param_name):
        self.publish_arm_values(self.param_list(param_name), self.get_float('step_duration_sec'))
        time.sleep(self.get_float('step_duration_sec'))

    def wait_for_trajectory_subscribers(self):
        deadline = time.time() + 3.0
        arm_topic = '/communication_base/arm_controller/joint_trajectory'
        hand_topic = '/communication_base/hand_controller/joint_trajectory'
        while rclpy.ok() and time.time() < deadline:
            arm_count = self.count_subscribers(arm_topic)
            hand_count = self.count_subscribers(hand_topic)
            if arm_count > 0 and hand_count > 0:
                return
            time.sleep(0.1)
        self.get_logger().warn(
            'trajectory controller subscriber not fully discovered before initial observe command')

    def settle_after_motion(self, param_name, label):
        duration = max(0.0, self.get_float(param_name))
        if duration <= 0.0:
            return
        self.get_logger().info(f'{label}: settle {duration:.2f}s before next grasp step')
        time.sleep(duration)

    def move_hand(self, joint_6):
        hand_position = float(joint_6)
        with self._state_lock:
            self.current_hand = hand_position
        msg = JointTrajectory()
        msg.joint_names = [str(self.get_parameter('grasp_joint_6_name').value)]
        point = JointTrajectoryPoint()
        point.positions = [hand_position]
        duration = self.get_float('step_duration_sec')
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self.publish_repeated(self.hand_pub, msg)
        self.publish_joint_state()
        time.sleep(duration)

    def publish_arm_values(self, values, duration):
        if len(values) != 5:
            self.get_logger().error(f'arm target must have 5 values, got {len(values)}')
            return
        arm_positions = [float(v) for v in values]
        with self._state_lock:
            self.current_arm = arm_positions
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self.publish_repeated(self.arm_pub, msg)
        self.publish_joint_state()

    def publish_phase(self, phase):
        """对外发布当前机械臂阶段(英文 key)。arm_phase_publisher.py 订阅 /arm_phase_raw 转发。"""
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)

    def publish_pick_state(self, state, action='', message=''):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'action': action,
            'message': message,
            'stamp': time.time(),
        }, ensure_ascii=False, separators=(',', ':'))
        self.pick_state_pub.publish(msg)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        with self._state_lock:
            positions = list(self.current_arm) + [self.current_hand]
        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        self.joint_state_pub.publish(msg)

    def publish_repeated(self, pub, msg):
        for _ in range(max(1, int(self.get_parameter('command_repeats').value))):
            pub.publish(msg)
            time.sleep(0.05)

    def publish_current_car_state(self):
        msg = PickAndPut()
        msg.car_state = float(self.current_car_state)
        msg.angle = float(self.current_car_angle)
        self.car_command_pub.publish(msg)

    def publish_car_state(self, car_state, angle):
        self.current_car_state = float(car_state)
        self.current_car_angle = float(angle)
        msg = PickAndPut()
        msg.car_state = self.current_car_state
        msg.angle = self.current_car_angle
        self.car_command_pub.publish(msg)

    def visual_ready_for_pick(self):
        if not bool(self.get_parameter('require_visual_ready_for_pick').value):
            return True
        # 原子快照: 避免读 target 和读 time 之间被视觉线程改写造成不一致。
        with self._visual_lock:
            target = self.last_visual_target
            last_time = self.last_visual_time
        if target is None or last_time is None:
            self.get_logger().warn('direct pick blocked: no visual target')
            return False
        max_age = float(self.get_parameter('pick_target_timeout_sec').value)
        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        if age > max_age:
            self.get_logger().warn(f'direct pick blocked: visual target stale age={age:.3f}s')
            return False
        if not target.correct:
            self.get_logger().warn('direct pick blocked: visual target not correct')
            return False
        color_location_y = self.color_location_for(target, 'color_location_y')
        color_location_x = self.color_location_for(target, 'color_location_x')
        angle_tolerance = self.get_float('angle_tolerance')
        angle_x_error = float(target.angle_x) - color_location_y
        if abs(angle_x_error) > angle_tolerance:
            self.get_logger().warn(
                f'direct pick blocked: angle_x={target.angle_x:.4f} '
                f'target={color_location_y:.4f} error={angle_x_error:.4f}')
            return False

        use_depth_distance = (
            bool(self.get_parameter('use_depth_distance_control').value) and
            float(target.distance) > 0.0
        )
        if use_depth_distance:
            target_distance = self.target_distance_for(target)
            distance_error = float(target.distance) - target_distance
            if not self.distance_ready_for_pick(target):
                self.get_logger().warn(
                    f'direct pick blocked: distance={target.distance:.3f} '
                    f'target={target_distance:.3f} error={distance_error:.3f} '
                    f'tolerance={self.get_float("distance_tolerance"):.3f}')
                return False
            self.get_logger().info(
                f'direct pick visual ready: angle_x={target.angle_x:.4f} '
                f'target={color_location_y:.4f} distance={target.distance:.3f} '
                f'target_distance={target_distance:.3f}')
            return True

        angle_y_error = float(target.angle_y) - color_location_x
        if abs(angle_y_error) > angle_tolerance:
            self.get_logger().warn(
                f'direct pick blocked: angle_y={target.angle_y:.4f} '
                f'target={color_location_x:.4f} error={angle_y_error:.4f}')
            return False
        self.get_logger().info(
            f'direct pick visual ready: angle_x={target.angle_x:.4f} '
            f'target_x={color_location_y:.4f} angle_y={target.angle_y:.4f} '
            f'target_y={color_location_x:.4f}')
        return True

    def distance_ready_for_pick(self, target):
        if bool(self.get_parameter('use_depth_distance_control').value) and target.distance > 0.0:
            if target.distance <= self.get_float('min_safe_distance'):
                return True
            return abs(target.distance - self.target_distance_for(target)) <= self.get_float('distance_tolerance')
        return True

    def _class_param(self, target, field):
        """读取检测到类别的 class_target.<名>.<field>; 返回 nan 表示未覆盖。"""
        name = getattr(target, 'color', None)
        if name in self._class_target_names:
            return float(self.get_parameter(f'class_target.{name}.{field}').value)
        return float('nan')

    def color_location_for(self, target, field):
        value = self._class_param(target, field)
        if not math.isnan(value):
            return value
        return self.get_float(field)

    def target_distance_for(self, target):
        value = self._class_param(target, 'target_distance')
        if not math.isnan(value):
            return value
        enabled = bool(self.get_parameter('class_3_target_distance_enabled').value)
        if enabled and target.color == '维生素胶囊':
            return float(self.get_parameter('class_3_target_distance').value)
        return self.get_float('target_distance')

    def param_list(self, name):
        return [float(v) for v in self.get_parameter(name).value]

    def get_float(self, name):
        return float(self.get_parameter(name).value)


def main(args=None):
    rclpy.init(args=args)
    node = DirectArmPickAndPut()
    # 多线程执行器: 抓取序列(数秒 time.sleep)与视觉更新跑在不同线程,抓取期间视觉不被堵死。
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
