#!/usr/bin/env python3
# coding=utf-8
"""
底盘药品定位节点 (ROS 2) — 适用于 mini_4wd_six_arm 移动机械臂小车
功能:
  1. 订阅 /color_position (SixArmPosition)，获取药品位置
  2. 使用 PD 控制器驱动底盘对准药品 (四驱车: x轴线速度 + z轴角速度)
  3. 药品定位完成后发布 /arm_state 通知机械臂夹取
  4. 订阅 /car_command (PickAndPut) 响应机械臂完成后的底盘旋转
  5. 利用 /odom 做定角旋转控制

参考: ROS 1 wheeltec_arm_pick/src/wheeltec_six_arm_pick/car_location_color.cpp
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import String

from robot_arm_control.msg import SixArmPosition, PickAndPut


class CarLocationColorNode(Node):
    def __init__(self):
        super().__init__('car_location_color_node')

        # ---- 声明参数 ----
        self.declare_parameter('x_p', 0.15)
        self.declare_parameter('x_d', 0.04)
        self.declare_parameter('z_p', 0.30)
        self.declare_parameter('z_d', 0.03)
        self.declare_parameter('color_location_x', 0.0909)
        self.declare_parameter('color_location_y', 0.2924)
        self.declare_parameter('location_threshold', 3)
        self.declare_parameter('speed_threshold_x', 0.003)
        self.declare_parameter('speed_threshold_z', 0.005)
        self.declare_parameter('rotation_speed', 0.6)
        self.declare_parameter('rotation_tolerance', 0.1)
        self.declare_parameter('error_timeout', 10000)
        self.declare_parameter('target_timeout_sec', 3.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.4)
        self.declare_parameter('pick_target_timeout_sec', 0.5)
        self.declare_parameter('use_depth_distance_control', True)
        self.declare_parameter('enable_angle_y_linear_control', False)
        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('class_3_target_distance_enabled', True)
        self.declare_parameter('class_3_target_distance', 0.25)
        self.declare_parameter('distance_tolerance', 0.025)
        self.declare_parameter('angle_tolerance', 0.012)
        self.declare_parameter('min_safe_distance', 0.26)
        self.declare_parameter('max_linear_speed', 0.015)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('max_angular_speed', 0.08)
        self.declare_parameter('min_angular_speed', 0.018)
        self.declare_parameter('linear_toward_sign', 1.0)
        self.declare_parameter('angle_y_linear_sign', 1.0)
        self.declare_parameter('angle_x_angular_sign', 1.0)
        self.declare_parameter('single_pick_mode', True)
        self.declare_parameter('enabled', False)
        self.declare_parameter('enable_topic', '/medicine_pick_enable')

        # 每类别甜点表: 不同药品(medicine/class_N)对准位置和抓取距离各不相同。
        # class_target_names 列出有独立标定的类别; 每个类别可单独覆盖
        # color_location_x(↔angle_y垂直) / color_location_y(↔angle_x横向) / target_distance。
        # 未列出的类别或未覆盖的字段(留 nan)自动回退到全局默认值,完全向后兼容。
        # dynamic_typing: 空列表默认会被推断成 BYTE_ARRAY 而拒绝 YAML 的字符串数组,故放开类型。
        self.declare_parameter('class_target_names', [],
                               ParameterDescriptor(dynamic_typing=True))
        self._class_target_names = [
            str(n) for n in (self.get_parameter('class_target_names').value or [])
        ]
        for name in self._class_target_names:
            self.declare_parameter(f'class_target.{name}.color_location_x', float('nan'))
            self.declare_parameter(f'class_target.{name}.color_location_y', float('nan'))
            self.declare_parameter(f'class_target.{name}.target_distance', float('nan'))

        self.refresh_params()

        # ---- PID 状态 ----
        self.last_error_x = 0.0
        self.last_error_z = 0.0
        self.distance_x = 0.0
        self.angular_z_val = 0.0

        # ---- 控制状态 ----
        self.target_liner_x = 0.0
        self.target_liner_y = 0.0
        self.target_angular_z = 0.0
        self.distance_ready = False
        self.location_flag = 0
        self.move_flag = 0
        self.car_state = 0
        self.target_angle = 0.0
        self.shake_hand = 0
        self.shake_hand_done = [0, 0, 0]

        # ---- 里程计状态 ----
        self.car_position_z = 0.0
        self.last_target_angle = 0.0
        self.target_position_z = 0.0

        # ---- 错误检测 ----
        self.error_flag = 1
        self.error_count = 0
        self.location_count = 0
        self.last_target_time = None
        self.last_target_msg = None
        self.pick_requested = False
        self.pick_complete = False
        self.enabled = bool(self.get_parameter('enabled').value)

        # ---- 发布 / 订阅 ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_state_pub = self.create_publisher(String, '/arm_state', 10)

        self.color_sub = self.create_subscription(
            SixArmPosition, '/color_position', self.color_location_callback, 10)
        self.car_cmd_sub = self.create_subscription(
            PickAndPut, '/car_command', self.car_command_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.car_pose_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, str(self.get_parameter('enable_topic').value), self.enable_callback, 10)

        # 主控制循环定时器 (~50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('car_location_color_node 初始化完成: 药品定位 (mini_4wd_six_arm)')

    # ================================================================
    #  回调函数
    # ================================================================

    def car_command_callback(self, msg):
        """接收机械臂发出的底盘运动指令"""
        self.refresh_params()
        if self.single_pick_mode:
            self.pick_complete = True
            self.location_flag = 0
            self.move_flag = 0
            self._stop_tracking_motion(publish=False)
            return
        self.car_state = int(msg.car_state)
        self.target_angle = msg.angle

    def enable_callback(self, msg):
        enabled = bool(msg.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        if enabled:
            self._reset_pick_state()
            self.get_logger().info('medicine pick alignment enabled')
        else:
            self._stop_tracking_motion(publish=True)
            self._publish_no_msg()
            self.get_logger().info('medicine pick alignment disabled')

    def color_location_callback(self, msg):
        """接收药品位置, 执行 PD 底盘定位"""
        self.refresh_params()
        if not self.enabled:
            self._stop_tracking_motion(publish=False)
            return
        # 检测到目标药品
        if (self.single_pick_mode and self.pick_requested) or self.pick_complete:
            self._stop_tracking_motion(publish=False)
            return

        if msg.correct and self.car_state == 0 and self.shake_hand == 0:
            self.error_flag = 0
            # 先按检测到的类别选用对应甜点,后续 PD 控制与到位判断都基于该类参数。
            self._apply_class_target(msg)
            self.last_target_time = self.get_clock().now()
            self.last_target_msg = msg

            # mini_4wd_six_arm: 使用 x轴线速度 + z轴角速度
            if self.location_flag == 0 and self.car_state == 0:
                self.angular_z_val = msg.angle_x
                self.target_liner_x = self._linear_x_from_target(msg)
                self.target_angular_z = self._angular_z_from_target(msg)

            # 判断定位是否完成: 必须看视觉误差本身，而不是只看控制输出速度。
            angle_error = self._angle_error(msg)
            distance_ready = self._distance_ready(msg)
            angle_ready = self._angle_ready(msg)
            motion_settled = (
                abs(self.target_liner_x) < self.speed_thresh_x and
                abs(self.target_angular_z) < self.speed_thresh_z)
            if (distance_ready and angle_ready and motion_settled and self.car_state == 0):
                self.location_count += 1
            else:
                self.location_count = 0

            if self.location_count >= self.location_threshold:
                self.location_flag = 1
                self.location_count = 0
                self.get_logger().info(
                    f'target aligned: angle_x={msg.angle_x:.4f} '
                    f'target={self.color_location_y:.4f} error={angle_error:.4f} '
                    f'distance={msg.distance:.3f}')

            # 定位过程中不需要 y 轴速度
            if self.location_flag == 0:
                self.target_liner_y = 0.0

            # 定位完成后停止底盘
            if self.location_flag == 1:
                self.target_liner_x = 0.0
                self.target_angular_z = 0.0

            self._publish_cmd_vel()

        # BPU 药品检测默认只发布 correct=True；保留旧逻辑兼容历史色块流程。
        elif (not msg.correct and self.car_state == 0 and
              self.location_flag == 0 and self.shake_hand == 0):
            self._stop_tracking_motion(publish=False)
            color_idx = {'yellow': 0, 'blue': 1, 'green': 2}.get(msg.color, -1)
            if 0 <= color_idx < 3 and self.shake_hand_done[color_idx] == 0:
                self.shake_hand = 1
                self.shake_hand_done[color_idx] = 1

    def car_pose_callback(self, msg):
        """利用 odom 做底盘定角旋转"""
        self.car_position_z = self._yaw_from_quaternion(msg.pose.pose.orientation)

        if self.last_target_angle != self.target_angle:
            self.target_position_z = self.car_position_z + self.target_angle

        # 夹取完成后底盘旋转
        if self.car_state == 1:
            yaw_error = self._normalize_angle(self.target_position_z - self.car_position_z)
            if yaw_error < -self.rotation_tol:
                self.target_angular_z = -self.rotation_speed
                self.move_flag = 1
            elif yaw_error > self.rotation_tol:
                self.target_angular_z = self.rotation_speed
                self.move_flag = 1
            else:
                self.target_angular_z = 0.0
                self.move_flag = 2
            self.last_target_angle = self.target_angle
            self._publish_cmd_vel()

        # 放置完成后底盘旋转回原位
        if self.car_state == 2:
            yaw_error = self._normalize_angle(self.target_position_z - self.car_position_z)
            if yaw_error < -self.rotation_tol:
                self.target_angular_z = -self.rotation_speed
                self.move_flag = 3
            elif yaw_error > self.rotation_tol:
                self.target_angular_z = self.rotation_speed
                self.move_flag = 3
            else:
                self.target_angular_z = 0.0
                self.move_flag = 4
            self.last_target_angle = self.target_angle
            self._publish_cmd_vel()

        self.last_target_angle = self.target_angle

    # ================================================================
    #  主控制循环
    # ================================================================

    def control_loop(self):
        self.refresh_params()
        if not self.enabled:
            return

        # 低帧率 BPU 下，速度命令要比目标状态更早过期:
        # 帧间先停车防止持续转动，但不清稳定计数；真正丢目标才重置定位状态。
        if self.car_state == 0 and not self.pick_requested:
            now = self.get_clock().now()
            if self.last_target_time is None:
                self.location_flag = 0
                self._stop_tracking_motion(publish=True)
            else:
                target_age = now - self.last_target_time
                if target_age > self.target_timeout:
                    self.location_flag = 0
                    self._stop_tracking_motion(publish=True)
                elif self.location_flag == 0 and target_age > self.cmd_vel_timeout:
                    self._stop_tracking_motion(
                        publish=True,
                        reset_location_count=False)

        # 一轮完整操作结束后清零标志位
        if self.move_flag == 4 or self.car_state == 3:
            self.car_state = 0
            self.location_flag = 0
            self.move_flag = 0
            self.pick_requested = False
            self.pick_complete = False
            self._stop_tracking_motion(publish=False)
            self.shake_hand_done = [0, 0, 0]

        # shake_hand 完成后清零
        if self.car_state == 4:
            self.car_state = 0
            self.location_flag = 0
            self.move_flag = 0
            self.pick_requested = False
            self.pick_complete = False
            self.shake_hand = 0
            self._stop_tracking_motion()

        # 发布机械臂状态
        self._publish_arm_state()

        # 错误检测: 长时间未检测到药品则停车
        if self.error_flag == 0:
            self.error_count = 0
        elif self.error_flag == 1 and self.location_flag == 0:
            self.error_count += 1
        self.error_flag = 1

        if self.error_count > self.error_timeout:
            self._stop_tracking_motion(publish=False)
            self.error_count = 0

    # ================================================================
    #  PD 控制器
    # ================================================================

    def _pid_x(self):
        error = self.distance_x - self.color_location_x
        output = self.x_d * self.last_error_x + self.x_p * error
        self.last_error_x = error
        return output

    def _linear_x_from_target(self, msg):
        if self.use_depth_distance_control and msg.distance > 0.0:
            if msg.distance <= self.min_safe_distance:
                self.distance_ready = True
                return 0.0

            target_distance = self._target_distance_for(msg)
            distance_error = msg.distance - target_distance
            if abs(distance_error) <= self.distance_tolerance:
                self.distance_ready = True
                return 0.0

            self.distance_ready = False

            # 纯 P 在距离误差变小时速度趋近 0,低于电机启动死区(四驱约 20-30mm/s)就推不动,
            # 车会在离目标几厘米处"软卡死",永远进不了容差 => 永不触发抓取。
            # 加最小速度兜底: 走到这里说明误差已超容差(≤容差上面已 return),直接套最小速度。
            # 已验证 min_linear_speed*pulse=12mm,从容差边缘冲过去落在 -4mm 仍在带内,不过冲不横跳。
            speed = min(self.max_linear_speed, self.x_p * abs(distance_error))
            if speed < self.min_linear_speed:
                speed = self.min_linear_speed

            if distance_error > 0.0:
                return self.linear_toward_sign * speed
            return -self.linear_toward_sign * speed

        self.distance_x = msg.angle_y
        if not self.enable_angle_y_linear_control:
            self.distance_ready = self._angle_y_ready(msg)
            return 0.0

        self.distance_ready = False
        return self._clamp(
            self.angle_y_linear_sign * self._pid_x(),
            -self.max_linear_speed,
            self.max_linear_speed)

    def _distance_ready(self, msg):
        if self.use_depth_distance_control and msg.distance > 0.0:
            if msg.distance <= self.min_safe_distance:
                return True
            return abs(msg.distance - self._target_distance_for(msg)) <= self.distance_tolerance
        if not self.enable_angle_y_linear_control:
            return True
        return abs(self.target_liner_x) < self.speed_thresh_x

    def _target_y_for(self, msg):
        # 横向甜点按类别取值; control_loop 每周期会 refresh_params 重置 self.color_location_y,
        # 而每类别值只在收帧回调里 apply, 故复核必须直接查类别表, 否则慢帧率下用的是全局默认值。
        entry = self._class_entry(msg)
        if 'color_location_y' in entry:
            return float(entry['color_location_y'])
        return float(self.color_location_y)

    def _target_x_for(self, msg):
        entry = self._class_entry(msg)
        if 'color_location_x' in entry:
            return float(entry['color_location_x'])
        return float(self.color_location_x)

    def _angle_error(self, msg):
        return msg.angle_x - self._target_y_for(msg)

    def _angle_ready(self, msg):
        return abs(self._angle_error(msg)) <= self.angle_tolerance

    def _angle_y_ready(self, msg):
        return abs(float(msg.angle_y) - self._target_x_for(msg)) <= self.angle_tolerance

    def _target_fresh_for_pick(self):
        if self.last_target_time is None:
            return False
        return self.get_clock().now() - self.last_target_time <= self.pick_target_timeout

    def _ready_to_pick(self):
        if self.last_target_msg is None or not self.last_target_msg.correct:
            return False
        return (
            self._target_fresh_for_pick() and
            self._distance_ready(self.last_target_msg) and
            self._angle_ready(self.last_target_msg))

    def _pid_z(self):
        error = self.angular_z_val - self.color_location_y
        output = self.z_d * self.last_error_z + self.z_p * error
        self.last_error_z = error
        return output

    def _angular_z_from_target(self, msg):
        angle_error = self._angle_error(msg)
        raw = self._pid_z()
        if self._angle_ready(msg):
            self.last_error_z = 0.0
            return 0.0

        angular = self._clamp(
            self.angle_x_angular_sign * raw,
            -self.max_angular_speed,
            self.max_angular_speed)
        # 接近容差边缘时不要强行套最小角速度，否则 BPU 慢帧率下容易越过目标后持续转圈。
        use_min_speed = abs(angle_error) > self.angle_tolerance * 1.5
        if use_min_speed and abs(angular) < self.min_angular_speed:
            angular = math.copysign(
                self.min_angular_speed,
                self.angle_x_angular_sign * (raw if raw != 0.0 else angle_error))
        return angular

    # ================================================================
    #  发布函数
    # ================================================================

    def _publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.target_liner_x
        msg.linear.y = self.target_liner_y
        msg.angular.z = self.target_angular_z
        self.cmd_vel_pub.publish(msg)

    def refresh_params(self):
        self.x_p = self.get_parameter('x_p').value
        self.x_d = self.get_parameter('x_d').value
        self.z_p = self.get_parameter('z_p').value
        self.z_d = self.get_parameter('z_d').value
        self.color_location_x = self.get_parameter('color_location_x').value
        self.color_location_y = self.get_parameter('color_location_y').value
        self.location_threshold = self.get_parameter('location_threshold').value
        self.speed_thresh_x = self.get_parameter('speed_threshold_x').value
        self.speed_thresh_z = self.get_parameter('speed_threshold_z').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.rotation_tol = self.get_parameter('rotation_tolerance').value
        self.error_timeout = self.get_parameter('error_timeout').value
        self.target_timeout = Duration(seconds=float(self.get_parameter('target_timeout_sec').value))
        self.cmd_vel_timeout = Duration(seconds=float(self.get_parameter('cmd_vel_timeout_sec').value))
        self.pick_target_timeout = Duration(seconds=float(self.get_parameter('pick_target_timeout_sec').value))
        self.use_depth_distance_control = self.get_parameter('use_depth_distance_control').value
        self.enable_angle_y_linear_control = bool(
            self.get_parameter('enable_angle_y_linear_control').value)
        self.target_distance = self.get_parameter('target_distance').value
        self.class_3_target_distance_enabled = bool(
            self.get_parameter('class_3_target_distance_enabled').value)
        self.class_3_target_distance = self.get_parameter('class_3_target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.linear_toward_sign = self.get_parameter('linear_toward_sign').value
        self.angle_y_linear_sign = self.get_parameter('angle_y_linear_sign').value
        self.angle_x_angular_sign = self.get_parameter('angle_x_angular_sign').value
        self.single_pick_mode = self.get_parameter('single_pick_mode').value

        # 重建每类别甜点表(只覆盖非 nan 的字段,其余回退全局默认)。
        self._class_targets = {}
        for name in self._class_target_names:
            entry = {}
            cx = float(self.get_parameter(f'class_target.{name}.color_location_x').value)
            cy = float(self.get_parameter(f'class_target.{name}.color_location_y').value)
            td = float(self.get_parameter(f'class_target.{name}.target_distance').value)
            if not math.isnan(cx):
                entry['color_location_x'] = cx
            if not math.isnan(cy):
                entry['color_location_y'] = cy
            if not math.isnan(td):
                entry['target_distance'] = td
            self._class_targets[name] = entry

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def _class_entry(self, msg):
        return self._class_targets.get(getattr(msg, 'color', None), {})

    def _apply_class_target(self, msg):
        """按检测到的类别覆盖横向/垂直甜点(查不到则保持全局默认)。"""
        entry = self._class_entry(msg)
        if 'color_location_x' in entry:
            self.color_location_x = entry['color_location_x']
        if 'color_location_y' in entry:
            self.color_location_y = entry['color_location_y']

    def _target_distance_for(self, msg):
        entry = self._class_entry(msg)
        if 'target_distance' in entry:
            return float(entry['target_distance'])
        # 兼容旧的 class_3(维生素胶囊)专用距离特例。
        if self.class_3_target_distance_enabled and msg.color == '维生素胶囊':
            return float(self.class_3_target_distance)
        return float(self.target_distance)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stop_tracking_motion(self, publish=False, reset_location_count=True):
        self.target_liner_x = 0.0
        self.target_liner_y = 0.0
        self.target_angular_z = 0.0
        self.distance_ready = False
        if reset_location_count:
            self.location_count = 0
        if publish:
            self._publish_cmd_vel()

    def _reset_pick_state(self):
        self.car_state = 0
        self.target_angle = 0.0
        self.shake_hand = 0
        self.location_flag = 0
        self.move_flag = 0
        self.error_flag = 1
        self.error_count = 0
        self.location_count = 0
        self.last_target_time = None
        self.last_target_msg = None
        self.pick_requested = False
        self.pick_complete = False
        self.shake_hand_done = [0, 0, 0]
        self._stop_tracking_motion(publish=True)

    def _publish_no_msg(self):
        msg = String()
        msg.data = 'no_msg'
        self.arm_state_pub.publish(msg)

    def _publish_arm_state(self):
        msg = String()

        if self.pick_complete or (self.single_pick_mode and self.pick_requested):
            msg.data = 'no_msg'
        elif self.shake_hand:
            msg.data = 'shake_hand'
        elif self.car_state == 1:
            # 六自由度默认使用云台旋转放置
            msg.data = 'rotate_put'
        elif self.location_flag == 1 and self.move_flag < 2 and self._ready_to_pick():
            msg.data = 'pick'
            if self.single_pick_mode:
                self.pick_requested = True
        elif self.location_flag == 1 and self.move_flag < 2:
            self.location_flag = 0
            msg.data = 'no_msg'
            if self.last_target_msg is not None:
                self.get_logger().warn(
                    f'pick blocked: stale or misaligned target '
                    f'angle_x={self.last_target_msg.angle_x:.4f} '
                    f'target={self._target_y_for(self.last_target_msg):.4f} '
                    f'distance={self.last_target_msg.distance:.3f}')
        elif self.move_flag in (2, 3):
            msg.data = 'put'
        else:
            msg.data = 'no_msg'

        self.arm_state_pub.publish(msg)

    def destroy_node(self):
        # 底盘固件无 cmd_vel 超时归零: 节点退出/被杀时若不补零速,
        # 底盘会一直执行最后一条速度指令 => 车失控前冲。退出前连发零速覆盖残留指令。
        try:
            stop = Twist()
            for _ in range(5):
                self.cmd_vel_pub.publish(stop)
                time.sleep(0.02)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarLocationColorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
