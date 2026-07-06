#!/usr/bin/env python3
# coding=utf-8
"""
串口桥接节点 (ROS 2)
功能: 订阅 /joint_states，将关节角度打包为 16 字节帧发送给 STM32 下位机
帧格式: [0xAA] [J1_H J1_L] [J2_H J2_L] ... [J6_H J6_L] [mode] [checksum] [0xBB]
参考: ROS 1 table_arm/src/wheeltec_table_arm.cpp
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import struct

try:
    import serial
except ImportError:
    serial = None

FRAME_HEADER = 0xAA
FRAME_TAIL   = 0xBB
MODE_DEFAULT  = 1
MODE_FOLLOWER = 2

# 串口帧需要发送的臂关节名称 (按下位机期望顺序)
ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/wheeltec_controller')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.ser = None
        if serial is None:
            self.get_logger().error('pyserial 未安装，请执行: pip install pyserial')
        else:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=2)
                self.get_logger().info(f'串口 {port} 已打开 (baud={baudrate})')
            except serial.SerialException as e:
                self.get_logger().error(f'串口打开失败: {e}')

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 100)

        self.get_logger().info('serial_bridge_node 初始化完成')

    def joint_callback(self, msg: JointState):
        if self.ser is None or not self.ser.is_open:
            return

        # 根据关节名称查找位置值
        positions = [0.0] * 6   # 6 个关节槽位
        for i, name in enumerate(ARM_JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        # positions[5] 保留为 0 (第 6 个槽位, 如需夹爪可扩展)

        tx = bytearray(16)
        tx[0] = FRAME_HEADER

        # 逐关节打包: float(rad) * 1000 → int16 → 高字节 + 低字节
        for i in range(6):
            val = int(positions[i] * 1000)
            val = max(-32768, min(32767, val))
            tx[1 + i * 2] = (val >> 8) & 0xFF   # 高 8 位
            tx[2 + i * 2] = val & 0xFF           # 低 8 位

        tx[13] = MODE_DEFAULT

        # XOR 校验 (tx[0] ~ tx[13])
        checksum = 0
        for j in range(14):
            checksum ^= tx[j]
        tx[14] = checksum
        tx[15] = FRAME_TAIL

        try:
            self.ser.write(tx)
        except Exception as e:
            self.get_logger().error(f'串口写入失败: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
