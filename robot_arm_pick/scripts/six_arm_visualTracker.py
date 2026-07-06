#!/usr/bin/env python3
# coding=utf-8
"""
色块识别节点 (ROS 2) — 适用于六自由度机械臂小车
参考: ROS 1 wheeltec_arm_pick/scripts/six_arm_visualTracker.py
"""

import numpy as np
import cv2
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from robot_arm_pick.msg import SixArmPosition as PositionMsg

np.seterr(all='raise')

KERNEL = np.ones((3, 3), dtype=np.uint8)

""" 判断图像中是否存在某种颜色 """    
def judge_color(hsv,lower,upper):
	pic = cv2.inRange(hsv, lower, upper)
	pic = cv2.erode(pic, KERNEL, iterations=5)
	pic = cv2.dilate(pic, KERNEL, iterations=3)
	contours = cv2.findContours(pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	for contour in sorted(contours, key=cv2.contourArea, reverse=True):
		area = cv2.contourArea(contour)
		if area>200:
			return True
		else:
			return False
		

class VisualTrackerNode(Node):
    def __init__(self):
        super().__init__('visual_tracker')
        self.bridge = CvBridge()
        self.i = 0
        self.m = 0  # 当前选择的颜色索引

        """ 参数声明与读取 """
        self.declare_parameter('picture_height', 480)
        self.declare_parameter('picture_width', 640)
        self.declare_parameter('vertical_angle', 0.4320)
        self.declare_parameter('horizontal_angle', 0.5236)
        self.declare_parameter('camera_topic', '/usb_cam/image_raw')
        self.declare_parameter('min_area', 200)
        self.declare_parameter('yellow_upper', [80, 100, 40])
        self.declare_parameter('yellow_lower', [100, 255, 255])
        self.declare_parameter('blue_upper', [0, 60, 40])
        self.declare_parameter('blue_lower', [30, 255, 255])
        self.declare_parameter('green_upper', [35, 43, 46])
        self.declare_parameter('green_lower', [77, 255, 255])
        self.declare_parameter('target_upper', [0, 43, 46])
        self.declare_parameter('target_lower', [10, 255, 255])

        self.pictureHeight = int(self.get_parameter('picture_height').value)
        self.pictureWidth = int(self.get_parameter('picture_width').value)
        vertAngle = float(self.get_parameter('vertical_angle').value)
        horizontalAngle = float(self.get_parameter('horizontal_angle').value)
        camera_topic = str(self.get_parameter('camera_topic').value)
        self.min_area = float(self.get_parameter('min_area').value)

        self.tanVertical = np.tan(vertAngle)
        self.tanHorizontal = np.tan(horizontalAngle)

        self.upper_yellow = np.array(self.get_parameter('yellow_upper').value)
        self.lower_yellow = np.array(self.get_parameter('yellow_lower').value)
        self.upper_blue = np.array(self.get_parameter('blue_upper').value)
        self.lower_blue = np.array(self.get_parameter('blue_lower').value)
        self.upper_green = np.array(self.get_parameter('green_upper').value)
        self.lower_green = np.array(self.get_parameter('green_lower').value)
        self.targetUpper = np.array(self.get_parameter('target_upper').value)
        self.targetLower = np.array(self.get_parameter('target_lower').value)

        """ 话题订阅发布 """
        self.image_sub = self.create_subscription(Image, camera_topic, self.trackObject, 10)
        self.positionPublisher = self.create_publisher(PositionMsg, '/object_tracker/current_position', 3)
        self.color_sub = self.create_subscription(Int8, '/color_flag', self.colorflag_callback, 10)
        self.visualflagPublisher = self.create_publisher(Int8, '/visual_clamp_flag', 1)

        self.get_logger().debug(str(self.targetUpper))
        self.get_logger().info('visualTracker init done')

    #色块夹取标志位发布函数
    def publish_flag(self):
        visual_clamp_flag = Int8()
        visual_clamp_flag.data = 1
        time.sleep(1.0)
        self.visualflagPublisher.publish(visual_clamp_flag)
        self.get_logger().info('a=%d' % visual_clamp_flag.data)

    def colorflag_callback(self, msg):
        self.m = msg.data

    def trackObject(self, image_data):
        m = self.m
        #convert both images to numpy arrays
        frame0 = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
        frame = frame0[int(self.pictureHeight/16):int(self.pictureHeight/16*12),int(self.pictureWidth/16*4):int(self.pictureWidth/16*15)]
        frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)#提高帧率

        """ 转换为HSV """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if self.i < 3:
            self.i = self.i + 1
            return
        elif self.i == 3:
            self.publish_flag()
            self.i = 4

        # select all the pixels that are in the range specified by the target
        if m == 0:
            org_mask = cv2.inRange(hsv, self.upper_yellow, self.lower_yellow)
            obj_color = 'yellow'
        elif m == 1:
            org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
            obj_color = 'blue'
        elif m == 2:
            org_mask = cv2.inRange(hsv, self.upper_green, self.lower_green)
            obj_color = 'green'
        elif m == 3:
            org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)
            obj_color = 'user-defined'
        else:
            org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
            obj_color = 'blue'
        # clean that up a little, the iterations are pretty much arbitrary
        mask = cv2.erode(org_mask, KERNEL, iterations=5)
        mask1 = cv2.dilate(mask, KERNEL, iterations=3)

        # 寻找目标轮廓并获取轮廓图像
        contours = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # go threw all the contours. starting with the bigest one
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < self.min_area:
                break

            # get position of object for this contour
            centerRaw, size, rotation = cv2.minAreaRect(contour)
            angleX = self.calculateAngleX(centerRaw)
            angleY = self.calculateAngleY(centerRaw)
            self.publishPosition(angleX, angleY, True, obj_color) # x轴y轴偏移量及是否找到目标色块
            return

        """ 如果没找到目标色块，则判断是否存在其他颜色色块 """
        for i in range(3):
            if i == m:
                continue
            elif i == 0:
                if judge_color(hsv, self.upper_yellow, self.lower_yellow):
                    self.publishPosition(float("inf"), float("inf"), False, 'yellow')
            elif i == 1:
                if judge_color(hsv, self.upper_blue, self.lower_blue):
                    self.publishPosition(float("inf"), float("inf"), False, 'blue')
            elif i == 2:
                if judge_color(hsv, self.upper_green, self.lower_green):
                    self.publishPosition(float("inf"), float("inf"), False, 'green')

    def publishPosition(self, angleX, angleY, correct, color):
        posMsg = PositionMsg()
        posMsg.angle_x = float(angleX)
        posMsg.angle_y = float(angleY)
        posMsg.correct = correct
        posMsg.color = color
        self.positionPublisher.publish(posMsg)

    def calculateAngleX(self, pos):
        '''calculates the X angle of displacement from straight ahead'''
        centerX = pos[0]
        displacement = 2 * centerX / self.pictureWidth - 1
        angle = -1 * np.arctan(displacement * self.tanHorizontal)
        return angle

    def calculateAngleY(self, pos):
        centerY = pos[1]
        displacement = 2 * centerY / self.pictureHeight - 1
        angle = -1 * np.arctan(displacement * self.tanVertical)
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = VisualTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
