#!/usr/bin/env python3
# coding=utf-8
"""Publish a camera image with a fixed grasp placement guide box."""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class GraspGuideOverlay(Node):
    def __init__(self):
        super().__init__('grasp_guide_overlay_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/camera/color/image_grasp_guide')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('horizontal_angle', 0.5236)
        self.declare_parameter('vertical_angle', 0.4320)
        # Same mapping as car_location_color/direct_arm_pick_and_put:
        # color_location_y is visual angle_x, color_location_x is visual angle_y.
        self.declare_parameter('color_location_x', -0.1094)
        self.declare_parameter('color_location_y', -0.0763)
        self.declare_parameter('guide_box_width', 80)
        self.declare_parameter('guide_box_height', 190)
        self.declare_parameter('guide_label', 'PLACE TARGET HERE')
        self.declare_parameter('draw_crosshair', True)

        self.bridge = CvBridge()
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)

        self.get_logger().info(
            f'grasp guide overlay: {self.input_topic} -> {self.output_topic}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        guide = self.draw_guide(frame, width, height)
        out = self.bridge.cv2_to_imgmsg(guide, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    def draw_guide(self, frame, width, height):
        image = frame.copy()
        cx, cy = self.guide_center(width, height)
        box_w, box_h = self.guide_size(width, height)

        x1 = int(round(max(0, cx - box_w * 0.5)))
        y1 = int(round(max(0, cy - box_h * 0.5)))
        x2 = int(round(min(width - 1, cx + box_w * 0.5)))
        y2 = int(round(min(height - 1, cy + box_h * 0.5)))
        cx_i = int(round(cx))
        cy_i = int(round(cy))

        color = (0, 255, 0)
        shadow = (0, 80, 0)
        cv2.rectangle(image, (x1 - 1, y1 - 1), (x2 + 1, y2 + 1), shadow, 4)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        if bool(self.get_parameter('draw_crosshair').value):
            cv2.line(image, (cx_i - 14, cy_i), (cx_i + 14, cy_i), color, 2)
            cv2.line(image, (cx_i, cy_i - 14), (cx_i, cy_i + 14), color, 2)
            cv2.circle(image, (cx_i, cy_i), 4, color, -1)

        label = str(self.get_parameter('guide_label').value)
        if label:
            cv2.putText(
                image, label, (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, shadow, 4, cv2.LINE_AA)
            cv2.putText(
                image, label, (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        return image

    def guide_center(self, width, height):
        input_size = float(self.get_parameter('input_size').value)
        horizontal_angle = float(self.get_parameter('horizontal_angle').value)
        vertical_angle = float(self.get_parameter('vertical_angle').value)
        angle_x = float(self.get_parameter('color_location_y').value)
        angle_y = float(self.get_parameter('color_location_x').value)

        x_in = (1.0 - math.tan(angle_x) / math.tan(horizontal_angle)) * 0.5 * input_size
        y_in = (1.0 - math.tan(angle_y) / math.tan(vertical_angle)) * 0.5 * input_size
        return x_in * width / input_size, y_in * height / input_size

    def guide_size(self, width, height):
        input_size = float(self.get_parameter('input_size').value)
        box_w = float(self.get_parameter('guide_box_width').value)
        box_h = float(self.get_parameter('guide_box_height').value)
        return box_w * width / input_size, box_h * height / input_size


def main(args=None):
    rclpy.init(args=args)
    node = GraspGuideOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
