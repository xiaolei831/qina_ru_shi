#!/usr/bin/env python3
# coding=utf-8
"""
药品视觉检测节点 (ROS 2) — BPU data2_best_bayese_640x640_nv12.bin

功能:
  1. 订阅 USB 摄像头图像
  2. 调用 hrt_model_exec 使用 BPU 运行药品检测模型
  3. 解析 YOLO/DFL 检测输出，选择目标药品框
  4. 发布 /color_position (SixArmPosition) 给底盘定位节点复用

说明:
  SixArmPosition.color 保持原字段名，但这里填入 medicine 或 class_N。
"""

import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageDraw as PILImageDraw, ImageFont as PILImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String

from robot_arm_control.msg import SixArmPosition


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x, axis=-1):
    x = x - np.max(x, axis=axis, keepdims=True)
    exp = np.exp(x)
    return exp / np.sum(exp, axis=axis, keepdims=True)


def nms(boxes, scores, iou_threshold):
    if len(boxes) == 0:
        return []

    boxes = np.asarray(boxes, dtype=np.float32)
    scores = np.asarray(scores, dtype=np.float32)
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        inter = w * h
        union = areas[i] + areas[order[1:]] - inter + 1e-9
        iou = inter / union
        order = order[1:][iou <= iou_threshold]

    return keep


MEDICINE_NAME_ALIASES = {
    '莲花清温颗粒': '莲花清瘟胶囊',
    '莲花清瘟颗粒': '莲花清瘟胶囊',
    '连花清温颗粒': '莲花清瘟胶囊',
    '连花清瘟颗粒': '莲花清瘟胶囊',
    '连花清瘟胶囊': '莲花清瘟胶囊',
    '银翅解毒片': '银翘解毒片',
}


class VisualTrackerNode(Node):
    def __init__(self):
        super().__init__('visual_tracker_node')

        default_model_path = str(
            Path(get_package_share_directory('robot_arm_control')) / 'model' / 'data2_best_bayese_640x640_nv12.bin'
        )

        self.declare_parameter('picture_height', 480)
        self.declare_parameter('picture_width', 640)
        self.declare_parameter('vertical_angle', 0.4320)
        self.declare_parameter('horizontal_angle', 0.5236)
        self.declare_parameter('camera_topic', '/usb_cam/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('model_name', 'data2_best_bayese_640x640_nv12')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('num_classes', 5)
        self.declare_parameter('reg_max', 16)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('target_class_id', -1)
        self.declare_parameter('class_names', ['medicine', 'class_1', 'class_2', 'class_3', 'class_4'])
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('log_detection_status', True)
        self.declare_parameter('direct_arm_trigger', True)
        self.declare_parameter('trigger_center_tolerance_x', 0.08)
        self.declare_parameter('trigger_center_tolerance_y', 0.08)
        self.declare_parameter('trigger_required_count', 3)
        self.declare_parameter('debug_image_topic', '/bpu_detection/image')
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('depth_roi_ratio', 0.33)
        self.declare_parameter('depth_offset', 0.03)
        self.declare_parameter('enabled', True)
        self.declare_parameter('enable_topic', '/medicine_pick_enable')
        self.declare_parameter('target_medicine_topic', '/target_medicine')

        self.pic_h = int(self.get_parameter('picture_height').value)
        self.pic_w = int(self.get_parameter('picture_width').value)
        vert_angle = float(self.get_parameter('vertical_angle').value)
        horiz_angle = float(self.get_parameter('horizontal_angle').value)
        camera_topic = str(self.get_parameter('camera_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.model_path = Path(str(self.get_parameter('model_path').value))
        self.model_name = str(self.get_parameter('model_name').value)
        self.input_size = int(self.get_parameter('input_size').value)
        self.num_classes = int(self.get_parameter('num_classes').value)
        self.reg_max = int(self.get_parameter('reg_max').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.class_names = list(self.get_parameter('class_names').value)
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.log_detection_status = bool(self.get_parameter('log_detection_status').value)
        self.direct_arm_trigger = bool(self.get_parameter('direct_arm_trigger').value)
        self.trigger_center_tolerance_x = float(self.get_parameter('trigger_center_tolerance_x').value)
        self.trigger_center_tolerance_y = float(self.get_parameter('trigger_center_tolerance_y').value)
        self.trigger_required_count = max(1, int(self.get_parameter('trigger_required_count').value))
        debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.depth_unit_scale = float(self.get_parameter('depth_unit_scale').value)
        self.depth_roi_ratio = float(self.get_parameter('depth_roi_ratio').value)
        self.depth_offset = float(self.get_parameter('depth_offset').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        target_medicine_topic = str(self.get_parameter('target_medicine_topic').value)
        self.target_medicine = ''
        self.centered_count = 0
        self.arm_triggered = False

        self.tan_v = np.tan(vert_angle)
        self.tan_h = np.tan(horiz_angle)
        self.frame_count = 0
        self.init_count = 0
        self.bridge = CvBridge()
        self.tmp_root = Path(tempfile.mkdtemp(prefix='robot_arm_bpu_detect_'))
        # cv2.putText 不支持中文(画成方块),改用 PIL + CJK 字体绘制标签。
        self.cn_label_size = 20
        self.cn_font = self._load_cn_font(self.cn_label_size)

        self.pos_pub = self.create_publisher(SixArmPosition, '/color_position', 10)
        self.flag_pub = self.create_publisher(Int8, '/visual_clamp_flag', 1)
        self.arm_state_pub = self.create_publisher(String, '/arm_state', 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.enable_callback, 10)
        self.target_medicine_sub = self.create_subscription(
            String, target_medicine_topic, self.target_medicine_callback, 10)
        self.image_sub = None
        self.depth_sub = None
        self.time_sync = None
        if self.use_depth:
            self.image_sub = message_filters.Subscriber(self, Image, camera_topic)
            self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self.time_sync = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_sub], 10, 0.1)
            self.time_sync.registerCallback(self.synced_image_callback)
            self.get_logger().info(f'visual_tracker_node 启用深度话题: {depth_topic}')
        else:
            self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)

        if not self.model_path.exists():
            self.get_logger().error(f'BPU model not found: {self.model_path}')

        self.get_logger().info(f'visual_tracker_node 使用 BPU 药品检测模型: {self.model_path}')

    def enable_callback(self, msg):
        enabled = bool(msg.data)
        if enabled != self.enabled:
            self.centered_count = 0
            self.arm_triggered = False
            self.get_logger().info(f'visual tracker enabled={enabled}')
        self.enabled = enabled

    def target_medicine_callback(self, msg):
        target = msg.data.strip()
        if target != self.target_medicine:
            self.centered_count = 0
            self.arm_triggered = False
            self.get_logger().info(
                f'visual tracker target medicine: {target if target else "any"}')
        self.target_medicine = target

    def destroy_node(self):
        shutil.rmtree(self.tmp_root, ignore_errors=True)
        return super().destroy_node()

    def synced_image_callback(self, image_msg, depth_msg):
        self.image_callback(image_msg, depth_msg)

    def image_callback(self, msg, depth_msg=None):
        if not self.enabled:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth_frame = None
        if depth_msg is not None:
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        frame = cv2.resize(frame, (self.input_size, self.input_size), interpolation=cv2.INTER_AREA)
        if depth_frame is not None:
            depth_frame = cv2.resize(depth_frame, (self.input_size, self.input_size), interpolation=cv2.INTER_NEAREST)

        if self.init_count < 3:
            self.init_count += 1
            return
        if self.init_count == 3:
            flag_msg = Int8()
            flag_msg.data = 1
            self.flag_pub.publish(flag_msg)
            self.init_count = 4

        detections = self.run_bpu_detection(frame)
        target = self.select_target(detections)
        if self.log_detection_status:
            self.log_detections(detections, target, depth_frame)
        self.publish_debug_image(frame, detections, target, msg.header)
        if target is None:
            self.centered_count = 0
            pos_msg = SixArmPosition()
            pos_msg.angle_x = 0.0
            pos_msg.angle_y = 0.0
            pos_msg.distance = 0.0
            pos_msg.correct = False
            pos_msg.color = 'none'
            self.pos_pub.publish(pos_msg)
            return

        x1, y1, x2, y2 = target['box']
        center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
        raw_distance = self.target_depth(target, depth_frame)
        distance = raw_distance + self.depth_offset if raw_distance > 0.0 else raw_distance

        pos_msg = SixArmPosition()
        pos_msg.angle_x = float(self.calc_angle_x(center))
        pos_msg.angle_y = float(self.calc_angle_y(center))
        pos_msg.distance = float(distance)
        pos_msg.correct = True
        pos_msg.color = self.class_name(target['class_id'])
        self.pos_pub.publish(pos_msg)
        self.maybe_trigger_arm(pos_msg)
        if self.log_detection_status:
            depth_info = self.target_depth_info(target, depth_frame)
            self.get_logger().info(
                'BPU medicine detection: publish target '
                f'class={pos_msg.color} score={target["score"]:.3f} '
                f'angle_x={pos_msg.angle_x:.4f} angle_y={pos_msg.angle_y:.4f} '
                f'distance={pos_msg.distance:.3f}m '
                f'center=({center[0]:.1f},{center[1]:.1f}) {depth_info}'
            )

    def log_detections(self, detections, target, depth_frame):
        if not detections:
            self.get_logger().info('BPU medicine detection: count=0 target=none')
            return

        target_box = target['box'] if target is not None else None
        parts = []
        for idx, det in enumerate(detections):
            x1, y1, x2, y2 = det['box']
            center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
            angle_x = float(self.calc_angle_x(center))
            angle_y = float(self.calc_angle_y(center))
            distance = self.target_depth(det, depth_frame)
            depth_info = self.target_depth_info(det, depth_frame)
            marker = '*' if target_box == det['box'] else '-'
            parts.append(
                f'{marker}#{idx} class={self.class_name(det["class_id"])} '
                f'id={det["class_id"]} score={det["score"]:.3f} '
                f'box=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}) '
                f'center=({center[0]:.1f},{center[1]:.1f}) '
                f'angle=({angle_x:.4f},{angle_y:.4f}) distance={distance:.3f}m {depth_info}'
            )

        target_desc = 'none'
        if target is not None:
            target_desc = f'{self.class_name(target["class_id"])} score={target["score"]:.3f}'
        self.get_logger().info(
            f'BPU medicine detection: count={len(detections)} target={target_desc} | ' + ' | '.join(parts)
        )

    def target_depth(self, target, depth_frame):
        info = self._target_depth_roi(target, depth_frame)
        if info is None:
            return 0.0

        _, _, _, _, roi = info
        values = roi.astype(np.float32).reshape(-1)
        values = values[np.isfinite(values)]
        values = values[values > 0]
        if values.size == 0:
            return 0.0
        return float(np.median(values) * self.depth_unit_scale)

    def target_depth_info(self, target, depth_frame):
        info = self._target_depth_roi(target, depth_frame)
        if info is None:
            return 'depth_roi=none valid_depth=0'

        rx1, ry1, rx2, ry2, roi = info
        values = roi.astype(np.float32).reshape(-1)
        values = values[np.isfinite(values)]
        values = values[values > 0]
        if values.size == 0:
            return f'depth_roi=({rx1},{ry1},{rx2},{ry2}) valid_depth=0'

        median_depth = float(np.median(values) * self.depth_unit_scale)
        return (
            f'depth_roi=({rx1},{ry1},{rx2},{ry2}) '
            f'valid_depth={values.size} median_depth={median_depth:.3f}m')

    def _target_depth_roi(self, target, depth_frame):
        if depth_frame is None:
            return None

        x1, y1, x2, y2 = [int(round(v)) for v in target['box']]
        width = max(1, x2 - x1)
        height = max(1, y2 - y1)
        roi_w = max(1, int(width * self.depth_roi_ratio))
        roi_h = max(1, int(height * self.depth_roi_ratio))
        cx = int(round((x1 + x2) * 0.5))
        cy = int(round((y1 + y2) * 0.5))
        rx1 = max(0, cx - roi_w // 2)
        ry1 = max(0, cy - roi_h // 2)
        rx2 = min(depth_frame.shape[1], cx + roi_w // 2 + 1)
        ry2 = min(depth_frame.shape[0], cy + roi_h // 2 + 1)
        roi = depth_frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return None
        return rx1, ry1, rx2, ry2, roi

    def maybe_trigger_arm(self, pos_msg):
        if not self.direct_arm_trigger or self.arm_triggered:
            return

        if (abs(pos_msg.angle_x) <= self.trigger_center_tolerance_x and
                abs(pos_msg.angle_y) <= self.trigger_center_tolerance_y):
            self.centered_count += 1
        else:
            self.centered_count = 0

        if self.centered_count < self.trigger_required_count:
            return

        arm_msg = String()
        arm_msg.data = 'pick'
        self.arm_state_pub.publish(arm_msg)
        self.arm_triggered = True
        self.get_logger().info('BPU medicine detection: target centered, publish /arm_state=pick')

    def run_bpu_detection(self, frame):
        if not self.model_path.exists():
            return []

        frame_id = self.frame_count
        image_path = self.tmp_root / f'frame_{frame_id}.jpg'
        dump_dir = self.tmp_root / f'dump_{frame_id}'
        if dump_dir.exists():
            shutil.rmtree(dump_dir)
        dump_dir.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(image_path), frame)
        cmd = [
            'hrt_model_exec', 'infer',
            '--model_file', str(self.model_path),
            '--model_name', self.model_name,
            '--input_file', str(image_path),
            '--enable_dump', 'true',
            '--dump_format', 'bin',
            '--dump_path', str(dump_dir),
        ]

        try:
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            return self.read_outputs(dump_dir, frame.shape)
        except (subprocess.CalledProcessError, FileNotFoundError, ValueError) as exc:
            self.get_logger().warning(f'BPU inference failed: {exc}')
            return []
        finally:
            image_path.unlink(missing_ok=True)
            shutil.rmtree(dump_dir, ignore_errors=True)

    def read_outputs(self, dump_dir, image_shape):
        outputs = [
            ('model_infer_output_0_output0.bin', (80, 80, self.num_classes),
             'model_infer_output_1_324.bin', (80, 80, 4 * self.reg_max), 8),
            ('model_infer_output_2_332.bin', (40, 40, self.num_classes),
             'model_infer_output_3_340.bin', (40, 40, 4 * self.reg_max), 16),
            ('model_infer_output_4_348.bin', (20, 20, self.num_classes),
             'model_infer_output_5_356.bin', (20, 20, 4 * self.reg_max), 32),
        ]

        detections = []
        for cls_name, cls_shape, reg_name, reg_shape, stride in outputs:
            cls_path = dump_dir / cls_name
            reg_path = dump_dir / reg_name
            cls_map = np.fromfile(cls_path, dtype=np.float32).reshape(cls_shape)
            reg_map = np.fromfile(reg_path, dtype=np.float32).reshape(reg_shape)
            detections.extend(self.decode_scale(cls_map, reg_map, stride, image_shape))

        final = []
        for class_id in range(self.num_classes):
            class_dets = [det for det in detections if det['class_id'] == class_id]
            if not class_dets:
                continue
            keep = nms([d['box'] for d in class_dets], [d['score'] for d in class_dets], self.nms_threshold)
            final.extend(class_dets[i] for i in keep)

        return sorted(final, key=lambda d: d['score'], reverse=True)

    def decode_scale(self, cls_map, reg_map, stride, image_shape):
        if cls_map.min() < 0 or cls_map.max() > 1:
            cls_scores = sigmoid(cls_map)
        else:
            cls_scores = cls_map

        best_cls = np.argmax(cls_scores, axis=-1)
        best_score = np.max(cls_scores, axis=-1)
        ys, xs = np.where(best_score >= self.confidence_threshold)
        if len(xs) == 0:
            return []

        reg = reg_map[ys, xs].reshape(-1, 4, self.reg_max)
        bins = np.arange(self.reg_max, dtype=np.float32)
        distances = (softmax(reg, axis=-1) * bins).sum(axis=-1) * stride
        cx = (xs.astype(np.float32) + 0.5) * stride
        cy = (ys.astype(np.float32) + 0.5) * stride

        x1 = cx - distances[:, 0]
        y1 = cy - distances[:, 1]
        x2 = cx + distances[:, 2]
        y2 = cy + distances[:, 3]
        src_h, src_w = image_shape[:2]

        detections = []
        for i in range(len(xs)):
            bx1 = float(np.clip(x1[i], 0, src_w - 1))
            by1 = float(np.clip(y1[i], 0, src_h - 1))
            bx2 = float(np.clip(x2[i], 0, src_w - 1))
            by2 = float(np.clip(y2[i], 0, src_h - 1))
            if bx2 <= bx1 or by2 <= by1:
                continue
            detections.append({
                'box': [bx1, by1, bx2, by2],
                'score': float(best_score[ys[i], xs[i]]),
                'class_id': int(best_cls[ys[i], xs[i]]),
            })
        return detections

    def select_target(self, detections):
        if self.target_class_id >= 0:
            detections = [det for det in detections if det['class_id'] == self.target_class_id]
        if self.target_medicine:
            target_name = self.normalize_medicine_name(self.target_medicine)
            detections = [
                det for det in detections
                if self.normalize_medicine_name(self.class_name(det['class_id'])) == target_name
            ]
        if not detections:
            return None

        def center_error(det):
            x1, y1, x2, y2 = det['box']
            center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
            angle_x = float(self.calc_angle_x(center))
            angle_y = float(self.calc_angle_y(center))
            return (abs(angle_x) + abs(angle_y), -det['score'])

        return min(detections, key=center_error)

    def publish_debug_image(self, frame, detections, target, header):
        debug = frame.copy()
        target_box = target['box'] if target is not None else None

        # 矩形框用 OpenCV 画(纯几何,无文字),中文标签统一用 PIL 画,
        # 因为 cv2.putText 不支持非 ASCII 字符(中文会渲染成 ??? / 方块)。
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['box']]
            is_target = target_box == det['box']
            color = (0, 255, 0) if is_target else (255, 0, 0)  # BGR
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)

        labels = []
        for det in detections:
            x1, y1, _, _ = [int(v) for v in det['box']]
            is_target = target_box == det['box']
            rgb = (0, 255, 0) if is_target else (255, 0, 0)
            labels.append((self.class_name(det['class_id']),
                           (x1, max(2, y1 - self.cn_label_size - 4)), rgb))
        status = 'target' if target is not None else 'no target'
        labels.append((f'BPU: {status}', (10, 6), (255, 255, 0)))
        debug = self._draw_cn_labels(debug, labels)

        image_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        image_msg.header = header
        self.debug_image_pub.publish(image_msg)

    def _load_cn_font(self, size):
        """加载中文字体; 找不到任何可用字体时返回 None(回退到 ASCII-only 的 cv2.putText)。"""
        candidates = [
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        ]
        for path in candidates:
            if Path(path).exists():
                try:
                    return PILImageFont.truetype(path, size)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(f'load CJK font failed {path}: {exc}')
        self.get_logger().warning('no CJK font found; debug image labels fall back to ASCII only')
        return None

    def _draw_cn_labels(self, bgr_image, labels):
        """用 PIL 在 BGR 图上绘制(可含中文)文字标签。
        labels: [(text, (x, y), (r, g, b)), ...]; 字体缺失时回退到 cv2.putText(仅 ASCII)。
        """
        if self.cn_font is None:
            for text, (x, y), (r, g, b) in labels:
                cv2.putText(bgr_image, text, (x, y + self.cn_label_size),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (b, g, r), 2, cv2.LINE_AA)
            return bgr_image
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        draw = PILImageDraw.Draw(pil)
        for text, (x, y), color in labels:
            draw.text((x, y), text, font=self.cn_font, fill=color)
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def class_name(self, class_id):
        if 0 <= class_id < len(self.class_names):
            return str(self.class_names[class_id])
        return f'class_{class_id}'

    @staticmethod
    def normalize_medicine_name(name):
        clean = str(name).strip()
        return MEDICINE_NAME_ALIASES.get(clean, clean)

    def calc_angle_x(self, pos):
        displacement = 2 * pos[0] / self.input_size - 1
        return -np.arctan(displacement * self.tan_h)

    def calc_angle_y(self, pos):
        displacement = 2 * pos[1] / self.input_size - 1
        return -np.arctan(displacement * self.tan_v)


def main(args=None):
    rclpy.init(args=args)
    node = VisualTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
