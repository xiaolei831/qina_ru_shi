#!/usr/bin/env python3
# coding=utf-8
"""药品识别节点 (Astra+ 相机 + BPU bin 模型 → vision_msgs/Detection2DArray).

订阅:
  - RGB:   /camera/color/image_raw   (sensor_msgs/Image, bgr8 / rgb8 / mono8)
  - Depth: /camera/depth/image_raw   (sensor_msgs/Image, 16UC1, mm)

发布:
  - /medicine_detections   (vision_msgs/Detection2DArray, header=color frame)
  - /medicine_detector/image_annotated  (sensor_msgs/Image, bgr8, 调试可视化)

每个 Detection2D 的 results[0] 中:
  - hypothesis.class_id : 字符串类别名 (medicine / class_1 / ...)
  - hypothesis.score    : 置信度
  - pose.pose.position.x = 框中心像素 cx (RGB 坐标)
  - pose.pose.position.y = 框中心像素 cy (RGB 坐标)
  - pose.pose.position.z = 深度 (米); 取不到深度时为 NaN
"""

import math
import time
import threading
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge

from hobot_dnn import pyeasy_dnn as dnn


# 模型常量, 跟 camera_medicine_detect.py 保持一致
INPUT_SIZE = 640
NUM_CLASSES = 5
REG_MAX = 16
DEFAULT_CLASS_NAMES = ["medicine", "class_1", "class_2", "class_3", "class_4"]
DEFAULT_MODEL = "/home/sunrise/qian_sai/data2_best_bayese_640x640_nv12.bin"
STRIDES = (8, 16, 32)


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x - np.max(x, axis=axis, keepdims=True)
    exp = np.exp(x)
    return exp / np.sum(exp, axis=axis, keepdims=True)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> List[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: List[int] = []
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
        order = order[1:][iou <= iou_thres]
    return keep


def decode_scale(cls_map: np.ndarray, reg_map: np.ndarray, stride: int,
                 conf_thres: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """对单个尺度 head 解码, 返回 (boxes_xyxy[N,4], scores[N], cls_ids[N]).

    boxes 是在 INPUT_SIZE x INPUT_SIZE 图上的像素坐标.
    """
    if cls_map.ndim == 4:
        cls_map = cls_map[0]
    if reg_map.ndim == 4:
        reg_map = reg_map[0]

    if cls_map.min() < 0 or cls_map.max() > 1:
        cls_scores = sigmoid(cls_map)
    else:
        cls_scores = cls_map

    best_cls = np.argmax(cls_scores, axis=-1)
    best_score = np.max(cls_scores, axis=-1)
    ys, xs = np.where(best_score >= conf_thres)
    if len(xs) == 0:
        return np.empty((0, 4), np.float32), np.empty((0,), np.float32), np.empty((0,), np.int32)

    reg = reg_map[ys, xs].reshape(-1, 4, REG_MAX)
    bins = np.arange(REG_MAX, dtype=np.float32)
    distances = (softmax(reg, axis=-1) * bins).sum(axis=-1) * stride

    cx = (xs.astype(np.float32) + 0.5) * stride
    cy = (ys.astype(np.float32) + 0.5) * stride
    x1 = cx - distances[:, 0]
    y1 = cy - distances[:, 1]
    x2 = cx + distances[:, 2]
    y2 = cy + distances[:, 3]

    boxes = np.stack([x1, y1, x2, y2], axis=1)
    boxes = np.clip(boxes, 0, INPUT_SIZE - 1)
    scores = best_score[ys, xs]
    cls_ids = best_cls[ys, xs].astype(np.int32)

    valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    return boxes[valid], scores[valid], cls_ids[valid]


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    """把 BGR HxWx3 转 NV12 (uint8, [Y plane | UV interleave])."""
    h, w = bgr.shape[:2]
    assert h % 2 == 0 and w % 2 == 0, "NV12 需要偶数尺寸"
    yuv_i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
    y_plane = yuv_i420[:h, :]
    u_plane = yuv_i420[h:h + h // 4, :].reshape(h // 2, w // 2)
    v_plane = yuv_i420[h + h // 4:, :].reshape(h // 2, w // 2)
    uv_plane = np.empty((h // 2, w), dtype=np.uint8)
    uv_plane[:, 0::2] = u_plane
    uv_plane[:, 1::2] = v_plane
    nv12 = np.concatenate([y_plane.reshape(-1), uv_plane.reshape(-1)], axis=0)
    return nv12


class MedicineDetector(Node):
    def __init__(self) -> None:
        super().__init__("medicine_detector")

        # ---- 参数 ----
        self.declare_parameter("model_path", DEFAULT_MODEL)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("detections_topic", "/medicine_detections")
        self.declare_parameter("annotated_topic", "/medicine_detector/image_annotated")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("nms_thres", 0.45)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("class_names", DEFAULT_CLASS_NAMES)
        self.declare_parameter("depth_window", 5)            # 中心点的局部邻域 +- N 像素
        self.declare_parameter("depth_grid", 7)              # 在检测框内均匀采样 grid x grid 个点
        self.declare_parameter("depth_min_valid_ratio", 0.05) # 最少有效像素占比, 不够则视为无深度
        self.declare_parameter("depth_min_m", 0.3)           # 有效深度下限 (m)
        self.declare_parameter("depth_max_m", 6.0)           # 有效深度上限 (m)

        gp = self.get_parameter
        self.model_path = str(gp("model_path").value)
        self.color_topic = str(gp("color_topic").value)
        self.depth_topic = str(gp("depth_topic").value)
        det_topic = str(gp("detections_topic").value)
        ann_topic = str(gp("annotated_topic").value)
        self.conf_thres = float(gp("conf_thres").value)
        self.nms_thres = float(gp("nms_thres").value)
        self.publish_annotated = bool(gp("publish_annotated").value)
        self.class_names = list(gp("class_names").value)
        self.depth_window = max(1, int(gp("depth_window").value))
        self.depth_grid = max(1, int(gp("depth_grid").value))
        self.depth_min_valid_ratio = float(gp("depth_min_valid_ratio").value)
        self.depth_min_m = float(gp("depth_min_m").value)
        self.depth_max_m = float(gp("depth_max_m").value)

        # ---- 模型 ----
        self.get_logger().info(f"Loading BPU model: {self.model_path}")
        self._models = dnn.load(self.model_path)
        self._model = self._models[0]
        self.get_logger().info(
            f"Model loaded. inputs={len(self._model.inputs)} "
            f"outputs={len(self._model.outputs)}"
        )

        # ---- IO ----
        self.bridge = CvBridge()
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_stamp = None
        self._lock = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Astra+ 默认彩色是 BEST_EFFORT, 深度是 RELIABLE; 这里分别匹配
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self.ann_pub = self.create_publisher(Image, ann_topic, 1) if self.publish_annotated else None

        # 深度先订阅, 这样彩色帧到的时候已经有最近一帧深度
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_cb, depth_qos
        )
        self.color_sub = self.create_subscription(
            Image, self.color_topic, self._color_cb, sensor_qos
        )

        self._last_log = time.time()
        self._frames = 0

        self.get_logger().info(
            f"Subscribing color={self.color_topic} depth={self.depth_topic}; "
            f"publishing detections={det_topic}"
        )

    # ---------- 回调 ----------
    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"depth convert failed: {exc}")
            return
        with self._lock:
            self._latest_depth = depth
            self._latest_depth_stamp = msg.header.stamp

    def _color_cb(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"color convert failed: {exc}")
            return

        t0 = time.time()
        boxes_xyxy_in, scores, cls_ids = self._infer(bgr)
        infer_ms = (time.time() - t0) * 1000.0

        # 把模型坐标 (640x640) 还原回原图坐标
        h0, w0 = bgr.shape[:2]
        sx = w0 / float(INPUT_SIZE)
        sy = h0 / float(INPUT_SIZE)
        boxes_src = boxes_xyxy_in.copy()
        if boxes_src.size:
            boxes_src[:, 0] *= sx
            boxes_src[:, 1] *= sy
            boxes_src[:, 2] *= sx
            boxes_src[:, 3] *= sy

        # 取深度
        with self._lock:
            depth = None if self._latest_depth is None else self._latest_depth.copy()

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        annotated = bgr.copy() if self.ann_pub is not None else None

        for box, score, cid in zip(boxes_src, scores, cls_ids):
            x1, y1, x2, y2 = box
            cx = float((x1 + x2) * 0.5)
            cy = float((y1 + y2) * 0.5)
            depth_m = self._sample_depth(depth, x1, y1, x2, y2, w0, h0)

            d2d = Detection2D()
            d2d.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            d2d.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            class_id = (
                self.class_names[int(cid)]
                if 0 <= int(cid) < len(self.class_names)
                else f"class_{int(cid)}"
            )
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score = float(score)
            hyp.pose.pose.position.x = cx
            hyp.pose.pose.position.y = cy
            hyp.pose.pose.position.z = (
                float(depth_m) if depth_m is not None else float("nan")
            )
            d2d.results.append(hyp)

            det_msg.detections.append(d2d)

            if annotated is not None:
                self._draw_one(annotated, box, class_id, float(score), depth_m)

        self.det_pub.publish(det_msg)

        if annotated is not None:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann_msg.header = msg.header
            self.ann_pub.publish(ann_msg)

        # 简单帧率/延迟日志, 每秒一行
        self._frames += 1
        now = time.time()
        if now - self._last_log >= 1.0:
            fps = self._frames / (now - self._last_log)
            self.get_logger().info(
                f"detections={len(det_msg.detections)} infer={infer_ms:.1f}ms fps={fps:.1f}"
            )
            self._frames = 0
            self._last_log = now

    # ---------- 推理 ----------
    def _infer(self, bgr: np.ndarray):
        # 先 letterbox-ish: 直接 resize, 与训练时一致
        resized = cv2.resize(bgr, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_AREA)
        nv12 = bgr_to_nv12(resized)
        outputs = self._model.forward(nv12)

        # outputs: [cls80, reg80, cls40, reg40, cls20, reg20]
        cls_maps = [outputs[0].buffer, outputs[2].buffer, outputs[4].buffer]
        reg_maps = [outputs[1].buffer, outputs[3].buffer, outputs[5].buffer]

        all_boxes: List[np.ndarray] = []
        all_scores: List[np.ndarray] = []
        all_ids: List[np.ndarray] = []
        for cls_m, reg_m, stride in zip(cls_maps, reg_maps, STRIDES):
            b, s, c = decode_scale(cls_m, reg_m, stride, self.conf_thres)
            if b.size:
                all_boxes.append(b)
                all_scores.append(s)
                all_ids.append(c)

        if not all_boxes:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,), np.int32)

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        ids = np.concatenate(all_ids, axis=0)

        # 按类做 NMS
        keep_all: List[int] = []
        for c in range(NUM_CLASSES):
            mask = ids == c
            if not np.any(mask):
                continue
            idxs = np.where(mask)[0]
            keep = nms(boxes[idxs], scores[idxs], self.nms_thres)
            keep_all.extend(int(idxs[k]) for k in keep)

        if not keep_all:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,), np.int32)
        keep_all = np.array(keep_all, dtype=np.int64)
        return boxes[keep_all], scores[keep_all], ids[keep_all]

    # ---------- 深度采样 ----------
    def _sample_depth(self, depth: Optional[np.ndarray],
                      x1c: float, y1c: float, x2c: float, y2c: float,
                      color_w: int, color_h: int) -> Optional[float]:
        """在检测框内多点采样深度, 返回中位数 (米); 没有有效值返回 None.

        策略:
          1. 按比例把彩色图坐标映射到深度图;
          2. 在框内做 grid x grid 均匀采样 + 框中心 +- depth_window 邻域;
          3. 过滤掉 0 (无效) 和超出 depth_min_m / depth_max_m 的点;
          4. 有效点至少占 depth_min_valid_ratio 才返回中位数, 否则视为无深度.
        """
        if depth is None:
            return None
        dh, dw = depth.shape[:2]
        scale = 0.001 if depth.dtype == np.uint16 else 1.0

        # 彩色 -> 深度坐标
        sx = dw / max(1, color_w)
        sy = dh / max(1, color_h)
        u1 = int(np.clip(round(x1c * sx), 0, dw - 1))
        v1 = int(np.clip(round(y1c * sy), 0, dh - 1))
        u2 = int(np.clip(round(x2c * sx), 0, dw - 1))
        v2 = int(np.clip(round(y2c * sy), 0, dh - 1))
        if u2 <= u1 or v2 <= v1:
            return None

        # 框内均匀网格采样: depth_grid x depth_grid 个点
        n = max(1, self.depth_grid)
        us = np.linspace(u1, u2, n).round().astype(np.int32)
        vs = np.linspace(v1, v2, n).round().astype(np.int32)
        uu, vv = np.meshgrid(us, vs)
        grid_samples = depth[vv.ravel(), uu.ravel()]

        # 中心邻域采样
        cu = (u1 + u2) // 2
        cv = (v1 + v2) // 2
        r = self.depth_window
        cu0, cu1 = max(0, cu - r), min(dw, cu + r + 1)
        cv0, cv1 = max(0, cv - r), min(dh, cv + r + 1)
        center_patch = depth[cv0:cv1, cu0:cu1].ravel()

        all_samples = np.concatenate([grid_samples, center_patch], axis=0)

        # 过滤无效 (0) 和量程外
        depth_min_raw = self.depth_min_m / scale
        depth_max_raw = self.depth_max_m / scale
        valid = all_samples[(all_samples > depth_min_raw) & (all_samples < depth_max_raw)]
        if valid.size == 0:
            return None
        if valid.size / max(1, all_samples.size) < self.depth_min_valid_ratio:
            return None
        return float(np.median(valid)) * scale

    # ---------- 可视化 ----------
    @staticmethod
    def _draw_one(image: np.ndarray, box, name: str, score: float,
                  depth_m: Optional[float]) -> None:
        x1, y1, x2, y2 = [int(round(v)) for v in box]
        color = (40, 220, 40)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        depth_txt = f"{depth_m:.2f}m" if depth_m is not None and not math.isnan(depth_m) else "n/a"
        label = f"{name} {score:.2f} d={depth_txt}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        ty = max(0, y1 - th - 8)
        cv2.rectangle(image, (x1, ty), (x1 + tw + 6, ty + th + 8), color, -1)
        cv2.putText(image, label, (x1 + 3, ty + th + 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)


def main():
    rclpy.init()
    node = MedicineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
