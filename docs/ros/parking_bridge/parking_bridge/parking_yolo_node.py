#!/usr/bin/env python3
"""YOLO ONNX parking-slot detector for the OS08A20 camera stream.

This node is perception-only. It publishes detections, slot candidates, and an
annotated image. It has no serial, CAN, motor, steering, brake, or throttle
interface.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import onnxruntime as ort
except Exception:  # pragma: no cover - runtime environment diagnostic path
    ort = None


def now_ns() -> int:
    return time.time_ns()


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def parse_csv(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


def normalized_name(value: str) -> str:
    return value.strip().lower().replace("-", "_").replace(" ", "_")


def compressed_jpeg_msg(frame: np.ndarray, stamp, frame_id: str, quality: int) -> CompressedImage | None:
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, int(quality)))],
    )
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


def letterbox(frame: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[float, float]]:
    h, w = frame.shape[:2]
    scale = min(float(size) / float(w), float(size) / float(h))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = float(size - new_w) / 2.0
    pad_y = float(size - new_h) / 2.0
    x0 = int(round(pad_x - 0.1))
    y0 = int(round(pad_y - 0.1))
    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas, scale, (float(x0), float(y0))


class ParkingYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_yolo")

        self.declare_parameter("input_topic", "/parking/camera/yolo_input_jpeg")
        self.declare_parameter("detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("view_topic", "/parking/yolo/parking_view")
        self.declare_parameter("state_topic", "/parking/perception/state")
        self.declare_parameter("model_path", "/home/ebaina/parking_models/parking_slot.onnx")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("class_names", "Parking")
        self.declare_parameter("empty_class_names", "empty,empty_space,vacant,available,free")
        self.declare_parameter("occupied_class_names", "occupied,ocupied,occupied_space,car,vehicle")
        self.declare_parameter("slot_class_names", "Parking,parking,parking_space,parking_slot,slot,space")
        self.declare_parameter("include_unknown_as_slot", True)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("process_stride", 3)
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("jpeg_quality", 88)
        self.declare_parameter("output_has_objectness", "auto")
        self.declare_parameter("status_period_sec", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.view_topic = str(self.get_parameter("view_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.class_names = parse_csv(self.get_parameter("class_names").value)
        self.empty_class_names = {normalized_name(v) for v in parse_csv(self.get_parameter("empty_class_names").value)}
        self.occupied_class_names = {normalized_name(v) for v in parse_csv(self.get_parameter("occupied_class_names").value)}
        self.slot_class_names = {normalized_name(v) for v in parse_csv(self.get_parameter("slot_class_names").value)}
        self.include_unknown_as_slot = as_bool(self.get_parameter("include_unknown_as_slot").value)
        self.input_size = max(64, int(self.get_parameter("input_size").value))
        self.process_stride = max(1, int(self.get_parameter("process_stride").value))
        self.confidence_threshold = max(0.01, min(0.99, float(self.get_parameter("confidence_threshold").value)))
        self.nms_threshold = max(0.01, min(0.99, float(self.get_parameter("nms_threshold").value)))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.output_has_objectness = str(self.get_parameter("output_has_objectness").value).strip().lower()

        self.view_pub = self.create_publisher(CompressedImage, self.view_topic, qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.session = None
        self.input_name = ""
        self.output_names: list[str] = []
        self.model_error = ""
        self.received_frames = 0
        self.processed_frames = 0
        self.decode_errors = 0
        self.last_frame_ns: int | None = None
        self.last_processed_ns: int | None = None
        self.last_payload: dict[str, Any] | None = None
        self.inference_times_ms: deque[float] = deque(maxlen=120)
        self.process_times_ns: deque[int] = deque(maxlen=120)
        self._load_model()

        self.timer = self.create_timer(float(self.get_parameter("status_period_sec").value), self._publish_state_timer)
        self.get_logger().info(
            "parking_yolo started: "
            f"input={self.input_topic}, detections={self.detections_topic}, view={self.view_topic}, "
            f"model={self.model_path}, motion_enabled=false"
        )

    def _load_model(self) -> None:
        if ort is None:
            self.model_error = "onnxruntime is not installed"
            self.get_logger().error(self.model_error)
            return
        if not self.model_path.exists():
            self.model_error = f"model file not found: {self.model_path}"
            self.get_logger().error(self.model_error)
            return
        try:
            self.session = ort.InferenceSession(str(self.model_path), providers=["CPUExecutionProvider"])
            input_meta = self.session.get_inputs()[0]
            self.input_name = input_meta.name
            self.output_names = [output.name for output in self.session.get_outputs()]
            shape = list(input_meta.shape or [])
            if len(shape) >= 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
                model_h = int(shape[2])
                model_w = int(shape[3])
                if model_h == model_w and model_h >= 64 and model_h != self.input_size:
                    self.get_logger().warn(f"overriding parking_yolo input_size {self.input_size} -> {model_h}")
                    self.input_size = model_h
        except Exception as exc:
            self.model_error = f"failed to load ONNX model: {exc}"
            self.get_logger().error(self.model_error)

    def _on_image(self, msg: CompressedImage) -> None:
        self.received_frames += 1
        self.last_frame_ns = now_ns()
        if self.session is None:
            return
        if self.received_frames % self.process_stride != 0:
            return

        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.decode_errors += 1
            return

        start_ns = now_ns()
        detections, inference_ms = self._detect(frame)
        end_ns = now_ns()
        slots = self._slot_candidates(detections, frame.shape[1], frame.shape[0])
        self.processed_frames += 1
        self.last_processed_ns = end_ns
        self.process_times_ns.append(end_ns)
        self.inference_times_ms.append(inference_ms)

        payload = {
            "schema_version": 1,
            "component": "parking_yolo",
            "time_ns": end_ns,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": self.camera_frame_id,
            "source_topic": self.input_topic,
            "source_image_size": [int(frame.shape[1]), int(frame.shape[0])],
            "coordinate_frame": "yolo_input_pixels",
            "model": self.model_path.name,
            "model_path": str(self.model_path),
            "class_names": self.class_names,
            "confidence_threshold": self.confidence_threshold,
            "nms_threshold": self.nms_threshold,
            "input_size": self.input_size,
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "decode_errors": self.decode_errors,
            "detections": detections,
            "detection_count": len(detections),
            "slot_candidates": slots,
            "slot_count": len(slots),
            "status": "slot_candidates" if slots else ("detections_no_slot" if detections else "no_detections"),
            "inference_ms": inference_ms,
            "pipeline_ms": float(end_ns - start_ns) / 1_000_000.0,
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        self.last_payload = payload

        out_msg = String()
        out_msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(out_msg)

        view = self._draw_view(frame, detections, slots, payload)
        view_msg = compressed_jpeg_msg(view, msg.header.stamp, self.camera_frame_id, self.jpeg_quality)
        if view_msg is not None:
            self.view_pub.publish(view_msg)
        self._publish_state(payload)

    def _detect(self, frame: np.ndarray) -> tuple[list[dict[str, Any]], float]:
        prepared, scale, pad = letterbox(frame, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        blob = rgb.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, :, :, :]

        start = time.perf_counter()
        outputs = self.session.run(self.output_names, {self.input_name: blob})
        inference_ms = (time.perf_counter() - start) * 1000.0

        pred = self._prediction_matrix(outputs)
        if pred is None or pred.shape[1] < 6:
            return [], inference_ms

        mask_feature_count = self._mask_feature_count(outputs)
        raw = self._decode_predictions(pred, frame.shape[1], frame.shape[0], scale, pad, mask_feature_count)
        if not raw:
            return [], inference_ms

        boxes = [item["bbox"] for item in raw]
        confidences = [float(item["confidence"]) for item in raw]
        nms_indices = cv2.dnn.NMSBoxes(
            bboxes=boxes,
            scores=confidences,
            score_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
        )
        if len(nms_indices) == 0:
            return [], inference_ms

        detections = [raw[int(index)] for index in np.array(nms_indices).reshape(-1)]
        detections.sort(key=lambda item: float(item["confidence"]), reverse=True)
        self._attach_segmentation_masks(detections, outputs, scale, pad, frame.shape[1], frame.shape[0])
        for index, det in enumerate(detections):
            det["id"] = index
        return detections, inference_ms

    @staticmethod
    def _prediction_matrix(outputs: list[np.ndarray]) -> np.ndarray | None:
        if not outputs:
            return None
        pred = np.asarray(outputs[0])
        pred = np.squeeze(pred)
        if pred.ndim == 1:
            pred = pred[None, :]
        if pred.ndim != 2:
            return None
        if pred.shape[0] < pred.shape[1] and pred.shape[0] <= 256:
            pred = pred.T
        return pred.astype(np.float32, copy=False)

    @staticmethod
    def _mask_feature_count(outputs: list[np.ndarray]) -> int:
        """Return YOLO segmentation mask coefficient count when present.

        Ultralytics YOLOv8/YOLO11 segmentation ONNX commonly exports:
        - output0: [1, 4 + classes + masks, anchors]
        - output1: [1, masks, mask_h, mask_w]

        The mask coefficients live at the tail of each prediction row. They
        must not be treated as class scores.
        """
        if len(outputs) < 2:
            return 0
        proto = np.asarray(outputs[1])
        if proto.ndim == 4 and proto.shape[1] > 0:
            return int(proto.shape[1])
        return 0

    def _decode_predictions(
        self,
        pred: np.ndarray,
        image_w: int,
        image_h: int,
        scale: float,
        pad: tuple[float, float],
        mask_feature_count: int,
    ) -> list[dict[str, Any]]:
        if pred.shape[1] == 6 and len(self.class_names) != 2:
            return self._decode_xyxy_score_class(pred, image_w, image_h, scale, pad)

        feature_count = pred.shape[1]
        has_objectness = self._has_objectness(feature_count, mask_feature_count)
        class_count = len(self.class_names)
        if has_objectness:
            class_start = 5
            objectness = pred[:, 4]
        else:
            class_start = 4
            objectness = np.ones((pred.shape[0],), dtype=np.float32)
        available_scores = max(0, feature_count - class_start - max(0, mask_feature_count))
        if class_count > 0:
            class_score_count = min(class_count, available_scores)
        else:
            class_score_count = available_scores
        class_scores = pred[:, class_start:class_start + class_score_count]
        if class_scores.size == 0:
            return []

        class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
        best_scores = class_scores[np.arange(class_scores.shape[0]), class_ids] * objectness
        keep = best_scores >= self.confidence_threshold
        if not np.any(keep):
            return []

        boxes_xywh = pred[keep, 0:4]
        class_ids = class_ids[keep]
        scores = best_scores[keep]
        mask_coefficients = None
        mask_start = class_start + class_score_count
        if mask_feature_count > 0 and pred.shape[1] >= mask_start + mask_feature_count:
            mask_coefficients = pred[keep, mask_start:mask_start + mask_feature_count]
        if np.max(boxes_xywh) <= 1.5:
            boxes_xywh = boxes_xywh * float(self.input_size)

        detections: list[dict[str, Any]] = []
        for index, (box, class_id, score) in enumerate(zip(boxes_xywh, class_ids, scores)):
            cx, cy, bw, bh = [float(x) for x in box[:4]]
            input_x1 = cx - bw / 2.0
            input_y1 = cy - bh / 2.0
            input_x2 = cx + bw / 2.0
            input_y2 = cy + bh / 2.0
            x1 = (input_x1 - pad[0]) / scale
            y1 = (input_y1 - pad[1]) / scale
            x2 = (input_x2 - pad[0]) / scale
            y2 = (input_y2 - pad[1]) / scale
            bbox = self._clip_xyxy_to_bbox(x1, y1, x2, y2, image_w, image_h)
            if bbox is None:
                continue
            det = self._detection_payload(bbox, int(class_id), float(score), image_w, image_h)
            if mask_coefficients is not None:
                det["_mask_coefficients"] = mask_coefficients[index].astype(np.float32, copy=False)
                det["_mask_input_bbox"] = [input_x1, input_y1, input_x2, input_y2]
            detections.append(det)
        return detections

    def _attach_segmentation_masks(
        self,
        detections: list[dict[str, Any]],
        outputs: list[np.ndarray],
        scale: float,
        pad: tuple[float, float],
        image_w: int,
        image_h: int,
    ) -> None:
        if len(outputs) < 2:
            return
        proto = np.asarray(outputs[1])
        if proto.ndim != 4 or proto.shape[0] < 1 or proto.shape[1] < 1:
            return
        proto_0 = proto[0].astype(np.float32, copy=False)
        proto_c, proto_h, proto_w = proto_0.shape
        proto_flat = proto_0.reshape(proto_c, -1)
        for detection in detections:
            coeffs = detection.pop("_mask_coefficients", None)
            input_bbox = detection.pop("_mask_input_bbox", None)
            if coeffs is None or input_bbox is None:
                continue
            coeffs_arr = np.asarray(coeffs, dtype=np.float32)
            if coeffs_arr.size != proto_c:
                continue
            mask = coeffs_arr @ proto_flat
            mask = 1.0 / (1.0 + np.exp(-mask))
            mask = mask.reshape(proto_h, proto_w)
            mask = cv2.resize(mask, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
            self._zero_mask_outside_input_bbox(mask, input_bbox)
            polygon, area_px = self._mask_polygon(mask, scale, pad, image_w, image_h)
            if polygon:
                detection["mask_polygon"] = polygon
                detection["mask_area_px"] = area_px
                detection["polygon_source"] = "mask"

    def _zero_mask_outside_input_bbox(self, mask: np.ndarray, input_bbox: list[float]) -> None:
        x1, y1, x2, y2 = input_bbox
        x1_i = max(0, min(self.input_size, int(math.floor(x1))))
        y1_i = max(0, min(self.input_size, int(math.floor(y1))))
        x2_i = max(0, min(self.input_size, int(math.ceil(x2))))
        y2_i = max(0, min(self.input_size, int(math.ceil(y2))))
        keep = np.zeros_like(mask, dtype=bool)
        if x2_i > x1_i and y2_i > y1_i:
            keep[y1_i:y2_i, x1_i:x2_i] = True
        mask[~keep] = 0.0

    def _mask_polygon(
        self,
        input_mask: np.ndarray,
        scale: float,
        pad: tuple[float, float],
        image_w: int,
        image_h: int,
    ) -> tuple[list[list[int]], int]:
        x0 = max(0, min(self.input_size - 1, int(round(pad[0]))))
        y0 = max(0, min(self.input_size - 1, int(round(pad[1]))))
        x1 = max(x0 + 1, min(self.input_size, int(round(pad[0] + image_w * scale))))
        y1 = max(y0 + 1, min(self.input_size, int(round(pad[1] + image_h * scale))))
        cropped = input_mask[y0:y1, x0:x1]
        if cropped.size == 0:
            return [], 0
        original_mask = cv2.resize(cropped, (image_w, image_h), interpolation=cv2.INTER_LINEAR)
        binary = (original_mask >= 0.5).astype(np.uint8) * 255
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return [], 0
        contour = max(contours, key=cv2.contourArea)
        area_px = int(round(float(cv2.contourArea(contour))))
        if area_px < 16:
            return [], area_px
        epsilon = max(2.0, 0.01 * cv2.arcLength(contour, True))
        approx = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if approx.shape[0] < 3:
            return [], area_px
        polygon: list[list[int]] = []
        for x, y in approx:
            polygon.append([
                int(max(0, min(image_w - 1, round(float(x))))),
                int(max(0, min(image_h - 1, round(float(y))))),
            ])
        return polygon, area_px

    def _decode_xyxy_score_class(
        self,
        pred: np.ndarray,
        image_w: int,
        image_h: int,
        scale: float,
        pad: tuple[float, float],
    ) -> list[dict[str, Any]]:
        detections: list[dict[str, Any]] = []
        rows = pred[pred[:, 4] >= self.confidence_threshold]
        if rows.size == 0:
            return detections
        coords = rows[:, 0:4]
        if np.max(coords) <= 1.5:
            coords = coords * float(self.input_size)
        for row, xyxy in zip(rows, coords):
            x1, y1, x2, y2 = [float(x) for x in xyxy]
            if max(x1, y1, x2, y2) <= float(self.input_size) + 2.0:
                x1 = (x1 - pad[0]) / scale
                y1 = (y1 - pad[1]) / scale
                x2 = (x2 - pad[0]) / scale
                y2 = (y2 - pad[1]) / scale
            bbox = self._clip_xyxy_to_bbox(x1, y1, x2, y2, image_w, image_h)
            if bbox is None:
                continue
            detections.append(self._detection_payload(bbox, int(round(float(row[5]))), float(row[4]), image_w, image_h))
        return detections

    def _has_objectness(self, feature_count: int, mask_feature_count: int = 0) -> bool:
        if self.output_has_objectness in {"true", "1", "yes", "on"}:
            return True
        if self.output_has_objectness in {"false", "0", "no", "off"}:
            return False
        if self.class_names:
            if feature_count == 5 + len(self.class_names):
                return True
            if feature_count == 4 + len(self.class_names):
                return False
            if mask_feature_count > 0:
                if feature_count == 5 + len(self.class_names) + mask_feature_count:
                    return True
                if feature_count == 4 + len(self.class_names) + mask_feature_count:
                    return False
        if feature_count == 85:
            return True
        return False

    @staticmethod
    def _clip_xyxy_to_bbox(
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        image_w: int,
        image_h: int,
    ) -> list[int] | None:
        left = max(0.0, min(float(image_w - 1), min(x1, x2)))
        top = max(0.0, min(float(image_h - 1), min(y1, y2)))
        right = max(0.0, min(float(image_w - 1), max(x1, x2)))
        bottom = max(0.0, min(float(image_h - 1), max(y1, y2)))
        bw = max(1.0, right - left)
        bh = max(1.0, bottom - top)
        if bw < 2.0 or bh < 2.0:
            return None
        return [int(round(left)), int(round(top)), int(round(bw)), int(round(bh))]

    def _detection_payload(
        self,
        bbox: list[int],
        class_id: int,
        score: float,
        image_w: int,
        image_h: int,
    ) -> dict[str, Any]:
        class_name = self._class_name(class_id)
        status = self._slot_status(class_name)
        x, y, w, h = bbox
        center = [x + w * 0.5, y + h * 0.5]
        return {
            "bbox": bbox,
            "bbox_xyxy": [x, y, x + w, y + h],
            "center_px": [round(center[0], 2), round(center[1], 2)],
            "center_norm": [round(center[0] / max(1, image_w), 4), round(center[1] / max(1, image_h), 4)],
            "confidence": round(score, 4),
            "class_id": class_id,
            "class_name": class_name,
            "slot_status": status,
            "frame_id": self.camera_frame_id,
        }

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f"class_{class_id}"

    def _slot_status(self, class_name: str) -> str:
        name = normalized_name(class_name)
        if name in self.empty_class_names:
            return "empty"
        if name in self.occupied_class_names:
            return "occupied"
        if name in self.slot_class_names:
            return "unknown"
        return "unknown"

    def _is_slot_candidate(self, detection: dict[str, Any]) -> bool:
        name = normalized_name(str(detection.get("class_name", "")))
        status = str(detection.get("slot_status", "unknown"))
        return (
            status in {"empty", "occupied"}
            or name in self.slot_class_names
            or self.include_unknown_as_slot
        )

    def _slot_candidates(self, detections: list[dict[str, Any]], image_w: int, image_h: int) -> list[dict[str, Any]]:
        slots: list[dict[str, Any]] = []
        for detection in detections:
            if not self._is_slot_candidate(detection):
                continue
            x, y, w, h = [int(v) for v in detection["bbox"]]
            polygon = detection.get("mask_polygon") or [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]
            slot = {
                "id": len(slots),
                "source": "yolo",
                "status": detection["slot_status"],
                "confidence": detection["confidence"],
                "class_id": detection["class_id"],
                "class_name": detection["class_name"],
                "bbox": detection["bbox"],
                "bbox_xyxy": detection["bbox_xyxy"],
                "center_px": detection["center_px"],
                "center_norm": detection["center_norm"],
                "polygon": polygon,
                "polygon_source": detection.get("polygon_source", "bbox"),
                "mask_area_px": detection.get("mask_area_px"),
                "image_size": [image_w, image_h],
            }
            slots.append(slot)
        return slots

    def _draw_view(
        self,
        frame: np.ndarray,
        detections: list[dict[str, Any]],
        slots: list[dict[str, Any]],
        payload: dict[str, Any],
    ) -> np.ndarray:
        view = frame.copy()
        for det in detections:
            x, y, w, h = [int(v) for v in det["bbox"]]
            status = str(det.get("slot_status", "unknown"))
            color = self._status_color(status)
            polygon = det.get("mask_polygon")
            if polygon:
                self._draw_polygon_overlay(view, polygon, color)
            else:
                cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            self._draw_label(view, label, x, y, color)
        for slot in slots:
            points = np.array(slot["polygon"], dtype=np.int32)
            cv2.polylines(view, [points], isClosed=True, color=self._status_color(slot["status"]), thickness=2)
        headline = (
            f"YOLO parking slots={payload['slot_count']} det={payload['detection_count']} "
            f"infer={payload['inference_ms']:.1f}ms motion=false"
        )
        cv2.rectangle(view, (0, 0), (min(view.shape[1], 900), 34), (20, 20, 20), -1)
        cv2.putText(view, headline, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return view

    @staticmethod
    def _draw_polygon_overlay(image: np.ndarray, polygon: list[list[int]], color: tuple[int, int, int]) -> None:
        points = np.array(polygon, dtype=np.int32)
        if points.ndim != 2 or points.shape[0] < 3:
            return
        overlay = image.copy()
        cv2.fillPoly(overlay, [points], color)
        cv2.addWeighted(overlay, 0.28, image, 0.72, 0.0, image)
        cv2.polylines(image, [points], isClosed=True, color=color, thickness=3)

    @staticmethod
    def _status_color(status: str) -> tuple[int, int, int]:
        if status == "empty":
            return (0, 220, 0)
        if status == "occupied":
            return (30, 30, 230)
        return (0, 210, 255)

    @staticmethod
    def _draw_label(image: np.ndarray, label: str, x: int, y: int, color: tuple[int, int, int]) -> None:
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        y0 = max(0, y - th - 8)
        cv2.rectangle(image, (x, y0), (min(image.shape[1] - 1, x + tw + 8), y), color, -1)
        cv2.putText(image, label, (x + 4, max(th + 2, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    def _publish_state(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps({
            "component": "parking_yolo",
            "time_ns": now_ns(),
            "ok": self.session is not None,
            "status": payload.get("status", "unknown"),
            "slot_count": int(payload.get("slot_count", 0)),
            "detection_count": int(payload.get("detection_count", 0)),
            "inference_ms": payload.get("inference_ms"),
            "processed_frames": self.processed_frames,
            "received_frames": self.received_frames,
            "decode_errors": self.decode_errors,
            "model_path": str(self.model_path),
            "model_error": self.model_error,
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }, ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)

    def _publish_state_timer(self) -> None:
        fps = 0.0
        if len(self.process_times_ns) >= 2:
            span = (self.process_times_ns[-1] - self.process_times_ns[0]) / 1_000_000_000.0
            if span > 0:
                fps = float(len(self.process_times_ns) - 1) / span
        inference_ms = float(np.mean(self.inference_times_ms)) if self.inference_times_ms else None
        payload = self.last_payload or {
            "status": "waiting",
            "slot_count": 0,
            "detection_count": 0,
            "inference_ms": inference_ms,
        }
        payload = dict(payload)
        payload["fps"] = fps
        payload["inference_ms_avg"] = inference_ms
        self._publish_state(payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
