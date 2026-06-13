#!/usr/bin/env python3
"""YOLO ONNX person detector for the OS08A20 camera stream.

The node is perception-only. It publishes diagnostics and annotated images,
and deliberately has no actuator, serial, CAN, motor, steering, brake, or
throttle interfaces.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
import json
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


COCO_PERSON_CLASS_ID = 0


def now_ns() -> int:
    return time.time_ns()


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


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


class YoloPersonNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_yolo_person")

        self.declare_parameter("input_topic", "/parking/camera/image_jpeg")
        self.declare_parameter("detections_topic", "/parking/yolo/person_detections")
        self.declare_parameter("view_topic", "/parking/yolo/person_view")
        self.declare_parameter("state_topic", "/parking/perception/state")
        self.declare_parameter("model_path", "/home/ebaina/parking_models/yolov8n.onnx")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("process_stride", 3)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("jpeg_quality", 82)
        self.declare_parameter("draw_all_frames", False)
        self.declare_parameter("status_period_sec", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.view_topic = str(self.get_parameter("view_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.input_size = max(64, int(self.get_parameter("input_size").value))
        self.process_stride = max(1, int(self.get_parameter("process_stride").value))
        self.confidence_threshold = max(0.01, min(0.99, float(self.get_parameter("confidence_threshold").value)))
        self.nms_threshold = max(0.01, min(0.99, float(self.get_parameter("nms_threshold").value)))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.draw_all_frames = as_bool(self.get_parameter("draw_all_frames").value)

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
            "parking_yolo_person started: "
            f"input={self.input_topic}, detections={self.detections_topic}, "
            f"view={self.view_topic}, model={self.model_path}, motion_enabled=false"
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
        providers = ["CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(str(self.model_path), providers=providers)
            input_meta = self.session.get_inputs()[0]
            self.input_name = input_meta.name
            self.output_names = [output.name for output in self.session.get_outputs()]
            shape = list(input_meta.shape or [])
            if len(shape) >= 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
                model_h = int(shape[2])
                model_w = int(shape[3])
                if model_h == model_w and model_h >= 64 and model_h != self.input_size:
                    self.get_logger().warn(
                        "overriding yolo input_size from "
                        f"{self.input_size} to static model size {model_h}"
                    )
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
        detections, inference_ms = self._detect_person(frame)
        end_ns = now_ns()
        self.processed_frames += 1
        self.last_processed_ns = end_ns
        self.process_times_ns.append(end_ns)
        self.inference_times_ms.append(inference_ms)

        payload = {
            "time_ns": end_ns,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": self.camera_frame_id,
            "model": self.model_path.name,
            "class_filter": "person",
            "class_id": COCO_PERSON_CLASS_ID,
            "confidence_threshold": self.confidence_threshold,
            "nms_threshold": self.nms_threshold,
            "input_size": self.input_size,
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "decode_errors": self.decode_errors,
            "detections": detections,
            "person_count": len(detections),
            "status": "person" if detections else "no_person",
            "inference_ms": inference_ms,
            "pipeline_ms": float(end_ns - start_ns) / 1_000_000.0,
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        self.last_payload = payload

        out_msg = String()
        out_msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(out_msg)

        view = self._draw_view(frame, detections, payload)
        view_msg = compressed_jpeg_msg(view, msg.header.stamp, self.camera_frame_id, self.jpeg_quality)
        if view_msg is not None:
            self.view_pub.publish(view_msg)

        self._publish_state(payload)

    def _detect_person(self, frame: np.ndarray) -> tuple[list[dict[str, Any]], float]:
        prepared, scale, pad = letterbox(frame, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        blob = rgb.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, :, :, :]

        start = time.perf_counter()
        outputs = self.session.run(self.output_names, {self.input_name: blob})
        inference_ms = (time.perf_counter() - start) * 1000.0

        pred = np.asarray(outputs[0])
        pred = np.squeeze(pred)
        if pred.ndim != 2:
            return [], inference_ms
        if pred.shape[0] < pred.shape[1] and pred.shape[0] in {84, 85, 116}:
            pred = pred.T

        if pred.shape[1] < 5:
            return [], inference_ms
        boxes_xywh = pred[:, 0:4]
        if pred.shape[1] == 85:
            scores = pred[:, 4] * pred[:, 5 + COCO_PERSON_CLASS_ID]
        else:
            scores = pred[:, 4 + COCO_PERSON_CLASS_ID]

        keep = scores >= self.confidence_threshold
        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        if boxes_xywh.size == 0:
            return [], inference_ms

        h, w = frame.shape[:2]
        pad_x, pad_y = pad
        boxes: list[list[int]] = []
        confidences: list[float] = []
        for box, score in zip(boxes_xywh, scores):
            cx, cy, bw, bh = [float(x) for x in box[:4]]
            x1 = (cx - bw / 2.0 - pad_x) / scale
            y1 = (cy - bh / 2.0 - pad_y) / scale
            x2 = (cx + bw / 2.0 - pad_x) / scale
            y2 = (cy + bh / 2.0 - pad_y) / scale
            x1 = max(0.0, min(float(w - 1), x1))
            y1 = max(0.0, min(float(h - 1), y1))
            x2 = max(0.0, min(float(w - 1), x2))
            y2 = max(0.0, min(float(h - 1), y2))
            bw_px = max(1.0, x2 - x1)
            bh_px = max(1.0, y2 - y1)
            boxes.append([int(round(x1)), int(round(y1)), int(round(bw_px)), int(round(bh_px))])
            confidences.append(float(score))

        nms_indices = cv2.dnn.NMSBoxes(
            bboxes=boxes,
            scores=confidences,
            score_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
        )
        if len(nms_indices) == 0:
            return [], inference_ms

        detections: list[dict[str, Any]] = []
        for raw_index in np.array(nms_indices).reshape(-1):
            index = int(raw_index)
            x, y, bw_px, bh_px = boxes[index]
            detections.append({
                "bbox": [x, y, bw_px, bh_px],
                "confidence": round(confidences[index], 4),
                "class_id": COCO_PERSON_CLASS_ID,
                "class_name": "person",
                "frame_id": self.camera_frame_id,
            })
        detections.sort(key=lambda item: float(item["confidence"]), reverse=True)
        return detections, inference_ms

    def _draw_view(self, frame: np.ndarray, detections: list[dict[str, Any]], payload: dict[str, Any]) -> np.ndarray:
        view = frame.copy()
        for det in detections:
            x, y, w, h = [int(v) for v in det["bbox"]]
            conf = float(det["confidence"])
            cv2.rectangle(view, (x, y), (x + w, y + h), (0, 220, 0), 2)
            label = f"person {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)
            y0 = max(0, y - th - 8)
            cv2.rectangle(view, (x, y0), (min(view.shape[1] - 1, x + tw + 8), y), (0, 150, 0), -1)
            cv2.putText(view, label, (x + 4, max(th + 2, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

        headline = (
            f"YOLO person: {payload['person_count']} "
            f"infer {payload['inference_ms']:.1f}ms "
            f"motion=false"
        )
        cv2.rectangle(view, (0, 0), (min(view.shape[1], 720), 34), (20, 20, 20), -1)
        cv2.putText(view, headline, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return view

    def _publish_state(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps({
            "component": "yolo_person",
            "time_ns": now_ns(),
            "ok": self.session is not None,
            "status": payload.get("status", "unknown"),
            "person_count": int(payload.get("person_count", 0)),
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
        now = now_ns()
        fps = 0.0
        if len(self.process_times_ns) >= 2:
            span = (self.process_times_ns[-1] - self.process_times_ns[0]) / 1_000_000_000.0
            if span > 0:
                fps = float(len(self.process_times_ns) - 1) / span
        inference_ms = None
        if self.inference_times_ms:
            inference_ms = float(np.mean(self.inference_times_ms))
        payload = self.last_payload or {
            "status": "waiting",
            "person_count": 0,
            "inference_ms": inference_ms,
        }
        state = dict(payload)
        state["time_ns"] = now
        state["fps"] = fps
        state["inference_ms_avg"] = inference_ms
        self._publish_state(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloPersonNode()
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
