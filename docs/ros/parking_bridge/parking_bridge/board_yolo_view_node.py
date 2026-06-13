#!/usr/bin/env python3
"""Render board-side YOLO JSON detections into a Foxglove image topic."""

from __future__ import annotations

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


def compressed_jpeg_msg(frame: np.ndarray, stamp, frame_id: str, quality: int) -> CompressedImage | None:
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


class BoardYoloViewNode(Node):
    def __init__(self) -> None:
        super().__init__("board_yolo_view")
        self.declare_parameter("detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("view_topic", "/parking/yolo/parking_view")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("jpeg_quality", 90)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.view_topic = str(self.get_parameter("view_topic").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.pub = self.create_publisher(CompressedImage, self.view_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(String, self.detections_topic, self._on_detections, 10)
        self.get_logger().info(
            f"board_yolo_view started: input={self.detections_topic}, view={self.view_topic}"
        )

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        view = self._draw(payload)
        out = compressed_jpeg_msg(view, self.get_clock().now().to_msg(), self.camera_frame_id, self.jpeg_quality)
        if out is not None:
            self.pub.publish(out)

    def _draw(self, payload: dict[str, Any]) -> np.ndarray:
        size = payload.get("source_image_size") or payload.get("image_size") or [640, 640]
        w = max(160, min(1920, int(float(size[0]))))
        h = max(120, min(1080, int(float(size[1]))))
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:, :] = (18, 20, 24)

        for x in range(0, w, max(1, w // 8)):
            cv2.line(frame, (x, 0), (x, h), (38, 42, 48), 1)
        for y in range(0, h, max(1, h // 8)):
            cv2.line(frame, (0, y), (w, y), (38, 42, 48), 1)

        detections = payload.get("detections", [])
        for det in detections:
            xyxy = det.get("bbox_xyxy") or [0, 0, 0, 0]
            x1, y1, x2, y2 = [int(round(float(v))) for v in xyxy[:4]]
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(x1 + 1, min(w - 1, x2))
            y2 = max(y1 + 1, min(h - 1, y2))
            conf = float(det.get("confidence", 0.0))
            label = str(det.get("class_name", "Parking"))
            polygon = self._polygon_points(det.get("mask_polygon"), w, h)
            if polygon is not None:
                overlay = frame.copy()
                cv2.fillPoly(overlay, [polygon], (0, 200, 255))
                cv2.addWeighted(overlay, 0.28, frame, 0.72, 0, frame)
                cv2.polylines(frame, [polygon], True, (0, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 64), 2)
            cv2.putText(
                frame,
                f"{label} {conf:.2f}",
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 64),
                2,
                cv2.LINE_AA,
            )

        status = str(payload.get("status", "unknown"))
        model = str(payload.get("model", "parking_slot.om"))
        count = int(payload.get("detection_count", len(detections)))
        packets = int(payload.get("received_packets", 0))
        cv2.putText(
            frame,
            f"board OM YOLO | model={model} | detections={count} | {status} | packets={packets}",
            (12, h - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        return frame

    @staticmethod
    def _polygon_points(raw: Any, image_w: int, image_h: int) -> np.ndarray | None:
        if not isinstance(raw, list) or len(raw) < 3:
            return None
        points = []
        for point in raw:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            try:
                x = int(round(float(point[0])))
                y = int(round(float(point[1])))
            except (TypeError, ValueError):
                continue
            points.append([max(0, min(image_w - 1, x)), max(0, min(image_h - 1, y))])
        if len(points) < 3:
            return None
        return np.array(points, dtype=np.int32)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardYoloViewNode()
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
