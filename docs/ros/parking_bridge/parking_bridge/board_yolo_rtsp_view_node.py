#!/usr/bin/env python3
"""Render board-side YOLO detections over the board RTSP camera stream."""

from __future__ import annotations

import json
import threading
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


class BoardYoloRtspViewNode(Node):
    def __init__(self) -> None:
        super().__init__("board_yolo_rtsp_view")
        self.declare_parameter("rtsp_url", "rtsp://192.168.137.2:554/live0")
        self.declare_parameter("detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("view_topic", "/parking/yolo/parking_view")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("jpeg_quality", 65)
        self.declare_parameter("publish_fps", 10.0)
        self.declare_parameter("output_width", 1280)
        self.declare_parameter("rotate180", True)

        self.rtsp_url = str(self.get_parameter("rtsp_url").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.view_topic = str(self.get_parameter("view_topic").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publish_fps = max(1.0, float(self.get_parameter("publish_fps").value))
        self.output_width = max(160, int(self.get_parameter("output_width").value))
        self.rotate180 = bool(self.get_parameter("rotate180").value)

        self.pub = self.create_publisher(CompressedImage, self.view_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(String, self.detections_topic, self._on_detections, 10)

        self._latest_payload: dict[str, Any] | None = None
        self._latest_frame: np.ndarray | None = None
        self._latest_frame_ns = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, name="rtsp_reader", daemon=True)
        self._reader.start()
        self.timer = self.create_timer(1.0 / self.publish_fps, self._publish_view)

        self.get_logger().info(
            f"board_yolo_rtsp_view started: rtsp={self.rtsp_url}, detections={self.detections_topic}, "
            f"view={self.view_topic}, output_width={self.output_width}, quality={self.jpeg_quality}"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        self._reader.join(timeout=2.0)
        return super().destroy_node()

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._latest_payload = payload

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                self.get_logger().warning("RTSP open failed; retrying")
                time.sleep(1.0)
                continue
            try:
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        break
                    with self._lock:
                        self._latest_frame = frame
                        self._latest_frame_ns = time.time_ns()
            finally:
                cap.release()
            time.sleep(0.2)

    def _publish_view(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            payload = self._latest_payload
            frame_age = (time.time_ns() - self._latest_frame_ns) / 1_000_000_000.0 if self._latest_frame_ns else None

        if frame is None:
            return
        if self.rotate180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        src_h, src_w = frame.shape[:2]
        out_w = min(self.output_width, src_w)
        out_h = max(1, int(round(src_h * (out_w / float(src_w)))))
        view = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)

        if payload is not None:
            self._draw_detections(view, payload)

        text = f"board RTSP + OM YOLO | {out_w}x{out_h}"
        if frame_age is not None:
            text += f" | frame_age={frame_age:.2f}s"
        cv2.rectangle(view, (0, 0), (min(out_w, 900), 34), (0, 0, 0), -1)
        cv2.putText(view, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (245, 245, 245), 2, cv2.LINE_AA)

        out = compressed_jpeg_msg(view, self.get_clock().now().to_msg(), self.camera_frame_id, self.jpeg_quality)
        if out is not None:
            self.pub.publish(out)

    def _draw_detections(self, view: np.ndarray, payload: dict[str, Any]) -> None:
        out_h, out_w = view.shape[:2]
        source_size = payload.get("source_image_size") or payload.get("image_size") or [640, 640]
        try:
            det_w = max(1.0, float(source_size[0]))
            det_h = max(1.0, float(source_size[1]))
        except (TypeError, ValueError, IndexError):
            det_w, det_h = 640.0, 640.0
        sx = out_w / det_w
        sy = out_h / det_h

        for det in payload.get("detections", []):
            polygon = self._polygon_points(det.get("mask_polygon"), sx, sy, out_w, out_h)
            xyxy = det.get("bbox_xyxy") or [0, 0, 0, 0]
            x1, y1, x2, y2 = self._scale_xyxy(xyxy, sx, sy, out_w, out_h)
            conf = float(det.get("confidence", 0.0))
            label = str(det.get("class_name", "Parking"))

            if polygon is not None:
                overlay = view.copy()
                cv2.fillPoly(overlay, [polygon], (0, 200, 255))
                cv2.addWeighted(overlay, 0.25, view, 0.75, 0, view)
                cv2.polylines(view, [polygon], True, (0, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 64), 2)
            self._draw_geometry(view, det.get("slot_geometry"), sx, sy)
            cv2.putText(
                view,
                f"{label} {conf:.2f}",
                (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 64),
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _scale_xyxy(raw: Any, sx: float, sy: float, image_w: int, image_h: int) -> tuple[int, int, int, int]:
        try:
            x1, y1, x2, y2 = [float(v) for v in raw[:4]]
        except (TypeError, ValueError):
            x1 = y1 = x2 = y2 = 0.0
        x1_i = max(0, min(image_w - 1, int(round(x1 * sx))))
        y1_i = max(0, min(image_h - 1, int(round(y1 * sy))))
        x2_i = max(x1_i + 1, min(image_w - 1, int(round(x2 * sx))))
        y2_i = max(y1_i + 1, min(image_h - 1, int(round(y2 * sy))))
        return x1_i, y1_i, x2_i, y2_i

    @staticmethod
    def _draw_geometry(view: np.ndarray, geometry: Any, sx: float, sy: float) -> None:
        if not isinstance(geometry, dict):
            return
        h, w = view.shape[:2]

        def point(raw: Any) -> tuple[int, int] | None:
            if not isinstance(raw, (list, tuple)) or len(raw) < 2:
                return None
            try:
                x = int(round(float(raw[0]) * sx))
                y = int(round(float(raw[1]) * sy))
            except (TypeError, ValueError):
                return None
            return max(0, min(w - 1, x)), max(0, min(h - 1, y))

        axis = geometry.get("approach_axis_px")
        if isinstance(axis, list) and len(axis) >= 2:
            p0 = point(axis[0])
            p1 = point(axis[1])
            if p0 is not None and p1 is not None:
                cv2.arrowedLine(view, p0, p1, (255, 0, 255), 2, cv2.LINE_AA, tipLength=0.18)

        entrance = geometry.get("entrance_edge_px")
        if isinstance(entrance, list) and len(entrance) >= 2:
            p0 = point(entrance[0])
            p1 = point(entrance[1])
            if p0 is not None and p1 is not None:
                cv2.line(view, p0, p1, (255, 255, 0), 3, cv2.LINE_AA)

        center = point(geometry.get("center_px"))
        if center is not None:
            cv2.circle(view, center, 5, (255, 0, 255), -1, cv2.LINE_AA)

    @staticmethod
    def _polygon_points(raw: Any, sx: float, sy: float, image_w: int, image_h: int) -> np.ndarray | None:
        if not isinstance(raw, list) or len(raw) < 3:
            return None
        points = []
        for point in raw:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            try:
                x = int(round(float(point[0]) * sx))
                y = int(round(float(point[1]) * sy))
            except (TypeError, ValueError):
                continue
            points.append([max(0, min(image_w - 1, x)), max(0, min(image_h - 1, y))])
        if len(points) < 3:
            return None
        return np.array(points, dtype=np.int32)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardYoloRtspViewNode()
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
