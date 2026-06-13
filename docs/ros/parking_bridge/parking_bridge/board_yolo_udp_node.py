#!/usr/bin/env python3
"""Bridge board-side parking YOLO UDP JSON into ROS diagnostics.

The board-side OM model owns camera and NPU inference. This node only receives
JSON detections and republishes them for Foxglove/planner use. It has no serial,
CAN, motor, steering, brake, or throttle interface.
"""

from __future__ import annotations

import json
import math
import socket
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def now_ns() -> int:
    return time.time_ns()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class BoardYoloUdpNode(Node):
    def __init__(self) -> None:
        super().__init__("board_yolo_udp")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 24580)
        self.declare_parameter("detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("state_topic", "/parking/perception/state")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("status_period_sec", 1.0)

        self.listen_host = str(self.get_parameter("listen_host").value)
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)

        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.listen_host, self.listen_port))

        self.received_packets = 0
        self.parse_errors = 0
        self.last_payload: dict[str, Any] | None = None
        self.last_packet_ns: int | None = None
        self.last_sender = ""

        self.poll_timer = self.create_timer(0.01, self._poll_socket)
        self.state_timer = self.create_timer(float(self.get_parameter("status_period_sec").value), self._publish_state)
        self.get_logger().info(
            "board_yolo_udp started: "
            f"listen={self.listen_host}:{self.listen_port}, detections={self.detections_topic}, "
            "motion_enabled=false"
        )

    def destroy_node(self) -> bool:
        try:
            self.sock.close()
        finally:
            return super().destroy_node()

    def _poll_socket(self) -> None:
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return

            self.received_packets += 1
            self.last_packet_ns = now_ns()
            self.last_sender = f"{addr[0]}:{addr[1]}"
            try:
                raw = json.loads(data.decode("utf-8", errors="replace").strip())
            except json.JSONDecodeError:
                self.parse_errors += 1
                continue

            payload = self._normalize_payload(raw)
            self.last_payload = payload
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self.detections_pub.publish(msg)
            self._publish_state()

    def _normalize_payload(self, raw: dict[str, Any]) -> dict[str, Any]:
        current_ns = now_ns()
        image_size = raw.get("image_size") or raw.get("source_image_size") or [1, 1]
        image_w = max(1, int(float(image_size[0])))
        image_h = max(1, int(float(image_size[1])))
        detections = []
        slots = []

        for idx, item in enumerate(raw.get("detections", [])):
            det = self._normalize_detection(idx, item, image_w, image_h)
            geometry = self._slot_geometry(det, image_w, image_h)
            if geometry:
                det["slot_geometry"] = geometry
            slot_center_px = geometry.get("center_px") or det["center_px"]
            slot_center_norm = geometry.get("center_norm") or det["center_norm"]
            detections.append(det)
            slots.append({
                "id": det["id"],
                "source": "board_yolo_om",
                "status": det["slot_status"],
                "confidence": det["confidence"],
                "class_id": det["class_id"],
                "class_name": det["class_name"],
                "bbox": det["bbox"],
                "bbox_xyxy": det["bbox_xyxy"],
                "center_px": slot_center_px,
                "center_norm": slot_center_norm,
                "polygon": det.get("mask_polygon") or self._bbox_polygon(det["bbox_xyxy"]),
                "geometry": geometry,
                "image_size": [image_w, image_h],
            })

        return {
            "schema_version": 1,
            "component": "board_yolo_udp",
            "time_ns": current_ns,
            "frame_id": self.camera_frame_id,
            "source": "board_parking_yolo_om_udp",
            "source_topic": "udp:24580",
            "source_image_size": [image_w, image_h],
            "coordinate_frame": "board_yolo_pixels",
            "model": raw.get("model", "parking_slot.om"),
            "model_path": "/opt/sample/parking_yolo_seg_safe/parking_slot.om",
            "class_names": ["Parking"],
            "received_packets": self.received_packets,
            "parse_errors": self.parse_errors,
            "sender": self.last_sender,
            "detections": detections,
            "detection_count": len(detections),
            "slot_candidates": slots,
            "slot_count": len(slots),
            "status": "slot_candidates" if slots else "no_detections",
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }

    def _normalize_detection(self, idx: int, item: dict[str, Any], image_w: int, image_h: int) -> dict[str, Any]:
        bbox = item.get("bbox") or [0, 0, 0, 0]
        x = clamp(float(bbox[0]), 0.0, float(image_w))
        y = clamp(float(bbox[1]), 0.0, float(image_h))
        w = clamp(float(bbox[2]), 0.0, float(image_w) - x)
        h = clamp(float(bbox[3]), 0.0, float(image_h) - y)
        xyxy = item.get("bbox_xyxy") or [x, y, x + w, y + h]
        x1 = clamp(float(xyxy[0]), 0.0, float(image_w))
        y1 = clamp(float(xyxy[1]), 0.0, float(image_h))
        x2 = clamp(float(xyxy[2]), x1, float(image_w))
        y2 = clamp(float(xyxy[3]), y1, float(image_h))
        center = item.get("center_px") or [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
        cx = clamp(float(center[0]), 0.0, float(image_w))
        cy = clamp(float(center[1]), 0.0, float(image_h))
        det = {
            "bbox": [round(x, 2), round(y, 2), round(w, 2), round(h, 2)],
            "bbox_xyxy": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
            "center_px": [round(cx, 2), round(cy, 2)],
            "center_norm": [round(cx / float(image_w), 4), round(cy / float(image_h), 4)],
            "confidence": float(item.get("confidence", item.get("score", 0.0))),
            "class_id": int(item.get("class_id", 0)),
            "class_name": str(item.get("class_name", "Parking")),
            "slot_status": str(item.get("slot_status", "unknown")),
            "frame_id": self.camera_frame_id,
            "id": int(item.get("id", idx)),
        }
        polygon = self._normalize_polygon(item.get("mask_polygon"), image_w, image_h)
        if polygon:
            det["mask_polygon"] = polygon
            det["polygon_source"] = str(item.get("polygon_source", "mask"))
        if "mask_area_px" in item:
            det["mask_area_px"] = int(float(item.get("mask_area_px", 0)))
        return det

    @staticmethod
    def _bbox_polygon(xyxy: list[float]) -> list[list[float]]:
        x1, y1, x2, y2 = xyxy
        return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]

    @staticmethod
    def _normalize_polygon(raw: Any, image_w: int, image_h: int) -> list[list[float]]:
        if not isinstance(raw, list):
            return []
        polygon: list[list[float]] = []
        for point in raw:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            try:
                x = clamp(float(point[0]), 0.0, float(image_w))
                y = clamp(float(point[1]), 0.0, float(image_h))
            except (TypeError, ValueError):
                continue
            polygon.append([round(x, 2), round(y, 2)])
        return polygon if len(polygon) >= 3 else []

    @staticmethod
    def _slot_geometry(det: dict[str, Any], image_w: int, image_h: int) -> dict[str, Any]:
        polygon = det.get("mask_polygon") or BoardYoloUdpNode._bbox_polygon(det["bbox_xyxy"])
        if not isinstance(polygon, list) or len(polygon) < 3:
            return {}

        pts = []
        for point in polygon:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            try:
                x = clamp(float(point[0]), 0.0, float(image_w))
                y = clamp(float(point[1]), 0.0, float(image_h))
            except (TypeError, ValueError):
                continue
            pts.append([x, y])
        if len(pts) < 3:
            return {}

        corners = BoardYoloUdpNode._oriented_bbox(pts)
        if len(corners) != 4:
            return {}

        edges = []
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            mx = (a[0] + b[0]) * 0.5
            my = (a[1] + b[1]) * 0.5
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            edges.append({"index": i, "a": a, "b": b, "mid": [mx, my], "length": length})

        entrance = max(edges, key=lambda edge: edge["mid"][1])
        back = edges[(entrance["index"] + 2) % 4]
        left_edge = edges[(entrance["index"] + 1) % 4]
        right_edge = edges[(entrance["index"] + 3) % 4]

        center = [
            (entrance["mid"][0] + back["mid"][0]) * 0.5,
            (entrance["mid"][1] + back["mid"][1]) * 0.5,
        ]
        approach_vec = [
            center[0] - entrance["mid"][0],
            center[1] - entrance["mid"][1],
        ]
        width_vec = [
            entrance["b"][0] - entrance["a"][0],
            entrance["b"][1] - entrance["a"][1],
        ]
        half_length = math.hypot(approach_vec[0], approach_vec[1])
        width_len = math.hypot(width_vec[0], width_vec[1])
        yaw_image_deg = math.degrees(math.atan2(approach_vec[1], approach_vec[0])) if half_length > 0 else 0.0

        return {
            "schema_version": 1,
            "coordinate_frame": "yolo_input_pixels",
            "calibrated_metric": False,
            "metric_reason": "camera_to_ground_homography_required",
            "source": "mask_polygon_min_area_oriented_bbox" if det.get("mask_polygon") else "bbox_axis_aligned",
            "image_size": [image_w, image_h],
            "corners_px": BoardYoloUdpNode._round_points(corners),
            "center_px": [round(center[0], 2), round(center[1], 2)],
            "center_norm": [round(center[0] / float(image_w), 4), round(center[1] / float(image_h), 4)],
            "entrance_edge_px": BoardYoloUdpNode._round_points([entrance["a"], entrance["b"]]),
            "back_edge_px": BoardYoloUdpNode._round_points([back["a"], back["b"]]),
            "left_edge_px": BoardYoloUdpNode._round_points([left_edge["a"], left_edge["b"]]),
            "right_edge_px": BoardYoloUdpNode._round_points([right_edge["a"], right_edge["b"]]),
            "approach_axis_px": BoardYoloUdpNode._round_points([entrance["mid"], center]),
            "width_axis_px": BoardYoloUdpNode._round_points([entrance["a"], entrance["b"]]),
            "length_px": round(half_length * 2.0, 2),
            "width_px": round(width_len, 2),
            "aspect_ratio": round((half_length * 2.0) / max(1.0, width_len), 4),
            "yaw_image_deg": round(yaw_image_deg, 2),
            "entrance_rule": "edge_with_largest_image_y_midpoint",
        }

    @staticmethod
    def _oriented_bbox(points: list[list[float]]) -> list[list[float]]:
        hull = BoardYoloUdpNode._convex_hull(points)
        if len(hull) < 3:
            return []

        best: tuple[float, list[list[float]]] | None = None
        for i in range(len(hull)):
            a = hull[i]
            b = hull[(i + 1) % len(hull)]
            angle = math.atan2(b[1] - a[1], b[0] - a[0])
            cos_a = math.cos(-angle)
            sin_a = math.sin(-angle)
            rotated = [
                [p[0] * cos_a - p[1] * sin_a, p[0] * sin_a + p[1] * cos_a]
                for p in hull
            ]
            xs = [p[0] for p in rotated]
            ys = [p[1] for p in rotated]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            area = (max_x - min_x) * (max_y - min_y)
            rect_rotated = [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]]
            cos_b = math.cos(angle)
            sin_b = math.sin(angle)
            rect = [
                [p[0] * cos_b - p[1] * sin_b, p[0] * sin_b + p[1] * cos_b]
                for p in rect_rotated
            ]
            if best is None or area < best[0]:
                best = (area, rect)
        return BoardYoloUdpNode._clockwise_from_top_left(best[1]) if best else []

    @staticmethod
    def _convex_hull(points: list[list[float]]) -> list[list[float]]:
        unique = sorted({(round(p[0], 4), round(p[1], 4)) for p in points})
        if len(unique) <= 1:
            return [[x, y] for x, y in unique]

        def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower = []
        for p in unique:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        upper = []
        for p in reversed(unique):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        hull = lower[:-1] + upper[:-1]
        return [[x, y] for x, y in hull]

    @staticmethod
    def _clockwise_from_top_left(points: list[list[float]]) -> list[list[float]]:
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        ordered = sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        start = min(range(len(ordered)), key=lambda i: (ordered[i][1] + ordered[i][0], ordered[i][1]))
        return ordered[start:] + ordered[:start]

    @staticmethod
    def _round_points(points: list[list[float]]) -> list[list[float]]:
        return [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in points]

    def _publish_state(self) -> None:
        current_ns = now_ns()
        age_sec = None
        if self.last_packet_ns is not None:
            age_sec = (current_ns - self.last_packet_ns) / 1_000_000_000.0
        state = {
            "schema_version": 1,
            "component": "board_yolo_udp",
            "time_ns": current_ns,
            "mode": "perception_only",
            "status": "receiving" if age_sec is not None and age_sec < 2.0 else "waiting_for_udp",
            "listen": f"{self.listen_host}:{self.listen_port}",
            "sender": self.last_sender,
            "received_packets": self.received_packets,
            "parse_errors": self.parse_errors,
            "last_packet_age_sec": age_sec,
            "last_detection_count": (self.last_payload or {}).get("detection_count", 0),
            "model": (self.last_payload or {}).get("model", "parking_slot.om"),
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardYoloUdpNode()
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
