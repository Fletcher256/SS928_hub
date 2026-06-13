#!/usr/bin/env python3
"""Transform YOLO slot pixel geometry into rear-axle ground coordinates.

This node is perception-only. It publishes JSON geometry diagnostics and never
opens an actuator, serial, CAN, motor, steering, brake, or throttle interface.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def now_ns() -> int:
    return time.time_ns()


def parse_points(value: Any) -> list[list[float]]:
    if value is None:
        return []
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        value = json.loads(text)
    if not isinstance(value, list):
        return []
    points: list[list[float]] = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        points.append([float(point[0]), float(point[1])])
    return points


def transform_point(homography: np.ndarray, point: list[float]) -> list[float]:
    src = np.array([float(point[0]), float(point[1]), 1.0], dtype=np.float64)
    dst = homography @ src
    if abs(float(dst[2])) < 1e-9:
        return [float("nan"), float("nan")]
    return [float(dst[0] / dst[2]), float(dst[1] / dst[2])]


def round_point(point: list[float]) -> list[float]:
    return [round(float(point[0]), 2), round(float(point[1]), 2)]


def transform_points(homography: np.ndarray, points: Any) -> list[list[float]]:
    if not isinstance(points, list):
        return []
    out = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        out.append(round_point(transform_point(homography, [float(point[0]), float(point[1])])))
    return out


def yaw_deg_from_axis(axis_cm: list[list[float]]) -> float | None:
    if len(axis_cm) < 2:
        return None
    p0, p1 = axis_cm[0], axis_cm[1]
    return math.degrees(math.atan2(float(p1[1]) - float(p0[1]), float(p1[0]) - float(p0[0])))


class SlotGeometryTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("slot_geometry_transform")

        self.declare_parameter("detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("slot_geometry_topic", "/parking/slot_geometry")
        self.declare_parameter("state_topic", "/parking/slot_geometry_state")
        self.declare_parameter("calibration_file", "/home/ebaina/parking_calibration/slot_homography_rear_axle.json")
        self.declare_parameter("image_points_px", "")
        self.declare_parameter("ground_points_cm", "")
        self.declare_parameter("vehicle_frame_id", "vehicle_rear_axle_cm")
        self.declare_parameter("stale_after_sec", 1.5)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.slot_geometry_topic = str(self.get_parameter("slot_geometry_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.calibration_file = Path(str(self.get_parameter("calibration_file").value)).expanduser()
        self.vehicle_frame_id = str(self.get_parameter("vehicle_frame_id").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)

        self.homography: np.ndarray | None = None
        self.calibration: dict[str, Any] = {}
        self.calibration_error = ""
        self._load_calibration()

        self.geometry_pub = self.create_publisher(String, self.slot_geometry_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sub = self.create_subscription(String, self.detections_topic, self._on_detections, 10)
        self.timer = self.create_timer(1.0, self._publish_state)
        self.last_input_ns: int | None = None
        self.last_output: dict[str, Any] | None = None

        self.get_logger().info(
            "slot_geometry_transform started: "
            f"input={self.detections_topic}, output={self.slot_geometry_topic}, frame={self.vehicle_frame_id}, "
            f"calibrated={self.homography is not None}"
        )

    def _load_calibration(self) -> None:
        image_points = parse_points(self.get_parameter("image_points_px").value)
        ground_points = parse_points(self.get_parameter("ground_points_cm").value)
        source = "ros_parameters"

        if (not image_points or not ground_points) and self.calibration_file.exists():
            try:
                data = json.loads(self.calibration_file.read_text(encoding="utf-8"))
                image_points = parse_points(data.get("image_points_px"))
                ground_points = parse_points(data.get("ground_points_cm"))
                source = str(self.calibration_file)
            except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
                self.calibration_error = f"failed_to_read_calibration_file:{exc}"

        if len(image_points) < 4 or len(ground_points) < 4 or len(image_points) != len(ground_points):
            self.homography = None
            if not self.calibration_error:
                self.calibration_error = "need_matching_image_points_px_and_ground_points_cm_at_least_4"
            self.calibration = {
                "source": source,
                "image_points_px": image_points,
                "ground_points_cm": ground_points,
                "valid": False,
                "error": self.calibration_error,
            }
            return

        src = np.array(image_points, dtype=np.float64)
        dst = np.array(ground_points, dtype=np.float64)
        homography, mask = cv2.findHomography(src, dst, method=0)
        if homography is None:
            self.homography = None
            self.calibration_error = "cv2_find_homography_failed"
            valid = False
        else:
            self.homography = homography.astype(np.float64)
            self.calibration_error = ""
            valid = True

        self.calibration = {
            "source": source,
            "image_points_px": [[round(x, 2), round(y, 2)] for x, y in image_points],
            "ground_points_cm": [[round(x, 2), round(y, 2)] for x, y in ground_points],
            "valid": valid,
            "homography_px_to_rear_axle_cm": self.homography.tolist() if self.homography is not None else None,
            "inlier_mask": mask.reshape(-1).astype(int).tolist() if mask is not None else None,
            "error": self.calibration_error,
        }

    def _on_detections(self, msg: String) -> None:
        self.last_input_ns = now_ns()
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        output = self._build_output(detections)
        self.last_output = output
        out_msg = String()
        out_msg.data = json.dumps(output, ensure_ascii=False, separators=(",", ":"))
        self.geometry_pub.publish(out_msg)
        self._publish_state()

    def _build_output(self, detections: dict[str, Any]) -> dict[str, Any]:
        current_ns = now_ns()
        base = {
            "schema_version": 1,
            "component": "slot_geometry_transform",
            "time_ns": current_ns,
            "input_topic": self.detections_topic,
            "source_frame": detections.get("coordinate_frame", "board_yolo_pixels"),
            "target_frame": self.vehicle_frame_id,
            "coordinate_convention": {
                "origin": "vehicle_rear_axle_center",
                "x_cm": "forward",
                "y_cm": "left",
                "yaw_deg": "counterclockwise_positive",
            },
            "calibration": self.calibration,
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        if self.homography is None:
            return {
                **base,
                "status": "waiting_for_calibration",
                "slot_count": 0,
                "slots": [],
                "selected_slot": None,
            }

        slots = []
        for slot in detections.get("slot_candidates", []):
            converted = self._convert_slot(slot)
            if converted:
                slots.append(converted)
        slots.sort(key=lambda row: float(row.get("confidence", 0.0)), reverse=True)
        for idx, slot in enumerate(slots):
            slot["id"] = idx

        return {
            **base,
            "status": "slot_geometry" if slots else "no_slots",
            "slot_count": len(slots),
            "slots": slots,
            "selected_slot": slots[0] if slots else None,
        }

    def _convert_slot(self, slot: dict[str, Any]) -> dict[str, Any]:
        if self.homography is None:
            return {}
        geometry = slot.get("geometry") if isinstance(slot.get("geometry"), dict) else {}
        polygon_px = geometry.get("corners_px") or slot.get("polygon") or []
        if not polygon_px:
            return {}

        corners_cm = transform_points(self.homography, polygon_px)
        center_px = geometry.get("center_px") or slot.get("center_px")
        center_cm = round_point(transform_point(self.homography, center_px)) if center_px else []
        entrance_cm = transform_points(self.homography, geometry.get("entrance_edge_px"))
        back_cm = transform_points(self.homography, geometry.get("back_edge_px"))
        approach_axis_cm = transform_points(self.homography, geometry.get("approach_axis_px"))
        width_axis_cm = transform_points(self.homography, geometry.get("width_axis_px"))
        yaw_ground = yaw_deg_from_axis(approach_axis_cm)

        width_cm = self._distance(width_axis_cm)
        length_cm = self._axis_length_from_centerline(approach_axis_cm)

        return {
            "id": int(slot.get("id", 0)),
            "source": slot.get("source", "board_yolo_om"),
            "status": slot.get("status", "unknown"),
            "confidence": float(slot.get("confidence", 0.0)),
            "class_id": slot.get("class_id"),
            "class_name": slot.get("class_name"),
            "pixel_geometry": geometry,
            "ground_geometry": {
                "coordinate_frame": self.vehicle_frame_id,
                "corners_cm": corners_cm,
                "center_cm": center_cm,
                "entrance_edge_cm": entrance_cm,
                "back_edge_cm": back_cm,
                "approach_axis_cm": approach_axis_cm,
                "width_axis_cm": width_axis_cm,
                "width_cm": width_cm,
                "length_cm": length_cm,
                "aspect_ratio": round(length_cm / max(1.0, width_cm), 4) if width_cm is not None and length_cm is not None else None,
                "yaw_ground_deg": round(yaw_ground, 2) if yaw_ground is not None else None,
            },
            "target_pose_vehicle_frame": {
                "x_cm": center_cm[0] if center_cm else None,
                "y_cm": center_cm[1] if center_cm else None,
                "yaw_deg": round(yaw_ground, 2) if yaw_ground is not None else None,
                "frame_id": self.vehicle_frame_id,
            },
            "calibrated_metric": True,
        }

    @staticmethod
    def _distance(points: list[list[float]]) -> float | None:
        if len(points) < 2:
            return None
        return round(math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1]), 2)

    @staticmethod
    def _axis_length_from_centerline(points: list[list[float]]) -> float | None:
        if len(points) < 2:
            return None
        half = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
        return round(half * 2.0, 2)

    def _publish_state(self) -> None:
        current_ns = now_ns()
        age_sec = None
        if self.last_input_ns is not None:
            age_sec = (current_ns - self.last_input_ns) / 1_000_000_000.0
        state = {
            "schema_version": 1,
            "component": "slot_geometry_transform",
            "time_ns": current_ns,
            "status": "calibrated" if self.homography is not None else "waiting_for_calibration",
            "input_fresh": age_sec is not None and age_sec <= self.stale_after_sec,
            "last_input_age_sec": age_sec,
            "slot_count": int((self.last_output or {}).get("slot_count", 0)),
            "calibration_valid": self.homography is not None,
            "calibration_error": self.calibration_error,
            "target_frame": self.vehicle_frame_id,
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlotGeometryTransformNode()
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
