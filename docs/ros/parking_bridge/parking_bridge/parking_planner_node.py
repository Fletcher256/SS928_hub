#!/usr/bin/env python3
"""Dry-run parking planner that prefers YOLO slot candidates.

The planner publishes JSON diagnostics only. It never opens an actuator,
serial, CAN, motor, steering, brake, or throttle interface.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def now_ns() -> int:
    return time.time_ns()


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class ParkingPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_planner")

        self.declare_parameter("yolo_detections_topic", "/parking/yolo/parking_detections")
        self.declare_parameter("pixel_candidates_topic", "/parking/parking_slot_candidates")
        self.declare_parameter("path_topic", "/parking/planner/path")
        self.declare_parameter("dry_run_cmd_topic", "/parking/controller/dry_run_cmd")
        self.declare_parameter("state_topic", "/parking/planner/state")
        self.declare_parameter("prefer_yolo", True)
        self.declare_parameter("fallback_to_pixel_candidates", True)
        self.declare_parameter("stale_after_sec", 1.5)
        self.declare_parameter("timer_period_sec", 0.2)
        self.declare_parameter("min_yolo_confidence", 0.35)
        self.declare_parameter("allow_unknown_yolo_slots", True)
        self.declare_parameter("vehicle_length_cm", 23.6)
        self.declare_parameter("vehicle_width_cm", 20.0)
        self.declare_parameter("slot_side_clearance_cm", 2.0)
        self.declare_parameter("slot_end_clearance_cm", 3.0)
        self.declare_parameter("max_steering_deg", 25.0)
        self.declare_parameter("nominal_reverse_speed_cm_s", 3.0)

        self.yolo_detections_topic = str(self.get_parameter("yolo_detections_topic").value)
        self.pixel_candidates_topic = str(self.get_parameter("pixel_candidates_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.dry_run_cmd_topic = str(self.get_parameter("dry_run_cmd_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.prefer_yolo = as_bool(self.get_parameter("prefer_yolo").value)
        self.fallback_to_pixel_candidates = as_bool(self.get_parameter("fallback_to_pixel_candidates").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)
        self.min_yolo_confidence = float(self.get_parameter("min_yolo_confidence").value)
        self.allow_unknown_yolo_slots = as_bool(self.get_parameter("allow_unknown_yolo_slots").value)
        self.vehicle_length_cm = float(self.get_parameter("vehicle_length_cm").value)
        self.vehicle_width_cm = float(self.get_parameter("vehicle_width_cm").value)
        self.slot_side_clearance_cm = float(self.get_parameter("slot_side_clearance_cm").value)
        self.slot_end_clearance_cm = float(self.get_parameter("slot_end_clearance_cm").value)
        self.max_steering_deg = float(self.get_parameter("max_steering_deg").value)
        self.nominal_reverse_speed_cm_s = float(self.get_parameter("nominal_reverse_speed_cm_s").value)

        self.path_pub = self.create_publisher(String, self.path_topic, 10)
        self.cmd_pub = self.create_publisher(String, self.dry_run_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.yolo_sub = self.create_subscription(String, self.yolo_detections_topic, self._on_yolo, 10)
        self.pixel_sub = self.create_subscription(String, self.pixel_candidates_topic, self._on_pixel, 10)

        self.last_yolo: dict[str, Any] | None = None
        self.last_yolo_ns: int | None = None
        self.last_pixel: dict[str, Any] | None = None
        self.last_pixel_ns: int | None = None
        self.last_plan: dict[str, Any] | None = None
        self.timer = self.create_timer(float(self.get_parameter("timer_period_sec").value), self._on_timer)

        self.get_logger().info(
            "parking_planner started: "
            f"yolo={self.yolo_detections_topic}, pixel={self.pixel_candidates_topic}, "
            f"path={self.path_topic}, dry_run_cmd={self.dry_run_cmd_topic}, motion_enabled=false"
        )

    def _on_yolo(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_yolo = payload
        self.last_yolo_ns = now_ns()
        self._publish_plan()

    def _on_pixel(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_pixel = payload
        self.last_pixel_ns = now_ns()
        self._publish_plan()

    def _on_timer(self) -> None:
        self._publish_plan()

    def _publish_plan(self) -> None:
        plan = self._build_plan()
        self.last_plan = plan
        path_msg = String()
        path_msg.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
        self.path_pub.publish(path_msg)

        cmd_msg = String()
        cmd_msg.data = json.dumps(self._dry_run_command(plan), ensure_ascii=False, separators=(",", ":"))
        self.cmd_pub.publish(cmd_msg)

        state_msg = String()
        state_msg.data = json.dumps(self._state_payload(plan), ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(state_msg)

    def _build_plan(self) -> dict[str, Any]:
        current_ns = now_ns()
        target = None
        if self.prefer_yolo and self._fresh(self.last_yolo_ns, current_ns):
            target = self._select_yolo_slot(self.last_yolo or {})
        if target is None and self.fallback_to_pixel_candidates and self._fresh(self.last_pixel_ns, current_ns):
            target = self._select_pixel_slot(self.last_pixel or {})

        base = {
            "schema_version": 1,
            "component": "parking_planner",
            "time_ns": current_ns,
            "mode": "dry_run_pixel_guidance",
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "vehicle": self._vehicle_payload(),
            "inputs": {
                "prefer_yolo": self.prefer_yolo,
                "yolo_fresh": self._fresh(self.last_yolo_ns, current_ns),
                "pixel_fresh": self._fresh(self.last_pixel_ns, current_ns),
                "fallback_to_pixel_candidates": self.fallback_to_pixel_candidates,
            },
        }
        if target is None:
            return {
                **base,
                "status": "waiting_for_slot",
                "selected_slot": None,
                "path": [],
                "path_norm": [],
                "control_hint": self._control_hint(None),
            }

        image_size = target.get("image_size") or [1, 1]
        image_w = max(1.0, float(image_size[0]))
        image_h = max(1.0, float(image_size[1]))
        center = target.get("center_px") or [image_w * 0.5, image_h * 0.7]
        cx = clamp(float(center[0]), 0.0, image_w)
        cy = clamp(float(center[1]), 0.0, image_h)
        cx_norm = cx / image_w
        cy_norm = cy / image_h
        path_norm = [
            [0.5, 1.0],
            [0.5 + (cx_norm - 0.5) * 0.35, 0.82],
            [cx_norm, clamp(cy_norm + 0.10, 0.0, 1.0)],
            [cx_norm, cy_norm],
        ]
        path = [[round(x * image_w, 2), round(y * image_h, 2)] for x, y in path_norm]
        return {
            **base,
            "status": "target_acquired",
            "selected_slot": target,
            "coordinate_frame": target.get("coordinate_frame", "image_pixels"),
            "image_size": [int(image_w), int(image_h)],
            "path": path,
            "path_norm": [[round(x, 4), round(y, 4)] for x, y in path_norm],
            "control_hint": self._control_hint(target),
        }

    def _fresh(self, timestamp_ns: int | None, current_ns: int) -> bool:
        if timestamp_ns is None:
            return False
        return (current_ns - timestamp_ns) / 1_000_000_000.0 <= self.stale_after_sec

    def _select_yolo_slot(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        image_size = payload.get("source_image_size") or [1, 1]
        candidates = []
        for slot in payload.get("slot_candidates", []):
            confidence = float(slot.get("confidence", 0.0))
            status = str(slot.get("status", "unknown"))
            if confidence < self.min_yolo_confidence:
                continue
            if status == "occupied":
                continue
            if status == "unknown" and not self.allow_unknown_yolo_slots:
                continue
            priority = 2 if status == "empty" else 1
            cx = float((slot.get("center_px") or [image_size[0] * 0.5, 0])[0])
            center_bias = 1.0 - abs((cx / max(1.0, float(image_size[0]))) - 0.5)
            candidates.append((priority, confidence, center_bias, slot))
        if not candidates:
            return None
        candidates.sort(key=lambda row: (row[0], row[1], row[2]), reverse=True)
        slot = dict(candidates[0][3])
        slot.setdefault("image_size", image_size)
        slot["source"] = "yolo"
        slot["coordinate_frame"] = "yolo_input_pixels"
        return slot

    def _select_pixel_slot(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        slots = list(payload.get("slots", []))
        if not slots:
            return None
        slots.sort(key=lambda row: float(row.get("confidence", 0.0)), reverse=True)
        slot = dict(slots[0])
        image_size = payload.get("processed_image_size") or [1, 1]
        polygon = slot.get("polygon") or []
        if polygon:
            xs = [float(point[0]) for point in polygon]
            ys = [float(point[1]) for point in polygon]
            slot["center_px"] = [round(sum(xs) / len(xs), 2), round(sum(ys) / len(ys), 2)]
        else:
            slot["center_px"] = [float(image_size[0]) * 0.5, float(image_size[1]) * 0.75]
        slot["source"] = "pixel_fallback"
        slot["image_size"] = image_size
        slot["coordinate_frame"] = "processed_image_pixels"
        return slot

    def _control_hint(self, target: dict[str, Any] | None) -> dict[str, Any]:
        if target is None:
            steering_deg = 0.0
            suggested_speed = 0.0
            lateral_error_norm = None
        else:
            image_size = target.get("image_size") or [1, 1]
            center = target.get("center_px") or [float(image_size[0]) * 0.5, 0.0]
            cx_norm = float(center[0]) / max(1.0, float(image_size[0]))
            lateral_error_norm = round(cx_norm - 0.5, 4)
            steering_deg = clamp(lateral_error_norm * 2.0 * self.max_steering_deg, -self.max_steering_deg, self.max_steering_deg)
            suggested_speed = -abs(self.nominal_reverse_speed_cm_s)
        return {
            "dry_run_only": True,
            "not_sent_to_vehicle": True,
            "simulated_steering_deg": round(steering_deg, 2),
            "simulated_suggested_reverse_speed_cm_s": round(suggested_speed, 2),
            "commanded_speed_cm_s": 0.0,
            "lateral_error_norm": lateral_error_norm,
            "max_steering_deg": self.max_steering_deg,
        }

    def _dry_run_command(self, plan: dict[str, Any]) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "component": "parking_controller_dry_run",
            "time_ns": now_ns(),
            "status": plan.get("status", "unknown"),
            "selected_slot_source": (plan.get("selected_slot") or {}).get("source"),
            "control_hint": plan.get("control_hint"),
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "serial_output_enabled": False,
            "can_output_enabled": False,
        }

    def _state_payload(self, plan: dict[str, Any]) -> dict[str, Any]:
        return {
            "component": "parking_planner",
            "time_ns": now_ns(),
            "ok": True,
            "status": plan.get("status", "unknown"),
            "target_source": (plan.get("selected_slot") or {}).get("source"),
            "path_points": len(plan.get("path") or []),
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }

    def _vehicle_payload(self) -> dict[str, Any]:
        required_width_cm = self.vehicle_width_cm + 2.0 * self.slot_side_clearance_cm
        required_length_cm = self.vehicle_length_cm + 2.0 * self.slot_end_clearance_cm
        return {
            "length_cm": round(self.vehicle_length_cm, 2),
            "width_cm": round(self.vehicle_width_cm, 2),
            "slot_side_clearance_cm": round(self.slot_side_clearance_cm, 2),
            "slot_end_clearance_cm": round(self.slot_end_clearance_cm, 2),
            "required_slot_width_cm": round(required_width_cm, 2),
            "required_slot_length_cm": round(required_length_cm, 2),
            "metric_fit_available": False,
            "metric_fit_reason": "camera_calibration_and_ground_scale_required",
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingPlannerNode()
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
