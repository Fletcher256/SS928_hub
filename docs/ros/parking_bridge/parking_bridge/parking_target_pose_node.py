#!/usr/bin/env python3
"""Generate a dry-run parking target pose from metric slot geometry."""

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


def point_add(a: list[float], b: list[float]) -> list[float]:
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1])]


def point_scale(v: list[float], scale: float) -> list[float]:
    return [float(v[0]) * scale, float(v[1]) * scale]


def point_sub(a: list[float], b: list[float]) -> list[float]:
    return [float(a[0]) - float(b[0]), float(a[1]) - float(b[1])]


def norm(v: list[float]) -> float:
    return math.hypot(float(v[0]), float(v[1]))


def unit(v: list[float]) -> list[float]:
    length = norm(v)
    if length < 1e-6:
        return [1.0, 0.0]
    return [float(v[0]) / length, float(v[1]) / length]


def rounded(point: list[float]) -> list[float]:
    return [round(float(point[0]), 2), round(float(point[1]), 2)]


class ParkingTargetPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_target_pose")

        self.declare_parameter("slot_geometry_topic", "/parking/slot_geometry")
        self.declare_parameter("target_pose_topic", "/parking/target_pose")
        self.declare_parameter("state_topic", "/parking/target_pose_state")
        self.declare_parameter("rear_axle_to_vehicle_center_cm", 11.0)
        self.declare_parameter("approach_distance_cm", 18.0)
        self.declare_parameter("stale_after_sec", 1.5)

        self.slot_geometry_topic = str(self.get_parameter("slot_geometry_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.rear_axle_to_vehicle_center_cm = float(self.get_parameter("rear_axle_to_vehicle_center_cm").value)
        self.approach_distance_cm = float(self.get_parameter("approach_distance_cm").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)

        self.pub = self.create_publisher(String, self.target_pose_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sub = self.create_subscription(String, self.slot_geometry_topic, self._on_slot_geometry, 10)
        self.timer = self.create_timer(1.0, self._publish_state)
        self.last_input_ns: int | None = None
        self.last_output: dict[str, Any] | None = None

        self.get_logger().info(
            "parking_target_pose started: "
            f"slot_geometry={self.slot_geometry_topic}, target={self.target_pose_topic}, "
            f"rear_axle_to_center={self.rear_axle_to_vehicle_center_cm}cm, motion_enabled=false"
        )

    def _on_slot_geometry(self, msg: String) -> None:
        self.last_input_ns = now_ns()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        output = self._build_output(payload)
        self.last_output = output
        out_msg = String()
        out_msg.data = json.dumps(output, ensure_ascii=False, separators=(",", ":"))
        self.pub.publish(out_msg)
        self._publish_state()

    def _build_output(self, payload: dict[str, Any]) -> dict[str, Any]:
        current_ns = now_ns()
        base = {
            "schema_version": 1,
            "component": "parking_target_pose",
            "time_ns": current_ns,
            "input_topic": self.slot_geometry_topic,
            "target_frame": "vehicle_rear_axle_cm",
            "coordinate_convention": {
                "origin": "vehicle_rear_axle_center",
                "x_cm": "forward",
                "y_cm": "left",
                "yaw_deg": "counterclockwise_positive",
            },
            "vehicle": {
                "rear_axle_to_vehicle_center_cm": round(self.rear_axle_to_vehicle_center_cm, 2),
            },
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }

        slot = payload.get("selected_slot")
        if not isinstance(slot, dict):
            return {**base, "status": "waiting_for_slot_geometry", "selected_slot": None}

        ground = slot.get("ground_geometry")
        if not isinstance(ground, dict):
            return {**base, "status": "waiting_for_ground_geometry", "selected_slot": None}

        center = ground.get("center_cm")
        approach_axis = ground.get("approach_axis_cm")
        entrance = ground.get("entrance_edge_cm")
        yaw = ground.get("yaw_ground_deg")
        if not isinstance(center, list) or not isinstance(approach_axis, list) or len(approach_axis) < 2:
            return {**base, "status": "invalid_slot_geometry", "selected_slot": slot}

        # approach_axis points from entrance edge toward slot center. To place the rear axle
        # while the vehicle center sits on the slot center, move the rear axle toward entrance.
        inward = unit(point_sub(approach_axis[1], approach_axis[0]))
        outward = point_scale(inward, -1.0)
        target_rear_axle = point_add(center, point_scale(outward, self.rear_axle_to_vehicle_center_cm))
        entrance_mid = approach_axis[0]
        approach_pose = point_add(entrance_mid, point_scale(outward, self.approach_distance_cm))

        path = [[0.0, 0.0], approach_pose, target_rear_axle]
        return {
            **base,
            "status": "target_pose",
            "selected_slot": slot,
            "target_rear_axle_pose_cm": {
                "x_cm": round(target_rear_axle[0], 2),
                "y_cm": round(target_rear_axle[1], 2),
                "yaw_deg": round(float(yaw), 2) if yaw is not None else None,
                "frame_id": "vehicle_rear_axle_cm",
                "meaning": "rear axle pose when vehicle center is aligned to slot center",
            },
            "approach_pose_cm": {
                "x_cm": round(approach_pose[0], 2),
                "y_cm": round(approach_pose[1], 2),
                "yaw_deg": round(float(yaw), 2) if yaw is not None else None,
                "frame_id": "vehicle_rear_axle_cm",
                "distance_before_entrance_cm": round(self.approach_distance_cm, 2),
            },
            "path_cm": [rounded(point) for point in path],
            "slot_center_cm": rounded(center),
            "slot_entrance_edge_cm": entrance,
            "slot_yaw_ground_deg": round(float(yaw), 2) if yaw is not None else None,
            "dry_run_only": True,
            "not_sent_to_vehicle": True,
        }

    def _publish_state(self) -> None:
        current_ns = now_ns()
        age_sec = None
        if self.last_input_ns is not None:
            age_sec = (current_ns - self.last_input_ns) / 1_000_000_000.0
        state = {
            "schema_version": 1,
            "component": "parking_target_pose",
            "time_ns": current_ns,
            "status": (self.last_output or {}).get("status", "waiting_for_slot_geometry"),
            "input_fresh": age_sec is not None and age_sec <= self.stale_after_sec,
            "last_input_age_sec": age_sec,
            "rear_axle_to_vehicle_center_cm": round(self.rear_axle_to_vehicle_center_cm, 2),
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingTargetPoseNode()
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
