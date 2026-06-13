#!/usr/bin/env python3
"""Dry-run metric reverse-parking planner (cm, rear-axle frame).

This node consumes the metric target pose (`/parking/target_pose`) and produces
a small-step, low-speed REVERSE parking plan expressed in centimetres in the
vehicle rear-axle frame. It publishes JSON diagnostics only. It never opens an
actuator, serial, UDP, CAN, motor, steering, brake, or throttle interface, and
it never commits to a full open-loop trajectory: every cycle it recommends a
single small reverse step, expecting the operator to stop and re-read fresh
perception after each step.

Frame note (rear-mounted camera -> reverse parking):
  origin  = vehicle rear axle centre
  +x_cm   = from rear axle toward the slot = rear-camera look direction
            = VEHICLE REVERSE DIRECTION (physically behind the car).
            Upstream nodes label this axis "forward"; for this rear-camera
            setup that label is a misnomer. Here it is the reverse direction.
  +y_cm   = lateral (upstream convention: left). Steering sign that maps a
            lateral/heading error to a servo direction is UNVERIFIED and must
            be confirmed by a tiny real-vehicle test before trusting it.
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


def unit(vx: float, vy: float) -> tuple[float, float]:
    length = math.hypot(vx, vy)
    if length < 1e-6:
        return (1.0, 0.0)
    return (vx / length, vy / length)


class ParkingMetricPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_metric_planner")

        self.declare_parameter("target_pose_topic", "/parking/target_pose")
        self.declare_parameter("path_topic", "/parking/planner/path_cm")
        self.declare_parameter("state_topic", "/parking/planner/path_cm_state")
        self.declare_parameter("stale_after_sec", 1.5)
        self.declare_parameter("timer_period_sec", 0.5)
        # Vehicle geometry.
        self.declare_parameter("rear_axle_to_vehicle_center_cm", 11.0)
        # Small-step / low-speed dry-run motion shaping.
        # step_cm is real ground distance per increment. Kept >> the measured 2cm
        # startup deadband so the deadband is a small, well-compensated fraction.
        self.declare_parameter("step_cm", 5.0)
        # Distance calibration (2026-06-09, V=1, landed two-point fit):
        #   actual_ground_cm = 1.0 * |stm32_command_D| - deadband_cm
        # i.e. to move N cm of real ground, command D = sign * (N + deadband_cm).
        self.declare_parameter("command_distance_deadband_cm", 2.0)
        self.declare_parameter("command_speed_gear", 1)
        self.declare_parameter("longitudinal_tolerance_cm", 2.0)
        self.declare_parameter("lateral_tolerance_cm", 2.0)
        self.declare_parameter("heading_tolerance_deg", 3.0)
        self.declare_parameter("preview_step_cm", 3.0)
        self.declare_parameter("max_preview_points", 40)
        # Steering hint (sign UNVERIFIED; tune k_* and steering_sign on the bench).
        self.declare_parameter("k_lateral_deg_per_cm", 1.5)
        self.declare_parameter("k_heading_deg_per_deg", 0.5)
        self.declare_parameter("max_steering_deg", 25.0)
        self.declare_parameter("steering_sign", 1.0)
        # Reverse pure-pursuit steering (heading-free; slot_yaw proved unreliable).
        self.declare_parameter("wheelbase_cm", 14.0)
        self.declare_parameter("min_lookahead_cm", 10.0)
        # Servo mapping (verified 2026-06-09, suspended): STE>90 => front wheels LEFT,
        # 135 full-left, 45 full-right, 90 center. steering_hint already carries the
        # (still UNVERIFIED) correction sign; positive hint => wheels left here.
        self.declare_parameter("servo_center_deg", 90.0)
        self.declare_parameter("servo_min_deg", 45.0)
        self.declare_parameter("servo_max_deg", 135.0)
        self.declare_parameter("steering_straight_deadzone_deg", 2.0)

        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)
        self.rear_axle_to_vehicle_center_cm = float(self.get_parameter("rear_axle_to_vehicle_center_cm").value)
        self.step_cm = abs(float(self.get_parameter("step_cm").value))
        self.command_distance_deadband_cm = abs(float(self.get_parameter("command_distance_deadband_cm").value))
        self.command_speed_gear = int(self.get_parameter("command_speed_gear").value)
        self.longitudinal_tolerance_cm = abs(float(self.get_parameter("longitudinal_tolerance_cm").value))
        self.lateral_tolerance_cm = abs(float(self.get_parameter("lateral_tolerance_cm").value))
        self.heading_tolerance_deg = abs(float(self.get_parameter("heading_tolerance_deg").value))
        self.preview_step_cm = abs(float(self.get_parameter("preview_step_cm").value))
        self.max_preview_points = max(2, int(self.get_parameter("max_preview_points").value))
        self.k_lateral_deg_per_cm = float(self.get_parameter("k_lateral_deg_per_cm").value)
        self.k_heading_deg_per_deg = float(self.get_parameter("k_heading_deg_per_deg").value)
        self.max_steering_deg = abs(float(self.get_parameter("max_steering_deg").value))
        self.steering_sign = float(self.get_parameter("steering_sign").value)
        self.wheelbase_cm = abs(float(self.get_parameter("wheelbase_cm").value))
        self.min_lookahead_cm = abs(float(self.get_parameter("min_lookahead_cm").value))
        self.servo_center_deg = float(self.get_parameter("servo_center_deg").value)
        self.servo_min_deg = float(self.get_parameter("servo_min_deg").value)
        self.servo_max_deg = float(self.get_parameter("servo_max_deg").value)
        self.steering_straight_deadzone_deg = abs(float(self.get_parameter("steering_straight_deadzone_deg").value))

        self.path_pub = self.create_publisher(String, self.path_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sub = self.create_subscription(String, self.target_pose_topic, self._on_target_pose, 10)
        self.timer = self.create_timer(float(self.get_parameter("timer_period_sec").value), self._on_timer)

        self.last_input_ns: int | None = None
        self.last_target: dict[str, Any] | None = None
        self.last_plan: dict[str, Any] | None = None

        self.get_logger().info(
            "parking_metric_planner started: "
            f"target_pose={self.target_pose_topic}, path={self.path_topic}, "
            f"maneuver=REVERSE, step_cm={self.step_cm}, motion_enabled=false"
        )

    def _on_target_pose(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_input_ns = now_ns()
        self.last_target = payload
        self._publish()

    def _on_timer(self) -> None:
        self._publish()

    def _publish(self) -> None:
        plan = self._build_plan()
        self.last_plan = plan
        path_msg = String()
        path_msg.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
        self.path_pub.publish(path_msg)
        state_msg = String()
        state_msg.data = json.dumps(self._state_payload(plan), ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(state_msg)

    def _fresh(self, current_ns: int) -> bool:
        if self.last_input_ns is None:
            return False
        return (current_ns - self.last_input_ns) / 1_000_000_000.0 <= self.stale_after_sec

    def _base(self, current_ns: int) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "component": "parking_metric_planner",
            "time_ns": current_ns,
            "mode": "dry_run_reverse_metric",
            "maneuver": "REVERSE",
            "input_topic": self.target_pose_topic,
            "target_frame": "vehicle_rear_axle_cm",
            "frame_note": (
                "+x_cm points from rear axle toward the slot = rear-camera look "
                "direction = vehicle REVERSE direction (physically behind the car). "
                "Upstream 'x_cm: forward' label is a misnomer for this rear-camera setup."
            ),
            "coordinate_convention": {
                "origin": "vehicle_rear_axle_center",
                "x_cm": "reverse_direction_toward_slot",
                "y_cm": "left",
                "yaw_deg": "counterclockwise_positive",
            },
            "vehicle": {
                "rear_axle_to_vehicle_center_cm": round(self.rear_axle_to_vehicle_center_cm, 2),
            },
            "distance_calibration": {
                "model": "actual_ground_cm = 1.0 * abs(stm32_command_D) - deadband_cm",
                "scale": 1.0,
                "deadband_cm": round(self.command_distance_deadband_cm, 2),
                "command_rule": "stm32_command_D = sign * (desired_ground_cm + deadband_cm)",
                "measured_points_cmd_to_ground_cm": [[10, 8], [20, 18]],
                "speed_gear_tested": self.command_speed_gear,
                "note": (
                    "single V=1 two-point landed fit 2026-06-09; deadband may vary "
                    "and small steps are less precise; verify before trusting"
                ),
            },
            "steering_convention": {
                "control_law": "reverse_pure_pursuit",
                "uses_heading": False,
                "wheelbase_cm": self.wheelbase_cm,
                "min_lookahead_cm": self.min_lookahead_cm,
                "servo_center_deg": self.servo_center_deg,
                "servo_min_deg": self.servo_min_deg,
                "servo_max_deg": self.servo_max_deg,
                "servo_mapping": "STE>90 => front wheels LEFT (verified 2026-06-09 suspended); 135 full-left, 45 full-right",
                "note": (
                    "rear-axle reverse pure-pursuit toward the target point; curvature self-reduces "
                    "as it aligns. Heading-free because slot_yaw proved unreliable. steering_sign=+1 "
                    "verified (STE>90 + reverse => rear LEFT => toward +y target). On-car validation pending."
                ),
            },
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "serial_output_enabled": False,
            "can_output_enabled": False,
            "dry_run_only": True,
            "not_sent_to_vehicle": True,
            "requires_operator_confirmation_per_step": True,
            "safety_note": (
                "dry-run only: recommends one small reverse step per cycle. Stop and "
                "re-read /parking/target_pose after every step."
            ),
        }

    def _build_plan(self) -> dict[str, Any]:
        current_ns = now_ns()
        base = self._base(current_ns)
        target = self.last_target or {}

        if not self._fresh(current_ns):
            return {**base, "status": "waiting_for_target", "input_fresh": False, "path_cm": []}
        if str(target.get("status")) != "target_pose":
            return {
                **base,
                "status": "no_target_pose",
                "input_fresh": True,
                "upstream_status": target.get("status"),
                "path_cm": [],
            }

        center = target.get("slot_center_cm")
        yaw = target.get("slot_yaw_ground_deg")
        if not isinstance(center, list) or len(center) < 2:
            return {**base, "status": "invalid_target", "input_fresh": True, "path_cm": []}
        cx, cy = float(center[0]), float(center[1])
        yaw_deg = float(yaw) if yaw is not None else 0.0

        # Inward direction = entrance -> deeper into slot. Prefer the measured
        # approach axis from the embedded slot geometry; fall back to slot yaw.
        inward = self._inward_from_target(target)
        if inward is None:
            inward = unit(math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)))
        left = (-inward[1], inward[0])  # +90 deg from inward

        # Reverse parking: rear axle sits DEEPER than the vehicle centre, because
        # the rear axle leads the car into the slot. (Forward-parking sign is the
        # opposite; the upstream target_pose_node uses the forward-parking sign.)
        r = self.rear_axle_to_vehicle_center_cm
        target_rear_axle = (cx + inward[0] * r, cy + inward[1] * r)

        # Errors are taken from the current rear-axle origin (0,0). The rear-axle
        # frame is vehicle-relative, so re-reading perception after each step
        # naturally shrinks these as the car backs up.
        longitudinal_remaining = target_rear_axle[0] * inward[0] + target_rear_axle[1] * inward[1]
        lateral_error = target_rear_axle[0] * left[0] + target_rear_axle[1] * left[1]
        heading_error = yaw_deg  # want slot aligned straight behind -> yaw ~ 0

        aligned = (
            abs(longitudinal_remaining) <= self.longitudinal_tolerance_cm
            and abs(lateral_error) <= self.lateral_tolerance_cm
            and abs(heading_error) <= self.heading_tolerance_deg
        )

        # Reverse pure-pursuit steering toward the target rear-axle POINT.
        # Uses only the target point (longitudinal_remaining, lateral_error); it does
        # NOT use slot_yaw/heading, which proved unreliable (read ~0 even while turning).
        # Curvature kappa = 2*lat/Ld^2 self-reduces as the rear axle aligns, so it
        # converges where a proportional steer-on-lateral law diverged (lat 2.9->7.8cm).
        lookahead = max(self.min_lookahead_cm, math.hypot(longitudinal_remaining, lateral_error))
        curvature = 2.0 * lateral_error / (lookahead * lookahead)  # 1/cm; +lat (target left) => +curvature
        delta_deg = math.degrees(math.atan(self.wheelbase_cm * curvature))
        steering_hint = clamp(delta_deg, -self.max_steering_deg, self.max_steering_deg) * self.steering_sign

        # Servo command via the verified mapping (STE>90 => wheels LEFT). steering_hint
        # already carries the (UNVERIFIED) correction sign; +hint => +offset => wheels left.
        servo = clamp(self.servo_center_deg + steering_hint, self.servo_min_deg, self.servo_max_deg)
        servo_i = int(round(servo))
        if aligned:
            next_step = {
                "direction": "STOP",
                "distance_cm": 0.0,
                "stm32_command_distance_cm": 0.0,
                "stm32_servo_deg": int(round(self.servo_center_deg)),
                "stm32_candidate_cmd": "STOP",
                "steering_hint_deg": 0.0,
                "reason": "within_tolerance_aligned",
            }
        else:
            step = min(self.step_cm, abs(longitudinal_remaining)) if longitudinal_remaining > 0 else self.step_cm
            # Deadband compensation: command D = -(ground_step + deadband) so the car
            # actually travels `step` cm of real ground (reverse => negative D).
            command_d = -(round(step, 2) + self.command_distance_deadband_cm)
            if abs(servo - self.servo_center_deg) <= self.steering_straight_deadzone_deg:
                candidate = f"MOVE D={round(command_d, 1)} V={self.command_speed_gear}"
            else:
                candidate = f"ARC D={round(command_d, 1)} STE={servo_i} V={self.command_speed_gear}"
            next_step = {
                "direction": "REVERSE",
                "distance_cm": round(step, 2),
                "stm32_command_distance_cm": round(command_d, 2),
                "stm32_servo_deg": servo_i,
                "stm32_candidate_cmd": candidate,
                "steering_hint_deg": round(steering_hint, 2),
                "reason": "reverse_one_small_step_then_replan",
            }
        next_step.update(
            {
                "steering_actuation_verified": True,
                "dry_run_only": True,
                "not_sent_to_vehicle": True,
                "requires_operator_confirmation": True,
            }
        )

        path_cm = self._preview_path(target_rear_axle)
        return {
            **base,
            "status": "aligned" if aligned else "planning",
            "input_fresh": True,
            "slot_center_cm": [round(cx, 2), round(cy, 2)],
            "slot_yaw_ground_deg": round(yaw_deg, 2),
            "inward_unit": [round(inward[0], 4), round(inward[1], 4)],
            "target_rear_axle_cm": [round(target_rear_axle[0], 2), round(target_rear_axle[1], 2)],
            "target_rear_axle_note": "rear axle pose when vehicle centre is aligned to slot centre (reverse-parked)",
            "errors": {
                "longitudinal_remaining_cm": round(longitudinal_remaining, 2),
                "lateral_error_cm": round(lateral_error, 2),
                "heading_error_deg": round(heading_error, 2),
            },
            "tolerances": {
                "longitudinal_cm": self.longitudinal_tolerance_cm,
                "lateral_cm": self.lateral_tolerance_cm,
                "heading_deg": self.heading_tolerance_deg,
            },
            "next_step": next_step,
            "path_cm": path_cm,
            "path_point_count": len(path_cm),
        }

    def _inward_from_target(self, target: dict[str, Any]) -> tuple[float, float] | None:
        slot = target.get("selected_slot")
        if not isinstance(slot, dict):
            return None
        ground = slot.get("ground_geometry")
        if not isinstance(ground, dict):
            return None
        axis = ground.get("approach_axis_cm")
        if not isinstance(axis, list) or len(axis) < 2:
            return None
        p0, p1 = axis[0], axis[1]
        try:
            return unit(float(p1[0]) - float(p0[0]), float(p1[1]) - float(p0[1]))
        except (TypeError, ValueError, IndexError):
            return None

    def _preview_path(self, target_rear_axle: tuple[float, float]) -> list[list[float]]:
        """Straight-line rear-axle waypoint preview from origin to target (for viz)."""
        tx, ty = target_rear_axle
        total = math.hypot(tx, ty)
        if total < 1e-6:
            return [[0.0, 0.0]]
        count = min(self.max_preview_points, max(2, int(math.ceil(total / max(0.5, self.preview_step_cm))) + 1))
        points = []
        for i in range(count):
            frac = i / (count - 1)
            points.append([round(tx * frac, 2), round(ty * frac, 2)])
        return points

    def _state_payload(self, plan: dict[str, Any]) -> dict[str, Any]:
        return {
            "component": "parking_metric_planner",
            "time_ns": plan.get("time_ns"),
            "ok": True,
            "status": plan.get("status"),
            "maneuver": "REVERSE",
            "input_fresh": plan.get("input_fresh", False),
            "next_step": plan.get("next_step"),
            "errors": plan.get("errors"),
            "path_point_count": plan.get("path_point_count", 0),
            "motion_enabled": False,
            "actuator_control_allowed": False,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingMetricPlannerNode()
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
