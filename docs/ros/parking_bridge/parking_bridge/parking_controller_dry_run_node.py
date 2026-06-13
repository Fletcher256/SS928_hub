#!/usr/bin/env python3
"""Dry-run controller that converts parking plans into candidate STM32 V2 commands.

This node never opens serial, UDP, CAN, motor, steering, brake, or throttle
interfaces. It only publishes JSON and text diagnostics showing what command
would be considered after the safety gates pass.
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


def get_path(payload: dict[str, Any], path: str, default: Any = None) -> Any:
    node: Any = payload
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


class ParkingControllerDryRunNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_controller_dry_run")

        self.declare_parameter("planner_path_topic", "/parking/planner/path")
        self.declare_parameter("stm32_health_topic", "/parking/stm32/health")
        self.declare_parameter("proposed_cmd_topic", "/parking/controller/proposed_cmd")
        self.declare_parameter("v2_candidate_topic", "/parking/controller/v2_candidate")
        self.declare_parameter("state_topic", "/parking/controller/state")
        self.declare_parameter("timer_period_sec", 0.2)
        self.declare_parameter("planner_stale_after_sec", 1.0)
        self.declare_parameter("stm32_stale_after_sec", 2.0)
        self.declare_parameter("require_stm32_health", False)
        self.declare_parameter("required_stable_frames", 5)
        self.declare_parameter("max_target_center_shift_norm", 0.08)
        self.declare_parameter("reverse_step_cm", 5.0)
        self.declare_parameter("approach_step_cm", 3.0)
        self.declare_parameter("speed_gear", 1)
        self.declare_parameter("servo_center_deg", 90.0)
        self.declare_parameter("servo_min_deg", 45.0)
        self.declare_parameter("servo_max_deg", 135.0)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("max_abs_steering_hint_deg", 25.0)
        self.declare_parameter("command_sequence_start", 1000)

        self.planner_path_topic = str(self.get_parameter("planner_path_topic").value)
        self.stm32_health_topic = str(self.get_parameter("stm32_health_topic").value)
        self.proposed_cmd_topic = str(self.get_parameter("proposed_cmd_topic").value)
        self.v2_candidate_topic = str(self.get_parameter("v2_candidate_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.planner_stale_after_sec = float(self.get_parameter("planner_stale_after_sec").value)
        self.stm32_stale_after_sec = float(self.get_parameter("stm32_stale_after_sec").value)
        self.require_stm32_health = as_bool(self.get_parameter("require_stm32_health").value)
        self.required_stable_frames = max(1, int(self.get_parameter("required_stable_frames").value))
        self.max_target_center_shift_norm = float(self.get_parameter("max_target_center_shift_norm").value)
        self.reverse_step_cm = abs(float(self.get_parameter("reverse_step_cm").value))
        self.approach_step_cm = abs(float(self.get_parameter("approach_step_cm").value))
        self.speed_gear = int(clamp(float(self.get_parameter("speed_gear").value), 0.0, 6.0))
        self.servo_center_deg = float(self.get_parameter("servo_center_deg").value)
        self.servo_min_deg = float(self.get_parameter("servo_min_deg").value)
        self.servo_max_deg = float(self.get_parameter("servo_max_deg").value)
        self.steering_sign = float(self.get_parameter("steering_sign").value)
        self.max_abs_steering_hint_deg = abs(float(self.get_parameter("max_abs_steering_hint_deg").value))
        self.sequence = int(self.get_parameter("command_sequence_start").value)

        self.proposed_pub = self.create_publisher(String, self.proposed_cmd_topic, 10)
        self.v2_pub = self.create_publisher(String, self.v2_candidate_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.path_sub = self.create_subscription(String, self.planner_path_topic, self._on_plan, 10)
        self.stm32_sub = self.create_subscription(String, self.stm32_health_topic, self._on_stm32_health, 10)

        self.last_plan: dict[str, Any] | None = None
        self.last_plan_ns: int | None = None
        self.last_stm32_health: dict[str, Any] | None = None
        self.last_stm32_health_ns: int | None = None
        self.last_target_center: tuple[float, float] | None = None
        self.stable_frames = 0
        self.last_payload: dict[str, Any] | None = None

        period = float(self.get_parameter("timer_period_sec").value)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "parking_controller_dry_run started: "
            f"plan={self.planner_path_topic}, stm32={self.stm32_health_topic}, "
            f"proposed={self.proposed_cmd_topic}, serial_output_enabled=false"
        )

    def _on_plan(self, msg: String) -> None:
        try:
            self.last_plan = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_plan_ns = now_ns()
        self._update_stability(self.last_plan)
        self._publish()

    def _on_stm32_health(self, msg: String) -> None:
        try:
            self.last_stm32_health = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_stm32_health_ns = now_ns()

    def _on_timer(self) -> None:
        self._publish()

    def _fresh(self, timestamp_ns: int | None, current_ns: int, stale_after_sec: float) -> bool:
        if timestamp_ns is None:
            return False
        return (current_ns - timestamp_ns) / 1_000_000_000.0 <= stale_after_sec

    def _target_center_norm(self, plan: dict[str, Any]) -> list[float] | None:
        center = get_path(plan, "selected_slot.center_norm")
        if isinstance(center, list) and len(center) >= 2:
            return [float(center[0]), float(center[1])]

        center_px = get_path(plan, "selected_slot.center_px")
        image_size = get_path(plan, "selected_slot.image_size") or plan.get("image_size")
        if (
            isinstance(center_px, list)
            and len(center_px) >= 2
            and isinstance(image_size, list)
            and len(image_size) >= 2
        ):
            image_w = max(1.0, float(image_size[0]))
            image_h = max(1.0, float(image_size[1]))
            return [
                clamp(float(center_px[0]) / image_w, 0.0, 1.0),
                clamp(float(center_px[1]) / image_h, 0.0, 1.0),
            ]

        path_norm = plan.get("path_norm")
        if isinstance(path_norm, list) and path_norm:
            last = path_norm[-1]
            if isinstance(last, list) and len(last) >= 2:
                return [float(last[0]), float(last[1])]
        return None

    def _update_stability(self, plan: dict[str, Any]) -> None:
        center = self._target_center_norm(plan)
        if center is None:
            self.last_target_center = None
            self.stable_frames = 0
            return
        current = (float(center[0]), float(center[1]))
        if self.last_target_center is None:
            self.stable_frames = 1
        else:
            dx = current[0] - self.last_target_center[0]
            dy = current[1] - self.last_target_center[1]
            shift = math.hypot(dx, dy)
            if shift <= self.max_target_center_shift_norm:
                self.stable_frames += 1
            else:
                self.stable_frames = 1
        self.last_target_center = current

    def _publish(self) -> None:
        payload = self._build_payload()
        self.last_payload = payload

        proposed = String()
        proposed.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.proposed_pub.publish(proposed)

        v2 = String()
        v2.data = str(payload.get("candidate_v2") or "")
        self.v2_pub.publish(v2)

        state = String()
        state.data = json.dumps(self._state_payload(payload), ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(state)

    def _build_payload(self) -> dict[str, Any]:
        current_ns = now_ns()
        plan = self.last_plan or {}
        planner_fresh = self._fresh(self.last_plan_ns, current_ns, self.planner_stale_after_sec)
        stm32_fresh = self._fresh(self.last_stm32_health_ns, current_ns, self.stm32_stale_after_sec)
        stm32_ok = bool(get_path(self.last_stm32_health or {}, "ok", False)) if stm32_fresh else False
        plan_status = str(plan.get("status", "missing_plan"))
        target = plan.get("selected_slot") if isinstance(plan.get("selected_slot"), dict) else None
        target_acquired = planner_fresh and plan_status == "target_acquired" and target is not None
        stable_enough = self.stable_frames >= self.required_stable_frames

        gates = {
            "planner_fresh": planner_fresh,
            "target_acquired": target_acquired,
            "target_stable": stable_enough,
            "stm32_health_fresh": stm32_fresh,
            "stm32_health_ok": stm32_ok,
            "stm32_required": self.require_stm32_health,
            "serial_output_enabled": False,
            "actuator_control_allowed": False,
            "operator_armed": False,
        }
        if self.require_stm32_health:
            gates["all_dry_run_gates_passed"] = target_acquired and stable_enough and stm32_ok
        else:
            gates["all_dry_run_gates_passed"] = target_acquired and stable_enough

        candidate = self._candidate_for_plan(plan, target, gates)
        blocked_reasons = self._blocked_reasons(gates)
        payload = {
            "schema_version": 1,
            "component": "parking_controller_dry_run",
            "time_ns": current_ns,
            "mode": "dry_run_only",
            "status": "candidate_ready" if candidate["candidate_v2"] and gates["all_dry_run_gates_passed"] else "blocked",
            "send_to_stm32": False,
            "motion_enabled": False,
            "serial_output_enabled": False,
            "actuator_control_allowed": False,
            "candidate_v2": candidate["candidate_v2"],
            "candidate": candidate,
            "gates": gates,
            "blocked_reasons": blocked_reasons,
            "planner": {
                "topic": self.planner_path_topic,
                "status": plan_status,
                "fresh": planner_fresh,
                "stable_frames": self.stable_frames,
                "required_stable_frames": self.required_stable_frames,
                "target_source": get_path(plan, "selected_slot.source"),
                "target_center_norm": self._target_center_norm(plan),
                "control_hint": plan.get("control_hint"),
            },
            "stm32": {
                "topic": self.stm32_health_topic,
                "fresh": stm32_fresh,
                "ok": stm32_ok,
                "required": self.require_stm32_health,
                "health": self.last_stm32_health,
            },
            "safety_note": (
                "dry-run only: this node never opens a serial/UDP/CAN/actuator path "
                "and never sends commands to STM32"
            ),
        }
        return payload

    def _candidate_for_plan(
        self,
        plan: dict[str, Any],
        target: dict[str, Any] | None,
        gates: dict[str, bool],
    ) -> dict[str, Any]:
        self.sequence += 1
        seq = self.sequence
        if not gates["planner_fresh"]:
            return self._candidate(seq, "WAIT", "", "planner_stale_or_missing")
        if target is None or plan.get("status") != "target_acquired":
            return self._candidate(seq, "STOP", f"@{seq} STOP", "no_stable_target")

        steering_hint = float(get_path(plan, "control_hint.simulated_steering_deg", 0.0) or 0.0)
        steering_hint = clamp(steering_hint, -self.max_abs_steering_hint_deg, self.max_abs_steering_hint_deg)
        servo = self.servo_center_deg + self.steering_sign * steering_hint
        servo = clamp(servo, self.servo_min_deg, self.servo_max_deg)
        servo_i = int(round(servo))
        v = self.speed_gear

        center = target.get("center_norm") or [0.5, 0.7]
        cy = float(center[1]) if isinstance(center, list) and len(center) >= 2 else 0.7
        distance = -self.reverse_step_cm
        action = "ARC"
        reason = "target_acquired_reverse_arc_step"
        if abs(servo - self.servo_center_deg) <= 2.0:
            action = "MOVE"
            distance = -self.approach_step_cm
            reason = "target_centered_reverse_move_step"
        if cy < 0.45:
            distance = -min(self.approach_step_cm, self.reverse_step_cm)
            reason += "_far_in_image"

        if action == "ARC":
            command = f"@{seq} ARC D={distance:.1f} STE={servo_i} V={v}"
        else:
            command = f"@{seq} MOVE D={distance:.1f} V={v}"
        return self._candidate(
            seq,
            action,
            command,
            reason,
            steering_hint_deg=round(steering_hint, 2),
            servo_deg=servo_i,
            distance_cm=round(distance, 2),
            speed_gear=v,
            steering_sign=self.steering_sign,
        )

    @staticmethod
    def _candidate(seq: int, action: str, command: str, reason: str, **kwargs: Any) -> dict[str, Any]:
        return {
            "seq": seq,
            "action": action,
            "candidate_v2": command,
            "reason": reason,
            "not_sent_to_vehicle": True,
            **kwargs,
        }

    def _blocked_reasons(self, gates: dict[str, bool]) -> list[str]:
        reasons = []
        if not gates["planner_fresh"]:
            reasons.append("planner_stale")
        if not gates["target_acquired"]:
            reasons.append("target_not_acquired")
        if not gates["target_stable"]:
            reasons.append("target_not_stable")
        if gates["stm32_required"] and not gates["stm32_health_ok"]:
            reasons.append("stm32_health_not_ok")
        reasons.append("operator_not_armed")
        reasons.append("serial_output_disabled_by_design")
        return reasons

    @staticmethod
    def _state_payload(payload: dict[str, Any]) -> dict[str, Any]:
        return {
            "component": "parking_controller_dry_run",
            "time_ns": payload.get("time_ns"),
            "ok": True,
            "status": payload.get("status"),
            "candidate_v2": payload.get("candidate_v2"),
            "send_to_stm32": False,
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "blocked_reasons": payload.get("blocked_reasons", []),
            "gates": payload.get("gates", {}),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingControllerDryRunNode()
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
