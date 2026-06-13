#!/usr/bin/env python3
"""Pixel-only parking-line preprocessor for uncalibrated camera bring-up."""

from __future__ import annotations

from collections import deque
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


class VisionPreprocessNode(Node):
    """Run low-risk pixel-level image processing before camera calibration.

    The node deliberately publishes only diagnostics and candidate geometry.
    It does not publish control commands, metric parking poses, or actuator
    requests.
    """

    def __init__(self) -> None:
        super().__init__("parking_vision_preprocess")

        self.declare_parameter("input_topic", "/parking/camera/image_jpeg")
        self.declare_parameter("debug_image_topic", "/parking/vision/line_debug")
        self.declare_parameter("candidates_topic", "/parking/parking_slot_candidates")
        self.declare_parameter("state_topic", "/parking/perception/state")
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("process_stride", 3)
        self.declare_parameter("resize_width", 960)
        self.declare_parameter("roi_top_fraction", 0.45)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("white_s_max", 80)
        self.declare_parameter("white_v_min", 145)
        self.declare_parameter("yellow_h_min", 15)
        self.declare_parameter("yellow_h_max", 42)
        self.declare_parameter("yellow_s_min", 45)
        self.declare_parameter("yellow_v_min", 90)
        self.declare_parameter("canny_low", 60)
        self.declare_parameter("canny_high", 160)
        self.declare_parameter("hough_threshold", 45)
        self.declare_parameter("min_line_length_px", 45)
        self.declare_parameter("max_line_gap_px", 24)
        self.declare_parameter("max_lines", 24)
        self.declare_parameter("slot_angle_tolerance_deg", 14.0)
        self.declare_parameter("slot_min_pair_gap_px", 70)
        self.declare_parameter("slot_max_pair_gap_px", 520)
        self.declare_parameter("slot_max_candidates", 6)
        self.declare_parameter("slot_occupied_edge_density", 0.080)
        self.declare_parameter("vehicle_length_cm", 23.6)
        self.declare_parameter("vehicle_width_cm", 20.0)
        self.declare_parameter("slot_side_clearance_cm", 2.0)
        self.declare_parameter("slot_end_clearance_cm", 3.0)
        self.declare_parameter("status_period_sec", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.candidates_topic = str(self.get_parameter("candidates_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.process_stride = max(1, int(self.get_parameter("process_stride").value))
        self.resize_width = max(0, int(self.get_parameter("resize_width").value))
        self.roi_top_fraction = min(0.95, max(0.0, float(self.get_parameter("roi_top_fraction").value)))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.yellow_h_min = int(self.get_parameter("yellow_h_min").value)
        self.yellow_h_max = int(self.get_parameter("yellow_h_max").value)
        self.yellow_s_min = int(self.get_parameter("yellow_s_min").value)
        self.yellow_v_min = int(self.get_parameter("yellow_v_min").value)
        self.canny_low = int(self.get_parameter("canny_low").value)
        self.canny_high = int(self.get_parameter("canny_high").value)
        self.hough_threshold = int(self.get_parameter("hough_threshold").value)
        self.min_line_length_px = int(self.get_parameter("min_line_length_px").value)
        self.max_line_gap_px = int(self.get_parameter("max_line_gap_px").value)
        self.max_lines = max(1, int(self.get_parameter("max_lines").value))
        self.slot_angle_tolerance_deg = float(self.get_parameter("slot_angle_tolerance_deg").value)
        self.slot_min_pair_gap_px = int(self.get_parameter("slot_min_pair_gap_px").value)
        self.slot_max_pair_gap_px = int(self.get_parameter("slot_max_pair_gap_px").value)
        self.slot_max_candidates = max(1, int(self.get_parameter("slot_max_candidates").value))
        self.slot_occupied_edge_density = float(self.get_parameter("slot_occupied_edge_density").value)
        self.vehicle_length_cm = float(self.get_parameter("vehicle_length_cm").value)
        self.vehicle_width_cm = float(self.get_parameter("vehicle_width_cm").value)
        self.slot_side_clearance_cm = float(self.get_parameter("slot_side_clearance_cm").value)
        self.slot_end_clearance_cm = float(self.get_parameter("slot_end_clearance_cm").value)

        self.debug_pub = self.create_publisher(CompressedImage, self.debug_image_topic, qos_profile_sensor_data)
        self.candidates_pub = self.create_publisher(String, self.candidates_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.received_frames = 0
        self.processed_frames = 0
        self.decode_errors = 0
        self.last_frame_ns: int | None = None
        self.last_processed_ns: int | None = None
        self.last_payload: dict[str, Any] | None = None
        self.process_times: deque[int] = deque(maxlen=120)
        self.timer = self.create_timer(float(self.get_parameter("status_period_sec").value), self._publish_state_timer)

        self.get_logger().info(
            "parking_vision_preprocess started: "
            f"input={self.input_topic}, candidates={self.candidates_topic}, "
            f"debug={self.debug_image_topic}, pixel_only=true, motion_enabled=false"
        )

    def _on_image(self, msg: CompressedImage) -> None:
        self.received_frames += 1
        self.last_frame_ns = now_ns()
        if self.received_frames % self.process_stride != 0:
            return

        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.decode_errors += 1
            return

        payload, debug = self._process(frame, msg.header.stamp)
        self.processed_frames += 1
        self.last_processed_ns = payload["time_ns"]
        self.last_payload = payload
        self.process_times.append(payload["time_ns"])

        candidates_msg = String()
        candidates_msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.candidates_pub.publish(candidates_msg)

        debug_msg = compressed_jpeg_msg(debug, msg.header.stamp, self.camera_frame_id, self.jpeg_quality)
        if debug_msg is not None:
            self.debug_pub.publish(debug_msg)

        self._publish_state(payload)

    def _process(self, frame: np.ndarray, stamp) -> tuple[dict[str, Any], np.ndarray]:
        original_h, original_w = frame.shape[:2]
        scale = 1.0
        if self.resize_width and original_w > self.resize_width:
            scale = float(self.resize_width) / float(original_w)
            frame = cv2.resize(
                frame,
                (self.resize_width, max(1, int(round(original_h * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        h, w = frame.shape[:2]
        roi_y0 = int(round(h * self.roi_top_fraction))
        roi = frame[roi_y0:h, 0:w]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi_gray = gray[roi_y0:h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, self.white_v_min), (179, self.white_s_max, 255))
        yellow_mask = cv2.inRange(
            hsv,
            (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
            (self.yellow_h_max, 255, 255),
        )
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        edges = cv2.Canny(mask, self.canny_low, self.canny_high)
        gray_edges = cv2.Canny(roi_gray, self.canny_low, self.canny_high)

        lines_raw = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length_px,
            maxLineGap=self.max_line_gap_px,
        )
        line_payloads = self._line_payloads(lines_raw, roi_y0)
        slot_payloads = self._slot_payloads(line_payloads, gray_edges, mask, roi_y0, w, h)

        debug = self._debug_image(frame, roi_y0, mask, edges, line_payloads, slot_payloads)
        brightness_mean = float(np.mean(roi_gray)) if roi_gray.size else 0.0
        brightness_std = float(np.std(roi_gray)) if roi_gray.size else 0.0
        if slot_payloads:
            status = "slot_candidates"
        elif line_payloads:
            status = "line_candidates"
        else:
            status = "no_lines"
        payload = {
            "schema_version": 2,
            "time_ns": now_ns(),
            "ros_stamp_sec": stamp.sec,
            "ros_stamp_nanosec": stamp.nanosec,
            "mode": "pixel_only_uncalibrated",
            "motion_enabled": False,
            "calibrated": False,
            "status": status,
            "source": "os08a20_camera_jpeg",
            "coordinate_frame": "processed_image_pixels",
            "original_image_size": [int(original_w), int(original_h)],
            "processed_image_size": [int(w), int(h)],
            "scale_from_original": scale,
            "vehicle": self._vehicle_payload(),
            "roi": {"x": 0, "y": roi_y0, "width": int(w), "height": int(h - roi_y0)},
            "brightness": {"mean": brightness_mean, "std": brightness_std},
            "mask_pixels": {
                "white": int(np.count_nonzero(white_mask)),
                "yellow": int(np.count_nonzero(yellow_mask)),
                "combined": int(np.count_nonzero(mask)),
            },
            "line_count": len(line_payloads),
            "lines": line_payloads,
            "slot_count": len(slot_payloads),
            "slots": slot_payloads,
            "limits": {
                "min_line_length_px": self.min_line_length_px,
                "max_lines": self.max_lines,
                "slot_min_pair_gap_px": self.slot_min_pair_gap_px,
                "slot_max_pair_gap_px": self.slot_max_pair_gap_px,
                "slot_angle_tolerance_deg": self.slot_angle_tolerance_deg,
            },
        }
        return payload, debug

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
            "metric_fit_reason": "pixel_only_uncalibrated_requires_ground_scale",
        }

    def _vehicle_fit_payload(self) -> dict[str, Any]:
        vehicle = self._vehicle_payload()
        return {
            "status": "metric_unavailable",
            "reason": vehicle["metric_fit_reason"],
            "required_slot_width_cm": vehicle["required_slot_width_cm"],
            "required_slot_length_cm": vehicle["required_slot_length_cm"],
        }

    def _line_payloads(self, lines_raw: np.ndarray | None, roi_y0: int) -> list[dict[str, Any]]:
        lines: list[dict[str, Any]] = []
        if lines_raw is None:
            return lines
        for item in lines_raw.reshape(-1, 4):
            x1, y1_roi, x2, y2_roi = [int(v) for v in item]
            y1 = y1_roi + roi_y0
            y2 = y2_roi + roi_y0
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.hypot(dx, dy)
            if length < float(self.min_line_length_px):
                continue
            angle = math.degrees(math.atan2(dy, dx))
            lines.append({
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "length_px": round(length, 2),
                "angle_deg": round(angle, 2),
            })
        lines.sort(key=lambda row: float(row["length_px"]), reverse=True)
        return lines[: self.max_lines]

    def _slot_payloads(
        self,
        lines: list[dict[str, Any]],
        gray_edges_roi: np.ndarray,
        paint_mask_roi: np.ndarray,
        roi_y0: int,
        width: int,
        height: int,
    ) -> list[dict[str, Any]]:
        usable: list[dict[str, Any]] = []
        for index, line in enumerate(lines):
            angle = self._normalize_line_angle(float(line["angle_deg"]))
            if abs(angle) < 18.0 or abs(angle) > 88.0:
                continue
            usable.append({**line, "index": index, "norm_angle_deg": angle})

        slots: list[dict[str, Any]] = []
        for left_index in range(len(usable)):
            for right_index in range(left_index + 1, len(usable)):
                a = usable[left_index]
                b = usable[right_index]
                angle_gap = abs(float(a["norm_angle_deg"]) - float(b["norm_angle_deg"]))
                if angle_gap > self.slot_angle_tolerance_deg:
                    continue

                ax = (float(a["x1"]) + float(a["x2"])) * 0.5
                ay = (float(a["y1"]) + float(a["y2"])) * 0.5
                bx = (float(b["x1"]) + float(b["x2"])) * 0.5
                by = (float(b["y1"]) + float(b["y2"])) * 0.5
                center_gap = math.hypot(ax - bx, ay - by)
                if center_gap < self.slot_min_pair_gap_px or center_gap > self.slot_max_pair_gap_px:
                    continue

                y_overlap = self._line_y_overlap(a, b)
                if y_overlap < 0.15:
                    continue

                polygon = self._slot_polygon(a, b, width, height)
                if len(polygon) < 4:
                    continue
                metrics = self._slot_image_metrics(polygon, gray_edges_roi, paint_mask_roi, roi_y0)
                confidence = min(
                    1.0,
                    0.25
                    + min(float(a["length_px"]), float(b["length_px"])) / 420.0
                    + y_overlap * 0.25
                    - angle_gap / max(1.0, self.slot_angle_tolerance_deg) * 0.10,
                )
                status = "unknown"
                if metrics["interior_pixels"] >= 100:
                    status = (
                        "possibly_occupied"
                        if metrics["interior_edge_density"] >= self.slot_occupied_edge_density
                        else "possibly_empty"
                    )
                slots.append({
                    "id": len(slots),
                    "status": status,
                    "vehicle_fit": self._vehicle_fit_payload(),
                    "confidence": round(float(confidence), 3),
                    "polygon": polygon,
                    "line_indices": [int(a["index"]), int(b["index"])],
                    "pair_gap_px": round(center_gap, 2),
                    "angle_gap_deg": round(angle_gap, 2),
                    "y_overlap_ratio": round(y_overlap, 3),
                    "interior_edge_density": metrics["interior_edge_density"],
                    "interior_paint_density": metrics["interior_paint_density"],
                    "interior_pixels": metrics["interior_pixels"],
                })

        slots.sort(key=lambda row: (float(row["confidence"]), int(row["interior_pixels"])), reverse=True)
        for index, slot in enumerate(slots[: self.slot_max_candidates]):
            slot["id"] = index
        return slots[: self.slot_max_candidates]

    @staticmethod
    def _normalize_line_angle(angle_deg: float) -> float:
        while angle_deg > 90.0:
            angle_deg -= 180.0
        while angle_deg < -90.0:
            angle_deg += 180.0
        return angle_deg

    @staticmethod
    def _line_y_overlap(a: dict[str, Any], b: dict[str, Any]) -> float:
        a_min = min(float(a["y1"]), float(a["y2"]))
        a_max = max(float(a["y1"]), float(a["y2"]))
        b_min = min(float(b["y1"]), float(b["y2"]))
        b_max = max(float(b["y1"]), float(b["y2"]))
        overlap = max(0.0, min(a_max, b_max) - max(a_min, b_min))
        span = max(a_max, b_max) - min(a_min, b_min)
        return overlap / span if span > 0.0 else 0.0

    @staticmethod
    def _slot_polygon(a: dict[str, Any], b: dict[str, Any], width: int, height: int) -> list[list[int]]:
        points = np.array(
            [
                [int(a["x1"]), int(a["y1"])],
                [int(a["x2"]), int(a["y2"])],
                [int(b["x1"]), int(b["y1"])],
                [int(b["x2"]), int(b["y2"])],
            ],
            dtype=np.int32,
        )
        hull = cv2.convexHull(points).reshape(-1, 2)
        if len(hull) < 4:
            return []
        hull[:, 0] = np.clip(hull[:, 0], 0, max(0, width - 1))
        hull[:, 1] = np.clip(hull[:, 1], 0, max(0, height - 1))
        return [[int(x), int(y)] for x, y in hull.tolist()]

    def _slot_image_metrics(
        self,
        polygon: list[list[int]],
        gray_edges_roi: np.ndarray,
        paint_mask_roi: np.ndarray,
        roi_y0: int,
    ) -> dict[str, Any]:
        roi_h, roi_w = gray_edges_roi.shape[:2]
        poly = np.array([[x, y - roi_y0] for x, y in polygon], dtype=np.int32)
        poly[:, 0] = np.clip(poly[:, 0], 0, max(0, roi_w - 1))
        poly[:, 1] = np.clip(poly[:, 1], 0, max(0, roi_h - 1))
        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        cv2.fillConvexPoly(mask, poly, 255)
        kernel = np.ones((9, 9), dtype=np.uint8)
        interior = cv2.erode(mask, kernel, iterations=1)
        interior_pixels = int(np.count_nonzero(interior))
        if interior_pixels <= 0:
            return {
                "interior_pixels": 0,
                "interior_edge_density": 0.0,
                "interior_paint_density": 0.0,
            }
        edge_pixels = int(np.count_nonzero(gray_edges_roi[interior > 0]))
        paint_pixels = int(np.count_nonzero(paint_mask_roi[interior > 0]))
        return {
            "interior_pixels": interior_pixels,
            "interior_edge_density": round(edge_pixels / interior_pixels, 4),
            "interior_paint_density": round(paint_pixels / interior_pixels, 4),
        }

    def _debug_image(
        self,
        frame: np.ndarray,
        roi_y0: int,
        mask: np.ndarray,
        edges: np.ndarray,
        lines: list[dict[str, Any]],
        slots: list[dict[str, Any]],
    ) -> np.ndarray:
        debug = frame.copy()
        h, w = debug.shape[:2]
        cv2.rectangle(debug, (0, roi_y0), (w - 1, h - 1), (255, 180, 0), 2)
        overlay = debug[roi_y0:h, 0:w].copy()
        overlay[mask > 0] = (0, 220, 255)
        debug[roi_y0:h, 0:w] = cv2.addWeighted(overlay, 0.25, debug[roi_y0:h, 0:w], 0.75, 0.0)
        for line in lines:
            p1 = (int(line["x1"]), int(line["y1"]))
            p2 = (int(line["x2"]), int(line["y2"]))
            cv2.line(debug, p1, p2, (0, 255, 0), 2, cv2.LINE_AA)
        for slot in slots:
            points = np.array(slot["polygon"], dtype=np.int32)
            color = (60, 210, 60) if slot["status"] == "possibly_empty" else (0, 165, 255)
            if slot["status"] == "possibly_occupied":
                color = (40, 40, 230)
            cv2.polylines(debug, [points], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
            cx = int(np.mean(points[:, 0]))
            cy = int(np.mean(points[:, 1]))
            cv2.putText(
                debug,
                f"S{slot['id']} {slot['status']}",
                (max(4, cx - 70), max(20, cy)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                f"S{slot['id']} {slot['status']}",
                (max(4, cx - 70), max(20, cy)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                1,
                cv2.LINE_AA,
            )
        edge_small = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        edge_small = cv2.resize(edge_small, (min(240, w), max(1, int((h - roi_y0) * min(240, w) / w))))
        debug[0:edge_small.shape[0], 0:edge_small.shape[1]] = edge_small
        text = f"pixel-only lines={len(lines)} slots={len(slots)} motion=false calibrated=false"
        cv2.putText(debug, text, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(debug, text, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        return debug

    def _rate(self) -> float:
        if len(self.process_times) < 2:
            return 0.0
        elapsed = (self.process_times[-1] - self.process_times[0]) / 1_000_000_000.0
        return (len(self.process_times) - 1) / elapsed if elapsed > 0.0 else 0.0

    def _state_payload(self, last: dict[str, Any] | None) -> dict[str, Any]:
        current_ns = now_ns()
        frame_age = None if self.last_frame_ns is None else (current_ns - self.last_frame_ns) / 1e9
        processed_age = None if self.last_processed_ns is None else (current_ns - self.last_processed_ns) / 1e9
        return {
            "schema_version": 2,
            "time_ns": current_ns,
            "mode": "perception_only",
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "calibrated": False,
            "camera_frame_ok": frame_age is not None and frame_age < 2.0,
            "camera_frame_age_sec": frame_age,
            "processed_age_sec": processed_age,
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "decode_errors": self.decode_errors,
            "processing_fps": self._rate(),
            "status": (last or {}).get("status", "waiting_for_camera"),
            "parking_line_candidates": int((last or {}).get("line_count", 0)),
            "parking_slot_candidates": int((last or {}).get("slot_count", 0)),
            "vehicle": (last or {}).get("vehicle", self._vehicle_payload()),
            "candidate_topic": self.candidates_topic,
            "debug_topic": self.debug_image_topic,
        }

    def _publish_state(self, payload: dict[str, Any] | None = None) -> None:
        msg = String()
        msg.data = json.dumps(self._state_payload(payload), ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(msg)

    def _publish_state_timer(self) -> None:
        self._publish_state(self.last_payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPreprocessNode()
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
