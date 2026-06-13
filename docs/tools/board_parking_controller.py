#!/usr/bin/env python3
"""Standalone ON-BOARD reverse-parking controller (no PC / no VM / no ROS).

Runs entirely on the SS928 board with pure stdlib:
  YOLO C binary (sample_parking_yolo_rtsp) --UDP 127.0.0.1:24580--> this program:
    1. receive detection JSON locally
    2. slot pixel geometry (convex hull -> min-area oriented bbox -> edges)   [pure py]
    3. apply hardcoded pixel->ground homography (3x3)                          [pure py]
    4. reverse pure-pursuit planner -> next MOVE/ARC command                   [pure py]
    5. drive STM32 over /dev/ttyUSB0 (closed-loop); read STAT (IMU yaw/odom)
    6. when vision drops, stop by default; optional dead-reckon is gated

Safety: motion requires --arm AND the arm file (default /tmp/parking_armed) to
exist; --dry-run computes and prints but never sends motion. Caps + divergence
guard + Ctrl-C STOP. Read-only PING/VER/STAT are always allowed.

This is a faithful pure-Python consolidation of the (verified) board_yolo_udp_node
slot geometry, slot_geometry_transform homography, parking_metric_planner
pure-pursuit, and parking_step_executor control loop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import socket
import subprocess
import sys
import time

try:
    from parking_fusion import PoseFuser, load_chassis_signs, parse_stm32_text, wrap_degrees
except ImportError:  # Keep older board deployments runnable if the helper is absent.
    PoseFuser = None
    load_chassis_signs = None
    parse_stm32_text = None
    wrap_degrees = None

# ----------------------------------------------------------------------------
# Hardcoded calibration (offline-computed; see tools/_compute_homography.py).
# Pixel(YOLO 640x640) -> ground cm in rear-axle frame [+x reverse/toward slot, +y left].
H = [
    [-0.002841148786767839, -0.0902385437276105, 64.96470339525213],
    [-0.06211726961004081, -6.827304779515962e-05, 21.07360636711848],
    [-4.793066796001953e-05, 4.628315527150238e-05, 1.0],
]

# Vehicle / control (verified this project).
REAR_AXLE_TO_CENTER_CM = 11.0   # rear axle sits this much deeper than vehicle centre
DEADBAND_CM = 2.0               # actual_ground ~= |commanded D| - deadband
WHEELBASE_CM = 14.0
MIN_LOOKAHEAD_CM = 10.0
MAX_STEER_DEG = 25.0
STEERING_SIGN = 1.0             # STE>90 => front wheels LEFT (verified)
SERVO_CENTER, SERVO_MIN, SERVO_MAX = 90.0, 45.0, 135.0
STEER_DEADZONE_DEG = 2.0
STEP_CM = 8.0                   # ground step per cycle
GEAR = 1

# Tolerances / safety caps.
LON_TOL_CM = 2.0
LAT_TOL_CM = 2.0
HEAD_TOL_DEG = 3.0
LATERAL_DIV_CM = 3.0            # divergence guard: lateral grew vs best
MAX_STEPS = 16
MAX_TOTAL_CM = 70.0
MAX_BLIND_CM = 30.0
DEFAULT_STABLE_FRAMES = 5
DEFAULT_MAX_CENTER_SHIFT_CM = 2.5
DEFAULT_MAX_AXIS_YAW_SHIFT_DEG = 8.0
DEFAULT_LAT_TEMPLATE_THRESHOLD_CM = 3.0
DEFAULT_HEAD_TEMPLATE_THRESHOLD_DEG = 8.0

# Pixel-servo defaults for direct image-space closed loop.
IMAGE_W = 640.0
IMAGE_H = 640.0
PIXEL_X_TARGET = IMAGE_W * 0.5
PIXEL_ANGLE_TARGET_DEG = -90.0
DEFAULT_PIXEL_X_TOL = 18.0
DEFAULT_PIXEL_ANGLE_TOL_DEG = 5.0
DEFAULT_PIXEL_STOP_CENTER_Y = 560.0
DEFAULT_PIXEL_STOP_BOX_H = 420.0
DEFAULT_CORRIDOR_SAMPLE_Y = 600.0
DEFAULT_CORRIDOR_ENTRY_Y = 560.0
DEFAULT_SUCCESS_CRITERIA = {
    "schema": "parking_success_criteria.v1",
    "version": "builtin",
    "done": {
        "slot_x_err_px_abs_max": 15.0,
        "slot_heading_err_deg_abs_max": 4.0,
        "slot_y_dist_cm_max": 10.0,
        "min_margin_px_min": 60.0,
        "required_stable_frames": 3,
    },
    "abort": {
        "min_margin_px_floor": 40.0,
        "vision_lost_sec": 0.5,
        "edge_recovery_enabled": True,
        "edge_recovery_min_margin_px": 30.0,
        "edge_recovery_predicted_min_margin_px": 40.0,
        "edge_recovery_min_margin_gain_px": 5.0,
        "edge_recovery_require_x_improve": True,
        "max_total_cm": 60.0,
        "max_steps": 12,
        "divergence_x_err_px": 200.0,
    },
}

DEFAULT_PERCEPTION_FILTER = {
    "schema": "perception_filter.v1",
    "required_frames": 5,
    "gate_center_shift_cm": 3.0,
    "gate_yaw_shift_deg": 6.0,
    "gate_static_scale": 0.5,
    "outlier_accept_consecutive": 3,
    "hold_grace_sec": 1.0,
    "hold_max_frames": 4,
    "divergence_debounce_frames": 2,
    "line_risk_debounce_frames": 1,
}

TTY = "/dev/ttyUSB0"
VID, PID = "1a86", "7523"
CH341_INIT = "/opt/parking/stm32_uart/ch341_user_init"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _deep_update(base, override):
    out = json.loads(json.dumps(base))
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_update(out[key], value)
        else:
            out[key] = value
    return out


def load_success_criteria(path):
    """Load parking done/abort thresholds, falling back to safe built-ins."""
    if not path:
        return json.loads(json.dumps(DEFAULT_SUCCESS_CRITERIA))
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except OSError:
        return json.loads(json.dumps(DEFAULT_SUCCESS_CRITERIA))
    return _deep_update(DEFAULT_SUCCESS_CRITERIA, data)


def load_perception_filter(path):
    """Load perception filtering thresholds, falling back to conservative built-ins."""
    if not path:
        return json.loads(json.dumps(DEFAULT_PERCEPTION_FILTER))
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except OSError:
        return json.loads(json.dumps(DEFAULT_PERCEPTION_FILTER))
    merged = json.loads(json.dumps(DEFAULT_PERCEPTION_FILTER))
    for key, value in data.items():
        if value is not None:
            merged[key] = value
    return merged


def _num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def evaluate_parking_criteria(slot_state, criteria, steps=0, total_cm=0.0):
    """Evaluate configured parked/abort gates against slot_relative_state."""
    done = criteria.get("done", {})
    abort = criteria.get("abort", {})
    corridor = slot_state.get("corridor") or {}
    image = slot_state.get("image") or {}
    ground = slot_state.get("ground_estimate") or {}
    gates = slot_state.get("gates") or {}

    slot_x_abs = abs(_num(corridor.get("slot_x_err_px")))
    heading_abs = abs(_num(image.get("slot_heading_err_deg")))
    y_dist = _num(ground.get("slot_y_dist_cm"))
    min_margin = _num(corridor.get("min_margin_px"), 9999.0)
    stable_frames = int(_num(slot_state.get("stable_frames"), 0))
    required_stable = int(_num(done.get("required_stable_frames"), 3))
    line_risk = bool(corridor.get("line_risk"))

    done_checks = {
        "slot_x_err_px_abs": round(slot_x_abs, 3),
        "slot_x_ok": slot_x_abs <= _num(done.get("slot_x_err_px_abs_max"), 15.0),
        "slot_heading_err_deg_abs": round(heading_abs, 3),
        "heading_ok": heading_abs <= _num(done.get("slot_heading_err_deg_abs_max"), 4.0),
        "slot_y_dist_cm": round(y_dist, 3),
        "distance_ok": y_dist <= _num(done.get("slot_y_dist_cm_max"), 10.0),
        "min_margin_px": round(min_margin, 3),
        "margin_ok": min_margin >= _num(done.get("min_margin_px_min"), 60.0),
        "stable_frames": stable_frames,
        "required_stable_frames": required_stable,
        "stable_ok": stable_frames >= required_stable and bool(gates.get("stable_enough", True)),
        "line_risk": line_risk,
    }
    abort_checks = {
        "min_margin_floor_ok": min_margin >= _num(abort.get("min_margin_px_floor"), 40.0),
        "slot_x_divergence_ok": slot_x_abs <= _num(abort.get("divergence_x_err_px"), 200.0),
        "max_steps_ok": int(steps) < int(_num(abort.get("max_steps"), 12)),
        "max_total_cm_ok": _num(total_cm) < _num(abort.get("max_total_cm"), 60.0),
        "line_risk_ok": not line_risk,
    }
    abort_reason_map = {
        "min_margin_floor_ok": "min_margin_below_floor",
        "slot_x_divergence_ok": "slot_x_error_diverged",
        "max_steps_ok": "max_steps_reached",
        "max_total_cm_ok": "max_total_cm_reached",
        "line_risk_ok": "line_risk",
    }
    abort_reasons = [abort_reason_map.get(key, key) for key, ok in abort_checks.items() if not ok]
    if abort_reasons:
        return {
            "verdict": "aborted",
            "reason": abort_reasons[0],
            "exit_code": 6,
            "done": done_checks,
            "abort": abort_checks,
        }
    parked = (
        done_checks["slot_x_ok"] and
        done_checks["heading_ok"] and
        done_checks["distance_ok"] and
        done_checks["margin_ok"] and
        done_checks["stable_ok"] and
        not line_risk
    )
    if parked:
        return {
            "verdict": "parked",
            "reason": "success_criteria_met",
            "exit_code": 0,
            "done": done_checks,
            "abort": abort_checks,
        }
    return {
        "verdict": "continue",
        "reason": "criteria_not_met",
        "exit_code": None,
        "done": done_checks,
        "abort": abort_checks,
    }


def planner_edge_recovery_context(state, criteria):
    abort = (criteria or {}).get("abort", {})
    enabled = bool(abort.get("edge_recovery_enabled", True))
    min_margin = planner_to_float(state.get("min_margin_px"))
    normal_floor = planner_to_float(abort.get("min_margin_px_floor"), 40.0)
    recovery_floor = planner_to_float(abort.get("edge_recovery_min_margin_px"), 30.0)
    pred_floor = planner_to_float(abort.get("edge_recovery_predicted_min_margin_px"), normal_floor)
    min_gain = planner_to_float(abort.get("edge_recovery_min_margin_gain_px"), 5.0)
    active = (
        enabled and
        not bool(state.get("line_risk")) and
        min_margin >= recovery_floor and
        min_margin < normal_floor
    )
    return {
        "active": bool(active),
        "min_margin_px": round(min_margin, 3),
        "normal_floor_px": round(normal_floor, 3),
        "recovery_floor_px": round(recovery_floor, 3),
        "predicted_min_margin_px": round(pred_floor, 3),
        "min_margin_gain_px": round(min_gain, 3),
        "require_x_improve": bool(abort.get("edge_recovery_require_x_improve", True)),
    }


# ============================ pixel slot geometry ============================
# (ported from board_yolo_udp_node: convex hull + min-area oriented bbox + edges)

def _convex_hull(points):
    uniq = sorted({(round(p[0], 4), round(p[1], 4)) for p in points})
    if len(uniq) <= 1:
        return [[x, y] for x, y in uniq]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in uniq:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(uniq):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [[x, y] for x, y in (lower[:-1] + upper[:-1])]


def _clockwise_from_top_left(points):
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    ordered = sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    start = min(range(len(ordered)), key=lambda i: (ordered[i][1] + ordered[i][0], ordered[i][1]))
    return ordered[start:] + ordered[:start]


def _oriented_bbox(points):
    hull = _convex_hull(points)
    if len(hull) < 3:
        return []
    best = None
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        angle = math.atan2(b[1] - a[1], b[0] - a[0])
        ca, sa = math.cos(-angle), math.sin(-angle)
        rot = [[p[0] * ca - p[1] * sa, p[0] * sa + p[1] * ca] for p in hull]
        xs = [p[0] for p in rot]
        ys = [p[1] for p in rot]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        area = (maxx - minx) * (maxy - miny)
        rr = [[minx, miny], [maxx, miny], [maxx, maxy], [minx, maxy]]
        cb, sb = math.cos(angle), math.sin(angle)
        rect = [[p[0] * cb - p[1] * sb, p[0] * sb + p[1] * cb] for p in rr]
        if best is None or area < best[0]:
            best = (area, rect)
    return _clockwise_from_top_left(best[1]) if best else []


def _edge_x_at_y(edge, y):
    a, b = edge
    dy = b[1] - a[1]
    if abs(dy) < 1e-6:
        return (a[0] + b[0]) * 0.5
    t = (y - a[1]) / dy
    return a[0] + t * (b[0] - a[0])


def _edge_mid(edge):
    return [(edge[0][0] + edge[1][0]) * 0.5, (edge[0][1] + edge[1][1]) * 0.5]


def _corridor_sample_from_edges(left_edge, right_edge, y):
    lx = _edge_x_at_y(left_edge, y)
    rx = _edge_x_at_y(right_edge, y)
    if lx > rx:
        lx, rx = rx, lx
    return {
        "y": float(y),
        "left_x": lx,
        "right_x": rx,
        "center_x": (lx + rx) * 0.5,
        "width_px": rx - lx,
    }


def slot_pixel_geometry(polygon):
    """polygon (mask points or bbox) -> dict with corners_px, center_px, approach_axis_px."""
    pts = [[float(p[0]), float(p[1])] for p in polygon if len(p) >= 2]
    if len(pts) < 3:
        return None
    corners = _oriented_bbox(pts)
    if len(corners) != 4:
        return None
    edges = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        edges.append({"index": i, "a": a, "b": b,
                      "mid": [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]})
    entrance = max(edges, key=lambda e: e["mid"][1])   # nearest edge = largest image y
    back = edges[(entrance["index"] + 2) % 4]
    side_a = edges[(entrance["index"] + 1) % 4]
    side_b = edges[(entrance["index"] + 3) % 4]
    side_a_edge = [side_a["a"], side_a["b"]]
    side_b_edge = [side_b["a"], side_b["b"]]
    sample_y = max(0.0, min(IMAGE_H - 1.0, DEFAULT_CORRIDOR_SAMPLE_Y))
    side_a_x = _edge_x_at_y(side_a_edge, sample_y)
    side_b_x = _edge_x_at_y(side_b_edge, sample_y)
    if side_a_x <= side_b_x:
        left_edge, right_edge = side_a_edge, side_b_edge
    else:
        left_edge, right_edge = side_b_edge, side_a_edge
    corridor_sample = _corridor_sample_from_edges(left_edge, right_edge, sample_y)
    center = [(entrance["mid"][0] + back["mid"][0]) * 0.5,
              (entrance["mid"][1] + back["mid"][1]) * 0.5]
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    axis_dx = center[0] - entrance["mid"][0]
    axis_dy = center[1] - entrance["mid"][1]
    entrance_dx = entrance["b"][0] - entrance["a"][0]
    entrance_dy = entrance["b"][1] - entrance["a"][1]
    return {
        "corners_px": corners,
        "center_px": center,
        "entrance_mid_px": entrance["mid"],
        "entrance_edge_px": [entrance["a"], entrance["b"]],
        "back_edge_px": [back["a"], back["b"]],
        "left_edge_px": left_edge,
        "right_edge_px": right_edge,
        "corridor_sample_px": corridor_sample,
        "approach_axis_px": [entrance["mid"], center],   # entrance -> deeper into slot
        "axis_angle_px_deg": math.degrees(math.atan2(axis_dy, axis_dx)),
        "entrance_angle_px_deg": math.degrees(math.atan2(entrance_dy, entrance_dx)),
        "bbox_w_px": max(xs) - min(xs),
        "bbox_h_px": max(ys) - min(ys),
        "bbox_area_px": (max(xs) - min(xs)) * (max(ys) - min(ys)),
    }


# ============================ homography -> ground cm ============================

def apply_h(px, py):
    a = H[0][0] * px + H[0][1] * py + H[0][2]
    b = H[1][0] * px + H[1][1] * py + H[1][2]
    w = H[2][0] * px + H[2][1] * py + H[2][2]
    if abs(w) < 1e-9:
        return (float("nan"), float("nan"))
    return (a / w, b / w)


def unit(vx, vy):
    L = math.hypot(vx, vy)
    return (1.0, 0.0) if L < 1e-6 else (vx / L, vy / L)


def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def axis_yaw_deg(axis_cm):
    return math.degrees(math.atan2(axis_cm[1][1] - axis_cm[0][1], axis_cm[1][0] - axis_cm[0][0]))


def mean(values):
    return sum(values) / float(len(values)) if values else 0.0


def median(values):
    vals = sorted(values)
    if not vals:
        return 0.0
    mid = len(vals) // 2
    if len(vals) % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def _mean_point(points):
    return [mean([p[0] for p in points]), mean([p[1] for p in points])]


def _mean_edge(edges):
    return [_mean_point([e[0] for e in edges]), _mean_point([e[1] for e in edges])]


def _median_point(points):
    return [median([p[0] for p in points]), median([p[1] for p in points])]


def _median_edge(edges):
    return [_median_point([e[0] for e in edges]), _median_point([e[1] for e in edges])]


# ============================ pure-pursuit planner ============================

def plan(slot_center_cm, approach_axis_cm):
    """Return dict with lon, lat, head, candidate command, servo, step (ground cm)."""
    cx, cy = slot_center_cm
    inward = unit(approach_axis_cm[1][0] - approach_axis_cm[0][0],
                  approach_axis_cm[1][1] - approach_axis_cm[0][1])
    left = (-inward[1], inward[0])
    r = REAR_AXLE_TO_CENTER_CM
    tx, ty = cx + inward[0] * r, cy + inward[1] * r          # target rear-axle point
    lon = tx * inward[0] + ty * inward[1]
    lat = tx * left[0] + ty * left[1]
    head = math.degrees(math.atan2(inward[1], inward[0]))    # approach-axis bearing (info)

    aligned = abs(lon) <= LON_TOL_CM and abs(lat) <= LAT_TOL_CM
    cmd, servo, step = pursuit_command(lon, lat)
    return {"lon": lon, "lat": lat, "head": head, "aligned": aligned,
            "cmd": cmd, "servo": servo, "step": step,
            "target": (tx, ty), "inward": inward}


def pursuit_command(lon, lat):
    lookahead = max(MIN_LOOKAHEAD_CM, math.hypot(lon, lat))
    curvature = 2.0 * lat / (lookahead * lookahead)
    delta = math.degrees(math.atan(WHEELBASE_CM * curvature))
    steer = clamp(delta, -MAX_STEER_DEG, MAX_STEER_DEG) * STEERING_SIGN
    servo = clamp(SERVO_CENTER + steer, SERVO_MIN, SERVO_MAX)
    servo_i = int(round(servo))
    step = min(STEP_CM, lon) if lon > 0 else STEP_CM
    cmd_d = -(round(step, 1) + DEADBAND_CM)
    if abs(servo - SERVO_CENTER) <= STEER_DEADZONE_DEG:
        cmd = "MOVE D=%.1f V=%d" % (cmd_d, GEAR)
    else:
        cmd = "ARC D=%.1f STE=%d V=%d" % (cmd_d, servo_i, GEAR)
    return cmd, servo_i, step


def predict_slot(anchor, cur_yaw, ds):
    """Dead-reckon target (lon,lat) from anchor using IMU yaw delta + commanded ds."""
    dpsi = math.radians(cur_yaw - anchor["yaw"])
    dx = ds * math.cos(dpsi / 2.0)
    dy = ds * math.sin(dpsi / 2.0)
    qx, qy = anchor["lon"] - dx, anchor["lat"] - dy
    c, s = math.cos(-dpsi), math.sin(-dpsi)
    return (c * qx - s * qy, s * qx + c * qy)


# ============================ STM32 serial ============================

def _usbdev_path():
    base = "/sys/bus/usb/devices"
    for name in os.listdir(base):
        d = os.path.join(base, name)
        try:
            v = open(os.path.join(d, "idVendor")).read().strip()
            p = open(os.path.join(d, "idProduct")).read().strip()
        except OSError:
            continue
        if v == VID and p == PID:
            bus = open(os.path.join(d, "busnum")).read().strip()
            dev = open(os.path.join(d, "devnum")).read().strip()
            return "/dev/bus/usb/%03d/%03d" % (int(bus), int(dev))
    return None


def serial_setup():
    usbdev = _usbdev_path()
    if usbdev and os.access(CH341_INIT, os.X_OK):
        subprocess.run([CH341_INIT, usbdev], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["stty", "-F", TTY, "9600", "cs8", "-cstopb", "-parenb", "-ixon", "-ixoff",
                    "-crtscts", "-hupcl", "clocal", "cread", "raw", "-echo", "min", "0", "time", "1"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


_seq = [1000]


def send_cmd(cmd, read_sec=6.0):
    """Write '@seq cmd\\r' to the tty, read response until terminal line or timeout."""
    _seq[0] += 1
    seq = _seq[0]
    fd = os.open(TTY, os.O_RDWR | os.O_NOCTTY)
    try:
        # drain
        end = time.time() + 0.4
        while time.time() < end:
            try:
                os.read(fd, 256)
            except OSError:
                break
        os.write(fd, ("@%d %s\r" % (seq, cmd)).encode())
        buf = b""
        end = time.time() + read_sec
        while time.time() < end:
            try:
                chunk = os.read(fd, 256)
            except OSError:
                chunk = b""
            if chunk:
                buf += chunk
                normalized = buf.replace(b"\r", b"\n")
                if ((b"\nDONE " in normalized or normalized.startswith(b"DONE ")) and
                    normalized.endswith(b"\n")):
                    break
                if ((b"\nERR " in normalized or normalized.startswith(b"ERR ")) and
                    normalized.endswith(b"\n")):
                    break
            else:
                time.sleep(0.05)
        return buf.decode("ascii", "replace")
    finally:
        os.close(fd)


def read_stat():
    resp = send_cmd("STAT", read_sec=3.0)
    yaw = d = None
    for tok in resp.replace("\r", " ").split():
        if tok.startswith("YAW="):
            try:
                yaw = float(tok[4:])
            except ValueError:
                pass
        elif tok.startswith("D="):
            try:
                d = float(tok[2:])
            except ValueError:
                pass
    return {"yaw": yaw, "d": d, "raw": resp.strip()}


def parse_stm32_events(text):
    if parse_stm32_text is None:
        return []
    try:
        return parse_stm32_text(text or "")
    except Exception as exc:
        return [{"type": "parse_error", "error": str(exc), "raw": text or ""}]


def stop():
    send_cmd("STOP", read_sec=2.0)


def final_stop_on_exit(args, motion_authorized, reason):
    if not getattr(args, "final_stop_on_exit", True):
        return
    if not motion_authorized:
        return
    try:
        serial_setup()
        resp = send_cmd("STOP", read_sec=2.0).strip()
        print("FINAL_STOP_ON_EXIT reason=%s resp=%s" % (reason, resp), flush=True)
        log_event(getattr(args, "log_jsonl", ""), {
            "event": "final_stop_on_exit",
            "reason": reason,
            "response": resp,
            "send_to_stm32": True,
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "time_unix": time.time(),
        })
    except Exception as exc:
        print("WARN: final STOP on exit failed: %s" % exc, file=sys.stderr, flush=True)
        log_event(getattr(args, "log_jsonl", ""), {
            "event": "final_stop_on_exit_error",
            "reason": reason,
            "error": str(exc),
            "send_to_stm32": False,
            "motion_enabled": False,
            "actuator_control_allowed": False,
            "time_unix": time.time(),
        })


def query_pwm_stat(read_sec=0.8):
    return send_cmd("PWM_STAT", read_sec=read_sec).strip()


# ============================ perception input ============================

def slot_infos_from_udp(raw):
    """raw detection JSON -> all structured slot candidates."""
    dets = raw.get("detections") or []
    infos = []
    for idx, det in enumerate(dets):
        conf = float(det.get("confidence", det.get("score", 0.0)))
        poly = det.get("mask_polygon")
        if not poly:
            bb = det.get("bbox_xyxy")
            if bb:
                poly = [[bb[0], bb[1]], [bb[2], bb[1]], [bb[2], bb[3]], [bb[0], bb[3]]]
        if not poly:
            continue
        geom = slot_pixel_geometry(poly)
        if not geom:
            continue
        center_cm = apply_h(*geom["center_px"])
        axis_cm = [apply_h(*geom["approach_axis_px"][0]), apply_h(*geom["approach_axis_px"][1])]
        if any(math.isnan(v) for v in (center_cm[0], center_cm[1])):
            continue
        p = plan(center_cm, axis_cm)
        infos.append({
            "confidence": conf,
            "class_name": det.get("class_name", "Parking"),
            "bbox_xyxy": det.get("bbox_xyxy"),
            "mask_polygon_px": poly,
            "mask_area_px": det.get("mask_area_px"),
            "center_px": geom["center_px"],
            "entrance_mid_px": geom["entrance_mid_px"],
            "entrance_edge_px": geom["entrance_edge_px"],
            "back_edge_px": geom["back_edge_px"],
            "left_edge_px": geom["left_edge_px"],
            "right_edge_px": geom["right_edge_px"],
            "corridor_sample_px": geom["corridor_sample_px"],
            "approach_axis_px": geom["approach_axis_px"],
            "axis_angle_px_deg": geom["axis_angle_px_deg"],
            "entrance_angle_px_deg": geom["entrance_angle_px_deg"],
            "bbox_w_px": geom["bbox_w_px"],
            "bbox_h_px": geom["bbox_h_px"],
            "bbox_area_px": geom["bbox_area_px"],
            "center_cm": center_cm,
            "approach_axis_cm": axis_cm,
            "axis_yaw_deg": axis_yaw_deg(axis_cm),
            "plan": p,
            "raw_detection_index": idx,
            "raw_detection_id": det.get("id", idx),
            "raw_detection_count": len(dets),
            "raw_time_ns": raw.get("time_ns"),
        })
    return infos


def summarize_slot_candidate(info, args):
    corr = corridor_metrics(info, args)
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    return {
        "raw_detection_index": info.get("raw_detection_index"),
        "raw_detection_id": info.get("raw_detection_id"),
        "confidence": round(float(info.get("confidence", 0.0)), 4),
        "center_px": [round(info["center_px"][0], 2), round(info["center_px"][1], 2)],
        "entrance_mid_px": [round(info["entrance_mid_px"][0], 2), round(info["entrance_mid_px"][1], 2)],
        "slot_x_err_px": round(corr["corridor_x_err"], 2),
        "slot_entry_x_err_px": round(corr["entry_x_err"], 2),
        "slot_heading_err_deg": round(angle_err, 3),
        "min_margin_px": round(corr["min_margin_px"], 2),
        "bbox_h_px": round(float(info.get("bbox_h_px", 0.0)), 2),
        "bbox_w_px": round(float(info.get("bbox_w_px", 0.0)), 2),
        "center_cm": [round(info["center_cm"][0], 3), round(info["center_cm"][1], 3)],
        "axis_yaw_deg": round(info["axis_yaw_deg"], 3),
    }


def best_slot_info_from_udp(raw):
    """raw detection JSON -> structured slot info or None."""
    infos = slot_infos_from_udp(raw)
    if not infos:
        return None
    return max(infos, key=lambda item: float(item.get("confidence", 0.0)))


def best_slot_from_udp(raw):
    """raw detection JSON -> (slot_center_cm, approach_axis_cm) or None."""
    info = best_slot_info_from_udp(raw)
    if not info:
        return None
    return (info["center_cm"], info["approach_axis_cm"])


def recv_latest(sock):
    """Drain the UDP socket, return the most recent parsed detection JSON, or None."""
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            break
        except OSError:
            break
        try:
            latest = json.loads(data.decode("utf-8", "replace").strip())
        except ValueError:
            continue
    return latest


class SlotStabilityFilter:
    def __init__(self, required_frames, max_center_shift_cm, max_axis_yaw_shift_deg,
                 outlier_accept_consecutive=3, hold_grace_sec=0.0, hold_max_frames=0,
                 gate_static_scale=0.5):
        self.required_frames = max(1, int(required_frames))
        self.max_center_shift_cm = float(max_center_shift_cm)
        self.max_axis_yaw_shift_deg = float(max_axis_yaw_shift_deg)
        self.outlier_accept_consecutive = max(1, int(outlier_accept_consecutive))
        self.hold_grace_sec = max(0.0, float(hold_grace_sec))
        self.hold_max_frames = max(0, int(hold_max_frames))
        self.gate_static_scale = max(0.05, min(1.0, float(gate_static_scale)))
        self.samples = []
        self.rejected = 0
        self.pending_outliers = []
        self.consecutive_outliers = 0
        self.last_good_ts = None
        self.hold_frames = 0

    def reset(self):
        self.samples = []
        self.pending_outliers = []
        self.consecutive_outliers = 0
        self.hold_frames = 0

    def _reference(self):
        if not self.samples:
            return None
        return self.fused()

    def _is_consistent_pending(self):
        if len(self.pending_outliers) < self.outlier_accept_consecutive:
            return False
        pending = self.pending_outliers[-self.outlier_accept_consecutive:]
        ref = pending[0]
        for item in pending[1:]:
            dc = math.hypot(
                item["center_cm"][0] - ref["center_cm"][0],
                item["center_cm"][1] - ref["center_cm"][1],
            )
            dyaw = abs(angle_diff_deg(item["axis_yaw_deg"], ref["axis_yaw_deg"]))
            if dc > self.max_center_shift_cm or dyaw > self.max_axis_yaw_shift_deg:
                return False
        return True

    def add(self, info, observing_static=True):
        now = time.time()
        gate_scale = self.gate_static_scale if observing_static else 1.0
        gate_center = self.max_center_shift_cm * gate_scale
        gate_yaw = self.max_axis_yaw_shift_deg * gate_scale
        if len(self.samples) >= self.required_frames:
            ref = self._reference()
            dc = math.hypot(
                info["center_cm"][0] - ref["center_cm"][0],
                info["center_cm"][1] - ref["center_cm"][1],
            )
            dyaw = abs(angle_diff_deg(info["axis_yaw_deg"], ref["axis_yaw_deg"]))
            if dc > gate_center or dyaw > gate_yaw:
                self.rejected += 1
                self.consecutive_outliers += 1
                self.pending_outliers.append(info)
                self.pending_outliers = self.pending_outliers[-self.outlier_accept_consecutive:]
                if self._is_consistent_pending():
                    self.samples = list(self.pending_outliers)
                    self.pending_outliers = []
                    self.consecutive_outliers = 0
                    self.last_good_ts = now
                    return self.is_stable(), {
                        **self.metrics(),
                        "accepted_outlier_cluster": True,
                        "center_shift_cm": round(dc, 3),
                        "axis_yaw_shift_deg": round(dyaw, 3),
                    }
                stable = self.is_stable()
                return stable, {
                    **self.metrics(),
                    "outlier_rejected": True,
                    "center_shift_cm": round(dc, 3),
                    "axis_yaw_shift_deg": round(dyaw, 3),
                    "consecutive_outliers": self.consecutive_outliers,
                    "gate_center_shift_cm": round(gate_center, 3),
                    "gate_yaw_shift_deg": round(gate_yaw, 3),
                }
        elif self.samples:
            prev = self.samples[-1]
            dc = math.hypot(
                info["center_cm"][0] - prev["center_cm"][0],
                info["center_cm"][1] - prev["center_cm"][1],
            )
            dyaw = abs(angle_diff_deg(info["axis_yaw_deg"], prev["axis_yaw_deg"]))
            if dc > gate_center or dyaw > gate_yaw:
                self.rejected += 1
                self.pending_outliers = [info]
                self.consecutive_outliers = 1
                return False, {
                    "stable_frames": len(self.samples),
                    "required_frames": self.required_frames,
                    "outlier_rejected": True,
                    "center_shift_cm": round(dc, 3),
                    "axis_yaw_shift_deg": round(dyaw, 3),
                    "consecutive_outliers": self.consecutive_outliers,
                }
        self.pending_outliers = []
        self.consecutive_outliers = 0
        self.hold_frames = 0
        self.samples.append(info)
        if len(self.samples) > self.required_frames:
            self.samples = self.samples[-self.required_frames:]
        self.last_good_ts = now
        return self.is_stable(), self.metrics()

    def tick_no_detection(self):
        if not self.samples or self.hold_grace_sec <= 0.0:
            return None, {"hold": False, "reason": "no_stable_state"}
        now = time.time()
        age = now - self.last_good_ts if self.last_good_ts else 999.0
        if age > self.hold_grace_sec:
            return None, {"hold": False, "reason": "hold_grace_expired", "age_sec": round(age, 3)}
        if self.hold_max_frames and self.hold_frames >= self.hold_max_frames:
            return None, {"hold": False, "reason": "hold_frame_cap", "age_sec": round(age, 3)}
        fused = self.fused()
        fused["coasted"] = True
        fused["coast_age_ms"] = int(round(age * 1000.0))
        self.hold_frames += 1
        return fused, {
            "hold": True,
            "reason": "hold_last_stable",
            "age_sec": round(age, 3),
            "hold_frames": self.hold_frames,
        }

    def is_stable(self):
        return len(self.samples) >= self.required_frames

    def metrics(self):
        if not self.samples:
            return {
                "stable_frames": 0,
                "required_frames": self.required_frames,
                "center_span_cm": None,
                "axis_yaw_span_deg": None,
                "confidence_mean": None,
            }
        xs = [s["center_cm"][0] for s in self.samples]
        ys = [s["center_cm"][1] for s in self.samples]
        yaws = [s["axis_yaw_deg"] for s in self.samples]
        yaw0 = yaws[0]
        yaw_offsets = [angle_diff_deg(y, yaw0) for y in yaws]
        cx0 = median(xs)
        cy0 = median(ys)
        center_span = max(math.hypot(x - cx0, y - cy0) for x, y in zip(xs, ys))
        return {
            "stable_frames": len(self.samples),
            "required_frames": self.required_frames,
            "center_span_cm": round(center_span, 3),
            "axis_yaw_span_deg": round(max(yaw_offsets) - min(yaw_offsets), 3),
            "confidence_mean": round(mean([s["confidence"] for s in self.samples]), 4),
            "rejected_resets": self.rejected,
        }

    def fused(self):
        if not self.samples:
            return None
        latest = dict(self.samples[-1])
        xs = [s["center_cm"][0] for s in self.samples]
        ys = [s["center_cm"][1] for s in self.samples]
        ax0 = [s["approach_axis_cm"][0][0] for s in self.samples]
        ay0 = [s["approach_axis_cm"][0][1] for s in self.samples]
        ax1 = [s["approach_axis_cm"][1][0] for s in self.samples]
        ay1 = [s["approach_axis_cm"][1][1] for s in self.samples]
        cpx = [s["center_px"][0] for s in self.samples]
        cpy = [s["center_px"][1] for s in self.samples]
        epx = [s["entrance_mid_px"][0] for s in self.samples]
        epy = [s["entrance_mid_px"][1] for s in self.samples]
        angles_px = [s["axis_angle_px_deg"] for s in self.samples]
        angle0 = angles_px[0]
        angle_px = angle0 + median([angle_diff_deg(a, angle0) for a in angles_px])
        latest["center_cm"] = (median(xs), median(ys))
        latest["approach_axis_cm"] = [(median(ax0), median(ay0)), (median(ax1), median(ay1))]
        latest["axis_yaw_deg"] = axis_yaw_deg(latest["approach_axis_cm"])
        latest["center_px"] = [median(cpx), median(cpy)]
        latest["entrance_mid_px"] = [median(epx), median(epy)]
        latest["entrance_edge_px"] = _median_edge([s["entrance_edge_px"] for s in self.samples])
        latest["back_edge_px"] = _median_edge([s["back_edge_px"] for s in self.samples])
        latest["left_edge_px"] = _median_edge([s["left_edge_px"] for s in self.samples])
        latest["right_edge_px"] = _median_edge([s["right_edge_px"] for s in self.samples])
        latest["corridor_sample_px"] = _corridor_sample_from_edges(
            latest["left_edge_px"], latest["right_edge_px"], DEFAULT_CORRIDOR_SAMPLE_Y)
        latest["axis_angle_px_deg"] = angle_px
        latest["bbox_w_px"] = median([s["bbox_w_px"] for s in self.samples])
        latest["bbox_h_px"] = median([s["bbox_h_px"] for s in self.samples])
        latest["bbox_area_px"] = median([s["bbox_area_px"] for s in self.samples])
        latest["confidence"] = mean([s["confidence"] for s in self.samples])
        latest["plan"] = plan(latest["center_cm"], latest["approach_axis_cm"])
        latest["stability"] = self.metrics()
        return latest


class SlotTargetSelector:
    def __init__(self, args):
        self.args = args
        self.locked = None
        self.rejected_switches = 0
        self.selected_count = 0

    def reset(self):
        self.locked = None
        self.rejected_switches = 0
        self.selected_count = 0

    def _score(self, info):
        summary = summarize_slot_candidate(info, self.args)
        confidence = float(info.get("confidence", 0.0))
        x_abs = abs(summary["slot_x_err_px"])
        entry_abs = abs(summary["slot_entry_x_err_px"])
        heading_abs = abs(summary["slot_heading_err_deg"])
        margin = summary["min_margin_px"]
        center_y = float(info["center_px"][1])
        bbox_h = float(info.get("bbox_h_px", 0.0))
        score = (
            x_abs * self.args.slot_select_x_weight +
            entry_abs * self.args.slot_select_entry_x_weight +
            heading_abs * self.args.slot_select_heading_weight -
            confidence * self.args.slot_select_confidence_weight -
            max(0.0, margin) * self.args.slot_select_margin_weight -
            center_y * self.args.slot_select_center_y_weight -
            bbox_h * self.args.slot_select_bbox_h_weight
        )
        lock = None
        if self.locked is not None:
            dc = math.hypot(
                info["center_cm"][0] - self.locked["center_cm"][0],
                info["center_cm"][1] - self.locked["center_cm"][1],
            )
            dyaw = abs(angle_diff_deg(info["axis_yaw_deg"], self.locked["axis_yaw_deg"]))
            dpx = math.hypot(
                info["center_px"][0] - self.locked["center_px"][0],
                info["center_px"][1] - self.locked["center_px"][1],
            )
            lock_ok = (
                dc <= self.args.slot_select_lock_max_center_shift_cm and
                dyaw <= self.args.slot_select_lock_max_yaw_shift_deg and
                dpx <= self.args.slot_select_lock_max_center_shift_px
            )
            score += dc * self.args.slot_select_lock_center_weight
            score += dyaw * self.args.slot_select_lock_yaw_weight
            lock = {
                "center_shift_cm": round(dc, 3),
                "center_shift_px": round(dpx, 2),
                "axis_yaw_shift_deg": round(dyaw, 3),
                "lock_ok": lock_ok,
            }
        summary["score"] = round(score, 3)
        summary["lock"] = lock
        return score, summary

    def select(self, raw):
        infos = slot_infos_from_udp(raw)
        if not infos:
            return None, {
                "schema": "slot_target_selection.v1",
                "status": "no_candidates",
                "candidate_count": 0,
                "locked": self.locked is not None,
                "rejected_switches": self.rejected_switches,
            }
        scored = []
        for info in infos:
            score, summary = self._score(info)
            scored.append((score, info, summary))
        scored.sort(key=lambda row: row[0])
        selected_score, selected, selected_summary = scored[0]
        rejected = False
        reason = "best_score"
        if self.locked is not None:
            lock = selected_summary.get("lock") or {}
            if not lock.get("lock_ok", False):
                rejected = True
                self.rejected_switches += 1
                reason = "locked_target_mismatch_wait"
        report = {
            "schema": "slot_target_selection.v1",
            "status": "rejected_switch" if rejected else "selected",
            "reason": reason,
            "candidate_count": len(scored),
            "selected_index": selected.get("raw_detection_index"),
            "selected_id": selected.get("raw_detection_id"),
            "locked": self.locked is not None,
            "rejected_switches": self.rejected_switches,
            "candidates": [row[2] for row in scored],
        }
        if rejected:
            return None, report
        selected = dict(selected)
        selected["target_selection"] = report
        self.locked = selected
        self.selected_count += 1
        return selected, report


def template_command(p, args):
    """Small-step action template. Keeps decisions easy to audit before real motion."""
    if p["aligned"]:
        return {
            "state": "DONE",
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "aligned_within_tolerance",
        }

    lat = p["lat"]
    head = p["head"]
    if abs(lat) >= args.lat_template_threshold_cm:
        # Real-car probe on 2026-06-11 showed negative lateral error improves
        # with servo > center, so lateral correction is opposite the raw lat sign.
        steer_dir = -1.0 if lat > 0 else 1.0
        servo = int(round(clamp(SERVO_CENTER + steer_dir * args.template_steer_deg * STEERING_SIGN,
                                SERVO_MIN, SERVO_MAX)))
        step = min(args.template_step_cm, max(1.0, p["lon"])) if p["lon"] > 0 else args.template_step_cm
        return {
            "state": "ALIGN_LATERAL",
            "action": "ARC",
            "cmd": "ARC D=%.1f STE=%d V=%d" % (-(round(step, 1) + DEADBAND_CM), servo, GEAR),
            "step": step,
            "servo": servo,
            "steer_dir": steer_dir,
            "reason": "lateral_error_template",
        }

    if abs(head) >= args.head_template_threshold_deg:
        steer_dir = 1.0 if head > 0 else -1.0
        servo = int(round(clamp(SERVO_CENTER + steer_dir * args.template_steer_deg * 0.5 * STEERING_SIGN,
                                SERVO_MIN, SERVO_MAX)))
        step = min(args.template_step_cm, max(1.0, p["lon"])) if p["lon"] > 0 else args.template_step_cm
        return {
            "state": "STRAIGHTEN",
            "action": "ARC",
            "cmd": "ARC D=%.1f STE=%d V=%d" % (-(round(step, 1) + DEADBAND_CM), servo, GEAR),
            "step": step,
            "servo": servo,
            "steer_dir": steer_dir,
            "reason": "heading_error_template",
        }

    step = min(args.template_step_cm, max(1.0, p["lon"])) if p["lon"] > 0 else args.template_step_cm
    return {
        "state": "FINAL_REVERSE",
        "action": "MOVE",
        "cmd": "MOVE D=%.1f V=%d" % (-(round(step, 1) + DEADBAND_CM), GEAR),
        "step": step,
        "servo": SERVO_CENTER,
        "steer_dir": 0.0,
        "reason": "centered_reverse_template",
    }


def _shortest_angle_deg(a, b):
    return angle_diff_deg(a, b)


def pixel_binding_control(x_err, angle_err, center_y, bbox_h, args):
    """Map pixel features directly to command distance, servo, and gear.

    The binding is intentionally simple and auditable:
      steer_offset = kx*x_err + ka*angle_err
      distance is long only when steering demand is mild and the slot is not near.
      gear defaults to 1 until higher gears are separately validated.
    """
    raw_offset = args.pixel_kx * x_err + args.pixel_ka * angle_err
    steer_offset = clamp(raw_offset, -args.pixel_max_steer_offset_deg, args.pixel_max_steer_offset_deg)
    servo = int(round(clamp(SERVO_CENTER + steer_offset * STEERING_SIGN, SERVO_MIN, SERVO_MAX)))
    abs_offset = abs(servo - SERVO_CENTER)

    closeness_y = center_y / max(1.0, args.pixel_stop_center_y)
    closeness_h = bbox_h / max(1.0, args.pixel_stop_bbox_h)
    closeness = max(closeness_y, closeness_h)
    abs_x = abs(x_err)

    if closeness >= args.pixel_near_ratio or abs_x <= args.pixel_x_tolerance_px:
        distance = args.pixel_near_d_cm
        distance_reason = "near_or_centered"
    elif abs_offset >= args.pixel_large_steer_offset_deg or abs_x >= args.pixel_large_x_err_px:
        distance = args.pixel_mid_d_cm
        distance_reason = "large_steer_or_x_error"
    else:
        distance = args.pixel_far_d_cm
        distance_reason = "far_mild_steer"
    distance = round(clamp(distance, args.pixel_min_command_abs_d_cm, args.pixel_max_command_abs_d_cm), 1)

    if args.pixel_max_gear <= 1:
        gear = 1
        gear_reason = "gear_limited"
    elif distance >= args.pixel_far_d_cm and abs_offset <= args.pixel_fast_max_steer_offset_deg:
        gear = min(args.pixel_max_gear, args.pixel_fast_gear)
        gear_reason = "fast_far_straight"
    else:
        gear = 1
        gear_reason = "slow_turn_or_near"

    return {
        "x_err": round(x_err, 3),
        "angle_err": round(angle_err, 3),
        "center_y": round(center_y, 3),
        "bbox_h": round(bbox_h, 3),
        "raw_steer_offset": round(raw_offset, 3),
        "steer_offset": round(servo - SERVO_CENTER, 3),
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "distance_cm": distance,
        "distance_reason": distance_reason,
        "closeness": round(closeness, 3),
        "gear": gear,
        "gear_reason": gear_reason,
    }


def pixel_servo_command(info, args):
    """Image-space visual servo using only YOLO pixel geometry."""
    cx, cy = info["center_px"]
    ex, ey = info["entrance_mid_px"]
    x_err = cx - args.pixel_target_x
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    bbox_h = info["bbox_h_px"]
    bbox_area = info["bbox_area_px"]
    near = cy >= args.pixel_stop_center_y and bbox_h >= args.pixel_stop_bbox_h
    aligned_x = abs(x_err) <= args.pixel_x_tolerance_px
    aligned_angle = abs(angle_err) <= args.pixel_angle_tolerance_deg
    binding = pixel_binding_control(x_err, angle_err, cy, bbox_h, args)
    step = binding["distance_cm"]
    cmd_d = -step

    if near and aligned_x and aligned_angle:
        return {
            "state": "PIXEL_DONE",
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "pixel_aligned_near",
            "pixel": {
                "cx": round(cx, 2),
                "cy": round(cy, 2),
                "entrance_y": round(ey, 2),
                "x_err": round(x_err, 2),
                "angle_deg": round(info["axis_angle_px_deg"], 2),
                "angle_err": round(angle_err, 2),
                "bbox_h": round(bbox_h, 2),
                "bbox_area": round(bbox_area, 2),
            },
        }

    if not aligned_x:
        state = "PIXEL_ALIGN_X"
        reason = "pixel_x_error"
    elif not aligned_angle:
        state = "PIXEL_ALIGN_ANGLE"
        reason = "pixel_angle_error"
    else:
        state = "PIXEL_REVERSE"
        reason = "pixel_centered_reverse"

    servo = binding["servo"]
    gear = binding["gear"]
    if abs(servo - SERVO_CENTER) <= STEER_DEADZONE_DEG:
        cmd = "MOVE D=%.1f V=%d" % (cmd_d, gear)
        servo = SERVO_CENTER
    else:
        cmd = "ARC D=%.1f STE=%d V=%d" % (cmd_d, servo, gear)

    return {
        "state": state,
        "action": "MOVE" if servo == SERVO_CENTER else "ARC",
        "cmd": cmd,
        "step": step,
        "servo": servo,
        "steer_dir": binding["steer_dir"],
        "binding": binding,
        "reason": reason,
        "pixel": {
            "cx": round(cx, 2),
            "cy": round(cy, 2),
            "entrance_y": round(ey, 2),
            "x_err": round(x_err, 2),
            "angle_deg": round(info["axis_angle_px_deg"], 2),
            "angle_err": round(angle_err, 2),
            "bbox_h": round(bbox_h, 2),
            "bbox_area": round(bbox_area, 2),
        },
    }


def corridor_metrics(info, args):
    sample_y = clamp(args.corridor_sample_y, 0.0, IMAGE_H - 1.0)
    entry_y = clamp(args.corridor_entry_y, 0.0, IMAGE_H - 1.0)
    left_edge = info["left_edge_px"]
    right_edge = info["right_edge_px"]
    sample = _corridor_sample_from_edges(left_edge, right_edge, sample_y)
    entry = _corridor_sample_from_edges(left_edge, right_edge, entry_y)
    target_x = args.pixel_target_x
    left_margin = target_x - sample["left_x"]
    right_margin = sample["right_x"] - target_x
    corridor_x_err = sample["center_x"] - target_x
    entry_x_err = entry["center_x"] - target_x
    cy = info["center_px"][1]
    bbox_h = info["bbox_h_px"]
    closeness_y = cy / max(1.0, args.pixel_stop_center_y)
    closeness_h = bbox_h / max(1.0, args.pixel_stop_bbox_h)
    closeness = max(closeness_y, closeness_h)
    min_margin = min(left_margin, right_margin)
    slot_width_px = max(1.0, float(sample["width_px"]))
    slot_height_px = max(1.0, float(info["bbox_h_px"]))
    corridor_x_err_norm = corridor_x_err / slot_width_px
    entry_x_err_norm = entry_x_err / slot_width_px
    left_margin_norm = left_margin / slot_width_px
    right_margin_norm = right_margin / slot_width_px
    min_margin_norm = min_margin / slot_width_px
    center_y_norm = cy / max(1.0, IMAGE_H)
    bbox_h_norm = bbox_h / max(1.0, IMAGE_H)
    risk_side = ""
    line_risk = False
    if closeness >= args.corridor_line_risk_min_closeness:
        if left_margin < args.corridor_min_line_margin_px:
            line_risk = True
            risk_side = "LEFT"
        if right_margin < args.corridor_min_line_margin_px:
            line_risk = True
            if not risk_side or right_margin < left_margin:
                risk_side = "RIGHT"
    return {
        "sample_y": round(sample_y, 2),
        "entry_y": round(entry_y, 2),
        "left_x": round(sample["left_x"], 2),
        "right_x": round(sample["right_x"], 2),
        "center_x": round(sample["center_x"], 2),
        "width_px": round(sample["width_px"], 2),
        "entry_left_x": round(entry["left_x"], 2),
        "entry_right_x": round(entry["right_x"], 2),
        "entry_center_x": round(entry["center_x"], 2),
        "corridor_x_err": round(corridor_x_err, 2),
        "entry_x_err": round(entry_x_err, 2),
        "left_margin_px": round(left_margin, 2),
        "right_margin_px": round(right_margin, 2),
        "min_margin_px": round(min_margin, 2),
        "slot_width_px": round(slot_width_px, 2),
        "slot_height_px": round(slot_height_px, 2),
        "corridor_x_err_norm": round(corridor_x_err_norm, 4),
        "entry_x_err_norm": round(entry_x_err_norm, 4),
        "left_margin_norm": round(left_margin_norm, 4),
        "right_margin_norm": round(right_margin_norm, 4),
        "min_margin_norm": round(min_margin_norm, 4),
        "center_y_norm": round(center_y_norm, 4),
        "bbox_h_norm": round(bbox_h_norm, 4),
        "closeness": round(closeness, 3),
        "line_risk": line_risk,
        "risk_side": risk_side,
    }


def slot_relative_state(info, args, stability=None):
    """Return the unified observation state used by action-template replanning.

    This state intentionally keeps image-space slot geometry as the primary
    signal. Ground-frame lon/lat/head remain useful, but they are treated as
    supporting estimates because the current homography is less trustworthy
    than direct YOLO slot-line geometry during close parking.
    """
    p = info["plan"]
    corr = corridor_metrics(info, args)
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    entrance_y = info["entrance_mid_px"][1]
    center_y = info["center_px"][1]
    bbox_h = info["bbox_h_px"]
    confidence = float(info.get("confidence", 0.0))
    stable_frames = 0
    required_stable_frames = max(1, int(args.stable_frames))
    center_span_cm = None
    axis_yaw_span_deg = None
    if stability:
        stable_frames = int(stability.get("stable_frames", 0) or 0)
        required_stable_frames = max(1, int(stability.get("required_frames", required_stable_frames) or required_stable_frames))
        center_span_cm = stability.get("center_span_cm")
        axis_yaw_span_deg = stability.get("axis_yaw_span_deg")

    near_ratio_y = center_y / max(1.0, args.pixel_stop_center_y)
    near_ratio_h = bbox_h / max(1.0, args.pixel_stop_bbox_h)
    line_margin_ok = corr["min_margin_px"] >= args.corridor_min_line_margin_px
    normalized_line_margin_ok = corr["min_margin_norm"] >= args.normalized_min_margin
    heading_ok = abs(angle_err) <= args.pixel_angle_tolerance_deg
    lateral_ok = abs(corr["corridor_x_err"]) <= args.corridor_x_tolerance_px
    normalized_lateral_ok = abs(corr["corridor_x_err_norm"]) <= args.normalized_x_tolerance
    stable_enough = stable_frames >= required_stable_frames
    pose_quality = 0.0
    pose_quality += min(1.0, max(0.0, confidence)) * 0.30
    pose_quality += (1.0 if stable_enough else min(1.0, stable_frames / max(1.0, required_stable_frames))) * 0.25
    pose_quality += min(1.0, max(0.0, corr["min_margin_px"] / max(1.0, args.corridor_min_line_margin_px * 3.0))) * 0.25
    pose_quality += max(0.0, 1.0 - min(1.0, abs(angle_err) / 45.0)) * 0.20

    if corr["line_risk"]:
        phase_hint = "line_risk_stop"
    elif corr["closeness"] < args.corridor_approach_closeness:
        phase_hint = "approach_entry"
    elif not lateral_ok or not heading_ok:
        phase_hint = "align_in_corridor"
    elif corr["closeness"] < args.corridor_final_stop_closeness:
        phase_hint = "straighten_or_enter"
    else:
        phase_hint = "final_stop_zone"

    return {
        "schema": "slot_relative_state.v1",
        "confidence": round(confidence, 4),
        "stable_frames": stable_frames,
        "required_stable_frames": required_stable_frames,
        "center_span_cm": None if center_span_cm is None else round(float(center_span_cm), 3),
        "axis_yaw_span_deg": None if axis_yaw_span_deg is None else round(float(axis_yaw_span_deg), 3),
        "pose_quality": round(pose_quality, 3),
        "phase_hint": phase_hint,
        "image": {
            "target_x_px": round(args.pixel_target_x, 2),
            "center_px": [round(info["center_px"][0], 2), round(info["center_px"][1], 2)],
            "entrance_mid_px": [round(info["entrance_mid_px"][0], 2), round(info["entrance_mid_px"][1], 2)],
            "entrance_y_px": round(entrance_y, 2),
            "center_y_px": round(center_y, 2),
            "bbox_h_px": round(bbox_h, 2),
            "bbox_w_px": round(info["bbox_w_px"], 2),
            "near_ratio_y": round(near_ratio_y, 3),
            "near_ratio_h": round(near_ratio_h, 3),
            "closeness": corr["closeness"],
            "axis_angle_px_deg": round(info["axis_angle_px_deg"], 2),
            "target_axis_angle_px_deg": round(args.pixel_target_angle_deg, 2),
            "slot_heading_err_deg": round(angle_err, 3),
        },
        "corridor": {
            "sample_y_px": corr["sample_y"],
            "entry_y_px": corr["entry_y"],
            "slot_x_err_px": corr["corridor_x_err"],
            "slot_entry_x_err_px": corr["entry_x_err"],
            "left_margin_px": corr["left_margin_px"],
            "right_margin_px": corr["right_margin_px"],
            "min_margin_px": corr["min_margin_px"],
            "width_px": corr["width_px"],
            "slot_width_px": corr["slot_width_px"],
            "slot_height_px": corr["slot_height_px"],
            "slot_x_err_norm": corr["corridor_x_err_norm"],
            "slot_entry_x_err_norm": corr["entry_x_err_norm"],
            "left_margin_norm": corr["left_margin_norm"],
            "right_margin_norm": corr["right_margin_norm"],
            "min_margin_norm": corr["min_margin_norm"],
            "center_y_norm": corr["center_y_norm"],
            "bbox_h_norm": corr["bbox_h_norm"],
            "line_risk": corr["line_risk"],
            "risk_side": corr["risk_side"],
        },
        "ground_estimate": {
            "slot_y_dist_cm": round(p["lon"], 3),
            "slot_lateral_cm": round(p["lat"], 3),
            "slot_axis_heading_deg": round(p["head"], 3),
            "slot_center_cm": [round(info["center_cm"][0], 3), round(info["center_cm"][1], 3)],
            "rear_target_cm": [round(p["target"][0], 3), round(p["target"][1], 3)],
            "aligned_by_ground_gate": bool(p["aligned"]),
        },
        "gates": {
            "stable_enough": stable_enough,
            "line_margin_ok": line_margin_ok,
            "normalized_line_margin_ok": normalized_line_margin_ok,
            "heading_ok": heading_ok,
            "lateral_ok": lateral_ok,
            "normalized_lateral_ok": normalized_lateral_ok,
        },
    }


def corridor_servo_command(info, args):
    """Parking-slot corridor controller using side lines instead of bbox center."""
    metrics = corridor_metrics(info, args)
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    abs_x = abs(metrics["corridor_x_err"])
    abs_angle = abs(angle_err)
    closeness = metrics["closeness"]

    if metrics["line_risk"]:
        return {
            "state": "LINE_RISK_%s" % (metrics["risk_side"] or "UNKNOWN"),
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "corridor_line_margin_too_small",
            "corridor": metrics,
            "binding": {
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "distance_reason": "line_risk_stop",
                "gear": 1,
                "gear_reason": "stopped",
            },
        }

    aligned_x = abs_x <= args.corridor_x_tolerance_px
    aligned_angle = abs_angle <= args.pixel_angle_tolerance_deg
    if closeness >= args.corridor_final_stop_closeness and aligned_x and aligned_angle:
        return {
            "state": "FINAL_STOP",
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "corridor_aligned_near",
            "corridor": metrics,
            "binding": {
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "distance_reason": "aligned_final_stop",
                "gear": 1,
                "gear_reason": "stopped",
            },
        }

    if closeness < args.corridor_approach_closeness:
        state = "APPROACH"
        step = args.corridor_approach_d_cm
        kx = args.corridor_kx
        max_offset = args.corridor_approach_max_steer_offset_deg
        distance_reason = "far_corridor_approach"
    elif not aligned_x or not aligned_angle:
        state = "ALIGN_CORRIDOR"
        step = args.corridor_align_d_cm
        kx = args.corridor_near_kx
        max_offset = args.corridor_align_max_steer_offset_deg
        distance_reason = "near_corridor_alignment"
    else:
        state = "ENTER_SLOT"
        step = args.corridor_enter_d_cm
        kx = args.corridor_near_kx
        max_offset = args.corridor_enter_max_steer_offset_deg
        distance_reason = "aligned_slot_entry"

    raw_offset = args.corridor_steer_sign * (kx * metrics["corridor_x_err"] + args.corridor_ka * angle_err)
    steer_offset = clamp(raw_offset, -max_offset, max_offset)
    servo = int(round(clamp(SERVO_CENTER + steer_offset * STEERING_SIGN, SERVO_MIN, SERVO_MAX)))
    step = round(clamp(step, args.corridor_min_command_abs_d_cm, args.pixel_max_command_abs_d_cm), 1)
    gear = 1

    if abs(servo - SERVO_CENTER) <= STEER_DEADZONE_DEG:
        cmd = "MOVE D=%.1f V=%d" % (-step, gear)
        servo = SERVO_CENTER
        action = "MOVE"
    else:
        cmd = "ARC D=%.1f STE=%d V=%d" % (-step, servo, gear)
        action = "ARC"

    return {
        "state": state,
        "action": action,
        "cmd": cmd,
        "step": step,
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "reason": "corridor_x_error" if not aligned_x else ("corridor_angle_error" if not aligned_angle else "corridor_enter"),
        "corridor": metrics,
        "binding": {
            "corridor_x_err": metrics["corridor_x_err"],
            "angle_err": round(angle_err, 3),
            "raw_steer_offset": round(raw_offset, 3),
            "corridor_steer_sign": args.corridor_steer_sign,
            "steer_offset": round(servo - SERVO_CENTER, 3),
            "servo": servo,
            "distance_cm": step,
            "distance_reason": distance_reason,
            "closeness": closeness,
            "gear": gear,
            "gear_reason": "corridor_safety_limited",
            "max_steer_offset": max_offset,
        },
    }


def normalized_corridor_servo_command(info, args):
    """Scale-invariant corridor controller.

    The controller uses the detected slot width as its unit. This avoids tuning
    directly against one physical parking-box size: large and small valid slots
    produce comparable normalized x error and line-margin values.
    """
    metrics = corridor_metrics(info, args)
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    x_norm = metrics["corridor_x_err_norm"]
    entry_norm = metrics["entry_x_err_norm"]
    margin_norm = metrics["min_margin_norm"]
    abs_x_norm = abs(x_norm)
    abs_angle = abs(angle_err)
    closeness = metrics["closeness"]

    norm_line_risk = margin_norm < args.normalized_min_margin
    if norm_line_risk:
        risk_side = "LEFT" if metrics["left_margin_norm"] < metrics["right_margin_norm"] else "RIGHT"
        return {
            "state": "NORM_LINE_RISK_%s" % risk_side,
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "normalized_line_margin_too_small",
            "corridor": metrics,
            "binding": {
                "corridor_x_err_norm": round(x_norm, 4),
                "entry_x_err_norm": round(entry_norm, 4),
                "min_margin_norm": round(margin_norm, 4),
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "distance_reason": "normalized_line_risk_stop",
                "gear": 1,
                "gear_reason": "stopped",
            },
        }

    aligned_x = abs_x_norm <= args.normalized_x_tolerance
    aligned_angle = abs_angle <= args.pixel_angle_tolerance_deg
    if closeness >= args.normalized_final_stop_closeness and aligned_x and aligned_angle:
        return {
            "state": "NORM_FINAL_STOP",
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "normalized_corridor_aligned_near",
            "corridor": metrics,
            "binding": {
                "corridor_x_err_norm": round(x_norm, 4),
                "entry_x_err_norm": round(entry_norm, 4),
                "min_margin_norm": round(margin_norm, 4),
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "distance_reason": "aligned_final_stop",
                "gear": 1,
                "gear_reason": "stopped",
            },
        }

    if closeness < args.normalized_approach_closeness:
        state = "NORM_APPROACH"
        step = args.normalized_approach_d_cm
        max_offset = args.normalized_approach_max_steer_offset_deg
        distance_reason = "normalized_far_approach"
    elif not aligned_x or not aligned_angle:
        state = "NORM_ALIGN_CORRIDOR"
        step = args.normalized_align_d_cm
        max_offset = args.normalized_align_max_steer_offset_deg
        distance_reason = "normalized_alignment"
    else:
        state = "NORM_ENTER_SLOT"
        step = args.normalized_enter_d_cm
        max_offset = args.normalized_enter_max_steer_offset_deg
        distance_reason = "normalized_enter"

    raw_offset = args.normalized_steer_sign * (
        args.normalized_kx * x_norm +
        args.normalized_entry_kx * entry_norm +
        args.normalized_ka * angle_err
    )
    if aligned_x and aligned_angle:
        raw_offset = 0.0
    steer_offset = clamp(raw_offset, -max_offset, max_offset)
    if (not aligned_x or not aligned_angle) and abs(steer_offset) > STEER_DEADZONE_DEG:
        min_offset = min(abs(max_offset), max(0.0, args.normalized_min_steer_offset_deg))
        if min_offset > 0.0 and abs(steer_offset) < min_offset:
            steer_offset = min_offset if steer_offset > 0 else -min_offset
    servo = int(round(clamp(SERVO_CENTER + steer_offset * STEERING_SIGN, SERVO_MIN, SERVO_MAX)))
    step = round(clamp(step, args.normalized_min_command_abs_d_cm, args.pixel_max_command_abs_d_cm), 1)
    gear = 1

    if abs(servo - SERVO_CENTER) <= STEER_DEADZONE_DEG:
        cmd = "MOVE D=%.1f V=%d" % (-step, gear)
        servo = SERVO_CENTER
        action = "MOVE"
    else:
        cmd = "ARC D=%.1f STE=%d V=%d" % (-step, servo, gear)
        action = "ARC"

    return {
        "state": state,
        "action": action,
        "cmd": cmd,
        "step": step,
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "reason": "normalized_x_error" if not aligned_x else ("normalized_angle_error" if not aligned_angle else "normalized_enter"),
        "corridor": metrics,
        "binding": {
            "corridor_x_err_norm": round(x_norm, 4),
            "entry_x_err_norm": round(entry_norm, 4),
            "min_margin_norm": round(margin_norm, 4),
            "angle_err": round(angle_err, 3),
            "raw_steer_offset": round(raw_offset, 3),
            "min_steer_offset": round(args.normalized_min_steer_offset_deg, 3),
            "normalized_steer_sign": args.normalized_steer_sign,
            "steer_offset": round(servo - SERVO_CENTER, 3),
            "servo": servo,
            "distance_cm": step,
            "distance_reason": distance_reason,
            "closeness": closeness,
            "gear": gear,
            "gear_reason": "normalized_corridor_safety_limited",
            "max_steer_offset": max_offset,
            "slot_width_px": metrics["slot_width_px"],
        },
    }


def _path_cmd(kind, step_cm, servo=None, gear=GEAR):
    step_cm = round(float(step_cm), 1)
    if kind == "MOVE" or servo is None or abs(float(servo) - SERVO_CENTER) <= STEER_DEADZONE_DEG:
        return {
            "action": "MOVE",
            "cmd": "MOVE D=%.1f V=%d" % (-step_cm, gear),
            "step": step_cm,
            "servo": SERVO_CENTER,
        }
    servo = int(round(clamp(float(servo), SERVO_MIN, SERVO_MAX)))
    return {
        "action": "ARC",
        "cmd": "ARC D=%.1f STE=%d V=%d" % (-step_cm, servo, gear),
        "step": step_cm,
        "servo": servo,
    }


def _path_build_template(template_id, commands, reason):
    total_cm = round(sum(c["step"] for c in commands), 1)
    return {
        "template_id": template_id,
        "reason": reason,
        "total_cm": total_cm,
        "commands": commands,
    }


def _path_score_template(template, desired_offset, abs_offset, closeness, metrics, args):
    commands = template["commands"]
    steer_offsets = [c["servo"] - SERVO_CENTER for c in commands if c["action"] == "ARC"]
    arc_count = len(steer_offsets)
    same_side_count = sum(1 for off in steer_offsets if off * desired_offset > 0.0)
    wrong_side_count = sum(1 for off in steer_offsets if off * desired_offset < 0.0)
    straight_only = arc_count == 0

    if abs_offset <= args.path_straight_offset_deg:
        side_cost = 0.0 if straight_only else 8.0 + 0.4 * arc_count
    elif straight_only:
        side_cost = abs_offset * args.path_straight_cost_gain
    elif same_side_count > 0 and wrong_side_count == 0:
        side_cost = max(0.0, abs_offset - max(abs(o) for o in steer_offsets)) * 0.15
    else:
        side_cost = args.path_wrong_side_penalty + abs_offset

    if abs_offset >= args.path_hard_offset_threshold_deg:
        target_arcs = 3
    elif abs_offset >= args.path_mid_offset_threshold_deg:
        target_arcs = 2
    elif abs_offset >= args.path_straight_offset_deg:
        target_arcs = 1
    else:
        target_arcs = 0
    arc_count_cost = abs(arc_count - target_arcs) * args.path_arc_count_penalty

    total_cm = template["total_cm"]
    target_total = args.path_near_total_cm if closeness >= args.path_near_closeness else args.path_far_total_cm
    distance_cost = abs(total_cm - target_total) * args.path_distance_cost_gain

    margin_norm = float(metrics["min_margin_norm"])
    margin_cost = 0.0
    if margin_norm < args.path_prefer_margin_norm:
        margin_cost = (args.path_prefer_margin_norm - margin_norm) * args.path_margin_cost_gain

    score = side_cost + arc_count_cost + distance_cost + margin_cost
    return {
        "cost": round(score, 3),
        "components": {
            "side_cost": round(side_cost, 3),
            "arc_count_cost": round(arc_count_cost, 3),
            "distance_cost": round(distance_cost, 3),
            "margin_cost": round(margin_cost, 3),
        },
        "arc_count": arc_count,
        "same_side_count": same_side_count,
        "wrong_side_count": wrong_side_count,
        "total_cm": total_cm,
    }


def path_template_planner_command(info, args):
    """Choose a short full-path template from one stable slot observation.

    This is deliberately more discrete than normalized_corridor_servo: it ranks
    whole command sequences, logs the selected path, then returns only the first
    command so the outer loop can re-observe and replan after every movement.
    """
    metrics = corridor_metrics(info, args)
    angle_err = _shortest_angle_deg(info["axis_angle_px_deg"], args.pixel_target_angle_deg)
    x_norm = float(metrics["corridor_x_err_norm"])
    entry_norm = float(metrics["entry_x_err_norm"])
    margin_norm = float(metrics["min_margin_norm"])
    closeness = float(metrics["closeness"])

    if margin_norm < args.path_template_min_margin_norm:
        return {
            "state": "PATH_LINE_RISK_%s" % (metrics["risk_side"] or "UNKNOWN"),
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "path_template_margin_too_small",
            "corridor": metrics,
            "binding": {
                "slot_x_err_norm": round(x_norm, 4),
                "slot_entry_x_err_norm": round(entry_norm, 4),
                "min_margin_norm": round(margin_norm, 4),
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "gear": GEAR,
                "gear_reason": "stopped",
            },
            "path_plan": {
                "selected_template_id": "STOP",
                "reason": "path_template_margin_too_small",
                "candidates": [],
                "commands": [],
            },
        }

    if (closeness >= args.path_final_stop_closeness and
        abs(x_norm) <= args.path_x_deadband_norm and
        abs(angle_err) <= args.path_heading_deadband_deg):
        return {
            "state": "PATH_FINAL_STOP",
            "action": "STOP",
            "cmd": "STOP",
            "step": 0.0,
            "servo": SERVO_CENTER,
            "reason": "path_template_aligned_near",
            "corridor": metrics,
            "binding": {
                "slot_x_err_norm": round(x_norm, 4),
                "slot_entry_x_err_norm": round(entry_norm, 4),
                "min_margin_norm": round(margin_norm, 4),
                "angle_err": round(angle_err, 3),
                "distance_cm": 0.0,
                "gear": GEAR,
                "gear_reason": "stopped",
            },
            "path_plan": {
                "selected_template_id": "STOP",
                "reason": "path_template_aligned_near",
                "candidates": [],
                "commands": [],
            },
        }

    step = round(clamp(args.path_step_cm, args.normalized_min_command_abs_d_cm,
                       args.pixel_max_command_abs_d_cm), 1)
    max_commands = max(1, int(args.path_max_commands))
    high_servo = int(round(clamp(args.path_arc_steer_high, SERVO_MIN, SERVO_MAX)))
    low_servo = int(round(clamp(args.path_arc_steer_low, SERVO_MIN, SERVO_MAX)))
    straight = [_path_cmd("MOVE", step) for _ in range(max_commands)]
    high2 = [_path_cmd("ARC", step, high_servo) for _ in range(min(2, max_commands))]
    low2 = [_path_cmd("ARC", step, low_servo) for _ in range(min(2, max_commands))]
    high3 = [_path_cmd("ARC", step, high_servo) for _ in range(min(3, max_commands))]
    low3 = [_path_cmd("ARC", step, low_servo) for _ in range(min(3, max_commands))]
    templates = [
        _path_build_template("straight_entry", straight, "straight_reverse_path"),
        _path_build_template("steer_high_then_straight",
                             high2 + [_path_cmd("MOVE", step) for _ in range(max(0, max_commands - len(high2)))],
                             "positive_servo_arc_then_straight"),
        _path_build_template("steer_low_then_straight",
                             low2 + [_path_cmd("MOVE", step) for _ in range(max(0, max_commands - len(low2)))],
                             "negative_servo_arc_then_straight"),
        _path_build_template("steer_high_hold",
                             high3 + [_path_cmd("MOVE", step) for _ in range(max(0, max_commands - len(high3)))],
                             "positive_servo_longer_arc"),
        _path_build_template("steer_low_hold",
                             low3 + [_path_cmd("MOVE", step) for _ in range(max(0, max_commands - len(low3)))],
                             "negative_servo_longer_arc"),
    ]

    desired_offset = args.path_steer_sign * (
        args.path_kx * x_norm +
        args.path_entry_kx * entry_norm +
        args.path_ka * angle_err
    )
    desired_offset = clamp(desired_offset, -args.path_max_steer_offset_deg,
                           args.path_max_steer_offset_deg)
    if abs(x_norm) <= args.path_x_deadband_norm and abs(angle_err) <= args.path_heading_deadband_deg:
        desired_offset = 0.0
    abs_offset = abs(desired_offset)

    candidates = []
    for template in templates:
        scoring = _path_score_template(template, desired_offset, abs_offset, closeness, metrics, args)
        row = dict(template)
        row.update(scoring)
        candidates.append(row)
    candidates.sort(key=lambda r: (r["cost"], r["template_id"]))
    selected = candidates[0]
    first = selected["commands"][0]
    action = first["action"]
    servo = first["servo"]
    state = "PATH_REVERSE_STRAIGHT" if action == "MOVE" else (
        "PATH_REVERSE_STEER_HIGH" if servo > SERVO_CENTER else "PATH_REVERSE_STEER_LOW")

    return {
        "state": state,
        "action": action,
        "cmd": first["cmd"],
        "step": first["step"],
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "reason": "path_template_best_sequence",
        "corridor": metrics,
        "binding": {
            "slot_x_err_norm": round(x_norm, 4),
            "slot_entry_x_err_norm": round(entry_norm, 4),
            "min_margin_norm": round(margin_norm, 4),
            "angle_err": round(angle_err, 3),
            "desired_steer_offset": round(desired_offset, 3),
            "path_steer_sign": args.path_steer_sign,
            "distance_cm": first["step"],
            "distance_reason": selected["template_id"],
            "closeness": closeness,
            "gear": GEAR,
            "gear_reason": "path_template_safety_limited",
            "selected_cost": selected["cost"],
        },
        "path_plan": {
            "schema": "parking_path_template_plan.v1",
            "selected_template_id": selected["template_id"],
            "selected_cost": selected["cost"],
            "desired_steer_offset": round(desired_offset, 3),
            "commands": selected["commands"],
            "candidates": candidates,
        },
    }


def _parse_motion_command_for_action(cmd):
    parts = cmd.split()
    kind = parts[0] if parts else ""
    kv = {}
    for part in parts[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            kv[k.upper()] = v
    d = float(kv.get("D", "0"))
    servo = int(float(kv.get("STE", str(SERVO_CENTER)))) if kind == "ARC" else int(SERVO_CENTER)
    return kind, abs(d), servo


def primitive_probe_command(_info, args):
    """Execute one operator-specified safe primitive after target stability."""
    cmd = parse_policy_actions(args.primitive_command, args.primitive_max_command_abs_d_cm)[0]
    kind, step, servo = _parse_motion_command_for_action(cmd)
    corridor = corridor_metrics(_info, args) if _info is not None else None
    return {
        "state": "PRIMITIVE_PROBE",
        "action": kind,
        "cmd": cmd,
        "step": round(step, 1),
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "reason": "fixed_calibration_primitive",
        "corridor": corridor,
        "binding": {
            "distance_cm": round(step, 1),
            "distance_reason": "fixed_calibration_primitive",
            "servo": servo,
            "gear": GEAR,
            "gear_reason": "primitive_probe_safety_limited",
        },
    }


def execute_logged_motion(action, args, pose_fuser=None):
    cmd = action["cmd"]
    step = planner_to_float(action.get("step"))
    st = read_stat()
    anchor = {"yaw": st["yaw"] if st["yaw"] is not None else 0.0}
    pwm_before = query_pwm_stat() if args.log_stm32_detail else ""
    pre_servo_resp = ""
    pwm_after_pre_servo = ""
    telemetry_on_resp = ""
    telemetry_off_resp = ""
    if (args.pre_steer_settle_sec > 0.0 and
        action.get("action") == "ARC" and
        action.get("servo") is not None):
        pre_servo_resp = send_cmd("SERVO A=%d" % int(round(action["servo"])), read_sec=2.0).strip()
        time.sleep(args.pre_steer_settle_sec)
        if args.log_stm32_detail:
            pwm_after_pre_servo = query_pwm_stat()
    if args.motion_telemetry:
        telemetry_on_resp = send_cmd("TEL ON", read_sec=2.0).strip()
    motion_resp = send_cmd(cmd, read_sec=args.move_read_sec).strip()
    if args.motion_telemetry:
        telemetry_off_resp = send_cmd("TEL OFF", read_sec=2.0).strip()
    pre_servo_events = parse_stm32_events(pre_servo_resp)
    telemetry_on_events = parse_stm32_events(telemetry_on_resp)
    motion_events = parse_stm32_events(motion_resp)
    telemetry_off_events = parse_stm32_events(telemetry_off_resp)
    fusion_motion_trace = []
    fusion_motion_final = None
    if pose_fuser is not None:
        try:
            for ev in motion_events:
                if ev.get("type") == "tlm":
                    fusion_motion_final = pose_fuser.ingest_tlm(ev)
                    fusion_motion_trace.append(fusion_motion_final)
        except Exception as exc:
            fusion_motion_trace.append({"schema": "fused_pose_error.v1", "error": str(exc)})
    pwm_after = query_pwm_stat() if args.log_stm32_detail else ""
    st_after = read_stat()
    odom_progress_cm = None
    if st_after.get("d") is not None:
        odom_progress_cm = abs(float(st_after["d"]))
    if (st.get("d") is not None and st_after.get("d") is not None and
        abs(float(st_after["d"])) < abs(float(st["d"]))):
        odom_progress_cm = abs(float(st_after["d"]))
    log_event(args.log_jsonl, {
        "event": "stm32_motion_result",
        "candidate_cmd": cmd,
        "pre_steer_settle_sec": args.pre_steer_settle_sec,
        "pre_servo_response": pre_servo_resp,
        "pre_servo_events": pre_servo_events,
        "telemetry_on_response": telemetry_on_resp,
        "telemetry_on_events": telemetry_on_events,
        "motion_response": motion_resp,
        "motion_events": motion_events,
        "telemetry_off_response": telemetry_off_resp,
        "telemetry_off_events": telemetry_off_events,
        "fusion_motion_trace": fusion_motion_trace,
        "fusion_motion_final": fusion_motion_final,
        "stat_before": st.get("raw", ""),
        "pwm_before": pwm_before,
        "pwm_after_pre_servo": pwm_after_pre_servo,
        "pwm_after": pwm_after,
        "stat_after": st_after.get("raw", ""),
        "commanded_step_cm": round(step, 3),
        "odom_progress_cm": None if odom_progress_cm is None else round(odom_progress_cm, 3),
        "odom_d_before_cm": st.get("d"),
        "odom_d_after_cm": st_after.get("d"),
        "final_blind_token": None,
        "counter_steer_result": None,
    })
    return st, st_after, motion_events


# ============================ planner core ============================

PLANNER_STATE_KEYS = [
    "slot_x_err_px",
    "slot_entry_x_err_px",
    "slot_heading_err_deg",
    "left_margin_px",
    "right_margin_px",
    "min_margin_px",
    "closeness",
    "slot_y_dist_cm",
    "slot_lateral_cm",
    "pose_quality",
]


def planner_to_float(value, default=0.0):
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        if isinstance(value, str) and value.lower() in ("true", "false"):
            return 1.0 if value.lower() == "true" else 0.0
        return float(value)
    except (TypeError, ValueError):
        return default


def planner_to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


def planner_load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def planner_load_response_model(path):
    try:
        data = planner_load_json(path)
    except OSError:
        return {"schema": "none", "records": []}
    records = [r for r in data.get("records", []) if r.get("action_id")]
    return {"schema": data.get("schema", "unknown"), "records": records}


def planner_flatten_slot_state(slot_state, stable=False, source=None):
    image = slot_state.get("image") or {}
    corridor = slot_state.get("corridor") or {}
    ground = slot_state.get("ground_estimate") or {}
    gates = slot_state.get("gates") or {}
    out = {
        "stable": bool(stable),
        "stable_enough": bool(gates.get("stable_enough")),
        "line_margin_ok": bool(gates.get("line_margin_ok")),
        "heading_ok": bool(gates.get("heading_ok")),
        "lateral_ok": bool(gates.get("lateral_ok")),
        "line_risk": bool(corridor.get("line_risk")),
        "phase_hint": slot_state.get("phase_hint") or "unknown",
        "confidence": planner_to_float(slot_state.get("confidence")),
        "pose_quality": planner_to_float(slot_state.get("pose_quality")),
        "slot_x_err_px": planner_to_float(corridor.get("slot_x_err_px")),
        "slot_entry_x_err_px": planner_to_float(corridor.get("slot_entry_x_err_px")),
        "slot_heading_err_deg": planner_to_float(image.get("slot_heading_err_deg")),
        "left_margin_px": planner_to_float(corridor.get("left_margin_px")),
        "right_margin_px": planner_to_float(corridor.get("right_margin_px")),
        "min_margin_px": planner_to_float(corridor.get("min_margin_px")),
        "closeness": planner_to_float(image.get("closeness")),
        "slot_y_dist_cm": planner_to_float(ground.get("slot_y_dist_cm")),
        "slot_lateral_cm": planner_to_float(ground.get("slot_lateral_cm")),
        "stable_frames": int(planner_to_float(slot_state.get("stable_frames"), 0)),
        "required_stable_frames": int(planner_to_float(slot_state.get("required_stable_frames"), 0)),
    }
    if source:
        out.update(source)
    return out


def planner_state_x_sign(value):
    if value > 3.0:
        return "+"
    if value < -3.0:
        return "-"
    return "0"


def planner_state_x_bin(value):
    v = abs(value)
    if v < 40.0:
        return "0-40"
    if v < 120.0:
        return "40-120"
    return "120+"


def planner_state_heading_bin(value):
    if value < -8.0:
        return "-8-"
    if value < 0.0:
        return "-8-0"
    if value <= 8.0:
        return "0-8"
    return "8+"


def planner_bucket_for_state(state):
    return {
        "phase": state.get("phase_hint", "unknown"),
        "x_err_sign": planner_state_x_sign(planner_to_float(state.get("slot_x_err_px"))),
        "x_err_bin": planner_state_x_bin(planner_to_float(state.get("slot_x_err_px"))),
        "heading_bin": planner_state_heading_bin(planner_to_float(state.get("slot_heading_err_deg"))),
    }


def planner_response_delta(record):
    return record.get("mean_delta") or record.get("delta") or {}


def planner_select_response_record(action_id, state, responses):
    records = [r for r in responses.get("records", []) if r.get("action_id") == action_id]
    if not records:
        return None, "prior"
    if responses.get("schema") != "parking_action_response_model.v2":
        return records[0], "measured"
    bucket = planner_bucket_for_state(state)
    exact = [r for r in records if r.get("bucket") == bucket]
    if exact:
        exact.sort(key=lambda r: planner_to_float(r.get("confidence")), reverse=True)
        return exact[0], "measured"
    same_sign = [
        r for r in records
        if (r.get("bucket") or {}).get("phase") == bucket.get("phase")
        and (r.get("bucket") or {}).get("x_err_sign") == bucket.get("x_err_sign")
    ]
    if same_sign:
        same_sign.sort(key=lambda r: planner_to_float(r.get("confidence")), reverse=True)
        neighbor = dict(same_sign[0])
        neighbor["confidence"] = round(planner_to_float(neighbor.get("confidence")) * 0.5, 3)
        neighbor["bucket_match"] = "same_phase_sign_neighbor"
        return neighbor, "measured_neighbor"
    records.sort(key=lambda r: planner_to_float(r.get("confidence")), reverse=True)
    fallback = dict(records[0])
    fallback["confidence"] = round(planner_to_float(fallback.get("confidence")) * 0.25, 3)
    fallback["bucket_match"] = "action_only_neighbor"
    return fallback, "measured_neighbor"


def planner_predicted_state(state, action, measured):
    source = measured or action
    delta = planner_response_delta(source) if measured else action.get("prior_delta")
    confidence = planner_to_float(source.get("confidence") if measured else action.get("prior_confidence"), 0.0)
    origin = measured.get("_origin", "measured") if measured else "prior"
    pred = dict(state)
    delta = delta or {}
    for key in [
        "slot_y_dist_cm",
        "slot_x_err_px",
        "slot_lateral_cm",
        "slot_heading_err_deg",
        "min_margin_px",
    ]:
        pred[key] = planner_to_float(pred.get(key)) + planner_to_float(delta.get(key), 0.0)
    pred["line_risk"] = pred["min_margin_px"] < 34.0
    return pred, confidence, origin


def planner_cost_state(pred, action, current, library, confidence, origin):
    scoring = library.get("scoring", {})
    target = scoring.get("target", {})
    weights = scoring.get("weights", {})

    slot_x_abs = abs(planner_to_float(pred.get("slot_x_err_px")) - planner_to_float(target.get("slot_x_err_px")))
    heading_abs = abs(planner_to_float(pred.get("slot_heading_err_deg")) - planner_to_float(target.get("slot_heading_err_deg")))
    lateral_abs = abs(planner_to_float(pred.get("slot_lateral_cm")) - planner_to_float(target.get("slot_lateral_cm")))
    min_margin = planner_to_float(pred.get("min_margin_px"))
    margin_shortfall = max(0.0, planner_to_float(target.get("min_margin_px"), 90.0) - min_margin)
    progress = max(0.0, planner_to_float(current.get("slot_y_dist_cm")) - planner_to_float(pred.get("slot_y_dist_cm")))
    phase_mismatch = 0.0 if current.get("phase_hint") in action.get("allowed_phases", []) else 1.0
    line_risk = 1.0 if pred.get("line_risk") else 0.0
    low_confidence = max(0.0, 1.0 - confidence)
    uncalibrated = 0.0 if origin.startswith("measured") else 1.0
    large_steer = abs(planner_to_float(action.get("servo"), SERVO_CENTER) - SERVO_CENTER) / 45.0

    parts = {
        "slot_x_err_abs": slot_x_abs * planner_to_float(weights.get("slot_x_err_abs"), 1.0),
        "slot_heading_err_abs": heading_abs * planner_to_float(weights.get("slot_heading_err_abs"), 4.0),
        "slot_lateral_abs": lateral_abs * planner_to_float(weights.get("slot_lateral_abs"), 8.0),
        "progress_bonus": -progress * planner_to_float(weights.get("progress"), 0.35),
        "min_margin_shortfall": margin_shortfall * planner_to_float(weights.get("min_margin_shortfall"), 2.5),
        "line_risk": line_risk * planner_to_float(weights.get("line_risk"), 1000.0),
        "phase_mismatch": phase_mismatch * planner_to_float(weights.get("phase_mismatch"), 25.0),
        "low_confidence": low_confidence * planner_to_float(weights.get("low_confidence"), 20.0),
        "uncalibrated": uncalibrated * planner_to_float(weights.get("uncalibrated"), 15.0),
        "large_steer": large_steer * planner_to_float(weights.get("large_steer"), 3.0),
    }
    return round(sum(parts.values()), 3), {k: round(v, 3) for k, v in parts.items()}


def planner_hard_block_reasons(action, state, pred, origin, response_verdict, real_motion, criteria):
    reasons = []
    if state.get("phase_hint") not in action.get("allowed_phases", []):
        reasons.append("phase_mismatch")
    abort = (criteria or {}).get("abort", {})
    min_margin_floor = planner_to_float(abort.get("min_margin_px_floor"), 40.0)
    recovery = planner_edge_recovery_context(state, criteria)
    pred_margin = planner_to_float(pred.get("min_margin_px"))
    if recovery.get("active"):
        if pred.get("line_risk") or pred_margin < planner_to_float(recovery.get("predicted_min_margin_px")):
            reasons.append("predicted_line_risk")
        margin_gain = pred_margin - planner_to_float(state.get("min_margin_px"))
        if margin_gain < planner_to_float(recovery.get("min_margin_gain_px")):
            reasons.append("edge_recovery_margin_not_improved")
        if recovery.get("require_x_improve"):
            cur_x_abs = abs(planner_to_float(state.get("slot_x_err_px")))
            pred_x_abs = abs(planner_to_float(pred.get("slot_x_err_px")))
            if pred_x_abs >= cur_x_abs:
                reasons.append("edge_recovery_x_not_improved")
    elif pred.get("line_risk") or pred_margin < min_margin_floor:
        reasons.append("predicted_line_risk")
    if real_motion:
        if action.get("requires_measured") and origin != "measured":
            reasons.append("no_exact_measured_response")
        if response_verdict == "worsened":
            reasons.append("measured_worsened")
    return reasons


def planner_score_actions(state, library, responses, real_motion=False, criteria=None):
    ranked = []
    recovery = planner_edge_recovery_context(state, criteria)
    for action in library.get("actions", []):
        measured, origin = planner_select_response_record(action.get("id"), state, responses)
        if measured:
            measured = dict(measured)
            measured["_origin"] = origin
        pred, confidence, origin = planner_predicted_state(state, action, measured)
        response_verdict = measured.get("dominant_verdict") or measured.get("verdict") if measured else None
        cost, parts = planner_cost_state(pred, action, state, library, confidence, origin)
        block_reasons = planner_hard_block_reasons(
            action, state, pred, origin, response_verdict, real_motion, criteria)
        ranked.append({
            "id": action.get("id"),
            "action_id": action.get("id"),
            "command": action.get("command"),
            "origin": origin,
            "confidence": round(confidence, 3),
            "response_bucket": measured.get("bucket") if measured else None,
            "response_match": measured.get("bucket_match", "exact") if measured else "prior",
            "response_verdict": response_verdict,
            "score": cost,
            "cost": cost,
            "cost_parts": parts,
            "hard_blocked": bool(block_reasons),
            "block_reasons": block_reasons,
            "edge_recovery": recovery,
            "predicted": {
                "slot_y_dist_cm": round(pred["slot_y_dist_cm"], 3),
                "slot_x_err_px": round(pred["slot_x_err_px"], 3),
                "slot_lateral_cm": round(pred["slot_lateral_cm"], 3),
                "slot_heading_err_deg": round(pred["slot_heading_err_deg"], 3),
                "min_margin_px": round(pred["min_margin_px"], 3),
                "line_risk": bool(pred["line_risk"]),
            },
            "notes": action.get("notes", ""),
        })
    ranked.sort(key=lambda item: (1 if item.get("hard_blocked") else 0, item["score"]))
    return ranked


def planner_servo_dir_from_command(command):
    try:
        kind, _step, servo = _parse_motion_command_for_action(command)
    except (ValueError, IndexError):
        return 0.0
    if kind != "ARC" or servo == SERVO_CENTER:
        return 0.0
    return 1.0 if servo > SERVO_CENTER else -1.0


def planner_apply_switch_penalty(ranking, last_action_id, penalty):
    if not last_action_id or penalty <= 0:
        return ranking
    last_item = None
    for item in ranking:
        if item.get("action_id") == last_action_id:
            last_item = item
            break
    if not last_item:
        return ranking
    last_dir = planner_servo_dir_from_command(last_item.get("command", ""))
    out = []
    for item in ranking:
        adjusted = dict(item)
        cur_dir = planner_servo_dir_from_command(item.get("command", ""))
        switch_penalty = 0.0
        if last_dir != 0.0 and cur_dir != 0.0 and cur_dir != last_dir:
            switch_penalty = penalty
        adjusted["switch_penalty"] = round(switch_penalty, 3)
        adjusted["score"] = round(planner_to_float(adjusted.get("score")) + switch_penalty, 3)
        adjusted["cost"] = adjusted["score"]
        out.append(adjusted)
    out.sort(key=lambda item: (1 if item.get("hard_blocked") else 0, item["score"]))
    return out


def planner_relax_prior_lateral_recovery_blocks(ranking, state, args):
    if not args.replanner_allow_prior_lateral_recovery:
        return ranking
    y_dist = planner_to_float(state.get("slot_y_dist_cm"))
    cur_lat_abs = abs(planner_to_float(state.get("slot_lateral_cm")))
    if y_dist > args.replanner_prior_lateral_recovery_max_y_dist_cm:
        return ranking
    if cur_lat_abs < args.replanner_prior_lateral_recovery_min_lateral_cm:
        return ranking
    out = []
    for item in ranking:
        adjusted = dict(item)
        reasons = list(adjusted.get("block_reasons") or [])
        predicted = adjusted.get("predicted") or {}
        pred_lat_abs = abs(planner_to_float(predicted.get("slot_lateral_cm")))
        pred_margin = planner_to_float(predicted.get("min_margin_px"))
        gain = cur_lat_abs - pred_lat_abs
        eligible = (
            adjusted.get("action_id") not in ("WAIT", "STOP") and
            "no_exact_measured_response" in reasons and
            "phase_mismatch" not in reasons and
            "measured_worsened" not in reasons and
            not bool(predicted.get("line_risk")) and
            pred_margin >= args.replanner_prior_lateral_recovery_min_margin_px and
            gain >= args.replanner_prior_lateral_recovery_min_gain_cm and
            pred_lat_abs <= args.replanner_prior_lateral_recovery_max_predicted_lateral_cm
        )
        if eligible:
            reasons = [r for r in reasons if r != "no_exact_measured_response"]
            bonus = args.replanner_prior_lateral_recovery_score_bonus
            adjusted["block_reasons"] = reasons
            adjusted["hard_blocked"] = bool(reasons)
            adjusted["score"] = round(planner_to_float(adjusted.get("score")) - bonus, 3)
            adjusted["cost"] = adjusted["score"]
            parts = dict(adjusted.get("cost_parts") or {})
            parts["prior_lateral_recovery_bonus"] = round(-bonus, 3)
            adjusted["cost_parts"] = parts
            adjusted["prior_lateral_recovery"] = {
                "active": True,
                "reason": "predicted_terminal_lateral_recovery",
                "current_lateral_abs_cm": round(cur_lat_abs, 3),
                "predicted_lateral_abs_cm": round(pred_lat_abs, 3),
                "predicted_lateral_gain_cm": round(gain, 3),
                "slot_y_dist_cm": round(y_dist, 3),
                "predicted_min_margin_px": round(pred_margin, 2),
                "score_bonus": round(bonus, 3),
            }
        else:
            adjusted["prior_lateral_recovery"] = {"active": False}
        out.append(adjusted)
    out.sort(key=lambda item: (1 if item.get("hard_blocked") else 0, item["score"]))
    return out


def planner_library_with_terminal_countersteer(library, state, args):
    if not args.replanner_allow_terminal_countersteer:
        return library
    if state.get("phase_hint") != "straighten_or_enter":
        return library
    lateral_abs = abs(planner_to_float(state.get("slot_lateral_cm")))
    heading_abs = abs(planner_to_float(state.get("slot_heading_err_deg")))
    if (lateral_abs < args.replanner_terminal_countersteer_min_lateral_cm and
        heading_abs < args.replanner_terminal_countersteer_min_heading_deg):
        return library
    allow_ids = set(
        item.strip() for item in args.replanner_terminal_countersteer_action_ids.split(",")
        if item.strip())
    patched = json.loads(json.dumps(library))
    for action in patched.get("actions", []):
        if action.get("id") not in allow_ids:
            continue
        phases = list(action.get("allowed_phases") or [])
        if "straighten_or_enter" not in phases:
            phases.append("straighten_or_enter")
        action["allowed_phases"] = phases
        action["terminal_countersteer_candidate"] = True
    return patched


def planner_chosen_from_ranked(item, reason):
    command = item.get("command", "")
    try:
        kind, step, servo = _parse_motion_command_for_action(command)
    except (ValueError, IndexError):
        kind, step, servo = "WAIT", 0.0, int(SERVO_CENTER)
    return {
        "id": item.get("action_id"),
        "action_id": item.get("action_id"),
        "command": command,
        "action": kind,
        "step": round(step, 1),
        "servo": servo,
        "reason": reason,
        "score": item.get("score"),
        "origin": item.get("origin"),
        "confidence": item.get("confidence"),
        "hard_blocked": bool(item.get("hard_blocked")),
        "block_reasons": item.get("block_reasons", []),
    }


def planner_synthetic_choice(action_id, command, reason):
    return {
        "id": action_id,
        "action_id": action_id,
        "command": command,
        "action": action_id,
        "step": 0.0,
        "servo": int(SERVO_CENTER),
        "reason": reason,
        "score": None,
        "origin": "gate",
        "confidence": None,
        "hard_blocked": False,
        "block_reasons": [],
    }


def action_replanner_command(slot_state, stable, args, library, responses, criteria,
                             real_motion=False, last_action_id=None):
    planner_state = planner_flatten_slot_state(slot_state, stable)
    recovery = planner_edge_recovery_context(planner_state, criteria)
    scoring_library = planner_library_with_terminal_countersteer(library, planner_state, args)
    ranking = planner_score_actions(planner_state, scoring_library, responses, real_motion, criteria)
    ranking = planner_relax_prior_lateral_recovery_blocks(ranking, planner_state, args)
    ranking = planner_apply_switch_penalty(ranking, last_action_id, args.replanner_switch_penalty)
    gates = {
        "stable": bool(stable),
        "stable_enough": bool(planner_state.get("stable_enough")),
        "line_risk": bool(planner_state.get("line_risk")),
        "line_margin_ok": bool(planner_state.get("line_margin_ok")),
        "edge_recovery": recovery,
        "real_motion": bool(real_motion),
    }
    if not stable or not planner_state.get("stable_enough"):
        chosen = planner_synthetic_choice("WAIT", "WAIT", "wait_unstable")
    elif planner_state.get("line_risk"):
        chosen = planner_synthetic_choice("STOP", "STOP", "line_risk")
    else:
        eligible = [item for item in ranking if not item.get("hard_blocked")]
        if not eligible:
            chosen = planner_synthetic_choice("STOP", "STOP", "none_eligible")
        else:
            best = eligible[0]
            chosen = planner_chosen_from_ranked(
                best,
                "edge_recovery_action" if recovery.get("active") else "best_ranked_action")
            if last_action_id:
                previous = None
                for item in eligible:
                    if item.get("action_id") == last_action_id:
                        previous = item
                        break
                if previous and planner_to_float(previous.get("score")) <= planner_to_float(best.get("score")) + args.replanner_hold_margin:
                    chosen = planner_chosen_from_ranked(previous, "hold_hysteresis")
    action = {
        "state": "ACTION_REPLANNER",
        "action": chosen["action"],
        "cmd": chosen["command"],
        "step": chosen["step"],
        "servo": chosen["servo"],
        "steer_dir": planner_servo_dir_from_command(chosen["command"]),
        "reason": chosen["reason"],
        "binding": {
            "action_id": chosen["action_id"],
            "score": chosen["score"],
            "origin": chosen["origin"],
            "confidence": chosen["confidence"],
        },
        "replanner": {
            "pre_state": planner_state,
            "ranking": ranking,
            "chosen": chosen,
            "gates": gates,
        },
    }
    return action


def replanner_lateral_recovery_context(action, current_lat_abs, args):
    replanner = action.get("replanner") or {}
    chosen = replanner.get("chosen") or {}
    action_id = chosen.get("action_id")
    if not args.replanner_allow_lateral_recovery:
        return {"active": False, "reason": "disabled"}
    if action.get("action") != "ARC":
        return {"active": False, "reason": "not_arc"}
    if not action_id or action_id in ("WAIT", "STOP"):
        return {"active": False, "reason": "no_action_id"}
    selected = None
    for item in replanner.get("ranking") or []:
        if item.get("action_id") == action_id:
            selected = item
            break
    if not selected:
        return {"active": False, "reason": "missing_ranked_action"}
    predicted = selected.get("predicted") or {}
    pred_lat = planner_to_float(predicted.get("slot_lateral_cm"), None)
    pred_margin = planner_to_float(predicted.get("min_margin_px"), 0.0)
    pred_line_risk = bool(predicted.get("line_risk"))
    if pred_lat is None:
        return {"active": False, "reason": "missing_predicted_lateral"}
    predicted_gain = abs(current_lat_abs) - abs(pred_lat)
    checks = {
        "predicted_lateral_improves": predicted_gain >= args.replanner_lateral_recovery_min_gain_cm,
        "predicted_margin_ok": pred_margin >= args.replanner_lateral_recovery_min_margin_px,
        "predicted_line_safe": not pred_line_risk,
        "chosen_not_blocked": not bool(selected.get("hard_blocked")),
    }
    return {
        "active": all(checks.values()),
        "reason": "predicted_lateral_recovery" if all(checks.values()) else "checks_failed",
        "checks": checks,
        "current_lateral_abs_cm": round(abs(current_lat_abs), 3),
        "predicted_lateral_abs_cm": round(abs(pred_lat), 3),
        "predicted_lateral_gain_cm": round(predicted_gain, 3),
        "predicted_min_margin_px": round(pred_margin, 2),
        "action_id": action_id,
    }


def log_event(log_file, event):
    if not log_file:
        return
    event = dict(event)
    event.setdefault("time_unix", time.time())
    with open(log_file, "a", encoding="utf-8") as f:
        f.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")


def load_chassis_kinematics(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except OSError:
        return {"schema": "missing", "steer_curvature": []}


def kinematics_row_for_ste(kinematics, ste):
    for row in (kinematics or {}).get("steer_curvature", []):
        try:
            if int(round(float(row.get("ste")))) == int(round(float(ste))):
                return row
        except (TypeError, ValueError):
            continue
    return None


def stat_yaw_from_text(text):
    for ev in parse_stm32_events(text or ""):
        if ev.get("type") == "stat" and ev.get("yaw") is not None:
            return planner_to_float(ev.get("yaw"))
    return None


def angle_delta_deg(delta):
    if wrap_degrees is not None:
        return wrap_degrees(delta)
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return delta


def final_pose_report_from_token(token, stat_before_text, stat_after_text, args):
    token = token or {}
    pre_state = token.get("pre_state") or {}
    heading_token = planner_to_float(pre_state.get("slot_heading_err_deg"))
    yaw_token = planner_to_float(token.get("yaw_token"), None)
    if yaw_token is None:
        yaw_token = stat_yaw_from_text((token.get("motion") or {}).get("stat_after", ""))
    if yaw_token is None:
        yaw_token = stat_yaw_from_text(stat_before_text)
    yaw_final = stat_yaw_from_text(stat_after_text)
    report = {
        "schema": "parking_final_pose_report.v1",
        "heading_vision_token_deg": round(heading_token, 3),
        "yaw_token_deg": None if yaw_token is None else round(yaw_token, 3),
        "yaw_final_deg": None if yaw_final is None else round(yaw_final, 3),
        "yaw_delta_deg": None,
        "final_heading_deg": None,
        "final_lateral_est_cm": planner_to_float(pre_state.get("slot_lateral_cm")),
        "depth_est_cm": planner_to_float(pre_state.get("slot_y_dist_cm")),
        "verdict": "unknown",
        "thresholds": {
            "straight_heading_abs_deg": args.final_pose_straight_heading_deg,
            "lateral_abs_cm": args.final_pose_lateral_cm,
        },
    }
    if yaw_token is None or yaw_final is None:
        report["verdict"] = "unknown_missing_yaw"
        return report
    yaw_delta = angle_delta_deg(yaw_final - yaw_token)
    if getattr(args, "_fusion_signs", None) is not None:
        yaw_delta *= args._fusion_signs.yaw_to_cw_sign
    final_heading = heading_token + yaw_delta
    report["yaw_delta_deg"] = round(yaw_delta, 3)
    report["final_heading_deg"] = round(final_heading, 3)
    if abs(final_heading) <= args.final_pose_straight_heading_deg:
        report["verdict"] = "parked_straight"
    else:
        report["verdict"] = "parked_crooked"
    if abs(report["final_lateral_est_cm"]) > args.final_pose_lateral_cm:
        report["verdict"] = "not_in" if report["verdict"] == "parked_crooked" else "parked_lateral_offset"
    return report


def counter_steer_result_from_motion(decision, stat_before_text, stat_after_text, args):
    yaw_before = stat_yaw_from_text(stat_before_text)
    yaw_after = stat_yaw_from_text(stat_after_text)
    result = {
        "schema": "parking_counter_steer_result.v1",
        "decision": decision,
        "yaw_before_deg": None if yaw_before is None else round(yaw_before, 3),
        "yaw_after_deg": None if yaw_after is None else round(yaw_after, 3),
        "measured_delta_heading_deg": None,
        "predicted_delta_heading_deg": decision.get("predicted_delta_heading_deg"),
        "pre_heading_deg": decision.get("slot_heading_err_deg"),
        "post_heading_est_deg": None,
        "verdict": "unknown_missing_yaw",
    }
    if yaw_before is None or yaw_after is None:
        return result
    yaw_delta = angle_delta_deg(yaw_after - yaw_before)
    if getattr(args, "_fusion_signs", None) is not None:
        yaw_delta *= args._fusion_signs.yaw_to_cw_sign
    pre_heading = planner_to_float(decision.get("slot_heading_err_deg"))
    post_heading = pre_heading + yaw_delta
    result["measured_delta_heading_deg"] = round(yaw_delta, 3)
    result["post_heading_est_deg"] = round(post_heading, 3)
    if abs(post_heading) <= args.counter_steer_heading_enter_deg:
        result["verdict"] = "straightened"
    elif abs(post_heading) < abs(pre_heading):
        result["verdict"] = "improved"
    else:
        result["verdict"] = "worse"
    return result


def counter_steer_decision_from_state(pre_state, args, kinematics):
    state = pre_state or {}
    heading = planner_to_float(state.get("slot_heading_err_deg"))
    lateral = planner_to_float(state.get("slot_lateral_cm"))
    phase = state.get("phase_hint")
    checks = {
        "enabled": bool(args.counter_steer_enable),
        "phase_ok": phase == "straighten_or_enter",
        "lateral_ok": abs(lateral) <= args.counter_steer_max_lateral_cm,
        "heading_need": abs(heading) > args.counter_steer_heading_enter_deg,
        "heading_cap": abs(heading) <= args.counter_steer_heading_stop_deg,
    }
    decision = {
        "schema": "parking_counter_steer_decision.v1",
        "enabled": bool(args.counter_steer_enable),
        "phase_hint": phase,
        "slot_heading_err_deg": round(heading, 3),
        "slot_lateral_cm": round(lateral, 3),
        "checks": checks,
        "verdict": "not_applicable",
        "candidate_cmd": "WAIT",
        "predicted_delta_heading_deg": 0.0,
    }
    if not all(checks.values()):
        decision["verdict"] = "gate_closed"
        return decision
    desired_delta = -heading
    hard = abs(heading) >= args.counter_steer_hard_heading_deg
    if desired_delta < 0.0:
        ste = args.counter_steer_left_hard_ste if hard else args.counter_steer_left_soft_ste
    else:
        ste = args.counter_steer_right_hard_ste if hard else args.counter_steer_right_soft_ste
    row = kinematics_row_for_ste(kinematics, ste)
    deg_per_cm = planner_to_float((row or {}).get("deg_per_cm"), 0.0)
    if deg_per_cm == 0.0:
        decision.update({
            "verdict": "missing_kinematics",
            "selected_ste": ste,
            "kinematics_row": row,
        })
        return decision
    if desired_delta * deg_per_cm <= 0.0:
        decision.update({
            "verdict": "wrong_curvature_sign",
            "selected_ste": ste,
            "deg_per_cm": round(deg_per_cm, 6),
            "desired_delta_heading_deg": round(desired_delta, 3),
        })
        return decision
    arc_deadband = planner_to_float((kinematics or {}).get("arc_deadband_cm"), args.counter_steer_arc_deadband_cm)
    min_cmd = planner_to_float((kinematics or {}).get("arc_min_effective_cmd_cm"), None)
    if min_cmd is None:
        min_cmd = args.counter_steer_min_command_cm
    actual_needed = abs(desired_delta) / abs(deg_per_cm)
    command_cm = clamp(actual_needed + arc_deadband, min_cmd, args.counter_steer_max_command_cm)
    predicted_delta = math.copysign(max(0.0, command_cm - arc_deadband) * abs(deg_per_cm), desired_delta)
    cmd = "ARC D=-%.1f STE=%d V=%d" % (round(command_cm, 1), int(round(ste)), GEAR)
    decision.update({
        "verdict": "counter_steer",
        "candidate_cmd": cmd,
        "action": "ARC",
        "selected_ste": int(round(ste)),
        "hard_arc": bool(hard),
        "deg_per_cm": round(deg_per_cm, 6),
        "arc_deadband_cm": round(arc_deadband, 3),
        "actual_needed_cm": round(actual_needed, 3),
        "command_cm": round(command_cm, 3),
        "predicted_delta_heading_deg": round(predicted_delta, 3),
        "predicted_final_heading_deg": round(heading + predicted_delta, 3),
        "kinematics_row": row,
    })
    return decision


def action_from_counter_steer(decision):
    try:
        kind, step, servo = _parse_motion_command_for_action(decision.get("candidate_cmd", ""))
    except (ValueError, IndexError):
        kind, step, servo = "WAIT", 0.0, int(SERVO_CENTER)
    return {
        "state": "COUNTER_STEER",
        "action": kind,
        "cmd": decision.get("candidate_cmd", "WAIT"),
        "step": round(step, 1),
        "servo": servo,
        "steer_dir": 0.0 if servo == SERVO_CENTER else (1.0 if servo > SERVO_CENTER else -1.0),
        "reason": "terminal_heading_counter_steer",
        "binding": {
            "action_id": "counter_steer_dynamic",
            "score": None,
            "origin": "chassis_kinematics",
            "confidence": None,
        },
        "counter_steer": decision,
    }


def final_blind_pre_state_review(pre_state, args):
    """Return gate details for allowing one terminal blind reverse token."""
    state = pre_state or {}
    heading_abs = abs(planner_to_float(state.get("slot_heading_err_deg")))
    lateral_abs = abs(planner_to_float(state.get("slot_lateral_cm")))
    desired_actual_cm = final_blind_desired_actual_cm_from_state(state, None, args)
    terminal_lateral_abs = lateral_abs + desired_actual_cm * math.sin(math.radians(heading_abs))
    heading_ok = heading_abs <= (
        args.final_blind_arc_max_heading_err_deg
        if args.final_blind_allow_heading_arc else args.final_blind_max_heading_err_deg)
    checks = {
        "stable_enough": bool(state.get("stable_enough")),
        "line_margin_ok": bool(state.get("line_margin_ok")),
        "line_risk_clear": not bool(state.get("line_risk")),
        "phase_ok": state.get("phase_hint") in ("straighten_or_enter", "final_stop_zone"),
        "x_err_ok": abs(planner_to_float(state.get("slot_x_err_px"))) <= args.final_blind_max_x_err_px,
        "lateral_ok": lateral_abs <= args.final_blind_max_lateral_cm,
        "heading_ok": heading_ok,
        "margin_ok": planner_to_float(state.get("min_margin_px")) >= args.final_blind_min_margin_px,
        "distance_ok": planner_to_float(state.get("slot_y_dist_cm")) <= args.final_blind_max_y_dist_cm,
        "predicted_terminal_lateral_ok": terminal_lateral_abs <= args.final_blind_max_terminal_lateral_cm,
    }
    if lateral_abs > args.final_blind_max_lateral_cm:
        mode = "reject_lateral_visible_correction_required"
    elif heading_abs <= args.final_blind_straight_heading_err_deg:
        mode = "straight"
    elif args.final_blind_allow_heading_arc and heading_abs <= args.final_blind_arc_max_heading_err_deg:
        mode = "heading_cancel_arc"
    else:
        mode = "reject_heading"
    return {
        "pass": all(checks.values()),
        "mode": mode,
        "checks": checks,
        "thresholds": {
            "max_x_err_px": args.final_blind_max_x_err_px,
            "max_lateral_cm": args.final_blind_max_lateral_cm,
            "straight_heading_err_deg": args.final_blind_straight_heading_err_deg,
            "max_heading_err_deg": args.final_blind_max_heading_err_deg,
            "arc_max_heading_err_deg": args.final_blind_arc_max_heading_err_deg,
            "min_margin_px": args.final_blind_min_margin_px,
            "max_y_dist_cm": args.final_blind_max_y_dist_cm,
            "max_terminal_lateral_cm": args.final_blind_max_terminal_lateral_cm,
        },
        "predicted": {
            "desired_actual_cm": round(desired_actual_cm, 3),
            "terminal_lateral_abs_cm": round(terminal_lateral_abs, 3),
        },
    }


def final_blind_desired_actual_cm_from_state(pre_state, token, args):
    configured = max(0.0, planner_to_float(args.final_blind_reverse_cm))
    if args.final_blind_distance_mode == "fixed":
        return configured
    state = pre_state or {}
    source = token or {}
    motion = source.get("motion") or {}
    y_dist = planner_to_float(state.get("slot_y_dist_cm"))
    progress = planner_to_float(motion.get("odom_progress_cm"))
    remaining = max(0.0, y_dist - progress - args.final_blind_target_y_dist_cm)
    if configured > 0.0:
        remaining = min(remaining, configured)
    return remaining


def final_blind_command_distance_cm(desired_actual_cm, args):
    if desired_actual_cm <= 0.0:
        return 0.0
    if not args.final_blind_compensate_deadband:
        return desired_actual_cm
    command_cm = desired_actual_cm + args.final_blind_deadband_cm - args.final_blind_coast_cm
    return max(desired_actual_cm, command_cm)


def final_blind_plan_from_token(token, args, remaining_total_cm):
    pre_state = (token or {}).get("pre_state") or {}
    review = (token or {}).get("pre_state_review") or {}
    mode = review.get("mode") or "straight"
    heading = planner_to_float(pre_state.get("slot_heading_err_deg"))
    desired_actual = final_blind_desired_actual_cm_from_state(pre_state, token, args)
    desired_actual = min(desired_actual, args.final_blind_max_actual_cm, max(0.0, remaining_total_cm))
    command_cm = final_blind_command_distance_cm(desired_actual, args)
    command_cm = round(min(command_cm, args.final_blind_max_command_cm, max(0.0, remaining_total_cm)), 1)
    if mode == "heading_cancel_arc" and args.final_blind_allow_heading_arc:
        arc_actual = math.radians(abs(heading)) * max(1.0, args.final_blind_arc_reff_cm)
        arc_actual = min(arc_actual, desired_actual)
        arc_command_cm = final_blind_command_distance_cm(arc_actual, args)
        command_cm = round(min(arc_command_cm, args.final_blind_max_command_cm, max(0.0, remaining_total_cm)), 1)
        servo = args.final_blind_arc_left_servo if heading > 0 else args.final_blind_arc_right_servo
        return {
            "action": "ARC",
            "cmd": "ARC D=%.1f STE=%d V=%d" % (-command_cm, int(round(servo)), GEAR),
            "command_cm": command_cm,
            "desired_actual_cm": round(desired_actual, 3),
            "arc_actual_cm": round(arc_actual, 3),
            "servo": int(round(servo)),
            "mode": mode,
            "heading_err_deg": round(heading, 3),
        }
    return {
        "action": "MOVE",
        "cmd": "MOVE D=%.1f V=%d" % (-command_cm, GEAR),
        "command_cm": command_cm,
        "desired_actual_cm": round(desired_actual, 3),
        "mode": "straight",
        "heading_err_deg": round(heading, 3),
    }


def write_final_blind_token(path, payload):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, separators=(",", ":"))
        f.write("\n")
    os.replace(tmp_path, path)


def read_final_blind_token(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def final_blind_token_review(token, args):
    computed_pre_state_review = (
        final_blind_pre_state_review(token.get("pre_state") or {}, args) if token else None)
    checks = {
        "exists": token is not None,
        "schema_ok": bool(token and token.get("schema") == "parking_final_blind_token.v1"),
        "not_consumed": bool(token and not token.get("consumed")),
        "fresh": bool(token and (time.time() - planner_to_float(token.get("time_unix"))) <= args.final_blind_token_max_age_sec),
    }
    if token:
        checks["pre_state_ok"] = bool((token.get("pre_state_review") or {}).get("pass"))
        chosen = token.get("chosen") or {}
        checks["chosen_action_ok"] = chosen.get("action") in ("MOVE", "ARC")
        checks["chosen_not_stop_wait"] = chosen.get("action_id") not in ("WAIT", "STOP")
    else:
        checks["pre_state_ok"] = False
        checks["chosen_action_ok"] = False
        checks["chosen_not_stop_wait"] = False
    if computed_pre_state_review is not None:
        checks["pre_state_ok"] = bool(computed_pre_state_review.get("pass"))
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "computed_pre_state_review": computed_pre_state_review,
    }


def consume_final_blind_token(path, token, result):
    if not token:
        return
    updated = dict(token)
    updated["consumed"] = True
    updated["consumed_time_unix"] = time.time()
    updated["consume_result"] = result
    try:
        write_final_blind_token(path, updated)
    except OSError:
        pass


def _cmd_from_plan_with_policy(p, step_cm, steer_deg, lateral_sign, max_abs_d_cm=None):
    lat = p["lat"]
    head = p["head"]
    step = min(step_cm, max(1.0, p["lon"])) if p["lon"] > 0 else step_cm
    abs_d = round(step, 1) + DEADBAND_CM
    if max_abs_d_cm is not None:
        abs_d = min(abs_d, max_abs_d_cm)
    cmd_d = -abs_d
    if abs(lat) >= DEFAULT_LAT_TEMPLATE_THRESHOLD_CM:
        raw_dir = 1.0 if lat > 0 else -1.0
        steer_dir = lateral_sign * raw_dir
        servo = int(round(clamp(SERVO_CENTER + steer_dir * steer_deg * STEERING_SIGN,
                                SERVO_MIN, SERVO_MAX)))
        return {
            "state": "FEEDBACK_ALIGN_LATERAL",
            "cmd": "ARC D=%.1f STE=%d V=%d" % (cmd_d, servo, GEAR),
            "step": step,
            "servo": servo,
            "steer_dir": steer_dir,
            "reason": "feedback_lateral",
        }
    if abs(head) >= DEFAULT_HEAD_TEMPLATE_THRESHOLD_DEG:
        raw_dir = 1.0 if head > 0 else -1.0
        steer_dir = raw_dir
        servo = int(round(clamp(SERVO_CENTER + steer_dir * steer_deg * 0.5 * STEERING_SIGN,
                                SERVO_MIN, SERVO_MAX)))
        return {
            "state": "FEEDBACK_STRAIGHTEN",
            "cmd": "ARC D=%.1f STE=%d V=%d" % (cmd_d, servo, GEAR),
            "step": step,
            "servo": servo,
            "steer_dir": steer_dir,
            "reason": "feedback_heading",
        }
    return {
        "state": "FEEDBACK_FINAL_REVERSE",
        "cmd": "MOVE D=%.1f V=%d" % (cmd_d, GEAR),
        "step": step,
        "servo": SERVO_CENTER,
        "steer_dir": 0.0,
        "reason": "feedback_centered",
    }


def collect_stable_plan(sock, args, timeout_sec):
    filt = SlotStabilityFilter(args.stable_frames, args.max_center_shift_cm, args.max_axis_yaw_shift_deg)
    end = time.time() + timeout_sec
    last = None
    while time.time() < end:
        info = acquire_info(sock, max(0.2, min(args.target_wait_sec, end - time.time())))
        if info is None:
            continue
        stable, metrics = filt.add(info)
        fused = filt.fused() if stable else info
        last = (stable, fused, metrics)
        if stable:
            return last
    return last


def _read_feedback_token(path, timeout_sec):
    print("  feedback: write '+', '-', '0', 'r', or 'q' to %s" % path, flush=True)
    deadline = time.time() + timeout_sec if timeout_sec > 0 else None
    while deadline is None or time.time() < deadline:
        try:
            with open(path, "r", encoding="utf-8") as f:
                token = f.read().strip().lower()
        except OSError:
            token = ""
        if token:
            try:
                with open(path, "w", encoding="utf-8") as f:
                    f.write("")
            except OSError:
                pass
            if token[0] in ["+", "-", "0", "r", "q"]:
                return token[0]
        time.sleep(0.2)
    return ""


def _auto_feedback(pre_p, post_p, min_lat_improve_cm):
    lat_delta = abs(pre_p["lat"]) - abs(post_p["lat"])
    lon_delta = pre_p["lon"] - post_p["lon"]
    heading_penalty = max(0.0, abs(post_p["head"]) - abs(pre_p["head"]))
    score = lat_delta + 0.25 * lon_delta - 0.12 * heading_penalty
    reward = 1 if lat_delta >= min_lat_improve_cm and lon_delta >= -1.0 else -1
    return reward, {
        "lat_delta_abs_cm": round(lat_delta, 3),
        "lon_delta_cm": round(lon_delta, 3),
        "heading_penalty_deg": round(heading_penalty, 3),
        "score": round(score, 3),
    }


def _bucket(value, cuts, labels):
    for cut, label in zip(cuts, labels):
        if value < cut:
            return label
    return labels[-1]


def policy_state_key(p):
    lon_b = _bucket(p["lon"], [12.0, 25.0, 45.0], ["near", "mid", "far", "very_far"])
    alat = abs(p["lat"])
    if alat < 2.5:
        lat_b = "center"
    elif alat < 6.0:
        lat_b = "left_small" if p["lat"] > 0 else "right_small"
    else:
        lat_b = "left_large" if p["lat"] > 0 else "right_large"
    ahead = abs(p["head"])
    if ahead < 2.0:
        head_b = "straight"
    elif ahead < 6.0:
        head_b = "yaw_pos_small" if p["head"] > 0 else "yaw_neg_small"
    else:
        head_b = "yaw_pos_large" if p["head"] > 0 else "yaw_neg_large"
    return "%s|%s|%s" % (lon_b, lat_b, head_b)


def parse_policy_actions(text, max_abs_d_cm):
    actions = []
    for raw in text.split("|"):
        cmd = raw.strip()
        if not cmd:
            continue
        if cmd == "STOP":
            actions.append(cmd)
            continue
        parts = cmd.split()
        kind = parts[0] if parts else ""
        kv = {}
        for part in parts[1:]:
            if "=" in part:
                k, v = part.split("=", 1)
                kv[k.upper()] = v
        if kind not in ["MOVE", "ARC"]:
            raise ValueError("unsupported action kind: %s" % cmd)
        d = float(kv.get("D", "0"))
        if d >= 0 or abs(d) > max_abs_d_cm:
            raise ValueError("unsafe action distance: %s" % cmd)
        if int(kv.get("V", str(GEAR))) > GEAR:
            raise ValueError("unsafe gear: %s" % cmd)
        if kind == "ARC":
            ste = int(float(kv.get("STE", str(SERVO_CENTER))))
            if ste < SERVO_MIN or ste > SERVO_MAX:
                raise ValueError("unsafe servo: %s" % cmd)
        actions.append(cmd)
    if not actions:
        raise ValueError("empty policy action list")
    return actions


def load_policy(path, actions):
    try:
        with open(path, "r", encoding="utf-8") as f:
            policy = json.load(f)
    except OSError:
        policy = {}
    policy.setdefault("schema_version", 1)
    policy.setdefault("actions", actions)
    policy.setdefault("q", {})
    policy.setdefault("counts", {})
    policy.setdefault("episodes", 0)
    return policy


def save_policy(path, policy):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(policy, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def choose_policy_action(policy, state_key, actions, epsilon):
    q_state = policy.setdefault("q", {}).setdefault(state_key, {})
    for action in actions:
        q_state.setdefault(action, 0.0)
    if random.random() < epsilon:
        return random.choice(actions), "explore"
    best_q = max(q_state.get(a, 0.0) for a in actions)
    tied = [a for a in actions if abs(q_state.get(a, 0.0) - best_q) < 1e-9]
    return random.choice(tied), "exploit"


def update_policy(policy, state_key, action, reward, alpha):
    q_state = policy.setdefault("q", {}).setdefault(state_key, {})
    old = float(q_state.get(action, 0.0))
    new = old + alpha * (reward - old)
    q_state[action] = round(new, 4)
    count_key = "%s||%s" % (state_key, action)
    counts = policy.setdefault("counts", {})
    counts[count_key] = int(counts.get(count_key, 0)) + 1
    return old, new, counts[count_key]


def reward_from_token(token, auto_reward):
    if token == "+":
        return 1.0, "manual_positive"
    if token == "-":
        return -1.0, "manual_negative"
    if token == "0":
        return 0.0, "manual_neutral"
    return float(auto_reward), "auto"


def wait_restart_or_quit(args, policy, reason, episode):
    log_event(args.log_jsonl, {
        "event": "learn_hold",
        "reason": reason,
        "episode": episode,
        "send_to_stm32": False,
        "motion_enabled": False,
        "actuator_control_allowed": False,
    })
    if not args.feedback_manual:
        return "quit"
    print("HOLD=%s; press SPACE for new round or q to quit." % reason, flush=True)
    while True:
        token = _read_feedback_token(args.feedback_file, 0)
        if token == "q":
            save_policy(args.learn_policy_file, policy)
            return "quit"
        if token == "r":
            return "restart"
        if token in ["+", "-", "0"]:
            print("  ignored feedback '%s' while holding; use SPACE or q." % token, flush=True)


def run_learn_policy(args):
    armed = args.arm and (args.dry_run or os.path.exists(args.arm_file))
    actions = parse_policy_actions(args.learn_actions, args.learn_max_command_abs_d_cm)
    policy = load_policy(args.learn_policy_file, actions)
    if not args.dry_run:
        serial_setup()
        st = read_stat()
        print("  STM32: %s" % st["raw"], flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((args.listen_host, args.listen_port))
    print("=== PARKING POLICY LEARNER ===", flush=True)
    print("  dry_run=%s arm=%s arm_file=%s(%s)" % (
        args.dry_run, args.arm, args.arm_file, os.path.exists(args.arm_file)), flush=True)
    print("  actions=%d epsilon=%.2f alpha=%.2f policy=%s" % (
        len(actions), args.learn_epsilon, args.learn_alpha, args.learn_policy_file), flush=True)
    print("  feedback: right=+ left=- space=restart q=quit via %s" % args.feedback_file, flush=True)

    total_cm = 0.0
    bad_streak = 0
    rollout = int(policy.get("rollouts", 0)) + 1
    episode = 1
    try:
        while args.learn_episodes <= 0 or episode <= args.learn_episodes:
            pre = collect_stable_plan(sock, args, args.feedback_vision_timeout_sec)
            if pre is None or not pre[0]:
                print("STOP=NO_STABLE_PRE_VISION episode=%d" % episode, flush=True)
                log_event(args.log_jsonl, {"event": "learn_stop", "reason": "no_stable_pre_vision",
                                           "episode": episode})
                if not args.dry_run:
                    stop()
                action = wait_restart_or_quit(args, policy, "NO_STABLE_PRE_VISION", episode)
                if action == "restart":
                    total_cm = 0.0
                    bad_streak = 0
                    rollout += 1
                    policy["rollouts"] = rollout
                    save_policy(args.learn_policy_file, policy)
                    continue
                return 6
            _stable, pre_fused, pre_metrics = pre
            pre_p = pre_fused["plan"]
            state_key = policy_state_key(pre_p)
            cmd, choose_reason = choose_policy_action(policy, state_key, actions, args.learn_epsilon)

            print("R%03d EP%04d %s PRE lon=%.2f lat=%.2f head=%.2f -> %s (%s)" % (
                rollout, episode, state_key, pre_p["lon"], pre_p["lat"], pre_p["head"], cmd, choose_reason), flush=True)
            log_event(args.log_jsonl, {
                "event": "learn_pre",
                "rollout": rollout,
                "episode": episode,
                "state_key": state_key,
                "choice": choose_reason,
                "dry_run": args.dry_run,
                "stability": pre_metrics,
                "confidence": round(pre_fused["confidence"], 4),
                "lon": round(pre_p["lon"], 3),
                "lat": round(pre_p["lat"], 3),
                "head": round(pre_p["head"], 3),
                "candidate_cmd": cmd,
                "send_to_stm32": False if args.dry_run else bool(armed and cmd != "STOP"),
                "motion_enabled": False if args.dry_run else bool(armed and cmd != "STOP"),
                "actuator_control_allowed": False if args.dry_run else bool(armed and cmd != "STOP"),
            })

            if cmd == "STOP":
                if not args.dry_run:
                    stop()
            elif args.dry_run:
                print("  [dry-run] would send: %s" % cmd, flush=True)
            else:
                if not armed:
                    print("REFUSING MOTION: learn-policy needs --arm and arm file.", flush=True)
                    stop()
                    return 4
                send_cmd(cmd, read_sec=args.move_read_sec)
                stop()
                # Use commanded distance as the conservative training cap.
                for part in cmd.split():
                    if part.startswith("D="):
                        total_cm += abs(float(part.split("=", 1)[1]))
                        break

            time.sleep(args.feedback_post_settle_sec)
            post = collect_stable_plan(sock, args, args.feedback_vision_timeout_sec)
            if post is None or not post[0]:
                print("STOP=NO_STABLE_POST_VISION episode=%d" % episode, flush=True)
                log_event(args.log_jsonl, {"event": "learn_stop", "reason": "no_stable_post_vision",
                                           "episode": episode})
                if not args.dry_run:
                    stop()
                action = wait_restart_or_quit(args, policy, "NO_STABLE_POST_VISION", episode)
                if action == "restart":
                    total_cm = 0.0
                    bad_streak = 0
                    rollout += 1
                    policy["rollouts"] = rollout
                    save_policy(args.learn_policy_file, policy)
                    continue
                return 7
            _stable, post_fused, post_metrics = post
            post_p = post_fused["plan"]
            auto_reward, auto_metrics = _auto_feedback(pre_p, post_p, args.feedback_min_lat_improve_cm)

            token = ""
            if args.feedback_manual:
                token = _read_feedback_token(args.feedback_file, args.feedback_timeout_sec)
            if token == "q":
                print("STOP=USER_QUIT", flush=True)
                if not args.dry_run:
                    stop()
                save_policy(args.learn_policy_file, policy)
                return 0
            if token == "r":
                policy["episodes"] = int(policy.get("episodes", 0)) + 1
                policy["rollouts"] = rollout + 1
                save_policy(args.learn_policy_file, policy)
                print("RESTART=NEW_ROUND requested", flush=True)
                total_cm = 0.0
                bad_streak = 0
                rollout += 1
                episode += 1
                continue

            reward, reward_source = reward_from_token(token, auto_reward if args.feedback_auto else 0.0)
            if reward < 0:
                bad_streak += 1
            else:
                bad_streak = 0
            old_q, new_q, count = update_policy(policy, state_key, cmd, reward, args.learn_alpha)
            policy["episodes"] = int(policy.get("episodes", 0)) + 1
            save_policy(args.learn_policy_file, policy)

            print("EP%03d POST lon=%.2f lat=%.2f head=%.2f reward=%.1f %s q %.2f->%.2f delta=%s" % (
                episode, post_p["lon"], post_p["lat"], post_p["head"], reward, reward_source,
                old_q, new_q, json.dumps(auto_metrics, separators=(",", ":"))), flush=True)
            log_event(args.log_jsonl, {
                "event": "learn_post",
                "rollout": rollout,
                "episode": episode,
                "state_key": state_key,
                "dry_run": args.dry_run,
                "stability": post_metrics,
                "confidence": round(post_fused["confidence"], 4),
                "lon": round(post_p["lon"], 3),
                "lat": round(post_p["lat"], 3),
                "head": round(post_p["head"], 3),
                "candidate_cmd": cmd,
                "reward": reward,
                "reward_source": reward_source,
                "auto_metrics": auto_metrics,
                "q_old": round(old_q, 4),
                "q_new": round(new_q, 4),
                "count": count,
                "send_to_stm32": False,
                "motion_enabled": False if args.dry_run else bool(armed),
                "actuator_control_allowed": False if args.dry_run else bool(armed),
            })

            if args.learn_stop_after_bad > 0 and bad_streak >= args.learn_stop_after_bad:
                print("STOP=NEGATIVE_STREAK %d" % bad_streak, flush=True)
                if not args.dry_run:
                    stop()
                action = wait_restart_or_quit(args, policy, "NEGATIVE_STREAK", episode)
                if action == "restart":
                    total_cm = 0.0
                    bad_streak = 0
                    rollout += 1
                    policy["rollouts"] = rollout
                    save_policy(args.learn_policy_file, policy)
                    episode += 1
                    continue
                return 8
            if args.learn_max_total_cm > 0 and total_cm >= args.learn_max_total_cm:
                print("STOP=LEARN_TOTAL_CAP total_cm=%.1f" % total_cm, flush=True)
                if not args.dry_run:
                    stop()
                action = wait_restart_or_quit(args, policy, "LEARN_TOTAL_CAP", episode)
                if action == "restart":
                    total_cm = 0.0
                    bad_streak = 0
                    rollout += 1
                    policy["rollouts"] = rollout
                    save_policy(args.learn_policy_file, policy)
                    episode += 1
                    continue
                return 0
            if abs(post_p["lat"]) <= args.feedback_success_lat_cm and post_p["lon"] <= args.feedback_success_lon_cm:
                print("STOP=LEARN_SUCCESS lon=%.2f lat=%.2f" % (post_p["lon"], post_p["lat"]), flush=True)
                if not args.dry_run:
                    stop()
                action = wait_restart_or_quit(args, policy, "LEARN_SUCCESS", episode)
                if action == "restart":
                    total_cm = 0.0
                    bad_streak = 0
                    rollout += 1
                    policy["rollouts"] = rollout
                    save_policy(args.learn_policy_file, policy)
                    episode += 1
                    continue
                return 0
            episode += 1

        print("STOP=LEARN_EPISODES_DONE", flush=True)
        if not args.dry_run:
            stop()
        return 0
    except KeyboardInterrupt:
        print("\nABORT (Ctrl-C) -> STOP", flush=True)
        if not args.dry_run:
            stop()
        save_policy(args.learn_policy_file, policy)
        return 130


def run_feedback_tune(args):
    armed = args.arm and (args.dry_run or os.path.exists(args.arm_file))
    if not args.dry_run:
        serial_setup()
        st = read_stat()
        print("  STM32: %s" % st["raw"], flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((args.listen_host, args.listen_port))
    print("=== PARKING FEEDBACK TUNER ===", flush=True)
    print("  dry_run=%s arm=%s arm_file=%s(%s)" % (
        args.dry_run, args.arm, args.arm_file, os.path.exists(args.arm_file)), flush=True)
    print("  episodes=%d step_cm=%.1f steer_deg=%.1f lateral_sign=%.1f" % (
        args.feedback_episodes, args.feedback_step_cm, args.feedback_steer_deg,
        args.feedback_lateral_sign), flush=True)
    print("  safety: per-step STOP=%s max_step_cm=%.1f" % (True, args.feedback_max_step_cm), flush=True)

    step_cm = min(args.feedback_step_cm, args.feedback_max_step_cm)
    steer_deg = args.feedback_steer_deg
    lateral_sign = args.feedback_lateral_sign
    bad_streak = 0
    total_cm = 0.0

    try:
        for episode in range(1, args.feedback_episodes + 1):
            pre = collect_stable_plan(sock, args, args.feedback_vision_timeout_sec)
            if pre is None or not pre[0]:
                print("STOP=NO_STABLE_PRE_VISION episode=%d" % episode, flush=True)
                log_event(args.log_jsonl, {"event": "feedback_stop", "reason": "no_stable_pre_vision",
                                           "episode": episode})
                if not args.dry_run:
                    stop()
                return 6

            _stable, pre_fused, pre_metrics = pre
            pre_p = pre_fused["plan"]
            action = _cmd_from_plan_with_policy(
                pre_p, step_cm, steer_deg, lateral_sign, args.feedback_max_command_abs_d_cm)
            cmd = action["cmd"]
            print("EP%02d PRE lon=%.2f lat=%.2f head=%.2f -> %s" % (
                episode, pre_p["lon"], pre_p["lat"], pre_p["head"], cmd), flush=True)

            log_event(args.log_jsonl, {
                "event": "feedback_pre",
                "episode": episode,
                "dry_run": args.dry_run,
                "stable": True,
                "stability": pre_metrics,
                "confidence": round(pre_fused["confidence"], 4),
                "lon": round(pre_p["lon"], 3),
                "lat": round(pre_p["lat"], 3),
                "head": round(pre_p["head"], 3),
                "candidate_cmd": cmd,
                "step_cm": round(step_cm, 3),
                "steer_deg": round(steer_deg, 3),
                "lateral_sign": lateral_sign,
                "send_to_stm32": False if args.dry_run else bool(armed),
                "motion_enabled": False if args.dry_run else bool(armed),
                "actuator_control_allowed": False if args.dry_run else bool(armed),
            })

            if args.dry_run:
                print("  [dry-run] would send: %s" % cmd, flush=True)
            else:
                if not armed:
                    print("REFUSING MOTION: feedback tuner needs --arm and arm file.", flush=True)
                    stop()
                    return 4
                send_cmd(cmd, read_sec=args.move_read_sec)
                stop()
                total_cm += action["step"]

            time.sleep(args.feedback_post_settle_sec)
            post = collect_stable_plan(sock, args, args.feedback_vision_timeout_sec)
            if post is None or not post[0]:
                print("STOP=NO_STABLE_POST_VISION episode=%d" % episode, flush=True)
                log_event(args.log_jsonl, {"event": "feedback_stop", "reason": "no_stable_post_vision",
                                           "episode": episode})
                if not args.dry_run:
                    stop()
                return 7

            _stable, post_fused, post_metrics = post
            post_p = post_fused["plan"]
            auto_reward, auto_metrics = _auto_feedback(pre_p, post_p, args.feedback_min_lat_improve_cm)
            token = ""
            if args.feedback_manual and not args.dry_run:
                token = _read_feedback_token(args.feedback_file, args.feedback_timeout_sec)
            if token == "q":
                print("STOP=USER_QUIT", flush=True)
                if not args.dry_run:
                    stop()
                return 0
            if token == "+":
                reward = 1
                source = "manual_positive"
            elif token == "-":
                reward = -1
                source = "manual_negative"
            elif token == "0":
                reward = 0
                source = "manual_neutral"
            else:
                reward = auto_reward if args.feedback_auto else 0
                source = "auto" if args.feedback_auto else "none"

            print("EP%02d POST lon=%.2f lat=%.2f head=%.2f reward=%s source=%s delta=%s" % (
                episode, post_p["lon"], post_p["lat"], post_p["head"], reward, source,
                json.dumps(auto_metrics, separators=(",", ":"))), flush=True)

            if reward > 0:
                bad_streak = 0
                step_cm = min(args.feedback_max_step_cm, step_cm + args.feedback_step_increment_cm)
                if abs(post_p["lat"]) < abs(pre_p["lat"]):
                    steer_deg = max(args.feedback_min_steer_deg, steer_deg - args.feedback_steer_increment_deg)
            elif reward < 0:
                bad_streak += 1
                step_cm = max(args.feedback_min_step_cm, step_cm * 0.6)
                steer_deg = min(args.feedback_max_steer_deg, steer_deg + args.feedback_steer_increment_deg)
                if bad_streak >= args.feedback_flip_after_bad:
                    lateral_sign *= -1.0
                    bad_streak = 0

            log_event(args.log_jsonl, {
                "event": "feedback_post",
                "episode": episode,
                "dry_run": args.dry_run,
                "stable": True,
                "stability": post_metrics,
                "confidence": round(post_fused["confidence"], 4),
                "lon": round(post_p["lon"], 3),
                "lat": round(post_p["lat"], 3),
                "head": round(post_p["head"], 3),
                "reward": reward,
                "reward_source": source,
                "auto_metrics": auto_metrics,
                "next_step_cm": round(step_cm, 3),
                "next_steer_deg": round(steer_deg, 3),
                "next_lateral_sign": lateral_sign,
                "send_to_stm32": False,
                "motion_enabled": False if args.dry_run else bool(armed),
                "actuator_control_allowed": False if args.dry_run else bool(armed),
            })

            if abs(post_p["lat"]) <= args.feedback_success_lat_cm and post_p["lon"] <= args.feedback_success_lon_cm:
                print("STOP=FEEDBACK_SUCCESS lon=%.2f lat=%.2f" % (post_p["lon"], post_p["lat"]), flush=True)
                if not args.dry_run:
                    stop()
                return 0
            if total_cm >= args.feedback_max_total_cm:
                print("STOP=FEEDBACK_TOTAL_CAP total_cm=%.1f" % total_cm, flush=True)
                if not args.dry_run:
                    stop()
                return 0

        print("STOP=FEEDBACK_EPISODES_DONE", flush=True)
        if not args.dry_run:
            stop()
        return 0
    except KeyboardInterrupt:
        print("\nABORT (Ctrl-C) -> STOP", flush=True)
        if not args.dry_run:
            stop()
        return 130


# ============================ control loop ============================

def acquire(sock, wait_sec):
    """Poll UDP up to wait_sec for a slot; tolerate dropouts. Returns (center_cm,axis_cm)|None."""
    end = time.time() + wait_sec
    while True:
        raw = recv_latest(sock)
        if raw is not None:
            slot = best_slot_from_udp(raw)
            if slot is not None:
                return slot
        if time.time() >= end:
            return None
        time.sleep(0.1)


def acquire_info(sock, wait_sec, selector=None):
    """Poll UDP up to wait_sec for structured slot info."""
    end = time.time() + wait_sec
    last_selection = None
    while True:
        raw = recv_latest(sock)
        if raw is not None:
            if selector is not None:
                info, selection = selector.select(raw)
                last_selection = selection
            else:
                info = best_slot_info_from_udp(raw)
                selection = None
            if info is not None:
                if selection is not None:
                    info["target_selection"] = selection
                return info
        if time.time() >= end:
            if last_selection is not None:
                if last_selection.get("status") == "no_candidates":
                    return None
                return {"target_selection_wait": last_selection}
            return None
        time.sleep(0.1)


def run(args):
    no_motion = args.dry_run or (args.strategy == "action_replanner" and args.replanner_dry_run)
    armed = args.arm and (no_motion or os.path.exists(args.arm_file))
    print("=== ON-BOARD PARKING CONTROLLER ===", flush=True)
    print("  dry_run=%s arm=%s arm_file=%s(%s)" % (
        args.dry_run, args.arm, args.arm_file, os.path.exists(args.arm_file)), flush=True)
    print("  safety: motion_requires_arm=%s no_motion_mode=%s replanner_dry_run=%s" % (
        True, no_motion, args.replanner_dry_run), flush=True)
    success_criteria = load_success_criteria(args.success_criteria_json)
    perception_filter = load_perception_filter(args.perception_filter_json)
    if args.stable_frames <= 0:
        args.stable_frames = int(perception_filter.get("required_frames", args.stable_frames))
    args.stable_frames = max(1, int(args.stable_frames))
    if args.max_center_shift_cm is None:
        args.max_center_shift_cm = float(perception_filter.get("gate_center_shift_cm", 3.0))
    if args.max_axis_yaw_shift_deg is None:
        args.max_axis_yaw_shift_deg = float(perception_filter.get("gate_yaw_shift_deg", 6.0))
    print("  strategy=%s stable_frames=%d log_jsonl=%s" % (
        args.strategy, args.stable_frames, args.log_jsonl or ""), flush=True)
    print("  success_criteria=%s schema=%s" % (
        args.success_criteria_json, success_criteria.get("schema", "")), flush=True)
    print("  perception_filter=%s gate_center=%.2f gate_yaw=%.2f hold=%.2fs" % (
        args.perception_filter_json or "builtin",
        args.max_center_shift_cm,
        args.max_axis_yaw_shift_deg,
        float(perception_filter.get("hold_grace_sec", 0.0))), flush=True)
    chassis_kinematics = load_chassis_kinematics(args.chassis_kinematics_json)
    print("  chassis_kinematics=%s schema=%s rows=%d counter_steer=%s" % (
        args.chassis_kinematics_json,
        chassis_kinematics.get("schema", ""),
        len(chassis_kinematics.get("steer_curvature", [])),
        args.counter_steer_enable), flush=True)
    planner_library = None
    planner_responses = None
    if args.strategy == "action_replanner":
        planner_library = planner_load_json(args.action_library_json)
        planner_responses = planner_load_response_model(args.response_model_json)
        print("  action_library=%s actions=%d" % (
            args.action_library_json, len(planner_library.get("actions", []))), flush=True)
        print("  response_model=%s schema=%s records=%d" % (
            args.response_model_json,
            planner_responses.get("schema", ""),
            len(planner_responses.get("records", []))), flush=True)
    pose_fuser = None
    if PoseFuser is not None and getattr(args, "_fusion_signs", None) is not None:
        pose_fuser = PoseFuser(args._fusion_signs)
        print("  fusion_pose=shadow_log_only", flush=True)
    if not no_motion:
        serial_setup()
        st = read_stat()
        print("  STM32: %s" % st["raw"], flush=True)
        if st["yaw"] is None:
            print("  WARN: no STM32 STAT response (check power/reset).", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((args.listen_host, args.listen_port))
    print("  listening UDP %s:%d" % (args.listen_host, args.listen_port), flush=True)

    filt = SlotStabilityFilter(
        args.stable_frames,
        args.max_center_shift_cm,
        args.max_axis_yaw_shift_deg,
        outlier_accept_consecutive=perception_filter.get("outlier_accept_consecutive", 3),
        hold_grace_sec=perception_filter.get("hold_grace_sec", 0.0),
        hold_max_frames=perception_filter.get("hold_max_frames", 0),
        gate_static_scale=perception_filter.get("gate_static_scale", 0.5),
    )
    target_selector = SlotTargetSelector(args) if args.slot_select_enable else None
    steps = 0
    total_cm = 0.0
    best_lat = None
    anchor = None
    ds_anchor = 0.0
    blind_cm = 0.0
    vision_lost_since = None
    last_stable_pixel_action = None
    last_sent_corridor_x_err = None
    last_replanner_action_id = None
    pixel_blind_finish_used = False
    final_blind_used = False
    deadline = time.time() + args.duration_sec if args.duration_sec > 0 else None
    try:
        while True:
            if deadline is not None and time.time() >= deadline:
                print("STOP=DURATION elapsed.", flush=True)
                log_event(args.log_jsonl, {"event": "duration_elapsed", "steps": steps, "total_cm": round(total_cm, 2)})
                if not no_motion:
                    stop()
                return 0

            wait_sec = args.target_wait_sec
            if deadline is not None:
                wait_sec = max(0.0, min(wait_sec, deadline - time.time()))
            info = acquire_info(sock, wait_sec, target_selector)
            if info is not None:
                if "target_selection_wait" in info:
                    selection = info.get("target_selection_wait")
                    coasted, hold_metrics = filt.tick_no_detection()
                    if coasted is not None:
                        log_event(args.log_jsonl, {
                            "event": "perception_hold_wait",
                            "dry_run": args.dry_run,
                            "selection": selection,
                            "hold": hold_metrics,
                            "coasted_state": slot_relative_state(coasted, args, coasted.get("stability")),
                            "send_to_stm32": False,
                            "motion_enabled": False,
                            "actuator_control_allowed": False,
                        })
                        print("WAIT=PERCEPTION_HOLD %s." % hold_metrics.get("reason"), flush=True)
                        time.sleep(args.settle_sec)
                        continue
                    log_event(args.log_jsonl, {
                        "event": "slot_target_selection_wait",
                        "dry_run": args.dry_run,
                        "selection": selection,
                        "hold": hold_metrics,
                        "lost_elapsed_sec": 0.0 if vision_lost_since is None else round(time.time() - vision_lost_since, 3),
                        "send_to_stm32": False,
                        "motion_enabled": False,
                        "actuator_control_allowed": False,
                    })
                    now = time.time()
                    if vision_lost_since is None:
                        vision_lost_since = now
                    lost_elapsed = now - vision_lost_since
                    if lost_elapsed >= args.replanner_vision_lost_stop_sec:
                        print("STOP=SLOT_TARGET_SELECTION_TIMEOUT.", flush=True)
                        if not no_motion:
                            stop()
                        return 0
                    print("WAIT=SLOT_TARGET_SELECTION %s." % (
                        selection.get("reason") if isinstance(selection, dict) else "no_stable_target"), flush=True)
                    time.sleep(args.settle_sec)
                    continue
                vision_lost_since = None
                stable, metrics = filt.add(info, observing_static=True)
                fused = filt.fused() if stable else info
                p = fused["plan"]
                if args.strategy == "pixel_servo":
                    action = pixel_servo_command(fused, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "corridor_servo":
                    action = corridor_servo_command(fused, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "normalized_corridor_servo":
                    action = normalized_corridor_servo_command(fused, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "path_template_planner":
                    action = path_template_planner_command(fused, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "primitive_probe":
                    action = primitive_probe_command(fused, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "action_replanner":
                    action = {
                        "state": "ACTION_REPLANNER_PENDING",
                        "action": "WAIT",
                        "cmd": "WAIT",
                        "step": 0.0,
                        "servo": SERVO_CENTER,
                        "reason": "waiting_for_slot_relative_state",
                    }
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                elif args.strategy == "template":
                    action = template_command(p, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]
                else:
                    cmd = p["cmd"]
                    step = p["step"]
                    state = "PURE_PURSUIT"
                    reason = "pure_pursuit"
                    action = {
                        "cmd": cmd,
                        "step": step,
                        "state": state,
                        "reason": reason,
                    }

                if args.strategy == "corridor_servo" and stable and action.get("action") != "STOP":
                    corr = action.get("corridor") or {}
                    cur_x_err = corr.get("corridor_x_err")
                    closeness = corr.get("closeness", 0.0)
                    if cur_x_err is not None and last_sent_corridor_x_err is not None:
                        same_side = (cur_x_err == 0) or (last_sent_corridor_x_err == 0) or (
                            cur_x_err > 0 and last_sent_corridor_x_err > 0) or (
                            cur_x_err < 0 and last_sent_corridor_x_err < 0)
                        growth = abs(cur_x_err) - abs(last_sent_corridor_x_err)
                        if (same_side and growth >= args.corridor_diverge_stop_px and
                            closeness >= args.corridor_diverge_min_closeness):
                            action = {
                                "state": "CORRIDOR_DIVERGING",
                                "action": "STOP",
                                "cmd": "STOP",
                                "step": 0.0,
                                "servo": SERVO_CENTER,
                                "reason": "corridor_error_increased_after_motion",
                                "corridor": corr,
                                "binding": {
                                    "prev_corridor_x_err": round(last_sent_corridor_x_err, 2),
                                    "corridor_x_err": round(cur_x_err, 2),
                                    "corridor_x_err_growth": round(growth, 2),
                                    "distance_cm": 0.0,
                                    "distance_reason": "corridor_diverge_stop",
                                    "closeness": closeness,
                                    "gear": 1,
                                    "gear_reason": "stopped",
                                },
                            }
                            cmd = action["cmd"]
                            step = action["step"]
                            state = action["state"]
                            reason = action["reason"]

                rel_state = slot_relative_state(fused, args, metrics)
                fusion_pose = None
                if pose_fuser is not None and stable:
                    try:
                        fusion_pose = pose_fuser.anchor_vision(rel_state)
                    except Exception as exc:
                        fusion_pose = {"schema": "fused_pose_error.v1", "error": str(exc)}
                parking_criteria = evaluate_parking_criteria(rel_state, success_criteria, steps, total_cm)
                if (
                    stable and
                    args.strategy == "action_replanner" and
                    parking_criteria.get("verdict") == "aborted" and
                    parking_criteria.get("reason") == "min_margin_below_floor"
                ):
                    recovery = planner_edge_recovery_context(
                        planner_flatten_slot_state(rel_state, stable), success_criteria)
                    if recovery.get("active"):
                        parking_criteria = dict(parking_criteria)
                        parking_criteria["verdict"] = "edge_recovery"
                        parking_criteria["reason"] = "edge_recovery_required"
                        parking_criteria["exit_code"] = None
                        parking_criteria["edge_recovery"] = recovery
                counter_steer_decision = None
                if stable and args.strategy == "action_replanner" and args.counter_steer_enable:
                    counter_steer_decision = counter_steer_decision_from_state(
                        planner_flatten_slot_state(rel_state, stable), args, chassis_kinematics)
                    log_event(args.log_jsonl, {
                        "event": "counter_steer_decision",
                        "dry_run": args.dry_run,
                        "no_motion_mode": no_motion,
                        "stable": stable,
                        "parking_criteria": parking_criteria,
                        "decision": counter_steer_decision,
                        "candidate_cmd": counter_steer_decision.get("candidate_cmd"),
                        "send_to_stm32": False,
                        "motion_enabled": False,
                        "actuator_control_allowed": False,
                    })
                criteria_exit_code = parking_criteria.get("exit_code")
                if stable and parking_criteria.get("verdict") in ("parked", "aborted"):
                    verdict = parking_criteria["verdict"]
                    state = "PARKED_BY_CRITERIA" if verdict == "parked" else "ABORT_BY_CRITERIA"
                    reason = parking_criteria.get("reason", verdict)
                    cmd = "STOP"
                    step = 0.0
                    action = dict(action)
                    action.update({
                        "state": state,
                        "action": "STOP",
                        "cmd": cmd,
                        "step": step,
                        "servo": SERVO_CENTER,
                        "reason": reason,
                        "parking_criteria": parking_criteria,
                    })
                elif args.strategy == "action_replanner":
                    if counter_steer_decision and counter_steer_decision.get("verdict") == "counter_steer":
                        action = action_from_counter_steer(counter_steer_decision)
                    else:
                        action = action_replanner_command(
                            rel_state, stable, args, planner_library, planner_responses,
                            success_criteria, real_motion=not no_motion,
                            last_action_id=last_replanner_action_id)
                    cmd = action["cmd"]
                    step = action["step"]
                    state = action["state"]
                    reason = action["reason"]

                cap_steps = steps if (not no_motion or args.dry_run_simulate_motion) else 0
                cap_total = total_cm if (not no_motion or args.dry_run_simulate_motion) else 0.0
                max_steps = min(MAX_STEPS, args.max_motion_steps) if args.max_motion_steps > 0 else MAX_STEPS
                max_total = min(MAX_TOTAL_CM, args.max_total_cm) if args.max_total_cm > 0 else MAX_TOTAL_CM
                cap_would_stop = cap_steps >= max_steps or cap_total + step > max_total
                cur_lat = abs(p["lat"])
                lateral_would_stop = best_lat is not None and cur_lat > best_lat + LATERAL_DIV_CM
                lateral_recovery = {"active": False, "reason": "not_checked"}
                if args.strategy in ("normalized_corridor_servo", "path_template_planner"):
                    # Ground-frame lateral cm is not reliable across different slot sizes
                    # and close-range perspective changes. For image-space controllers,
                    # rely on normalized margins, vision loss, and hard caps instead.
                    lateral_would_stop = False
                elif args.strategy == "action_replanner" and lateral_would_stop:
                    lateral_recovery = replanner_lateral_recovery_context(action, cur_lat, args)
                    if lateral_recovery.get("active"):
                        lateral_would_stop = False
                hold_action = cmd == "WAIT" or action.get("action") == "WAIT"
                stop_action = cmd == "STOP" or action.get("action") == "STOP"
                motion_gate_open = bool(armed and stable and not no_motion)
                will_execute_motion = bool(
                    motion_gate_open and
                    action.get("action") in ("MOVE", "ARC") and
                    not stop_action and
                    not hold_action and
                    not p["aligned"] and
                    not lateral_would_stop and
                    not cap_would_stop
                )

                event = {
                    "event": "candidate",
                    "dry_run": args.dry_run,
                    "no_motion_mode": no_motion,
                    "replanner_dry_run": args.replanner_dry_run,
                    "stable": stable,
                    "stability": metrics,
                    "state": state,
                    "reason": reason,
                    "confidence": round(fused["confidence"], 4),
                    "center_cm": [round(p["target"][0], 2), round(p["target"][1], 2)],
                    "slot_center_cm": [round(fused["center_cm"][0], 2), round(fused["center_cm"][1], 2)],
                    "axis_yaw_deg": round(fused["axis_yaw_deg"], 2),
                    "lon": round(p["lon"], 2),
                    "lat": round(p["lat"], 2),
                    "head": round(p["head"], 2),
                    "pixel": action.get("pixel"),
                    "corridor": action.get("corridor"),
                    "slot_relative_state": rel_state,
                    "fusion_pose": fusion_pose,
                    "parking_criteria": parking_criteria,
                    "verdict": parking_criteria.get("verdict"),
                    "binding": action.get("binding"),
                    "counter_steer": action.get("counter_steer"),
                    "path_plan": action.get("path_plan"),
                    "slot_polygon_px": fused.get("mask_polygon_px"),
                    "slot_edges_px": {
                        "entrance": fused.get("entrance_edge_px"),
                        "back": fused.get("back_edge_px"),
                        "left": fused.get("left_edge_px"),
                        "right": fused.get("right_edge_px"),
                    },
                    "target_selection": fused.get("target_selection"),
                    "candidate_cmd": cmd,
                    "motion_gate_open": motion_gate_open,
                    "cap_would_stop": cap_would_stop,
                    "lateral_would_stop": lateral_would_stop,
                    "lateral_recovery": lateral_recovery,
                    "will_execute_motion": will_execute_motion,
                    "send_to_stm32": will_execute_motion,
                    "motion_enabled": will_execute_motion,
                    "actuator_control_allowed": will_execute_motion,
                }
                replanner = action.get("replanner")
                if replanner:
                    event["ranking"] = replanner.get("ranking")
                    event["chosen"] = replanner.get("chosen")
                    event["replanner_gates"] = replanner.get("gates")
                log_event(args.log_jsonl, event)
                if replanner:
                    replanner_gates = dict(replanner.get("gates") or {})
                    replanner_gates.update({
                        "motion_gate_open": motion_gate_open,
                        "will_execute_motion": will_execute_motion,
                        "cap_would_stop": cap_would_stop,
                        "lateral_would_stop": lateral_would_stop,
                        "lateral_recovery": lateral_recovery,
                        "dry_run": args.dry_run,
                        "replanner_dry_run": args.replanner_dry_run,
                    })
                    log_event(args.log_jsonl, {
                        "event": "replanner_step",
                        "step": steps + 1,
                        "pre_state": replanner.get("pre_state"),
                        "ranking": replanner.get("ranking"),
                        "chosen": replanner.get("chosen"),
                        "gates": replanner_gates,
                        "stm32": {
                            "sent": "",
                            "ack": "",
                            "done": "",
                            "pwm_stat": "",
                            "stat_after": "",
                        },
                        "post_state": {},
                        "delta": {},
                        "verdict": parking_criteria.get("verdict", "unknown"),
                        "parking_criteria": parking_criteria,
                        "totals": {
                            "steps_done": steps,
                            "total_cm": round(total_cm, 2),
                        },
                    })

                print("VIS stable=%s frames=%d lon=%.1f lat=%.1f head=%.1f state=%s -> %s" % (
                    stable, metrics.get("stable_frames", 0), p["lon"], p["lat"], p["head"], state, cmd), flush=True)
                if not stable:
                    print("  WAIT=UNSTABLE %s" % json.dumps(metrics, separators=(",", ":")), flush=True)
                    continue
                if args.strategy in ("pixel_servo", "corridor_servo", "normalized_corridor_servo", "path_template_planner"):
                    last_stable_pixel_action = {
                        "cmd": cmd,
                        "step": step,
                        "servo": action.get("servo", SERVO_CENTER),
                        "lon": p["lon"],
                        "lat": p["lat"],
                        "head": p["head"],
                        "pixel": action.get("pixel"),
                        "corridor": action.get("corridor"),
                        "binding": action.get("binding"),
                        "path_plan": action.get("path_plan"),
                    }
                if hold_action:
                    print("  WAIT=%s %s" % (state, reason), flush=True)
                    time.sleep(args.settle_sec)
                    continue
                if stop_action:
                    print("STOP=%s %s." % (state, reason), flush=True)
                    if not no_motion:
                        stop()
                    return int(criteria_exit_code) if criteria_exit_code is not None else 0
                if p["aligned"]:
                    print("STOP=ALIGNED parked.", flush=True)
                    if not no_motion:
                        stop()
                    return 0
                if lateral_would_stop:
                    print("STOP=DIVERGING lateral %.1f->%.1f." % (best_lat, cur_lat), flush=True)
                    if not no_motion:
                        stop()
                    return 5
                best_lat = cur_lat if best_lat is None else min(best_lat, cur_lat)
                if cap_would_stop:
                    print("STOP=CAP steps/total.", flush=True)
                    if not no_motion:
                        stop()
                    return 0
                anchor = {"lon": p["lon"], "lat": p["lat"]}
                ds_anchor = 0.0
                odom_progress_cm = None
                if no_motion:
                    print("  [no-motion] would send: %s" % cmd, flush=True)
                else:
                    if args.confirm_each_step:
                        print("CONFIRM action=%s cmd=%s ; type y then Enter to execute, anything else to stop:" % (
                            action.get("replanner", {}).get("chosen", {}).get("action_id", action.get("action")),
                            cmd), flush=True)
                        answer = sys.stdin.readline().strip().lower()
                        if answer != "y":
                            print("STOP=CONFIRM_REJECTED.", flush=True)
                            stop()
                            return 7
                    st = read_stat()
                    anchor["yaw"] = st["yaw"] if st["yaw"] is not None else 0.0
                    pwm_before = query_pwm_stat() if args.log_stm32_detail else ""
                    pre_servo_resp = ""
                    pwm_after_pre_servo = ""
                    telemetry_on_resp = ""
                    telemetry_off_resp = ""
                    if (args.pre_steer_settle_sec > 0.0 and
                        action.get("action") == "ARC" and
                        action.get("servo") is not None):
                        pre_servo_resp = send_cmd("SERVO A=%d" % int(round(action["servo"])), read_sec=2.0).strip()
                        time.sleep(args.pre_steer_settle_sec)
                        if args.log_stm32_detail:
                            pwm_after_pre_servo = query_pwm_stat()
                    if args.motion_telemetry:
                        telemetry_on_resp = send_cmd("TEL ON", read_sec=2.0).strip()
                    motion_resp = send_cmd(cmd, read_sec=args.move_read_sec).strip()
                    if args.motion_telemetry:
                        telemetry_off_resp = send_cmd("TEL OFF", read_sec=2.0).strip()
                    pre_servo_events = parse_stm32_events(pre_servo_resp)
                    telemetry_on_events = parse_stm32_events(telemetry_on_resp)
                    motion_events = parse_stm32_events(motion_resp)
                    telemetry_off_events = parse_stm32_events(telemetry_off_resp)
                    fusion_motion_trace = []
                    fusion_motion_final = None
                    if pose_fuser is not None and stable:
                        try:
                            pose_fuser.anchor_vision(rel_state)
                            for ev in motion_events:
                                if ev.get("type") == "tlm":
                                    fusion_motion_trace.append(pose_fuser.ingest_tlm(ev))
                            fusion_motion_final = pose_fuser.snapshot()
                        except Exception as exc:
                            fusion_motion_final = {"schema": "fused_pose_error.v1", "error": str(exc)}
                    pwm_after = query_pwm_stat() if args.log_stm32_detail else ""
                    st_after = read_stat()
                    odom_progress_cm = None
                    if st_after.get("d") is not None:
                        candidate_progress = abs(float(st_after["d"]))
                        if 0.0 <= candidate_progress <= max(1.0, step * 1.8):
                            odom_progress_cm = candidate_progress
                    final_blind_token_event = None
                    if args.strategy == "action_replanner" and replanner and action.get("action") in ("MOVE", "ARC"):
                        chosen = replanner.get("chosen") or {}
                        pre_state = replanner.get("pre_state") or {}
                        pre_state_review = final_blind_pre_state_review(pre_state, args)
                        progress_for_token = odom_progress_cm if odom_progress_cm is not None else step
                        final_blind_token_event = {
                            "enabled": bool(args.allow_final_blind_reverse),
                            "path": args.final_blind_token,
                            "pre_state_review": pre_state_review,
                            "written": False,
                        }
                        if args.allow_final_blind_reverse and pre_state_review.get("pass"):
                            token = {
                                "schema": "parking_final_blind_token.v1",
                                "time_unix": time.time(),
                                "consumed": False,
                                "reason": "recent_stable_visual_action_entered_terminal_blind_zone",
                                "source_log_jsonl": args.log_jsonl,
                                "pre_state": pre_state,
                                "pre_state_review": pre_state_review,
                                "yaw_token": st_after.get("yaw"),
                                "chosen": chosen,
                                "motion": {
                                    "candidate_cmd": cmd,
                                    "commanded_step_cm": round(step, 3),
                                    "odom_progress_cm": None if odom_progress_cm is None else round(odom_progress_cm, 3),
                                    "stat_before": st.get("raw", ""),
                                    "stat_after": st_after.get("raw", ""),
                                },
                                "totals_after_action": {
                                    "steps_done": steps + 1,
                                    "total_cm": round(total_cm + progress_for_token, 3),
                                },
                            }
                            try:
                                write_final_blind_token(args.final_blind_token, token)
                                final_blind_token_event["written"] = True
                            except OSError as exc:
                                final_blind_token_event["error"] = str(exc)
                    counter_steer_result = None
                    if action.get("counter_steer"):
                        counter_steer_result = counter_steer_result_from_motion(
                            action.get("counter_steer"), st.get("raw", ""), st_after.get("raw", ""), args)
                        log_event(args.log_jsonl, {
                            "event": "counter_steer_result",
                            "candidate_cmd": cmd,
                            "counter_steer_result": counter_steer_result,
                            "stat_before": st.get("raw", ""),
                            "stat_after": st_after.get("raw", ""),
                        })
                    log_event(args.log_jsonl, {
                        "event": "stm32_motion_result",
                        "candidate_cmd": cmd,
                        "pre_steer_settle_sec": args.pre_steer_settle_sec,
                        "pre_servo_response": pre_servo_resp,
                        "pre_servo_events": pre_servo_events,
                        "telemetry_on_response": telemetry_on_resp,
                        "telemetry_on_events": telemetry_on_events,
                        "motion_response": motion_resp,
                        "motion_events": motion_events,
                        "telemetry_off_response": telemetry_off_resp,
                        "telemetry_off_events": telemetry_off_events,
                        "fusion_motion_trace": fusion_motion_trace,
                        "fusion_motion_final": fusion_motion_final,
                        "stat_before": st["raw"],
                        "pwm_before": pwm_before,
                        "pwm_after_pre_servo": pwm_after_pre_servo,
                        "pwm_after": pwm_after,
                        "stat_after": st_after.get("raw", ""),
                        "commanded_step_cm": round(step, 3),
                        "odom_progress_cm": None if odom_progress_cm is None else round(odom_progress_cm, 3),
                        "odom_d_before_cm": None if st.get("d") is None else round(float(st["d"]), 3),
                        "odom_d_after_cm": None if st_after.get("d") is None else round(float(st_after["d"]), 3),
                        "final_blind_token": final_blind_token_event,
                        "counter_steer_result": counter_steer_result,
                    })
                if args.strategy == "corridor_servo" and action.get("action") != "STOP":
                    corr = action.get("corridor") or {}
                    cur_x_err = corr.get("corridor_x_err")
                    if cur_x_err is not None and (not no_motion or args.dry_run_simulate_motion):
                        last_sent_corridor_x_err = cur_x_err
                if args.strategy == "action_replanner":
                    chosen_id = action.get("replanner", {}).get("chosen", {}).get("action_id")
                    if chosen_id and chosen_id not in ("WAIT", "STOP"):
                        last_replanner_action_id = chosen_id
                if not no_motion or args.dry_run_simulate_motion:
                    steps += 1
                    progress_cm = step
                    if not no_motion and odom_progress_cm is not None:
                        progress_cm = odom_progress_cm
                    total_cm += progress_cm
                    ds_anchor += progress_cm
                time.sleep(args.settle_sec)
                continue

            # vision lost
            now = time.time()
            if vision_lost_since is None:
                vision_lost_since = now
            lost_elapsed = now - vision_lost_since
            coasted, hold_metrics = filt.tick_no_detection()
            if coasted is not None:
                log_event(args.log_jsonl, {
                    "event": "perception_hold_wait",
                    "dry_run": args.dry_run,
                    "lost_elapsed_sec": round(lost_elapsed, 3),
                    "hold": hold_metrics,
                    "coasted_state": slot_relative_state(coasted, args, coasted.get("stability")),
                    "send_to_stm32": False,
                    "motion_enabled": False,
                    "actuator_control_allowed": False,
                    "steps": steps,
                    "total_cm": round(total_cm, 2),
                })
                print("WAIT=PERCEPTION_HOLD %s." % hold_metrics.get("reason"), flush=True)
                time.sleep(args.settle_sec)
                continue
            log_event(args.log_jsonl, {
                "event": "vision_lost",
                "dry_run": args.dry_run,
                "lost_elapsed_sec": round(lost_elapsed, 3),
                "hold": hold_metrics,
                "send_to_stm32": False,
                "motion_enabled": False,
                "actuator_control_allowed": False,
                "steps": steps,
                "total_cm": round(total_cm, 2),
            })
            if args.strategy == "action_replanner" and args.allow_final_blind_reverse:
                if lost_elapsed < args.final_blind_vision_lost_sec:
                    print("WAIT=FINAL_BLIND_VISION_LOST %.2fs/%.2fs." % (
                        lost_elapsed, args.final_blind_vision_lost_sec), flush=True)
                    time.sleep(args.settle_sec)
                    continue
                token = read_final_blind_token(args.final_blind_token)
                token_review = final_blind_token_review(token, args)
                max_steps = min(MAX_STEPS, args.max_motion_steps) if args.max_motion_steps > 0 else MAX_STEPS
                max_total = min(MAX_TOTAL_CM, args.max_total_cm) if args.max_total_cm > 0 else MAX_TOTAL_CM
                remaining_total = max(0.0, max_total - total_cm)
                final_blind_plan = final_blind_plan_from_token(token, args, remaining_total)
                step = final_blind_plan["command_cm"]
                can_execute_final_blind = (
                    token_review.get("pass") and
                    not final_blind_used and
                    step >= args.final_blind_min_command_cm and
                    steps < max_steps
                )
                cmd = final_blind_plan["cmd"]
                log_event(args.log_jsonl, {
                    "event": "final_blind_reverse_candidate",
                    "dry_run": args.dry_run,
                    "no_motion_mode": no_motion,
                    "lost_elapsed_sec": round(lost_elapsed, 3),
                    "token_path": args.final_blind_token,
                    "token_review": token_review,
                    "final_blind_plan": final_blind_plan,
                    "candidate_cmd": cmd,
                    "step": step,
                    "final_blind_used": final_blind_used,
                    "steps": steps,
                    "max_steps": max_steps,
                    "total_cm": round(total_cm, 2),
                    "remaining_total_cm": round(remaining_total, 2),
                    "send_to_stm32": bool(can_execute_final_blind and armed and not no_motion),
                    "motion_enabled": bool(can_execute_final_blind and armed and not no_motion),
                    "actuator_control_allowed": bool(can_execute_final_blind and armed and not no_motion),
                })
                if not can_execute_final_blind:
                    if not token_review.get("pass") and lost_elapsed < args.replanner_vision_lost_stop_sec:
                        log_event(args.log_jsonl, {
                            "event": "replanner_vision_reacquire_wait",
                            "dry_run": args.dry_run,
                            "no_motion_mode": no_motion,
                            "lost_elapsed_sec": round(lost_elapsed, 3),
                            "stop_after_sec": args.replanner_vision_lost_stop_sec,
                            "token_review": token_review,
                            "send_to_stm32": False,
                            "motion_enabled": False,
                            "actuator_control_allowed": False,
                        })
                        print("WAIT=REPLANNER_VISION_REACQUIRE %.2fs/%.2fs." % (
                            lost_elapsed, args.replanner_vision_lost_stop_sec), flush=True)
                        time.sleep(args.settle_sec)
                        continue
                    print("STOP=FINAL_BLIND_GATE_CLOSED.", flush=True)
                    if no_motion and deadline is not None:
                        time.sleep(args.settle_sec)
                        continue
                    if not no_motion:
                        stop()
                    return 0
                print("FINAL_BLIND_REVERSE -> %s" % cmd, flush=True)
                final_blind_used = True
                if no_motion:
                    print("  [no-motion] would send final blind reverse: %s" % cmd, flush=True)
                else:
                    st = read_stat()
                    telemetry_on_resp = ""
                    telemetry_off_resp = ""
                    if args.motion_telemetry:
                        telemetry_on_resp = send_cmd("TEL ON", read_sec=2.0).strip()
                    motion_resp = send_cmd(cmd, read_sec=args.move_read_sec).strip()
                    if args.motion_telemetry:
                        telemetry_off_resp = send_cmd("TEL OFF", read_sec=2.0).strip()
                    motion_events = parse_stm32_events(motion_resp)
                    st_after = read_stat()
                    progress_cm = step
                    if st_after.get("d") is not None:
                        candidate_progress = abs(float(st_after["d"]))
                        if 0.0 <= candidate_progress <= max(1.0, step * 1.8):
                            progress_cm = candidate_progress
                    final_pose_report = final_pose_report_from_token(
                        token, st.get("raw", ""), st_after.get("raw", ""), args)
                    log_event(args.log_jsonl, {
                        "event": "final_blind_reverse_result",
                        "candidate_cmd": cmd,
                        "final_blind_plan": final_blind_plan,
                        "final_pose_report": final_pose_report,
                        "telemetry_on_response": telemetry_on_resp,
                        "telemetry_on_events": parse_stm32_events(telemetry_on_resp),
                        "motion_response": motion_resp,
                        "motion_events": motion_events,
                        "telemetry_off_response": telemetry_off_resp,
                        "telemetry_off_events": parse_stm32_events(telemetry_off_resp),
                        "stat_before": st.get("raw", ""),
                        "stat_after": st_after.get("raw", ""),
                        "commanded_step_cm": round(step, 3),
                        "odom_progress_cm": round(progress_cm, 3),
                    })
                    consume_final_blind_token(args.final_blind_token, token, {
                        "dry_run": False,
                        "candidate_cmd": cmd,
                        "step": step,
                        "final_blind_plan": final_blind_plan,
                        "final_pose_report": final_pose_report,
                        "stat_before": st.get("raw", ""),
                        "stat_after": st_after.get("raw", ""),
                    })
                    log_event(args.log_jsonl, {
                        "event": "final_pose_report",
                        "source": "final_blind_reverse_result",
                        "candidate_cmd": cmd,
                        "final_pose_report": final_pose_report,
                        "stat_before": st.get("raw", ""),
                        "stat_after": st_after.get("raw", ""),
                    })
                    stop()
                    total_cm += progress_cm
                    steps += 1
                print("STOP=FINAL_BLIND_REVERSE_DONE.", flush=True)
                return 0
            if (anchor is None or "yaw" not in anchor) and not (
                args.strategy == "primitive_probe" and args.primitive_no_vision):
                print("STOP=NO_TARGET (no slot / no anchor).", flush=True)
                if no_motion and deadline is not None:
                    time.sleep(args.settle_sec)
                    continue
                if not no_motion:
                    stop()
                return 0
            if args.strategy in ("pixel_servo", "corridor_servo", "normalized_corridor_servo"):
                if lost_elapsed < args.pixel_vision_lost_stop_sec:
                    print("WAIT=PIXEL_VISION_LOST %.2fs/%.2fs." % (
                        lost_elapsed, args.pixel_vision_lost_stop_sec), flush=True)
                    time.sleep(args.settle_sec)
                    continue
                if (args.strategy == "pixel_servo" and
                    args.pixel_blind_finish_cm > 0.0 and not pixel_blind_finish_used and
                    last_stable_pixel_action is not None and
                    steps >= args.pixel_blind_finish_min_steps and
                    last_stable_pixel_action["lon"] <= args.pixel_blind_finish_max_lon_cm):
                    servo = int(round(last_stable_pixel_action.get("servo", SERVO_CENTER)))
                    if abs(servo - SERVO_CENTER) > args.pixel_blind_finish_max_steer_offset_deg:
                        print("STOP=PIXEL_BLIND_FINISH_STEER_CAP.", flush=True)
                        if not no_motion:
                            stop()
                        return 0
                    max_total = min(MAX_TOTAL_CM, args.max_total_cm) if args.max_total_cm > 0 else MAX_TOTAL_CM
                    remaining_cap = max(0.0, max_total - total_cm)
                    step = round(min(args.pixel_blind_finish_cm, remaining_cap, args.pixel_max_command_abs_d_cm), 1)
                    if step < args.pixel_min_command_abs_d_cm:
                        print("STOP=PIXEL_BLIND_FINISH_CAP.", flush=True)
                        if not no_motion:
                            stop()
                        return 0
                    if abs(servo - SERVO_CENTER) <= STEER_DEADZONE_DEG:
                        cmd = "MOVE D=%.1f V=%d" % (-step, 1)
                    else:
                        cmd = "ARC D=%.1f STE=%d V=%d" % (-step, servo, 1)
                    log_event(args.log_jsonl, {
                        "event": "pixel_blind_finish",
                        "dry_run": args.dry_run,
                        "reason": "vision_lost_after_near_stable_detection",
                        "last_stable": last_stable_pixel_action,
                        "candidate_cmd": cmd,
                        "step": step,
                        "send_to_stm32": False if no_motion else bool(armed),
                        "motion_enabled": False if no_motion else bool(armed),
                        "actuator_control_allowed": False if no_motion else bool(armed),
                        "steps": steps,
                        "total_cm": round(total_cm, 2),
                    })
                    print("PIXEL_BLIND_FINISH -> %s" % cmd, flush=True)
                    pixel_blind_finish_used = True
                    if no_motion:
                        print("  [no-motion] would send blind finish: %s" % cmd, flush=True)
                    else:
                        send_cmd(cmd, read_sec=args.move_read_sec)
                        stop()
                    steps += 1
                    total_cm += step
                    print("STOP=PIXEL_BLIND_FINISH_DONE.", flush=True)
                    return 0
                print("STOP=PIXEL_VISION_LOST.", flush=True)
                if no_motion and deadline is not None:
                    time.sleep(args.settle_sec)
                    continue
                if not no_motion:
                    stop()
                return 0
            max_steps = min(MAX_STEPS, args.max_motion_steps) if args.max_motion_steps > 0 else MAX_STEPS
            max_total = min(MAX_TOTAL_CM, args.max_total_cm) if args.max_total_cm > 0 else MAX_TOTAL_CM
            if steps >= max_steps or total_cm >= max_total:
                print("STOP=CAP_BEFORE_DEADRECKON steps/total.", flush=True)
                if not no_motion:
                    stop()
                return 0
            if args.strategy == "primitive_probe":
                if args.primitive_no_vision:
                    action = primitive_probe_command(None, args)
                    cmd = action["cmd"]
                    step = action["step"]
                    cap_would_stop = steps >= max_steps or total_cm + step > max_total
                    motion_gate_open = bool(armed and not no_motion)
                    will_execute_motion = bool(
                        motion_gate_open and
                        action.get("action") in ("MOVE", "ARC") and
                        not cap_would_stop
                    )
                    log_event(args.log_jsonl, {
                        "event": "primitive_no_vision_candidate",
                        "dry_run": args.dry_run,
                        "no_motion_mode": no_motion,
                        "state": action["state"],
                        "reason": "fixed_calibration_primitive_no_vision",
                        "candidate_cmd": cmd,
                        "binding": action.get("binding"),
                        "motion_gate_open": motion_gate_open,
                        "cap_would_stop": cap_would_stop,
                        "will_execute_motion": will_execute_motion,
                        "send_to_stm32": will_execute_motion,
                        "motion_enabled": will_execute_motion,
                        "actuator_control_allowed": will_execute_motion,
                    })
                    print("PRIMITIVE_NO_VISION -> %s" % cmd, flush=True)
                    if cap_would_stop:
                        print("STOP=CAP primitive_no_vision.", flush=True)
                        if not no_motion:
                            stop()
                        return 0
                    if no_motion:
                        print("  [no-motion] would send primitive no-vision: %s" % cmd, flush=True)
                    elif will_execute_motion:
                        execute_logged_motion(action, args, pose_fuser)
                    else:
                        print("STOP=NOT_ARMED primitive_no_vision.", flush=True)
                        return 7
                    steps += 1
                    total_cm += step
                    print("STOP=PRIMITIVE_NO_VISION_DONE.", flush=True)
                    return 0
                print("STOP=PRIMITIVE_PROBE_VISION_LOST.", flush=True)
                if no_motion and deadline is not None:
                    time.sleep(args.settle_sec)
                    continue
                if not no_motion:
                    stop()
                return 0
            if not args.allow_dead_reckon_after_loss:
                print("STOP=VISION_LOST_DEADRECKON_DISABLED.", flush=True)
                if no_motion and deadline is not None:
                    time.sleep(args.settle_sec)
                    continue
                if not no_motion:
                    stop()
                return 0
            if no_motion:
                print("  [no-motion] vision lost, would dead-reckon.", flush=True)
                if deadline is not None:
                    time.sleep(args.settle_sec)
                    continue
                return 0
            cur = read_stat()
            if cur["yaw"] is None:
                print("STOP=NO_STAT for dead-reckon.", flush=True)
                stop()
                return 0
            lon_p, lat_p = predict_slot(anchor, cur["yaw"], ds_anchor)
            print("LOST->DR lon=%.1f lat=%.1f (yaw %.1f vs %.1f, ds=%.1f)" % (
                lon_p, lat_p, cur["yaw"], anchor["yaw"], ds_anchor), flush=True)
            if lon_p <= LON_TOL_CM:
                print("STOP=PARKED (dead-reckoned).", flush=True)
                stop()
                return 0
            if blind_cm >= MAX_BLIND_CM:
                print("STOP=DEADRECKON_CAP.", flush=True)
                stop()
                return 0
            cmd, _servo, step_g = pursuit_command(lon_p, lat_p)
            remaining_total = max_total - total_cm
            remaining_blind = MAX_BLIND_CM - blind_cm
            step_g = min(step_g, remaining_total, remaining_blind)
            if step_g <= 0.0:
                print("STOP=DEADRECKON_STEP_CAP.", flush=True)
                stop()
                return 0
            if "ARC " in cmd:
                cmd = "ARC D=%.1f STE=%d V=%d" % (-step_g, int(round(_servo)), GEAR)
            else:
                cmd = "MOVE D=%.1f V=%d" % (-step_g, GEAR)
            print("  DR step %.1fcm: %s" % (step_g, cmd), flush=True)
            send_cmd(cmd, read_sec=args.move_read_sec)
            blind_cm += step_g
            ds_anchor += step_g
            steps += 1
            total_cm += step_g
            time.sleep(args.settle_sec)
    except KeyboardInterrupt:
        print("\nABORT (Ctrl-C) -> STOP", flush=True)
        if not no_motion:
            stop()
        return 130


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--listen-host", default="127.0.0.1")
    ap.add_argument("--listen-port", type=int, default=24580)
    ap.add_argument("--dry-run", action="store_true", help="compute & print only, never send motion")
    ap.add_argument("--arm", action="store_true", help="allow motion (also needs the arm file)")
    ap.add_argument("--arm-file", default="/tmp/parking_armed")
    ap.add_argument("--feedback-tune", action="store_true",
                    help="run bounded step-by-step feedback tuning instead of normal parking control")
    ap.add_argument("--learn-policy", action="store_true",
                    help="learn a persistent state-action policy from +/-/restart feedback")
    ap.add_argument("--target-wait-sec", type=float, default=6.0)
    ap.add_argument("--settle-sec", type=float, default=0.6)
    ap.add_argument("--move-read-sec", type=float, default=6.0)
    ap.add_argument("--pre-steer-settle-sec", type=float, default=0.0,
                    help="before a real ARC command, optionally send SERVO A=<steer> and wait this long")
    ap.add_argument("--log-stm32-detail", action="store_true",
                    help="log STAT/PWM_STAT and STM32 command responses around each real motion command")
    ap.add_argument("--motion-telemetry", action="store_true",
                    help="wrap each real motion with TEL ON/OFF and parse TLM rows into fusion_motion_trace")
    ap.add_argument("--final-stop-on-exit", action=argparse.BooleanOptionalAction, default=True,
                    help="for authorized real runs, send one final STOP before process exit")
    ap.add_argument("--duration-sec", type=float, default=0.0, help="0 = run until stopped; useful for bounded dry-run capture")
    ap.add_argument("--log-jsonl", default="", help="write structured dry-run/control events to this JSONL file")
    ap.add_argument("--max-motion-steps", type=int, default=0,
                    help="0 uses the built-in MAX_STEPS; positive value further limits real/simulated motion steps")
    ap.add_argument("--max-total-cm", type=float, default=0.0,
                    help="0 uses the built-in MAX_TOTAL_CM; positive value further limits real/simulated commanded distance")
    ap.add_argument("--allow-dead-reckon-after-loss", action="store_true",
                    help="allow IMU/odometry dead-reckon continuation after vision is lost; default stops")
    ap.add_argument("--allow-final-blind-reverse", action="store_true",
                    help="allow one token-gated terminal straight reverse after action_replanner vision loss")
    ap.add_argument("--final-blind-token", default="/tmp/parking_final_blind_token.json",
                    help="one-shot token path written after a safe visible action and consumed in terminal blind zone")
    ap.add_argument("--final-blind-token-max-age-sec", type=float, default=180.0,
                    help="maximum age of final-blind token before it is rejected")
    ap.add_argument("--final-blind-vision-lost-sec", type=float, default=0.5,
                    help="wait this long after YOLO loss before consuming the final blind token")
    ap.add_argument("--replanner-vision-lost-stop-sec", type=float, default=3.0,
                    help="for action_replanner, wait this long for YOLO reacquire when no valid final-blind token exists")
    ap.add_argument("--final-blind-reverse-cm", type=float, default=0.0,
                    help="desired actual reverse distance for the one-shot final blind command; 0 disables execution")
    ap.add_argument("--final-blind-distance-mode", choices=["fixed", "dynamic"], default="fixed",
                    help="fixed uses --final-blind-reverse-cm; dynamic estimates remaining distance from the token state")
    ap.add_argument("--final-blind-target-y-dist-cm", type=float, default=8.0,
                    help="dynamic mode target remaining slot_y_dist_cm")
    ap.add_argument("--final-blind-compensate-deadband", action="store_true",
                    help="command desired distance plus drivetrain deadband minus expected coast")
    ap.add_argument("--final-blind-deadband-cm", type=float, default=2.0,
                    help="terminal reverse command deadband compensation")
    ap.add_argument("--final-blind-coast-cm", type=float, default=1.0,
                    help="expected post-DONE coast to subtract from the deadband-compensated command")
    ap.add_argument("--final-blind-min-command-cm", type=float, default=1.0,
                    help="minimum one-shot final blind reverse command distance")
    ap.add_argument("--final-blind-max-command-cm", type=float, default=6.0,
                    help="maximum one-shot final blind reverse command distance")
    ap.add_argument("--final-blind-max-actual-cm", type=float, default=8.0,
                    help="maximum desired actual final blind travel before command compensation")
    ap.add_argument("--final-blind-max-x-err-px", type=float, default=25.0,
                    help="token write gate: maximum absolute slot x error in the last visible state")
    ap.add_argument("--final-blind-max-lateral-cm", type=float, default=1.5,
                    help="token write gate: maximum absolute rear-target lateral error in cm")
    ap.add_argument("--final-blind-max-terminal-lateral-cm", type=float, default=1.8,
                    help="token write gate: conservative predicted terminal lateral error cap")
    ap.add_argument("--final-blind-straight-heading-err-deg", type=float, default=2.0,
                    help="heading error below this uses straight final blind")
    ap.add_argument("--final-blind-max-heading-err-deg", type=float, default=5.0,
                    help="token write gate: maximum absolute heading error in the last visible state")
    ap.add_argument("--final-blind-allow-heading-arc", action="store_true",
                    help="allow one token-gated ARC to cancel small terminal heading error")
    ap.add_argument("--final-blind-arc-max-heading-err-deg", type=float, default=6.0,
                    help="maximum heading error for optional terminal heading-cancel ARC")
    ap.add_argument("--final-blind-arc-reff-cm", type=float, default=87.0,
                    help="effective turn radius for optional terminal heading-cancel ARC")
    ap.add_argument("--final-blind-arc-left-servo", type=float, default=60.0,
                    help="servo value used when terminal heading-cancel ARC turns left")
    ap.add_argument("--final-blind-arc-right-servo", type=float, default=105.0,
                    help="servo value used when terminal heading-cancel ARC turns right")
    ap.add_argument("--final-blind-min-margin-px", type=float, default=80.0,
                    help="token write gate: minimum line margin in the last visible state")
    ap.add_argument("--final-blind-max-y-dist-cm", type=float, default=35.0,
                    help="token write gate: maximum estimated remaining visible distance before blind zone")
    ap.add_argument("--success-criteria-json", default="/opt/parking/autopark/parking_success_criteria.json",
                    help="parking done/abort criteria JSON; missing file falls back to built-in safe defaults")
    ap.add_argument("--chassis-signs-json", default="/opt/parking/autopark/chassis_signs.json",
                    help="C0 sign configuration for future fusion/PoseFuser use")
    ap.add_argument("--chassis-kinematics-json", default="/opt/parking/autopark/chassis_kinematics.json",
                    help="measured steering curvature table used by dynamic terminal counter-steer")
    ap.add_argument("--require-fusion-signs", action="store_true",
                    help="refuse startup unless --chassis-signs-json exists and has no null sign fields")
    ap.add_argument("--final-pose-straight-heading-deg", type=float, default=2.0,
                    help="final_pose_report: maximum absolute final heading considered straight")
    ap.add_argument("--final-pose-lateral-cm", type=float, default=1.5,
                    help="final_pose_report: maximum absolute final lateral estimate considered in-slot")
    ap.add_argument("--strategy", choices=[
        "template",
        "pure_pursuit",
        "pixel_servo",
        "corridor_servo",
        "normalized_corridor_servo",
        "path_template_planner",
        "primitive_probe",
        "action_replanner",
    ], default="template")
    ap.add_argument("--replanner-dry-run", action="store_true",
                    help="for --strategy action_replanner, score and log only; never open serial or send motion")
    ap.add_argument("--confirm-each-step", action="store_true",
                    help="before each real action_replanner motion command, require stdin 'y'")
    ap.add_argument("--action-library-json", default="/opt/parking/autopark/parking_action_library.json",
                    help="action template library JSON for --strategy action_replanner")
    ap.add_argument("--response-model-json", default="/opt/parking/autopark/parking_action_response_model.json",
                    help="measured/prior action response model JSON for --strategy action_replanner")
    ap.add_argument("--replanner-switch-penalty", type=float, default=5.0,
                    help="score penalty for reversing steering direction relative to the previous replanner action")
    ap.add_argument("--replanner-hold-margin", type=float, default=3.0,
                    help="keep the previous replanner action if it remains eligible within this score margin")
    ap.add_argument("--replanner-allow-lateral-recovery", action=argparse.BooleanOptionalAction, default=True,
                    help="allow action_replanner to execute a predicted-safe ARC recovery instead of stopping on lateral divergence")
    ap.add_argument("--replanner-lateral-recovery-min-gain-cm", type=float, default=1.0,
                    help="minimum predicted lateral improvement required to override the lateral divergence stop")
    ap.add_argument("--replanner-lateral-recovery-min-margin-px", type=float, default=80.0,
                    help="minimum predicted line margin required to override the lateral divergence stop")
    ap.add_argument("--replanner-allow-prior-lateral-recovery", action=argparse.BooleanOptionalAction, default=True,
                    help="allow a safe predicted ARC recovery near the terminal zone even without an exact measured bucket")
    ap.add_argument("--replanner-prior-lateral-recovery-max-y-dist-cm", type=float, default=30.0,
                    help="only relax exact-measured ARC blocking when the visible terminal zone is this close")
    ap.add_argument("--replanner-prior-lateral-recovery-min-lateral-cm", type=float, default=1.8,
                    help="minimum current lateral error required before relaxing exact-measured ARC blocking")
    ap.add_argument("--replanner-prior-lateral-recovery-min-gain-cm", type=float, default=1.0,
                    help="minimum predicted lateral improvement required for prior-based terminal recovery")
    ap.add_argument("--replanner-prior-lateral-recovery-max-predicted-lateral-cm", type=float, default=1.0,
                    help="maximum predicted lateral error after a prior-based terminal recovery ARC")
    ap.add_argument("--replanner-prior-lateral-recovery-min-margin-px", type=float, default=120.0,
                    help="minimum predicted line margin for prior-based terminal recovery")
    ap.add_argument("--replanner-prior-lateral-recovery-score-bonus", type=float, default=8.0,
                    help="score bonus for a safe prior-based terminal lateral recovery ARC")
    ap.add_argument("--replanner-allow-terminal-countersteer", action="store_true",
                    help="allow selected ARC actions into straighten_or_enter candidate ranking")
    ap.add_argument("--replanner-terminal-countersteer-action-ids",
                    default="reverse_left_hard_6,reverse_right_soft_6",
                    help="comma-separated action IDs eligible for terminal visible counter-steer ranking")
    ap.add_argument("--replanner-terminal-countersteer-min-lateral-cm", type=float, default=1.5,
                    help="minimum absolute lateral error before terminal counter-steer candidates are enabled")
    ap.add_argument("--replanner-terminal-countersteer-min-heading-deg", type=float, default=2.0,
                    help="minimum absolute heading error before terminal counter-steer candidates are enabled")
    ap.add_argument("--counter-steer-enable", action="store_true",
                    help="enable dynamic measured-kinematics counter-steer in straighten_or_enter")
    ap.add_argument("--counter-steer-max-lateral-cm", type=float, default=1.5,
                    help="counter-steer gate: maximum absolute lateral error before blind-safe heading correction")
    ap.add_argument("--counter-steer-heading-enter-deg", type=float, default=2.0,
                    help="counter-steer gate: heading error at or below this proceeds to normal planner/token flow")
    ap.add_argument("--counter-steer-heading-stop-deg", type=float, default=6.0,
                    help="counter-steer gate: above this heading error stops instead of blind heading correction")
    ap.add_argument("--counter-steer-hard-heading-deg", type=float, default=3.0,
                    help="heading error threshold for selecting hard 60/120 arcs instead of soft 75/105 arcs")
    ap.add_argument("--counter-steer-left-hard-ste", type=float, default=60.0)
    ap.add_argument("--counter-steer-left-soft-ste", type=float, default=75.0)
    ap.add_argument("--counter-steer-right-hard-ste", type=float, default=120.0)
    ap.add_argument("--counter-steer-right-soft-ste", type=float, default=105.0)
    ap.add_argument("--counter-steer-arc-deadband-cm", type=float, default=2.0,
                    help="fallback ARC deadband when chassis_kinematics lacks arc_deadband_cm")
    ap.add_argument("--counter-steer-min-command-cm", type=float, default=4.0,
                    help="minimum ARC command distance for counter-steer")
    ap.add_argument("--counter-steer-max-command-cm", type=float, default=6.0,
                    help="maximum ARC command distance for counter-steer")
    ap.add_argument("--perception-filter-json", default="/opt/parking/autopark/perception_filter.json",
                    help="perception jitter/dropout filter configuration")
    ap.add_argument("--stable-frames", type=int, default=0,
                    help="override perception filter required_frames; 0 uses --perception-filter-json")
    ap.add_argument("--max-center-shift-cm", type=float, default=None,
                    help="override perception filter center-shift gate")
    ap.add_argument("--max-axis-yaw-shift-deg", type=float, default=None,
                    help="override perception filter yaw-shift gate")
    ap.add_argument("--lat-template-threshold-cm", type=float, default=DEFAULT_LAT_TEMPLATE_THRESHOLD_CM)
    ap.add_argument("--head-template-threshold-deg", type=float, default=DEFAULT_HEAD_TEMPLATE_THRESHOLD_DEG)
    ap.add_argument("--template-step-cm", type=float, default=5.0)
    ap.add_argument("--template-steer-deg", type=float, default=10.0)
    ap.add_argument("--pixel-step-cm", type=float, default=7.0)
    ap.add_argument("--pixel-max-command-abs-d-cm", type=float, default=40.0)
    ap.add_argument("--pixel-steer-deg", type=float, default=12.0)
    ap.add_argument("--pixel-kx", type=float, default=0.14,
                    help="servo offset degrees per pixel of x error")
    ap.add_argument("--pixel-ka", type=float, default=0.35,
                    help="servo offset degrees per degree of pixel angle error")
    ap.add_argument("--pixel-max-steer-offset-deg", type=float, default=24.0)
    ap.add_argument("--pixel-large-steer-offset-deg", type=float, default=18.0)
    ap.add_argument("--pixel-large-x-err-px", type=float, default=70.0)
    ap.add_argument("--pixel-min-command-abs-d-cm", type=float, default=8.0)
    ap.add_argument("--pixel-near-d-cm", type=float, default=10.0)
    ap.add_argument("--pixel-mid-d-cm", type=float, default=20.0)
    ap.add_argument("--pixel-far-d-cm", type=float, default=40.0)
    ap.add_argument("--pixel-near-ratio", type=float, default=0.9)
    ap.add_argument("--pixel-vision-lost-stop-sec", type=float, default=0.5)
    ap.add_argument("--pixel-blind-finish-cm", type=float, default=0.0,
                    help="one final capped reverse step after near-slot YOLO loss; 0 disables")
    ap.add_argument("--pixel-blind-finish-max-lon-cm", type=float, default=25.0,
                    help="allow blind finish only if last stable lon is at or below this")
    ap.add_argument("--pixel-blind-finish-min-steps", type=int, default=2,
                    help="allow blind finish only after at least this many visible closed-loop steps")
    ap.add_argument("--pixel-blind-finish-max-steer-offset-deg", type=float, default=18.0,
                    help="refuse blind finish if last servo offset exceeds this")
    ap.add_argument("--pixel-max-gear", type=int, default=1)
    ap.add_argument("--pixel-fast-gear", type=int, default=2)
    ap.add_argument("--pixel-fast-max-steer-offset-deg", type=float, default=6.0)
    ap.add_argument("--pixel-target-x", type=float, default=PIXEL_X_TARGET)
    ap.add_argument("--pixel-target-angle-deg", type=float, default=PIXEL_ANGLE_TARGET_DEG)
    ap.add_argument("--slot-select-enable", action=argparse.BooleanOptionalAction, default=True,
                    help="choose and lock one parking-slot target before stability filtering")
    ap.add_argument("--slot-select-x-weight", type=float, default=0.25,
                    help="slot target selection cost per pixel of corridor x error")
    ap.add_argument("--slot-select-entry-x-weight", type=float, default=0.08,
                    help="slot target selection cost per pixel of entrance x error")
    ap.add_argument("--slot-select-heading-weight", type=float, default=3.0,
                    help="slot target selection cost per degree of image heading error")
    ap.add_argument("--slot-select-confidence-weight", type=float, default=70.0,
                    help="slot target selection confidence reward")
    ap.add_argument("--slot-select-margin-weight", type=float, default=0.08,
                    help="slot target selection reward per pixel of line margin")
    ap.add_argument("--slot-select-center-y-weight", type=float, default=0.01,
                    help="slot target selection reward for nearer/lower image center")
    ap.add_argument("--slot-select-bbox-h-weight", type=float, default=0.01,
                    help="slot target selection reward for taller slot box")
    ap.add_argument("--slot-select-lock-max-center-shift-cm", type=float, default=3.0,
                    help="locked target reject threshold for frame-to-frame ground center shift")
    ap.add_argument("--slot-select-lock-max-center-shift-px", type=float, default=90.0,
                    help="locked target reject threshold for frame-to-frame image center shift")
    ap.add_argument("--slot-select-lock-max-yaw-shift-deg", type=float, default=6.0,
                    help="locked target reject threshold for frame-to-frame axis yaw shift")
    ap.add_argument("--slot-select-lock-center-weight", type=float, default=8.0,
                    help="slot target selection cost per cm away from locked target")
    ap.add_argument("--slot-select-lock-yaw-weight", type=float, default=5.0,
                    help="slot target selection cost per degree away from locked target")
    ap.add_argument("--pixel-x-tolerance-px", type=float, default=DEFAULT_PIXEL_X_TOL)
    ap.add_argument("--pixel-angle-tolerance-deg", type=float, default=DEFAULT_PIXEL_ANGLE_TOL_DEG)
    ap.add_argument("--pixel-stop-center-y", type=float, default=DEFAULT_PIXEL_STOP_CENTER_Y)
    ap.add_argument("--pixel-stop-bbox-h", type=float, default=DEFAULT_PIXEL_STOP_BOX_H)
    ap.add_argument("--corridor-sample-y", type=float, default=DEFAULT_CORRIDOR_SAMPLE_Y,
                    help="image y row used to measure slot corridor center and line margins")
    ap.add_argument("--corridor-entry-y", type=float, default=DEFAULT_CORRIDOR_ENTRY_Y,
                    help="secondary image y row used to monitor entry-center trend")
    ap.add_argument("--corridor-x-tolerance-px", type=float, default=24.0)
    ap.add_argument("--corridor-min-line-margin-px", type=float, default=34.0)
    ap.add_argument("--corridor-line-risk-min-closeness", type=float, default=0.92)
    ap.add_argument("--corridor-final-stop-closeness", type=float, default=1.08)
    ap.add_argument("--corridor-approach-closeness", type=float, default=0.82)
    ap.add_argument("--corridor-kx", type=float, default=0.12,
                    help="servo offset degrees per pixel of corridor center error while far")
    ap.add_argument("--corridor-near-kx", type=float, default=0.18,
                    help="servo offset degrees per pixel of corridor center error near entry")
    ap.add_argument("--corridor-ka", type=float, default=0.25,
                    help="servo offset degrees per degree of corridor angle error")
    ap.add_argument("--corridor-steer-sign", type=float, choices=[-1.0, 1.0], default=-1.0,
                    help="empirical reverse steering sign for corridor error; -1 flips the failed 2026-06-11 real-run direction")
    ap.add_argument("--corridor-diverge-stop-px", type=float, default=10.0,
                    help="stop if corridor x error grows by at least this after a real/simulated corridor step")
    ap.add_argument("--corridor-diverge-min-closeness", type=float, default=0.8,
                    help="apply corridor divergence stop only when the slot is near enough for the metric to matter")
    ap.add_argument("--corridor-approach-d-cm", type=float, default=20.0)
    ap.add_argument("--corridor-align-d-cm", type=float, default=8.0)
    ap.add_argument("--corridor-enter-d-cm", type=float, default=6.0)
    ap.add_argument("--corridor-min-command-abs-d-cm", type=float, default=5.0)
    ap.add_argument("--corridor-approach-max-steer-offset-deg", type=float, default=18.0)
    ap.add_argument("--corridor-align-max-steer-offset-deg", type=float, default=26.0)
    ap.add_argument("--corridor-enter-max-steer-offset-deg", type=float, default=16.0)
    ap.add_argument("--normalized-x-tolerance", type=float, default=0.06,
                    help="allowed corridor center error as fraction of detected slot width")
    ap.add_argument("--normalized-min-margin", type=float, default=0.12,
                    help="minimum safe side margin as fraction of detected slot width")
    ap.add_argument("--normalized-approach-closeness", type=float, default=0.82)
    ap.add_argument("--normalized-final-stop-closeness", type=float, default=1.08)
    ap.add_argument("--normalized-kx", type=float, default=80.0,
                    help="servo offset degrees per normalized corridor x error")
    ap.add_argument("--normalized-entry-kx", type=float, default=35.0,
                    help="servo offset degrees per normalized entry x error")
    ap.add_argument("--normalized-ka", type=float, default=0.25,
                    help="servo offset degrees per degree of slot angle error")
    ap.add_argument("--normalized-min-steer-offset-deg", type=float, default=14.0,
                    help="minimum nonzero steering offset once normalized correction is required")
    ap.add_argument("--normalized-steer-sign", type=float, choices=[-1.0, 1.0], default=-1.0,
                    help="empirical reverse steering sign for normalized corridor errors")
    ap.add_argument("--normalized-approach-d-cm", type=float, default=12.0)
    ap.add_argument("--normalized-align-d-cm", type=float, default=6.0)
    ap.add_argument("--normalized-enter-d-cm", type=float, default=6.0)
    ap.add_argument("--normalized-min-command-abs-d-cm", type=float, default=5.0)
    ap.add_argument("--normalized-approach-max-steer-offset-deg", type=float, default=16.0)
    ap.add_argument("--normalized-align-max-steer-offset-deg", type=float, default=24.0)
    ap.add_argument("--normalized-enter-max-steer-offset-deg", type=float, default=14.0)
    ap.add_argument("--path-step-cm", type=float, default=6.0,
                    help="distance per command in path_template_planner")
    ap.add_argument("--path-max-commands", type=int, default=6,
                    help="number of commands in each planned path template")
    ap.add_argument("--path-arc-steer-high", type=float, default=112.0,
                    help="positive-servo ARC value used by path templates")
    ap.add_argument("--path-arc-steer-low", type=float, default=68.0,
                    help="negative-servo ARC value used by path templates")
    ap.add_argument("--path-x-deadband-norm", type=float, default=0.05,
                    help="normalized corridor x error treated as centered by path planner")
    ap.add_argument("--path-heading-deadband-deg", type=float, default=5.0,
                    help="heading error treated as aligned by path planner")
    ap.add_argument("--path-template-min-margin-norm", type=float, default=0.10,
                    help="minimum normalized side margin before path planner stops")
    ap.add_argument("--path-final-stop-closeness", type=float, default=1.08,
                    help="closeness threshold for final stop when aligned")
    ap.add_argument("--path-kx", type=float, default=95.0,
                    help="desired steering offset degrees per normalized x error")
    ap.add_argument("--path-entry-kx", type=float, default=40.0,
                    help="desired steering offset degrees per normalized entry x error")
    ap.add_argument("--path-ka", type=float, default=0.35,
                    help="desired steering offset degrees per slot heading degree")
    ap.add_argument("--path-steer-sign", type=float, choices=[-1.0, 1.0], default=-1.0,
                    help="empirical reverse steering sign for path template selection")
    ap.add_argument("--path-max-steer-offset-deg", type=float, default=32.0)
    ap.add_argument("--path-straight-offset-deg", type=float, default=6.0)
    ap.add_argument("--path-mid-offset-threshold-deg", type=float, default=14.0)
    ap.add_argument("--path-hard-offset-threshold-deg", type=float, default=24.0)
    ap.add_argument("--path-straight-cost-gain", type=float, default=1.2)
    ap.add_argument("--path-wrong-side-penalty", type=float, default=80.0)
    ap.add_argument("--path-arc-count-penalty", type=float, default=5.0)
    ap.add_argument("--path-distance-cost-gain", type=float, default=0.15)
    ap.add_argument("--path-prefer-margin-norm", type=float, default=0.22)
    ap.add_argument("--path-margin-cost-gain", type=float, default=20.0)
    ap.add_argument("--path-near-closeness", type=float, default=0.92)
    ap.add_argument("--path-near-total-cm", type=float, default=24.0)
    ap.add_argument("--path-far-total-cm", type=float, default=36.0)
    ap.add_argument("--primitive-command", default="ARC D=-6.0 STE=120 V=1",
                    help="single safe primitive for --strategy primitive_probe")
    ap.add_argument("--primitive-max-command-abs-d-cm", type=float, default=8.0,
                    help="maximum reverse distance accepted by --primitive-command")
    ap.add_argument("--primitive-no-vision", action="store_true",
                    help="allow primitive_probe to execute one capped calibration primitive without YOLO target stability")
    ap.add_argument("--dry-run-simulate-motion", action="store_true",
                    help="in dry-run, virtually accumulate steps/caps as if commands were executed")
    ap.add_argument("--feedback-episodes", type=int, default=8)
    ap.add_argument("--feedback-step-cm", type=float, default=2.0)
    ap.add_argument("--feedback-min-step-cm", type=float, default=1.0)
    ap.add_argument("--feedback-max-step-cm", type=float, default=3.0)
    ap.add_argument("--feedback-max-command-abs-d-cm", type=float, default=3.0)
    ap.add_argument("--feedback-max-total-cm", type=float, default=18.0)
    ap.add_argument("--feedback-step-increment-cm", type=float, default=0.5)
    ap.add_argument("--feedback-steer-deg", type=float, default=10.0)
    ap.add_argument("--feedback-min-steer-deg", type=float, default=4.0)
    ap.add_argument("--feedback-max-steer-deg", type=float, default=16.0)
    ap.add_argument("--feedback-steer-increment-deg", type=float, default=2.0)
    ap.add_argument("--feedback-lateral-sign", type=float, choices=[-1.0, 1.0], default=-1.0)
    ap.add_argument("--feedback-flip-after-bad", type=int, default=2)
    ap.add_argument("--feedback-vision-timeout-sec", type=float, default=5.0)
    ap.add_argument("--feedback-post-settle-sec", type=float, default=0.8)
    ap.add_argument("--feedback-file", default="/tmp/parking_feedback")
    ap.add_argument("--feedback-timeout-sec", type=float, default=8.0)
    ap.add_argument("--feedback-manual", action="store_true",
                    help="after each real step, wait for +, -, 0, or q in --feedback-file")
    ap.add_argument("--feedback-auto", action="store_true",
                    help="use visual pre/post deltas as feedback when no manual token is provided")
    ap.add_argument("--feedback-min-lat-improve-cm", type=float, default=0.25)
    ap.add_argument("--feedback-success-lat-cm", type=float, default=2.5)
    ap.add_argument("--feedback-success-lon-cm", type=float, default=8.0)
    ap.add_argument("--learn-policy-file", default="/opt/parking/autopark/parking_policy.json")
    ap.add_argument("--learn-actions",
                    default="MOVE D=-7.0 V=1|ARC D=-7.0 STE=50 V=1|ARC D=-7.0 STE=60 V=1|"
                            "ARC D=-7.0 STE=70 V=1|ARC D=-7.0 STE=80 V=1|"
                            "ARC D=-7.0 STE=90 V=1|ARC D=-7.0 STE=100 V=1|"
                            "ARC D=-7.0 STE=110 V=1|ARC D=-7.0 STE=120 V=1|"
                            "ARC D=-7.0 STE=130 V=1")
    ap.add_argument("--learn-max-command-abs-d-cm", type=float, default=7.0)
    ap.add_argument("--learn-max-total-cm", type=float, default=70.0)
    ap.add_argument("--learn-alpha", type=float, default=0.35)
    ap.add_argument("--learn-epsilon", type=float, default=0.25)
    ap.add_argument("--learn-episodes", type=int, default=0,
                    help="0 = keep running until q; positive value is for bounded tests")
    ap.add_argument("--learn-stop-after-bad", type=int, default=0,
                    help="0 = do not hold on negative streak; positive value holds until SPACE/q")
    args = ap.parse_args()
    args._fusion_signs = None
    if load_chassis_signs is None:
        msg = "FUSION_SIGNS=UNAVAILABLE helper_module_missing"
        print(msg, flush=True)
        if args.require_fusion_signs:
            print("REFUSING START: --require-fusion-signs set but parking_fusion.py is unavailable.",
                  file=sys.stderr)
            return 6
    else:
        try:
            signs = load_chassis_signs(args.chassis_signs_json)
            args._fusion_signs = signs
            print("FUSION_SIGNS=OK %s yaw_cw_positive=%s odom_d_reverse_negative=%s "
                  "odom_x_right_positive=%s vision_lateral_left_negative=%s" % (
                      args.chassis_signs_json,
                      signs.yaw_cw_positive,
                      signs.odom_d_reverse_negative,
                      signs.odom_x_right_positive,
                      signs.vision_lateral_left_negative,
                  ), flush=True)
        except Exception as exc:
            print("FUSION_SIGNS=INVALID path=%s error=%s" % (args.chassis_signs_json, exc), flush=True)
            if args.require_fusion_signs:
                print("REFUSING START: fusion signs are required and invalid.", file=sys.stderr)
                return 6
    safe_replanner_dry_run = args.strategy == "action_replanner" and args.replanner_dry_run
    if not args.dry_run and not safe_replanner_dry_run and not (args.arm and os.path.exists(args.arm_file)):
        print("REFUSING MOTION: need --arm AND arm file %s (or use --dry-run)." % args.arm_file,
              file=sys.stderr)
        return 4
    motion_authorized = not args.dry_run and not safe_replanner_dry_run and args.arm and os.path.exists(args.arm_file)
    reason = "normal_return"
    try:
        if args.learn_policy:
            return run_learn_policy(args)
        if args.feedback_tune:
            return run_feedback_tune(args)
        return run(args)
    except BaseException:
        reason = "exception"
        raise
    finally:
        final_stop_on_exit(args, motion_authorized, reason)


if __name__ == "__main__":
    raise SystemExit(main())
