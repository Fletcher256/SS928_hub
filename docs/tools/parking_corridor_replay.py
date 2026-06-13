#!/usr/bin/env python3
"""Offline analysis/replay for pixel_servo vs corridor_servo parking logs.

Historical controller logs usually contain only candidate decisions, not raw
YOLO polygons. Those rows can be audited but not geometrically replayed. Logs
that contain raw YOLO messages with "detections" can be fully replayed through
board_parking_controller.best_slot_info_from_udp() and corridor_servo_command().
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import board_parking_controller as bpc


def default_control_args(overrides: argparse.Namespace) -> SimpleNamespace:
    values = {
        "pixel_target_x": bpc.PIXEL_X_TARGET,
        "pixel_target_angle_deg": bpc.PIXEL_ANGLE_TARGET_DEG,
        "pixel_x_tolerance_px": bpc.DEFAULT_PIXEL_X_TOL,
        "pixel_angle_tolerance_deg": bpc.DEFAULT_PIXEL_ANGLE_TOL_DEG,
        "pixel_stop_center_y": bpc.DEFAULT_PIXEL_STOP_CENTER_Y,
        "pixel_stop_bbox_h": bpc.DEFAULT_PIXEL_STOP_BOX_H,
        "pixel_min_command_abs_d_cm": 8.0,
        "pixel_max_command_abs_d_cm": 40.0,
        "pixel_kx": overrides.pixel_kx,
        "pixel_ka": overrides.pixel_ka,
        "pixel_max_steer_offset_deg": overrides.pixel_max_steer_offset_deg,
        "pixel_large_steer_offset_deg": overrides.pixel_large_steer_offset_deg,
        "pixel_large_x_err_px": overrides.pixel_large_x_err_px,
        "pixel_near_d_cm": overrides.pixel_near_d_cm,
        "pixel_mid_d_cm": overrides.pixel_mid_d_cm,
        "pixel_far_d_cm": overrides.pixel_far_d_cm,
        "pixel_near_ratio": overrides.pixel_near_ratio,
        "pixel_max_gear": 1,
        "pixel_fast_gear": 2,
        "pixel_fast_max_steer_offset_deg": 6.0,
        "corridor_min_command_abs_d_cm": overrides.corridor_min_command_abs_d_cm,
        "corridor_sample_y": overrides.corridor_sample_y,
        "corridor_entry_y": overrides.corridor_entry_y,
        "corridor_x_tolerance_px": overrides.corridor_x_tolerance_px,
        "corridor_min_line_margin_px": overrides.corridor_min_line_margin_px,
        "corridor_line_risk_min_closeness": overrides.corridor_line_risk_min_closeness,
        "corridor_final_stop_closeness": overrides.corridor_final_stop_closeness,
        "corridor_approach_closeness": overrides.corridor_approach_closeness,
        "corridor_kx": overrides.corridor_kx,
        "corridor_near_kx": overrides.corridor_near_kx,
        "corridor_ka": overrides.corridor_ka,
        "corridor_steer_sign": overrides.corridor_steer_sign,
        "corridor_diverge_stop_px": overrides.corridor_diverge_stop_px,
        "corridor_diverge_min_closeness": overrides.corridor_diverge_min_closeness,
        "corridor_approach_d_cm": overrides.corridor_approach_d_cm,
        "corridor_align_d_cm": overrides.corridor_align_d_cm,
        "corridor_enter_d_cm": overrides.corridor_enter_d_cm,
        "corridor_approach_max_steer_offset_deg": overrides.corridor_approach_max_steer_offset_deg,
        "corridor_align_max_steer_offset_deg": overrides.corridor_align_max_steer_offset_deg,
        "corridor_enter_max_steer_offset_deg": overrides.corridor_enter_max_steer_offset_deg,
    }
    return SimpleNamespace(**values)


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield lineno, json.loads(line)
            except ValueError as exc:
                yield lineno, {"event": "parse_error", "error": str(exc), "raw": line[:200]}


def compact_candidate(row: dict) -> dict:
    pixel = row.get("pixel") or {}
    binding = row.get("binding") or {}
    corridor = row.get("corridor") or {}
    return {
        "event": row.get("event"),
        "stable": row.get("stable"),
        "state": row.get("state"),
        "reason": row.get("reason"),
        "candidate_cmd": row.get("candidate_cmd"),
        "send_to_stm32": row.get("send_to_stm32"),
        "confidence": row.get("confidence"),
        "lon": row.get("lon"),
        "lat": row.get("lat"),
        "head": row.get("head"),
        "pixel_x_err": pixel.get("x_err"),
        "pixel_cy": pixel.get("cy"),
        "pixel_bbox_h": pixel.get("bbox_h"),
        "pixel_closeness": binding.get("closeness"),
        "servo": binding.get("servo"),
        "distance_cm": binding.get("distance_cm"),
        "distance_reason": binding.get("distance_reason"),
        "corridor_x_err": corridor.get("corridor_x_err"),
        "corridor_left_margin_px": corridor.get("left_margin_px"),
        "corridor_right_margin_px": corridor.get("right_margin_px"),
        "corridor_min_margin_px": corridor.get("min_margin_px"),
        "corridor_line_risk": corridor.get("line_risk"),
        "corridor_risk_side": corridor.get("risk_side"),
    }


def raw_yolo_from_row(row: dict):
    raw = row.get("raw_yolo") or row.get("raw")
    if isinstance(raw, dict) and isinstance(raw.get("detections"), list):
        return raw
    if isinstance(row.get("detections"), list):
        return row

    poly = row.get("slot_polygon_px")
    if poly:
        det = {
            "confidence": row.get("confidence", 1.0),
            "class_name": row.get("class_name", "Parking"),
            "mask_polygon": poly,
            "bbox_xyxy": row.get("bbox_xyxy"),
        }
        return {"detections": [det], "time_ns": row.get("raw_time_ns")}
    return None


def compact_action(action: dict) -> dict:
    binding = action.get("binding") or {}
    corridor = action.get("corridor") or {}
    pixel = action.get("pixel") or {}
    return {
        "state": action.get("state"),
        "reason": action.get("reason"),
        "cmd": action.get("cmd"),
        "action": action.get("action"),
        "servo": action.get("servo"),
        "step": action.get("step"),
        "distance_cm": binding.get("distance_cm"),
        "distance_reason": binding.get("distance_reason"),
        "pixel_x_err": pixel.get("x_err"),
        "pixel_cy": pixel.get("cy"),
        "pixel_bbox_h": pixel.get("bbox_h"),
        "closeness": binding.get("closeness"),
        "corridor_x_err": corridor.get("corridor_x_err"),
        "corridor_min_margin_px": corridor.get("min_margin_px"),
        "corridor_line_risk": corridor.get("line_risk"),
        "corridor_risk_side": corridor.get("risk_side"),
    }


def replay_file(path: Path, args: argparse.Namespace) -> dict:
    control_args = default_control_args(args)
    rows = []
    counts = {
        "total_rows": 0,
        "candidate_rows": 0,
        "stable_candidate_rows": 0,
        "raw_detection_rows": 0,
        "replayed_pixel_rows": 0,
        "replayed_corridor_rows": 0,
        "replayed_compare_rows": 0,
        "pixel_corridor_cmd_diff_rows": 0,
        "pixel_corridor_state_diff_rows": 0,
        "vision_lost_rows": 0,
        "parse_errors": 0,
    }
    last_stable = None
    last_candidate = None
    last_replayed = None
    last_compare = None

    for lineno, row in iter_jsonl(path):
        counts["total_rows"] += 1
        event = row.get("event")
        if event == "parse_error":
            counts["parse_errors"] += 1
            continue
        if event == "candidate":
            counts["candidate_rows"] += 1
            compact = compact_candidate(row)
            compact["lineno"] = lineno
            rows.append(compact)
            last_candidate = compact
            if row.get("stable"):
                counts["stable_candidate_rows"] += 1
                last_stable = compact
        elif event == "vision_lost":
            counts["vision_lost_rows"] += 1

        raw = raw_yolo_from_row(row)
        if raw is not None:
            counts["raw_detection_rows"] += 1
            info = bpc.best_slot_info_from_udp(raw)
            if info:
                pixel_action = bpc.pixel_servo_command(info, control_args)
                corridor_action = bpc.corridor_servo_command(info, control_args)
                counts["replayed_pixel_rows"] += 1
                counts["replayed_corridor_rows"] += 1
                counts["replayed_compare_rows"] += 1
                if pixel_action.get("cmd") != corridor_action.get("cmd"):
                    counts["pixel_corridor_cmd_diff_rows"] += 1
                if pixel_action.get("state") != corridor_action.get("state"):
                    counts["pixel_corridor_state_diff_rows"] += 1
                last_compare = {
                    "lineno": lineno,
                    "original_candidate_cmd": row.get("candidate_cmd"),
                    "pixel": compact_action(pixel_action),
                    "corridor": compact_action(corridor_action),
                }
                last_replayed = {
                    "lineno": lineno,
                    "state": corridor_action.get("state"),
                    "reason": corridor_action.get("reason"),
                    "candidate_cmd": corridor_action.get("cmd"),
                    "corridor": corridor_action.get("corridor"),
                    "binding": corridor_action.get("binding"),
                }

    failure_note = None
    if last_stable and counts["vision_lost_rows"] > 0:
        failure_note = {
            "kind": "vision_lost_after_stable_candidate",
            "last_stable": last_stable,
        }

    return {
        "file": str(path),
        "counts": counts,
        "can_replay_corridor": counts["replayed_corridor_rows"] > 0,
        "evidence_gap": None if counts["replayed_corridor_rows"] > 0
        else "no raw YOLO detections/polygons in this log; only old candidate decisions can be audited",
        "last_stable_candidate": last_stable,
        "last_candidate": last_candidate,
        "last_replayed_corridor": last_replayed,
        "last_pixel_corridor_compare": last_compare,
        "failure_note": failure_note,
        "candidate_tail": rows[-args.tail:],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("artifacts/autopark_baseline/corridor_replay_report.json"))
    ap.add_argument("--tail", type=int, default=8)
    ap.add_argument("--pixel-kx", type=float, default=0.14)
    ap.add_argument("--pixel-ka", type=float, default=0.35)
    ap.add_argument("--pixel-max-steer-offset-deg", type=float, default=24.0)
    ap.add_argument("--pixel-large-steer-offset-deg", type=float, default=18.0)
    ap.add_argument("--pixel-large-x-err-px", type=float, default=70.0)
    ap.add_argument("--pixel-near-d-cm", type=float, default=10.0)
    ap.add_argument("--pixel-mid-d-cm", type=float, default=20.0)
    ap.add_argument("--pixel-far-d-cm", type=float, default=40.0)
    ap.add_argument("--pixel-near-ratio", type=float, default=0.9)
    ap.add_argument("--corridor-sample-y", type=float, default=bpc.DEFAULT_CORRIDOR_SAMPLE_Y)
    ap.add_argument("--corridor-entry-y", type=float, default=bpc.DEFAULT_CORRIDOR_ENTRY_Y)
    ap.add_argument("--corridor-x-tolerance-px", type=float, default=24.0)
    ap.add_argument("--corridor-min-line-margin-px", type=float, default=34.0)
    ap.add_argument("--corridor-line-risk-min-closeness", type=float, default=0.92)
    ap.add_argument("--corridor-final-stop-closeness", type=float, default=1.08)
    ap.add_argument("--corridor-approach-closeness", type=float, default=0.82)
    ap.add_argument("--corridor-kx", type=float, default=0.12)
    ap.add_argument("--corridor-near-kx", type=float, default=0.18)
    ap.add_argument("--corridor-ka", type=float, default=0.25)
    ap.add_argument("--corridor-steer-sign", type=float, choices=[-1.0, 1.0], default=-1.0)
    ap.add_argument("--corridor-diverge-stop-px", type=float, default=10.0)
    ap.add_argument("--corridor-diverge-min-closeness", type=float, default=0.8)
    ap.add_argument("--corridor-approach-d-cm", type=float, default=20.0)
    ap.add_argument("--corridor-align-d-cm", type=float, default=8.0)
    ap.add_argument("--corridor-enter-d-cm", type=float, default=6.0)
    ap.add_argument("--corridor-min-command-abs-d-cm", type=float, default=5.0)
    ap.add_argument("--corridor-approach-max-steer-offset-deg", type=float, default=18.0)
    ap.add_argument("--corridor-align-max-steer-offset-deg", type=float, default=26.0)
    ap.add_argument("--corridor-enter-max-steer-offset-deg", type=float, default=16.0)
    args = ap.parse_args()

    report = {"logs": [replay_file(path, args) for path in args.logs]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("CORRIDOR_REPLAY_REPORT", args.out)
    for item in report["logs"]:
        c = item["counts"]
        print(
            "LOG file=%s candidates=%d stable=%d raw=%d compare=%d cmd_diff=%d state_diff=%d vision_lost=%d" % (
                item["file"], c["candidate_rows"], c["stable_candidate_rows"],
                c["raw_detection_rows"], c["replayed_compare_rows"],
                c["pixel_corridor_cmd_diff_rows"], c["pixel_corridor_state_diff_rows"],
                c["vision_lost_rows"],
            )
        )
        if item["evidence_gap"]:
            print("  EVIDENCE_GAP", item["evidence_gap"])
        if item["last_stable_candidate"]:
            print("  LAST_STABLE", json.dumps(item["last_stable_candidate"], ensure_ascii=False, separators=(",", ":")))
        elif item["last_candidate"]:
            print("  LAST_CANDIDATE", json.dumps(item["last_candidate"], ensure_ascii=False, separators=(",", ":")))
        if item["last_replayed_corridor"]:
            print("  LAST_REPLAYED", json.dumps(item["last_replayed_corridor"], ensure_ascii=False, separators=(",", ":")))
        if item["last_pixel_corridor_compare"]:
            print("  LAST_COMPARE", json.dumps(item["last_pixel_corridor_compare"], ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
