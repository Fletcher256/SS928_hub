#!/usr/bin/env python3
"""Extract and summarize slot-relative observation states from parking logs.

Stage 1 of the new autopark architecture is reliable observation:

YOLO polygon -> slot_relative_state

This tool works offline on JSONL logs. It reuses existing states when present,
or recomputes them from logged slot polygons and edge geometry.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import board_parking_controller as bpc


def default_args() -> SimpleNamespace:
    return SimpleNamespace(
        pixel_target_x=bpc.PIXEL_X_TARGET,
        pixel_target_angle_deg=bpc.PIXEL_ANGLE_TARGET_DEG,
        pixel_stop_center_y=bpc.DEFAULT_PIXEL_STOP_CENTER_Y,
        pixel_stop_bbox_h=bpc.DEFAULT_PIXEL_STOP_BOX_H,
        pixel_angle_tolerance_deg=bpc.DEFAULT_PIXEL_ANGLE_TOL_DEG,
        corridor_sample_y=bpc.DEFAULT_CORRIDOR_SAMPLE_Y,
        corridor_entry_y=bpc.DEFAULT_CORRIDOR_ENTRY_Y,
        corridor_x_tolerance_px=24.0,
        corridor_min_line_margin_px=34.0,
        corridor_line_risk_min_closeness=0.92,
        corridor_approach_closeness=0.82,
        corridor_final_stop_closeness=1.08,
        stable_frames=bpc.DEFAULT_STABLE_FRAMES,
    )


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8-sig", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield lineno, json.loads(line)
            except ValueError as exc:
                yield lineno, {"event": "parse_error", "error": str(exc)}


def info_from_candidate(row: dict):
    poly = row.get("slot_polygon_px")
    if not poly:
        return None
    geom = bpc.slot_pixel_geometry(poly)
    if not geom:
        return None
    center_cm = bpc.apply_h(*geom["center_px"])
    axis_cm = [
        bpc.apply_h(*geom["approach_axis_px"][0]),
        bpc.apply_h(*geom["approach_axis_px"][1]),
    ]
    plan = bpc.plan(center_cm, axis_cm)
    return {
        "confidence": float(row.get("confidence", 0.0)),
        "class_name": row.get("class_name", "Parking"),
        "bbox_xyxy": row.get("bbox_xyxy"),
        "mask_polygon_px": poly,
        "mask_area_px": row.get("mask_area_px"),
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
        "axis_yaw_deg": bpc.axis_yaw_deg(axis_cm),
        "plan": plan,
        "raw_detection_count": row.get("raw_detection_count", 1),
        "raw_time_ns": row.get("raw_time_ns"),
    }


def extract_state(row: dict, args: SimpleNamespace):
    state = row.get("slot_relative_state")
    if state:
        return state
    info = info_from_candidate(row)
    if not info:
        return None
    stability = dict(row.get("stability") or {})
    if row.get("stable") and "required_frames" not in stability:
        stability["required_frames"] = stability.get("stable_frames", args.stable_frames)
    return bpc.slot_relative_state(info, args, stability)


def flatten_state(path: Path, lineno: int, row: dict, state: dict) -> dict:
    image = state.get("image") or {}
    corridor = state.get("corridor") or {}
    ground = state.get("ground_estimate") or {}
    gates = state.get("gates") or {}
    return {
        "file": str(path),
        "lineno": lineno,
        "event": row.get("event"),
        "stable": row.get("stable"),
        "candidate_cmd": row.get("candidate_cmd"),
        "will_execute_motion": row.get("will_execute_motion", row.get("send_to_stm32")),
        "confidence": state.get("confidence"),
        "pose_quality": state.get("pose_quality"),
        "phase_hint": state.get("phase_hint"),
        "stable_frames": state.get("stable_frames"),
        "slot_x_err_px": corridor.get("slot_x_err_px"),
        "slot_entry_x_err_px": corridor.get("slot_entry_x_err_px"),
        "slot_heading_err_deg": image.get("slot_heading_err_deg"),
        "left_margin_px": corridor.get("left_margin_px"),
        "right_margin_px": corridor.get("right_margin_px"),
        "min_margin_px": corridor.get("min_margin_px"),
        "line_risk": corridor.get("line_risk"),
        "risk_side": corridor.get("risk_side"),
        "closeness": image.get("closeness"),
        "slot_y_dist_cm": ground.get("slot_y_dist_cm"),
        "slot_lateral_cm": ground.get("slot_lateral_cm"),
        "slot_axis_heading_deg": ground.get("slot_axis_heading_deg"),
        "stable_enough": gates.get("stable_enough"),
        "line_margin_ok": gates.get("line_margin_ok"),
        "heading_ok": gates.get("heading_ok"),
        "lateral_ok": gates.get("lateral_ok"),
    }


def numeric(values):
    return [float(v) for v in values if isinstance(v, (int, float))]


def stats(values):
    vals = numeric(values)
    if not vals:
        return None
    return {
        "count": len(vals),
        "min": round(min(vals), 3),
        "max": round(max(vals), 3),
        "mean": round(statistics.fmean(vals), 3),
        "stdev": round(statistics.pstdev(vals), 3) if len(vals) > 1 else 0.0,
    }


def analyze(paths: list[Path], args: SimpleNamespace) -> dict:
    rows = []
    parse_errors = 0
    candidate_rows = 0
    for path in paths:
        for lineno, row in iter_jsonl(path):
            if row.get("event") == "parse_error":
                parse_errors += 1
                continue
            if row.get("event") != "candidate":
                continue
            candidate_rows += 1
            state = extract_state(row, args)
            if not state:
                continue
            rows.append(flatten_state(path, lineno, row, state))

    stable_rows = [r for r in rows if r.get("stable")]
    summary = {
        "input_files": [str(p) for p in paths],
        "candidate_rows": candidate_rows,
        "state_rows": len(rows),
        "stable_state_rows": len(stable_rows),
        "parse_errors": parse_errors,
        "phase_counts": {},
        "line_risk_rows": sum(1 for r in rows if r.get("line_risk")),
        "motion_candidate_rows": sum(1 for r in rows if r.get("will_execute_motion")),
        "metrics": {
            "slot_x_err_px": stats([r.get("slot_x_err_px") for r in stable_rows]),
            "slot_entry_x_err_px": stats([r.get("slot_entry_x_err_px") for r in stable_rows]),
            "slot_heading_err_deg": stats([r.get("slot_heading_err_deg") for r in stable_rows]),
            "min_margin_px": stats([r.get("min_margin_px") for r in stable_rows]),
            "slot_y_dist_cm": stats([r.get("slot_y_dist_cm") for r in stable_rows]),
            "slot_lateral_cm": stats([r.get("slot_lateral_cm") for r in stable_rows]),
            "pose_quality": stats([r.get("pose_quality") for r in stable_rows]),
        },
        "tail": rows[-8:],
    }
    for row in rows:
        phase = row.get("phase_hint") or "unknown"
        summary["phase_counts"][phase] = summary["phase_counts"].get(phase, 0) + 1
    return {"summary": summary, "rows": rows}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("artifacts/autopark_baseline/slot_state_summary.json"))
    ap.add_argument("--csv", type=Path, default=Path("artifacts/autopark_baseline/slot_state_rows.csv"))
    ns = ap.parse_args()
    report = analyze(ns.logs, default_args())
    ns.out.parent.mkdir(parents=True, exist_ok=True)
    ns.out.write_text(json.dumps(report["summary"], indent=2, ensure_ascii=False), encoding="utf-8")
    if report["rows"]:
        ns.csv.parent.mkdir(parents=True, exist_ok=True)
        with ns.csv.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(report["rows"][0].keys()))
            writer.writeheader()
            writer.writerows(report["rows"])
    print(json.dumps({
        "out": str(ns.out),
        "csv": str(ns.csv),
        "candidate_rows": report["summary"]["candidate_rows"],
        "state_rows": report["summary"]["state_rows"],
        "stable_state_rows": report["summary"]["stable_state_rows"],
        "phase_counts": report["summary"]["phase_counts"],
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
