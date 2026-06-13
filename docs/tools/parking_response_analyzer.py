#!/usr/bin/env python3
"""Analyze real parking logs and estimate steering-response quality.

This is an offline tool. It pairs each executed STM32 motion command with the
next visual candidate row, then reports whether the observed slot metrics moved
in the expected direction. It is intended to prevent repeated full-run tests
when the low-level path response is still unknown.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

try:
    import board_parking_controller as bpc
except Exception:
    bpc = None


COMMAND_RE = re.compile(r"^(?P<kind>MOVE|ARC)\s+D=(?P<d>-?\d+(?:\.\d+)?)(?:\s+STE=(?P<ste>\d+))?")


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


def parse_command(text: str) -> dict:
    match = COMMAND_RE.search(text or "")
    if not match:
        return {"kind": "UNKNOWN", "raw": text}
    steer = match.group("ste")
    return {
        "kind": match.group("kind"),
        "distance_cm": float(match.group("d")),
        "servo": int(steer) if steer is not None else 90,
        "raw": text,
    }


def default_corridor_args():
    return SimpleNamespace(
        corridor_sample_y=getattr(bpc, "DEFAULT_CORRIDOR_SAMPLE_Y", 500.0),
        corridor_entry_y=getattr(bpc, "DEFAULT_CORRIDOR_ENTRY_Y", 430.0),
        pixel_target_x=getattr(bpc, "PIXEL_X_TARGET", 320.0),
        pixel_stop_center_y=getattr(bpc, "DEFAULT_PIXEL_STOP_CENTER_Y", 560.0),
        pixel_stop_bbox_h=getattr(bpc, "DEFAULT_PIXEL_STOP_BOX_H", 520.0),
        corridor_line_risk_min_closeness=0.92,
        corridor_min_line_margin_px=34.0,
    )


def fallback_corridor_metrics(row: dict) -> dict:
    if bpc is None:
        return {}
    poly = row.get("slot_polygon_px")
    if not poly:
        return {}
    geom = bpc.slot_pixel_geometry(poly)
    if not geom:
        return {}
    xs = [float(p[0]) for p in poly]
    ys = [float(p[1]) for p in poly]
    info = dict(geom)
    info["center_px"] = [sum(xs) / len(xs), sum(ys) / len(ys)]
    info["bbox_h_px"] = max(ys) - min(ys)
    try:
        return bpc.corridor_metrics(info, default_corridor_args())
    except Exception:
        return {}


def candidate_metrics(row: dict) -> dict:
    corridor = row.get("corridor") or fallback_corridor_metrics(row)
    return {
        "lineno": row.get("_lineno"),
        "candidate_cmd": row.get("candidate_cmd"),
        "state": row.get("state"),
        "stable": row.get("stable"),
        "will_execute_motion": row.get("will_execute_motion", row.get("send_to_stm32")),
        "lon_cm": row.get("lon"),
        "lat_cm": row.get("lat"),
        "heading_deg": row.get("head"),
        "corridor_x_err_px": corridor.get("corridor_x_err"),
        "corridor_min_margin_px": corridor.get("min_margin_px"),
        "corridor_left_margin_px": corridor.get("left_margin_px"),
        "corridor_right_margin_px": corridor.get("right_margin_px"),
        "closeness": corridor.get("closeness"),
    }


def finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def delta(after, before, key):
    if finite(after.get(key)) and finite(before.get(key)):
        return round(after[key] - before[key], 3)
    return None


def abs_delta(after, before, key):
    if finite(after.get(key)) and finite(before.get(key)):
        return round(abs(after[key]) - abs(before[key]), 3)
    return None


def classify_transition(before: dict, after: dict, command: dict) -> dict:
    dx_abs = abs_delta(after, before, "corridor_x_err_px")
    dlat_abs = abs_delta(after, before, "lat_cm")
    dmargin = delta(after, before, "corridor_min_margin_px")
    score = 0
    notes = []
    if dx_abs is not None:
        if dx_abs < -1.0:
            score += 1
            notes.append("corridor_x_error_improved")
        elif dx_abs > 1.0:
            score -= 1
            notes.append("corridor_x_error_worsened")
    if dlat_abs is not None:
        if dlat_abs < -0.2:
            score += 1
            notes.append("lat_error_improved")
        elif dlat_abs > 0.2:
            score -= 1
            notes.append("lat_error_worsened")
    if dmargin is not None:
        if dmargin > 2.0:
            score += 1
            notes.append("line_margin_improved")
        elif dmargin < -2.0:
            score -= 1
            notes.append("line_margin_worsened")
    if not notes:
        notes.append("insufficient_metric_change")
    if command.get("kind") == "ARC":
        if command.get("servo", 90) < 90:
            turn_bucket = "servo_left_of_center"
        elif command.get("servo", 90) > 90:
            turn_bucket = "servo_right_of_center"
        else:
            turn_bucket = "servo_center"
    else:
        turn_bucket = "straight"
    return {
        "score": score,
        "verdict": "improved" if score > 0 else ("worsened" if score < 0 else "neutral"),
        "notes": notes,
        "turn_bucket": turn_bucket,
        "delta": {
            "lon_cm": delta(after, before, "lon_cm"),
            "lat_cm": delta(after, before, "lat_cm"),
            "abs_lat_cm": dlat_abs,
            "heading_deg": delta(after, before, "heading_deg"),
            "corridor_x_err_px": delta(after, before, "corridor_x_err_px"),
            "abs_corridor_x_err_px": dx_abs,
            "corridor_min_margin_px": dmargin,
        },
    }


def analyze(paths: list[Path]) -> dict:
    transitions = []
    all_candidates = []
    for path in paths:
        rows = []
        for lineno, row in iter_jsonl(path):
            row["_lineno"] = lineno
            row["_file"] = str(path)
            rows.append(row)
        for idx, row in enumerate(rows):
            if row.get("event") != "stm32_motion_result":
                if row.get("event") == "candidate":
                    all_candidates.append(candidate_metrics(row) | {"file": str(path)})
                continue
            before = None
            after = None
            for prev in reversed(rows[:idx]):
                if prev.get("event") == "candidate":
                    before = candidate_metrics(prev)
                    break
            for nxt in rows[idx + 1:]:
                if nxt.get("event") == "candidate":
                    after = candidate_metrics(nxt)
                    break
            command = parse_command(row.get("candidate_cmd") or "")
            item = {
                "file": str(path),
                "motion_lineno": row.get("_lineno"),
                "command": command,
                "stm32": {
                    "pre_servo_response": row.get("pre_servo_response", ""),
                    "motion_response": row.get("motion_response", ""),
                    "pwm_after_pre_servo": row.get("pwm_after_pre_servo", ""),
                    "stat_after": row.get("stat_after", ""),
                },
                "before": before,
                "after": after,
            }
            if before and after:
                item["classification"] = classify_transition(before, after, command)
            else:
                item["classification"] = {
                    "score": 0,
                    "verdict": "unknown",
                    "notes": ["missing_pre_or_post_candidate"],
                }
            transitions.append(item)

    bucket_summary = {}
    for item in transitions:
        cls = item["classification"]
        bucket = cls.get("turn_bucket", "unknown")
        cur = bucket_summary.setdefault(bucket, {"count": 0, "score_sum": 0, "verdicts": {}})
        cur["count"] += 1
        cur["score_sum"] += cls.get("score", 0)
        cur["verdicts"][cls.get("verdict", "unknown")] = cur["verdicts"].get(cls.get("verdict", "unknown"), 0) + 1

    recommendation = {
        "do_not_full_run_until": [
            "left/right steering response has at least one improving primitive in current camera pose",
            "same primitive is repeated twice without increasing line-risk metrics",
            "candidate logs show will_execute_motion only for actually sent commands",
        ],
        "next_calibration_primitives": [
            "ARC D=-6.0 STE=60 V=1",
            "ARC D=-6.0 STE=120 V=1",
            "MOVE D=-6.0 V=1",
        ],
        "promotion_rule": "Only expand from 6 cm to 12-20 cm after the 6 cm primitive improves abs corridor_x_err or abs lat and does not reduce min line margin.",
    }
    if transitions and transitions[-1]["classification"]["verdict"] == "worsened":
        recommendation["latest_warning"] = "Latest executed primitive worsened at least one key metric; use calibration, not closed-loop full parking."

    return {
        "input_files": [str(p) for p in paths],
        "candidate_rows": len(all_candidates),
        "executed_motion_count": len(transitions),
        "bucket_summary": bucket_summary,
        "transitions": transitions,
        "last_candidate": all_candidates[-1] if all_candidates else None,
        "recommendation": recommendation,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("artifacts/autopark_baseline/parking_response_report.json"))
    args = ap.parse_args()
    report = analyze(args.logs)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({
        "out": str(args.out),
        "executed_motion_count": report["executed_motion_count"],
        "bucket_summary": report["bucket_summary"],
        "latest_warning": report["recommendation"].get("latest_warning", ""),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
