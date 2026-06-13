#!/usr/bin/env python3
"""Offline one-step action scorer for the action-template parking plan.

This is Stage 2 software only. It reads slot-relative observation states from
JSONL logs or CSV rows, predicts each bounded action with measured/prior
response deltas, and ranks the actions. It does not send motion commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import parking_slot_state_analyzer as state_analyzer


bpc = state_analyzer.bpc


STATE_KEYS = [
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


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def to_float(value, default=0.0):
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


def to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


def read_states(path: Path) -> list[dict]:
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8-sig", newline="") as fh:
            rows = []
            for row in csv.DictReader(fh):
                rows.append(normalize_row(row))
            return rows
    report = state_analyzer.analyze([path], state_analyzer.default_args())
    return [normalize_row(row) for row in report["rows"]]


def normalize_row(row: dict) -> dict:
    out = dict(row)
    for key in STATE_KEYS:
        out[key] = to_float(out.get(key))
    out["stable"] = to_bool(out.get("stable"))
    out["stable_enough"] = to_bool(out.get("stable_enough"))
    out["line_margin_ok"] = to_bool(out.get("line_margin_ok"))
    out["heading_ok"] = to_bool(out.get("heading_ok"))
    out["lateral_ok"] = to_bool(out.get("lateral_ok"))
    out["line_risk"] = to_bool(out.get("line_risk"))
    out["phase_hint"] = out.get("phase_hint") or "unknown"
    return out


def load_response_records(path: Path) -> dict:
    if not path.exists():
        return {"schema": "none", "records": []}
    data = load_json(path)
    if data.get("schema") == "parking_action_response_model.v2":
        return data
    records = []
    for record in data.get("records", []):
        if record.get("action_id"):
            records.append(record)
    return {"schema": data.get("schema", "parking_action_response_model.v1"), "records": records}


def state_x_sign(value: float) -> str:
    if value > 3.0:
        return "+"
    if value < -3.0:
        return "-"
    return "0"


def state_x_bin(value: float) -> str:
    v = abs(value)
    if v < 40.0:
        return "0-40"
    if v < 120.0:
        return "40-120"
    return "120+"


def state_heading_bin(value: float) -> str:
    if value < -8.0:
        return "-8-"
    if value < 0.0:
        return "-8-0"
    if value <= 8.0:
        return "0-8"
    return "8+"


def bucket_for_state(state: dict) -> dict:
    return {
        "phase": state.get("phase_hint", "unknown"),
        "x_err_sign": state_x_sign(to_float(state.get("slot_x_err_px"))),
        "x_err_bin": state_x_bin(to_float(state.get("slot_x_err_px"))),
        "heading_bin": state_heading_bin(to_float(state.get("slot_heading_err_deg"))),
    }


def response_delta(record: dict) -> dict:
    return record.get("mean_delta") or record.get("delta") or {}


def select_response_record(action_id: str, state: dict, responses: dict) -> tuple[dict | None, str]:
    records = [r for r in responses.get("records", []) if r.get("action_id") == action_id]
    if not records:
        return None, "prior"
    if responses.get("schema") != "parking_action_response_model.v2":
        return records[0], "measured"
    bucket = bucket_for_state(state)
    exact = [r for r in records if r.get("bucket") == bucket]
    if exact:
        exact.sort(key=lambda r: to_float(r.get("confidence")), reverse=True)
        return exact[0], "measured"
    same_sign = [
        r for r in records
        if (r.get("bucket") or {}).get("phase") == bucket.get("phase")
        and (r.get("bucket") or {}).get("x_err_sign") == bucket.get("x_err_sign")
    ]
    if same_sign:
        same_sign.sort(key=lambda r: to_float(r.get("confidence")), reverse=True)
        neighbor = dict(same_sign[0])
        neighbor["confidence"] = round(to_float(neighbor.get("confidence")) * 0.5, 3)
        neighbor["bucket_match"] = "same_phase_sign_neighbor"
        return neighbor, "measured_neighbor"
    records.sort(key=lambda r: to_float(r.get("confidence")), reverse=True)
    fallback = dict(records[0])
    fallback["confidence"] = round(to_float(fallback.get("confidence")) * 0.25, 3)
    fallback["bucket_match"] = "action_only_neighbor"
    return fallback, "measured_neighbor"


def predicted_state(state: dict, action: dict, measured: dict | None) -> tuple[dict, float, str]:
    source = measured or action
    delta = response_delta(source) if measured else action.get("prior_delta")
    confidence = to_float(source.get("confidence") if measured else action.get("prior_confidence"), 0.0)
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
        pred[key] = to_float(pred.get(key)) + to_float(delta.get(key), 0.0)
    pred["line_risk"] = pred["min_margin_px"] < 34.0
    return pred, confidence, origin


def cost_state(pred: dict, action: dict, current: dict, library: dict, confidence: float, origin: str) -> tuple[float, dict]:
    scoring = library.get("scoring", {})
    target = scoring.get("target", {})
    weights = scoring.get("weights", {})

    slot_x_abs = abs(to_float(pred.get("slot_x_err_px")) - to_float(target.get("slot_x_err_px")))
    heading_abs = abs(to_float(pred.get("slot_heading_err_deg")) - to_float(target.get("slot_heading_err_deg")))
    lateral_abs = abs(to_float(pred.get("slot_lateral_cm")) - to_float(target.get("slot_lateral_cm")))
    min_margin = to_float(pred.get("min_margin_px"))
    margin_shortfall = max(0.0, to_float(target.get("min_margin_px"), 90.0) - min_margin)
    progress = max(0.0, to_float(current.get("slot_y_dist_cm")) - to_float(pred.get("slot_y_dist_cm")))
    phase_mismatch = 0.0 if current.get("phase_hint") in action.get("allowed_phases", []) else 1.0
    line_risk = 1.0 if pred.get("line_risk") else 0.0
    low_confidence = max(0.0, 1.0 - confidence)
    uncalibrated = 0.0 if origin.startswith("measured") else 1.0
    large_steer = abs(to_float(action.get("servo"), 90.0) - 90.0) / 45.0

    parts = {
        "slot_x_err_abs": slot_x_abs * to_float(weights.get("slot_x_err_abs"), 1.0),
        "slot_heading_err_abs": heading_abs * to_float(weights.get("slot_heading_err_abs"), 4.0),
        "slot_lateral_abs": lateral_abs * to_float(weights.get("slot_lateral_abs"), 8.0),
        "progress_bonus": -progress * to_float(weights.get("progress"), 0.35),
        "min_margin_shortfall": margin_shortfall * to_float(weights.get("min_margin_shortfall"), 2.5),
        "line_risk": line_risk * to_float(weights.get("line_risk"), 1000.0),
        "phase_mismatch": phase_mismatch * to_float(weights.get("phase_mismatch"), 25.0),
        "low_confidence": low_confidence * to_float(weights.get("low_confidence"), 20.0),
        "uncalibrated": uncalibrated * to_float(weights.get("uncalibrated"), 15.0),
        "large_steer": large_steer * to_float(weights.get("large_steer"), 3.0),
    }
    return round(sum(parts.values()), 3), {k: round(v, 3) for k, v in parts.items()}


def score_actions(state: dict, library: dict, responses: dict) -> list[dict]:
    return bpc.planner_score_actions(state, library, responses, real_motion=False, criteria=None)


def select_states(rows: list[dict], tail: int) -> list[dict]:
    stable = [row for row in rows if row.get("stable") and row.get("stable_enough") and not row.get("line_risk")]
    if not stable:
        stable = [row for row in rows if row.get("stable")]
    return stable[-tail:] if tail > 0 else stable


def analyze(paths: list[Path], library_path: Path, response_path: Path, tail: int) -> dict:
    library = load_json(library_path)
    responses = load_response_records(response_path)
    states = []
    for path in paths:
        states.extend(read_states(path))
    selected = select_states(states, tail)
    decisions = []
    for state in selected:
        ranked = score_actions(state, library, responses)
        decisions.append({
            "source": {
                "file": state.get("file"),
                "lineno": state.get("lineno"),
                "phase_hint": state.get("phase_hint"),
                "slot_x_err_px": round(to_float(state.get("slot_x_err_px")), 3),
                "slot_heading_err_deg": round(to_float(state.get("slot_heading_err_deg")), 3),
                "slot_lateral_cm": round(to_float(state.get("slot_lateral_cm")), 3),
                "slot_y_dist_cm": round(to_float(state.get("slot_y_dist_cm")), 3),
                "min_margin_px": round(to_float(state.get("min_margin_px")), 3),
                "pose_quality": round(to_float(state.get("pose_quality")), 3),
            },
            "best_action": ranked[0] if ranked else None,
            "ranked_actions": ranked,
        })
    top_counts = {}
    for decision in decisions:
        best = decision.get("best_action") or {}
        action_id = best.get("action_id", "none")
        top_counts[action_id] = top_counts.get(action_id, 0) + 1
    return {
        "schema": "parking_action_score_report.v1",
        "library": str(library_path),
        "response_model": str(response_path),
        "input_files": [str(p) for p in paths],
        "state_rows": len(states),
        "selected_state_rows": len(selected),
        "top_action_counts": top_counts,
        "decisions": decisions,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("states", nargs="+", type=Path, help="slot-state JSONL logs or analyzer CSV files")
    ap.add_argument("--library", type=Path, default=ROOT / "configs" / "parking_action_library.json")
    ap.add_argument("--responses", type=Path, default=ROOT / "configs" / "parking_action_response_model.json")
    ap.add_argument("--tail", type=int, default=8)
    ap.add_argument("--out", type=Path, default=ROOT / "artifacts" / "autopark_baseline" / "parking_action_scores.json")
    ns = ap.parse_args()
    report = analyze(ns.states, ns.library, ns.responses, ns.tail)
    ns.out.parent.mkdir(parents=True, exist_ok=True)
    ns.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({
        "out": str(ns.out),
        "state_rows": report["state_rows"],
        "selected_state_rows": report["selected_state_rows"],
        "top_action_counts": report["top_action_counts"],
        "latest_best": report["decisions"][-1]["best_action"] if report["decisions"] else None,
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
