#!/usr/bin/env python3
"""Validate parking success/abort criteria without motion or hardware access."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import board_parking_controller as bpc


def synthetic_state(slot_x, heading, y_dist, margin, stable_frames=3, line_risk=False):
    return {
        "schema": "slot_relative_state.v1.synthetic",
        "confidence": 1.0,
        "stable_frames": stable_frames,
        "required_stable_frames": 3,
        "pose_quality": 1.0,
        "phase_hint": "straighten_or_enter",
        "image": {"slot_heading_err_deg": heading, "closeness": 1.0},
        "corridor": {
            "slot_x_err_px": slot_x,
            "slot_entry_x_err_px": slot_x,
            "left_margin_px": margin,
            "right_margin_px": margin,
            "min_margin_px": margin,
            "line_risk": line_risk,
            "risk_side": "LEFT" if line_risk else "",
        },
        "ground_estimate": {
            "slot_y_dist_cm": y_dist,
            "slot_lateral_cm": 0.0,
            "slot_axis_heading_deg": 0.0,
        },
        "gates": {
            "stable_enough": stable_frames >= 3,
            "line_margin_ok": margin >= 60,
            "heading_ok": abs(heading) <= 4.0,
            "lateral_ok": abs(slot_x) <= 15,
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--criteria", type=Path, default=ROOT / "configs" / "parking_success_criteria.json")
    ap.add_argument("--out", type=Path, default=ROOT / "artifacts" / "autopark_baseline" / "parking_success_criteria_check.json")
    args = ap.parse_args()
    criteria = bpc.load_success_criteria(str(args.criteria))
    cases = {
        "done": synthetic_state(slot_x=8.0, heading=2.0, y_dist=7.0, margin=90.0),
        "continue": synthetic_state(slot_x=76.0, heading=-3.5, y_dist=48.0, margin=93.0),
        "abort": synthetic_state(slot_x=8.0, heading=2.0, y_dist=7.0, margin=35.0, line_risk=True),
    }
    results = {
        name: bpc.evaluate_parking_criteria(state, criteria, steps=0, total_cm=0.0)
        for name, state in cases.items()
    }
    report = {
        "criteria": str(args.criteria),
        "schema": criteria.get("schema"),
        "results": results,
        "passed": (
            results["done"]["verdict"] == "parked" and
            results["continue"]["verdict"] == "continue" and
            results["abort"]["verdict"] == "aborted"
        ),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({
        "out": str(args.out),
        "passed": report["passed"],
        "verdicts": {name: result["verdict"] for name, result in results.items()},
    }, ensure_ascii=False))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
