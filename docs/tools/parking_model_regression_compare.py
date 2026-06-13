#!/usr/bin/env python3
"""Compare two dry-run JSONL logs before/after a YOLO model update.

Inputs are board_parking_controller.py JSONL logs. The comparison intentionally
focuses on control-impacting outputs: detection confidence, slot center,
entrance/axis angle, and candidate command family. It is offline-only and never
connects to hardware.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


def load_candidates(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("event") == "candidate":
            rows.append(row)
    return rows


def cmd_family(row: dict[str, Any]) -> str:
    cmd = str(row.get("candidate_cmd") or "")
    return cmd.split()[0] if cmd else ""


def center_delta(a: dict[str, Any], b: dict[str, Any]) -> float | None:
    ca = a.get("slot_center_cm")
    cb = b.get("slot_center_cm")
    if not (isinstance(ca, list) and isinstance(cb, list) and len(ca) >= 2 and len(cb) >= 2):
        return None
    return math.hypot(float(ca[0]) - float(cb[0]), float(ca[1]) - float(cb[1]))


def deltas(before: list[dict[str, Any]], after: list[dict[str, Any]]) -> dict[str, Any]:
    n = min(len(before), len(after))
    conf_delta = []
    center_delta_cm = []
    axis_yaw_delta = []
    family_changes = 0
    direction_reversal_risk = 0
    for idx in range(n):
        a, b = before[idx], after[idx]
        if isinstance(a.get("confidence"), (int, float)) and isinstance(b.get("confidence"), (int, float)):
            conf_delta.append(float(b["confidence"]) - float(a["confidence"]))
        dc = center_delta(a, b)
        if dc is not None:
            center_delta_cm.append(dc)
        if isinstance(a.get("axis_yaw_deg"), (int, float)) and isinstance(b.get("axis_yaw_deg"), (int, float)):
            axis_yaw_delta.append(abs(float(b["axis_yaw_deg"]) - float(a["axis_yaw_deg"])))
        if cmd_family(a) != cmd_family(b):
            family_changes += 1
        if isinstance(a.get("lat"), (int, float)) and isinstance(b.get("lat"), (int, float)):
            if float(a["lat"]) * float(b["lat"]) < 0:
                direction_reversal_risk += 1
    return {
        "paired_candidates": n,
        "before_candidates": len(before),
        "after_candidates": len(after),
        "confidence_delta_mean": round(mean(conf_delta), 4) if conf_delta else None,
        "center_delta_cm_mean": round(mean(center_delta_cm), 4) if center_delta_cm else None,
        "center_delta_cm_max": round(max(center_delta_cm), 4) if center_delta_cm else None,
        "axis_yaw_delta_deg_mean": round(mean(axis_yaw_delta), 4) if axis_yaw_delta else None,
        "axis_yaw_delta_deg_max": round(max(axis_yaw_delta), 4) if axis_yaw_delta else None,
        "command_family_changes": family_changes,
        "lateral_sign_reversal_risk": direction_reversal_risk,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--before", required=True, type=Path)
    ap.add_argument("--after", required=True, type=Path)
    ap.add_argument("--output", type=Path)
    args = ap.parse_args()

    result = {
        "schema_version": 1,
        "before": str(args.before),
        "after": str(args.after),
        "comparison": deltas(load_candidates(args.before), load_candidates(args.after)),
        "interpretation": {
            "center_delta_cm_mean": "lower is better; investigate if above 2-3 cm on fixed scene",
            "command_family_changes": "MOVE/ARC/STOP family changes on the same scene need review",
            "lateral_sign_reversal_risk": "nonzero means candidate steering direction may reverse",
        },
    }
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
