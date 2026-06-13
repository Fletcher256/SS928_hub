#!/usr/bin/env python3
"""Replay slot-relative states through the offline action planner.

This is a PC-side validation tool for T4. It reads historical slot-state JSONL
or analyzer CSV files, reuses parking_action_scorer for all action ranking, and
writes row-by-row ranking/chosen decisions. It never talks to the board, STM32,
camera, YOLO process, or any actuator.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import parking_action_scorer as scorer


SCHEMA = "parking_replay_planner_report.v1"
WAIT_ACTION_ID = "WAIT"
STOP_ACTION_ID = "STOP"
ACTIONABLE_BLOCKLIST = {WAIT_ACTION_ID, STOP_ACTION_ID}

STATE_EXPORT_KEYS = [
    "file",
    "lineno",
    "event",
    "stable",
    "stable_enough",
    "line_risk",
    "line_margin_ok",
    "heading_ok",
    "lateral_ok",
    "phase_hint",
    "confidence",
    "pose_quality",
    "slot_x_err_px",
    "slot_entry_x_err_px",
    "slot_heading_err_deg",
    "slot_y_dist_cm",
    "slot_lateral_cm",
    "left_margin_px",
    "right_margin_px",
    "min_margin_px",
    "closeness",
]

CSV_FIELDS = [
    "source_file",
    "lineno",
    "stable",
    "stable_enough",
    "line_risk",
    "phase_hint",
    "slot_x_err_px",
    "slot_y_dist_cm",
    "min_margin_px",
    "chosen_action_id",
    "chosen_command",
    "chosen_reason",
    "best_cost",
    "best_origin",
    "best_confidence",
    "top3",
]


def to_float(value, default=0.0) -> float:
    return scorer.to_float(value, default)


def to_bool(value) -> bool:
    return scorer.to_bool(value)


def round_value(value):
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, float)):
        return round(float(value), 3)
    return value


def parse_servo_from_command(command) -> float | None:
    if not isinstance(command, str):
        return None
    match = re.search(r"\bSTE\s*=\s*(-?\d+(?:\.\d+)?)", command)
    if not match:
        if command.startswith("MOVE"):
            return 90.0
        return None
    return float(match.group(1))


def compact_state(state: dict) -> dict:
    return {key: round_value(state.get(key)) for key in STATE_EXPORT_KEYS if key in state}


def compact_rank_item(item: dict) -> dict:
    out = {
        "action_id": item.get("action_id"),
        "command": item.get("command"),
        "origin": item.get("origin"),
        "confidence": round_value(item.get("confidence")),
        "response_match": item.get("response_match"),
        "response_verdict": item.get("response_verdict"),
        "cost": round_value(item.get("cost")),
        "cost_parts": item.get("cost_parts"),
        "predicted": item.get("predicted"),
        "notes": item.get("notes", ""),
    }
    if item.get("response_bucket") is not None:
        out["response_bucket"] = item.get("response_bucket")
    return out


def make_synthetic_choice(action_id: str, command: str, reason: str) -> dict:
    return {
        "action_id": action_id,
        "command": command,
        "command_servo": parse_servo_from_command(command),
        "reason": reason,
        "cost": None,
        "origin": "gate",
        "confidence": None,
    }


def make_ranked_choice(best: dict, reason: str) -> dict:
    command = best.get("command")
    return {
        "action_id": best.get("action_id"),
        "command": command,
        "command_servo": parse_servo_from_command(command),
        "reason": reason,
        "cost": round_value(best.get("cost")),
        "origin": best.get("origin"),
        "confidence": round_value(best.get("confidence")),
        "response_match": best.get("response_match"),
        "response_verdict": best.get("response_verdict"),
    }


def choose_action(state: dict, ranking: list[dict]) -> dict:
    stable = to_bool(state.get("stable"))
    stable_enough = to_bool(state.get("stable_enough"))
    line_risk = to_bool(state.get("line_risk"))
    if not stable or not stable_enough:
        return make_synthetic_choice(WAIT_ACTION_ID, "WAIT", "wait_unstable")
    if line_risk:
        return make_synthetic_choice(STOP_ACTION_ID, "STOP", "line_risk")
    if not ranking:
        return make_synthetic_choice(WAIT_ACTION_ID, "WAIT", "no_ranked_actions")
    return make_ranked_choice(ranking[0], "best_ranked_action")


def is_stable_actionable(state: dict, chosen: dict) -> bool:
    action_id = chosen.get("action_id")
    return (
        to_bool(state.get("stable"))
        and to_bool(state.get("stable_enough"))
        and not to_bool(state.get("line_risk"))
        and action_id not in ACTIONABLE_BLOCKLIST
    )


def count_switches(action_ids: list[str]) -> int:
    switches = 0
    previous = None
    for action_id in action_ids:
        if previous is None:
            previous = action_id
            continue
        if action_id != previous:
            switches += 1
            previous = action_id
    return switches


def is_right_correction(chosen: dict) -> bool:
    action_id = str(chosen.get("action_id") or "")
    if action_id.startswith("reverse_right_"):
        return True
    servo = chosen.get("command_servo")
    return isinstance(servo, (int, float)) and servo > 90.0


def direction_review(rows: list[dict], threshold_px: float) -> dict:
    right_offset_rows = []
    checked = []
    wrong = []
    skipped = []
    for row in rows:
        state = row["pre_state"]
        chosen = row["chosen"]
        if to_float(state.get("slot_x_err_px")) <= threshold_px:
            continue
        item = {
            "row_index": row["row_index"],
            "file": state.get("file"),
            "lineno": state.get("lineno"),
            "slot_x_err_px": round_value(state.get("slot_x_err_px")),
            "chosen_action_id": chosen.get("action_id"),
            "chosen_command": chosen.get("command"),
            "chosen_reason": chosen.get("reason"),
        }
        right_offset_rows.append(item)
        if not is_stable_actionable(state, chosen):
            skipped.append(item)
            continue
        checked.append(item)
        if not is_right_correction(chosen):
            wrong.append(item)
    return {
        "threshold_px": threshold_px,
        "right_offset_rows": len(right_offset_rows),
        "checked_rows": len(checked),
        "skipped_wait_or_stop_rows": len(skipped),
        "wrong_direction_rows": wrong,
        "pass": len(checked) > 0 and not wrong,
    }


def top_counts(action_ids: list[str]) -> dict:
    counts = {}
    for action_id in action_ids:
        counts[action_id] = counts.get(action_id, 0) + 1
    return counts


def csv_row(row: dict) -> dict:
    state = row["pre_state"]
    chosen = row["chosen"]
    ranking = row["ranking"]
    best = ranking[0] if ranking else {}
    top3 = [
        {
            "action_id": item.get("action_id"),
            "command": item.get("command"),
            "cost": item.get("cost"),
        }
        for item in ranking[:3]
    ]
    return {
        "source_file": state.get("file"),
        "lineno": state.get("lineno"),
        "stable": state.get("stable"),
        "stable_enough": state.get("stable_enough"),
        "line_risk": state.get("line_risk"),
        "phase_hint": state.get("phase_hint"),
        "slot_x_err_px": state.get("slot_x_err_px"),
        "slot_y_dist_cm": state.get("slot_y_dist_cm"),
        "min_margin_px": state.get("min_margin_px"),
        "chosen_action_id": chosen.get("action_id"),
        "chosen_command": chosen.get("command"),
        "chosen_reason": chosen.get("reason"),
        "best_cost": best.get("cost"),
        "best_origin": best.get("origin"),
        "best_confidence": best.get("confidence"),
        "top3": json.dumps(top3, ensure_ascii=False, separators=(",", ":")),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(csv_row(row))


def replay(paths: list[Path], library_path: Path, response_path: Path, max_switches: int, expected_rows: int, right_x_threshold: float) -> dict:
    library = scorer.load_json(library_path)
    responses = scorer.load_response_records(response_path)
    states = []
    for path in paths:
        states.extend(scorer.read_states(path))

    rows = []
    for index, state in enumerate(states, 1):
        ranking = [compact_rank_item(item) for item in scorer.score_actions(state, library, responses)]
        chosen = choose_action(state, ranking)
        rows.append({
            "row_index": index,
            "pre_state": compact_state(state),
            "ranking": ranking,
            "chosen": chosen,
        })

    chosen_ids = [str(row["chosen"].get("action_id") or "") for row in rows]
    stable_action_ids = [
        str(row["chosen"].get("action_id") or "")
        for row in rows
        if is_stable_actionable(row["pre_state"], row["chosen"])
    ]
    review = direction_review(rows, right_x_threshold)
    stable_switches = count_switches(stable_action_ids)
    all_switches = count_switches(chosen_ids)
    expected_rows_enabled = expected_rows > 0
    row_count_pass = (len(states) == expected_rows) if expected_rows_enabled else True
    acceptance = {
        "pass": bool(row_count_pass and stable_switches <= max_switches and review["pass"]),
        "row_count_pass": row_count_pass,
        "expected_rows": expected_rows if expected_rows_enabled else None,
        "max_switches": max_switches,
        "switch_count_pass": stable_switches <= max_switches,
        "direction_review_pass": review["pass"],
    }
    return {
        "schema": SCHEMA,
        "inputs": {
            "state_logs": [str(path) for path in paths],
            "library": str(library_path),
            "response_model": str(response_path),
        },
        "counts": {
            "state_rows": len(states),
            "stable_rows": sum(1 for state in states if to_bool(state.get("stable"))),
            "stable_enough_rows": sum(1 for state in states if to_bool(state.get("stable_enough"))),
            "stable_actionable_rows": len(stable_action_ids),
            "line_risk_rows": sum(1 for state in states if to_bool(state.get("line_risk"))),
            "wait_rows": chosen_ids.count(WAIT_ACTION_ID),
            "stop_rows": chosen_ids.count(STOP_ACTION_ID),
        },
        "top_action_counts": top_counts(chosen_ids),
        "stable_top_action_counts": top_counts(stable_action_ids),
        "action_switch_count_stable": stable_switches,
        "chosen_switch_count_all": all_switches,
        "direction_review": review,
        "acceptance": acceptance,
        "rows": rows,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("states", nargs="+", type=Path, help="slot-state JSONL logs or analyzer CSV files")
    ap.add_argument("--library", type=Path, default=ROOT / "configs" / "parking_action_library.json")
    ap.add_argument("--responses", type=Path, default=ROOT / "configs" / "parking_action_response_model.json")
    ap.add_argument("--out", type=Path, default=ROOT / "artifacts" / "autopark_baseline" / "parking_replay_planner_report.json")
    ap.add_argument("--csv", type=Path, default=ROOT / "artifacts" / "autopark_baseline" / "parking_replay_planner_rows.csv")
    ap.add_argument("--max-switches", type=int, default=2)
    ap.add_argument("--expected-rows", type=int, default=33, help="Set <=0 to disable row-count acceptance check")
    ap.add_argument("--right-x-threshold", type=float, default=40.0)
    ns = ap.parse_args()

    report = replay(ns.states, ns.library, ns.responses, ns.max_switches, ns.expected_rows, ns.right_x_threshold)
    ns.out.parent.mkdir(parents=True, exist_ok=True)
    ns.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    write_csv(ns.csv, report["rows"])
    print(json.dumps({
        "out": str(ns.out),
        "csv": str(ns.csv),
        "state_rows": report["counts"]["state_rows"],
        "stable_actionable_rows": report["counts"]["stable_actionable_rows"],
        "top_action_counts": report["top_action_counts"],
        "stable_top_action_counts": report["stable_top_action_counts"],
        "action_switch_count_stable": report["action_switch_count_stable"],
        "direction_review_pass": report["direction_review"]["pass"],
        "acceptance_pass": report["acceptance"]["pass"],
    }, ensure_ascii=False))
    return 0 if report["acceptance"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
