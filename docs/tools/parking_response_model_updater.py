#!/usr/bin/env python3
"""Update the parking action response model from a one-step probe JSONL log.

The updater is hardware-free. It reads an existing controller JSONL log, finds
each executed STM32 motion event, averages candidate states before and after the
motion, computes the observed delta, assigns a verdict, and writes the v2
bucketed response model.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

import parking_slot_state_analyzer as state_analyzer


MODEL_SCHEMA = "parking_action_response_model.v2"
DEFAULT_NOISE_3SIGMA = {
    "slot_x_err_px": 3.0,
    "slot_heading_err_deg": 0.6,
    "slot_lateral_cm": 0.2,
    "min_margin_px": 2.0,
}


METRIC_KEYS = [
    "slot_y_dist_cm",
    "slot_x_err_px",
    "slot_entry_x_err_px",
    "slot_lateral_cm",
    "slot_heading_err_deg",
    "min_margin_px",
    "left_margin_px",
    "right_margin_px",
    "confidence",
    "pose_quality",
]


COMMAND_RE = re.compile(r"^(?P<kind>MOVE|ARC)\s+D=(?P<d>-?\d+(?:\.\d+)?)(?:\s+STE=(?P<ste>\d+))?")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8-sig", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError as exc:
                row = {"event": "parse_error", "error": str(exc)}
            row["_lineno"] = lineno
            yield row


def to_float(value, default=0.0):
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def mean(values):
    vals = [to_float(v) for v in values if v is not None and v != ""]
    return sum(vals) / len(vals) if vals else 0.0


def parse_command(text: str) -> dict:
    match = COMMAND_RE.search(text or "")
    if not match:
        return {"kind": "UNKNOWN", "raw": text, "distance_cm": 0.0, "servo": 90}
    steer = match.group("ste")
    return {
        "kind": match.group("kind"),
        "distance_cm": float(match.group("d")),
        "servo": int(float(steer)) if steer is not None else 90,
        "raw": text,
    }


def canonical_command(text: str) -> str:
    parsed = parse_command(text)
    if parsed["kind"] == "MOVE":
        return "MOVE D=%.1f V=1" % parsed["distance_cm"]
    if parsed["kind"] == "ARC":
        return "ARC D=%.1f STE=%d V=1" % (parsed["distance_cm"], parsed["servo"])
    return text.strip()


def command_to_action_id(library: dict) -> dict:
    mapping = {}
    for action in library.get("actions", []):
        mapping[canonical_command(action.get("command", ""))] = action.get("id")
    return mapping


def candidate_state(row: dict):
    if row.get("event") != "candidate":
        return None
    state = state_analyzer.extract_state(row, state_analyzer.default_args())
    if not state:
        return None
    flat = state_analyzer.flatten_state(Path(row.get("_file", "")), row.get("_lineno", 0), row, state)
    return flat


def average_states(states: list[dict]) -> dict:
    if not states:
        return {}
    out = {
        "count": len(states),
        "linenos": [s.get("lineno") for s in states],
        "phase_hint": states[-1].get("phase_hint"),
        "stable_rows": sum(1 for s in states if s.get("stable")),
    }
    for key in METRIC_KEYS:
        out[key] = round(mean([s.get(key) for s in states]), 3)
    return out


def x_sign(value: float) -> str:
    if value > 3.0:
        return "+"
    if value < -3.0:
        return "-"
    return "0"


def x_bin(value: float) -> str:
    v = abs(value)
    if v < 40.0:
        return "0-40"
    if v < 120.0:
        return "40-120"
    return "120+"


def heading_bin(value: float) -> str:
    if value < -8.0:
        return "-8-"
    if value < 0.0:
        return "-8-0"
    if value <= 8.0:
        return "0-8"
    return "8+"


def state_bucket(pre: dict) -> dict:
    return {
        "phase": pre.get("phase_hint", "unknown"),
        "x_err_sign": x_sign(to_float(pre.get("slot_x_err_px"))),
        "x_err_bin": x_bin(to_float(pre.get("slot_x_err_px"))),
        "heading_bin": heading_bin(to_float(pre.get("slot_heading_err_deg"))),
    }


def compute_delta(pre: dict, post: dict) -> dict:
    delta = {}
    for key in [
        "slot_y_dist_cm",
        "slot_x_err_px",
        "slot_entry_x_err_px",
        "slot_lateral_cm",
        "slot_heading_err_deg",
        "min_margin_px",
        "left_margin_px",
        "right_margin_px",
    ]:
        delta[key] = round(to_float(post.get(key)) - to_float(pre.get(key)), 3)
    return delta


def classify(pre: dict, post: dict, delta: dict, noise: dict) -> tuple[str, dict]:
    x_abs_delta = abs(to_float(post.get("slot_x_err_px"))) - abs(to_float(pre.get("slot_x_err_px")))
    lat_abs_delta = abs(to_float(post.get("slot_lateral_cm"))) - abs(to_float(pre.get("slot_lateral_cm")))
    heading_abs_delta = abs(to_float(post.get("slot_heading_err_deg"))) - abs(to_float(pre.get("slot_heading_err_deg")))
    margin_delta = to_float(delta.get("min_margin_px"))

    x_thr = to_float(noise.get("slot_x_err_px"), 3.0)
    lat_thr = to_float(noise.get("slot_lateral_cm"), 0.2)
    heading_thr = to_float(noise.get("slot_heading_err_deg"), 0.6)
    margin_thr = to_float(noise.get("min_margin_px"), 2.0)

    flags = {
        "x_abs_delta": round(x_abs_delta, 3),
        "lat_abs_delta": round(lat_abs_delta, 3),
        "heading_abs_delta": round(heading_abs_delta, 3),
        "margin_delta": round(margin_delta, 3),
        "x_improved": x_abs_delta < -x_thr,
        "x_worsened": x_abs_delta > x_thr,
        "lat_improved": lat_abs_delta < -lat_thr,
        "lat_worsened": lat_abs_delta > lat_thr,
        "heading_improved": heading_abs_delta < -heading_thr,
        "heading_worsened": heading_abs_delta > heading_thr,
        "margin_improved": margin_delta > margin_thr,
        "margin_worsened": margin_delta < -margin_thr,
    }
    if flags["x_improved"] and flags["lat_improved"] and margin_delta >= -5.0:
        return "improved", flags
    if flags["x_worsened"] or flags["lat_worsened"] or flags["heading_worsened"] or flags["margin_worsened"]:
        return "worsened", flags
    if flags["x_improved"] or flags["lat_improved"] or flags["heading_improved"] or flags["margin_improved"]:
        return "mixed", flags
    return "neutral", flags


def confidence_for_n(n: int) -> float:
    return round(float(n) / float(n + 2), 3) if n > 0 else 0.0


def empty_model(version="2026-06-12") -> dict:
    return {
        "schema": MODEL_SCHEMA,
        "version": version,
        "noise_3sigma": dict(DEFAULT_NOISE_3SIGMA),
        "records": [],
        "legacy_records": [],
    }


def load_or_migrate_model(path: Path) -> dict:
    if not path.exists():
        return empty_model()
    data = load_json(path)
    if data.get("schema") == MODEL_SCHEMA:
        data.setdefault("noise_3sigma", dict(DEFAULT_NOISE_3SIGMA))
        data.setdefault("records", [])
        return data
    model = empty_model()
    model["legacy_records"] = data.get("records", [])
    return model


def find_record(model: dict, action_id: str, bucket: dict):
    for record in model.get("records", []):
        if record.get("action_id") == action_id and record.get("bucket") == bucket:
            return record
    return None


def recompute_record(record: dict):
    samples = record.get("samples", [])
    record["n"] = len(samples)
    record["confidence"] = confidence_for_n(record["n"])
    mean_delta = {}
    for key in [
        "slot_y_dist_cm",
        "slot_x_err_px",
        "slot_entry_x_err_px",
        "slot_lateral_cm",
        "slot_heading_err_deg",
        "min_margin_px",
        "left_margin_px",
        "right_margin_px",
    ]:
        mean_delta[key] = round(mean([(s.get("delta") or {}).get(key) for s in samples]), 3)
    record["mean_delta"] = mean_delta
    verdict_counts = {}
    for sample in samples:
        verdict = sample.get("verdict", "unknown")
        verdict_counts[verdict] = verdict_counts.get(verdict, 0) + 1
    record["verdict_counts"] = verdict_counts
    if verdict_counts:
        record["dominant_verdict"] = sorted(verdict_counts.items(), key=lambda kv: (-kv[1], kv[0]))[0][0]


def upsert_sample(model: dict, action_id: str, command: str, bucket: dict, sample: dict):
    record = find_record(model, action_id, bucket)
    if record is None:
        record = {
            "action_id": action_id,
            "command": command,
            "bucket": bucket,
            "samples": [],
            "mean_delta": {},
            "n": 0,
            "confidence": 0.0,
        }
        model.setdefault("records", []).append(record)
    sample_id = sample["sample_id"]
    record["samples"] = [s for s in record.get("samples", []) if s.get("sample_id") != sample_id]
    record["samples"].append(sample)
    record["samples"].sort(key=lambda s: s.get("sample_id", ""))
    recompute_record(record)


def transition_samples(log_path: Path, library: dict, pre_window: int, post_window: int) -> list[dict]:
    command_map = command_to_action_id(library)
    rows = []
    for row in iter_jsonl(log_path):
        row["_file"] = str(log_path)
        rows.append(row)
    samples = []
    for idx, row in enumerate(rows):
        if row.get("event") != "stm32_motion_result":
            continue
        command = canonical_command(row.get("candidate_cmd") or "")
        action_id = command_map.get(command)
        if not action_id:
            continue
        before_candidates = [candidate_state(r) for r in rows[:idx] if r.get("event") == "candidate"]
        after_candidates = [candidate_state(r) for r in rows[idx + 1:] if r.get("event") == "candidate"]
        before_candidates = [s for s in before_candidates if s]
        after_candidates = [s for s in after_candidates if s]
        if not before_candidates or not after_candidates:
            continue

        stable_pre = [s for s in before_candidates if s.get("stable")]
        stable_post = [s for s in after_candidates if s.get("stable")]
        pre_states = (stable_pre or before_candidates)[-pre_window:]
        post_states = (stable_post or after_candidates)[:post_window]
        pre = average_states(pre_states)
        post = average_states(post_states)
        delta = compute_delta(pre, post)
        bucket = state_bucket(pre)
        verdict, verdict_flags = classify(pre, post, delta, DEFAULT_NOISE_3SIGMA)
        sample = {
            "sample_id": "%s:%s" % (log_path.as_posix(), row.get("_lineno")),
            "date": "2026-06-12",
            "log": log_path.as_posix(),
            "motion_lineno": row.get("_lineno"),
            "action_id": action_id,
            "command": command,
            "bucket": bucket,
            "pre": pre,
            "post": post,
            "delta": delta,
            "verdict": verdict,
            "verdict_flags": verdict_flags,
            "stm32": {
                "pre_servo_response": row.get("pre_servo_response", ""),
                "pwm_after_pre_servo": row.get("pwm_after_pre_servo", ""),
                "motion_response": row.get("motion_response", ""),
                "stat_after": row.get("stat_after", ""),
            },
        }
        samples.append(sample)
    return samples


def update_model(logs: list[Path], model_path: Path, library_path: Path, pre_window: int, post_window: int) -> dict:
    library = load_json(library_path)
    model = load_or_migrate_model(model_path)
    model["schema"] = MODEL_SCHEMA
    model["noise_3sigma"] = dict(DEFAULT_NOISE_3SIGMA)
    added = []
    for log_path in logs:
        for sample in transition_samples(log_path, library, pre_window, post_window):
            upsert_sample(model, sample["action_id"], sample["command"], sample["bucket"], sample)
            added.append({
                "sample_id": sample["sample_id"],
                "action_id": sample["action_id"],
                "command": sample["command"],
                "bucket": sample["bucket"],
                "delta": sample["delta"],
                "verdict": sample["verdict"],
            })
    model["updated_from"] = added
    model["records"].sort(key=lambda r: (r.get("action_id", ""), json.dumps(r.get("bucket", {}), sort_keys=True)))
    return model


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--model", type=Path, default=ROOT / "configs" / "parking_action_response_model.json")
    ap.add_argument("--library", type=Path, default=ROOT / "configs" / "parking_action_library.json")
    ap.add_argument("--out", type=Path, default=None, help="default: overwrite --model")
    ap.add_argument("--pre-window", type=int, default=10)
    ap.add_argument("--post-window", type=int, default=10)
    args = ap.parse_args()
    out = args.out or args.model
    model = update_model(args.logs, args.model, args.library, args.pre_window, args.post_window)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(model, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({
        "out": str(out),
        "schema": model.get("schema"),
        "record_count": len(model.get("records", [])),
        "updated_from": model.get("updated_from", []),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
