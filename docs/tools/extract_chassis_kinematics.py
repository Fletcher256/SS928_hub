#!/usr/bin/env python3
"""Extract chassis steering curvature from existing parking JSONL logs.

This is an offline-only tool. It reads previously captured controller logs and
the C2 steering summary, then writes configs/chassis_kinematics.json.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from statistics import mean, pstdev
from typing import Any

from parking_fusion import parse_stm32_line, wrap_degrees


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = ROOT / "artifacts" / "autopark_baseline"
DEFAULT_SUMMARY = DEFAULT_LOG_DIR / "c2_steering_response_summary_20260613.json"
DEFAULT_OUT = ROOT / "configs" / "chassis_kinematics.json"
DEFAULT_AUDIT_OUT = DEFAULT_LOG_DIR / "chassis_kinematics_audit_20260613.json"


ARC_RE = re.compile(r"\bARC\b.*\bSTE=([-+]?\d+(?:\.\d+)?)")
D_RE = re.compile(r"\bD=([-+]?\d+(?:\.\d+)?)")
DATE_RE = re.compile(r"(20\d{6})")
DEFAULT_YAW_FAULT_PATTERNS = [
    "c0_yaw_static_before_zero",
    "c0_yaw_static_after_zero",
]
MIN_CURVATURE_COMMAND_CM = 5.0


def to_float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None or value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def command_servo(command: str) -> int | None:
    m = ARC_RE.search(command or "")
    if not m:
        return None
    return int(round(float(m.group(1))))


def command_distance_abs(command: str) -> float | None:
    m = D_RE.search(command or "")
    if not m:
        return None
    return abs(float(m.group(1)))


def source_log(path_or_name: Path | str | None) -> str:
    if path_or_name is None:
        return ""
    text = str(path_or_name)
    try:
        return str(Path(text).relative_to(ROOT)).replace("\\", "/")
    except (ValueError, OSError):
        return text.replace("\\", "/")


def log_date_from_source(text: str) -> str | None:
    m = DATE_RE.search(text or "")
    if not m:
        return None
    value = m.group(1)
    return f"{value[0:4]}-{value[4:6]}-{value[6:8]}"


def yaw_source_review(source: str, patterns: list[str]) -> tuple[bool, str]:
    lowered = (source or "").lower()
    for pattern in patterns:
        if pattern.lower() in lowered:
            return False, f"matched_yaw_fault_pattern:{pattern}"
    return True, "ok"


def mark_curvature_use(sample: dict[str, Any]) -> dict[str, Any]:
    command_abs = command_distance_abs(sample.get("command") or "")
    sample["command_abs_d_cm"] = None if command_abs is None else round(command_abs, 6)
    if not sample.get("yaw_source_ok", True):
        sample["use_for_curvature"] = False
        sample["curvature_exclusion_reason"] = sample.get("yaw_source_reason") or "yaw_source_not_ok"
    elif command_abs is not None and command_abs < MIN_CURVATURE_COMMAND_CM:
        sample["use_for_curvature"] = False
        sample["curvature_exclusion_reason"] = "small_deadband_probe"
    else:
        sample["use_for_curvature"] = True
        sample["curvature_exclusion_reason"] = ""
    return sample


def parse_stat(text: str) -> dict[str, Any]:
    for line in (text or "").splitlines():
        event = parse_stm32_line(line)
        if event.get("type") == "stat":
            return event
    return {}


def done_event(events: list[dict[str, Any]]) -> dict[str, Any]:
    for event in reversed(events or []):
        if event.get("type") == "done" and event.get("cmd") in ("ARC", "MOVE"):
            return event
    return {}


def tlm_delta(events: list[dict[str, Any]]) -> tuple[float | None, float | None]:
    tlms = [e for e in events or [] if e.get("type") == "tlm"]
    if len(tlms) < 2:
        return None, None
    first = tlms[0]
    last = tlms[-1]
    yaw0 = to_float(first.get("yaw"))
    yaw1 = to_float(last.get("yaw"))
    d0 = to_float(first.get("d"))
    d1 = to_float(last.get("d"))
    if yaw0 is None or yaw1 is None or d0 is None or d1 is None:
        return None, None
    return wrap_degrees(yaw1 - yaw0), abs(d1 - d0)


def sample_from_motion_event(
    path: Path,
    lineno: int,
    row: dict[str, Any],
    yaw_fault_patterns: list[str],
) -> dict[str, Any] | None:
    command = row.get("candidate_cmd") or row.get("cmd") or ""
    servo = command_servo(command)
    if servo is None:
        return None
    before = parse_stat(row.get("stat_before", ""))
    after = parse_stat(row.get("stat_after", ""))
    yaw_before = to_float(before.get("yaw"))
    yaw_after = to_float(after.get("yaw"))
    dist_stat = to_float(after.get("d"))
    if yaw_before is None or yaw_after is None or dist_stat is None or dist_stat <= 0.0:
        return None
    events = row.get("motion_events") or []
    done = done_event(events)
    yaw_done = to_float(done.get("yaw"))
    dist_done = to_float(done.get("d"))
    yaw_tlm, dist_tlm = tlm_delta(events)
    yaw_stat = wrap_degrees(yaw_after - yaw_before)
    source = source_log(path)
    yaw_ok, yaw_reason = yaw_source_review(source, yaw_fault_patterns)
    return mark_curvature_use({
        "source": "jsonl_motion_event",
        "file": source,
        "source_log": source,
        "log_date": log_date_from_source(source),
        "time_unix": row.get("time_unix"),
        "yaw_source_ok": yaw_ok,
        "yaw_source_reason": yaw_reason,
        "lineno": lineno,
        "command": command,
        "servo": servo,
        "yaw_change_stat_deg": round(yaw_stat, 6),
        "dist_stat_cm": round(dist_stat, 6),
        "deg_per_cm_stat": round(yaw_stat / dist_stat, 6),
        "yaw_change_done_deg": None if yaw_done is None else round(wrap_degrees(yaw_done - yaw_before), 6),
        "dist_done_cm": None if dist_done is None else round(dist_done, 6),
        "deg_per_cm_done": (
            None if yaw_done is None or dist_done is None or dist_done <= 0.0
            else round(wrap_degrees(yaw_done - yaw_before) / dist_done, 6)
        ),
        "yaw_change_tlm_deg": None if yaw_tlm is None else round(yaw_tlm, 6),
        "dist_tlm_cm": None if dist_tlm is None else round(dist_tlm, 6),
        "deg_per_cm_tlm": (
            None if yaw_tlm is None or dist_tlm is None or dist_tlm <= 0.0
            else round(yaw_tlm / dist_tlm, 6)
        ),
    })


def sample_from_counter_steer_result(
    path: Path,
    lineno: int,
    row: dict[str, Any],
    yaw_fault_patterns: list[str],
) -> dict[str, Any] | None:
    result = row.get("counter_steer_result") or {}
    if not result:
        return None
    command = row.get("candidate_cmd") or (result.get("decision") or {}).get("command") or ""
    servo = command_servo(command)
    if servo is None:
        return None
    before = parse_stat(row.get("stat_before", ""))
    after = parse_stat(row.get("stat_after", ""))
    yaw_before = to_float(before.get("yaw"), result.get("yaw_before_deg"))
    yaw_after = to_float(after.get("yaw"), result.get("yaw_after_deg"))
    dist_stat = to_float(after.get("d"))
    if yaw_before is None or yaw_after is None or dist_stat is None or dist_stat <= 0.0:
        return None
    yaw_stat = wrap_degrees(yaw_after - yaw_before)
    source = source_log(path)
    yaw_ok, yaw_reason = yaw_source_review(source, yaw_fault_patterns)
    return mark_curvature_use({
        "source": "counter_steer_result",
        "file": source,
        "source_log": source,
        "log_date": log_date_from_source(source),
        "time_unix": row.get("time_unix"),
        "yaw_source_ok": yaw_ok,
        "yaw_source_reason": yaw_reason,
        "lineno": lineno,
        "command": command,
        "servo": servo,
        "yaw_change_stat_deg": round(yaw_stat, 6),
        "dist_stat_cm": round(dist_stat, 6),
        "deg_per_cm_stat": round(yaw_stat / dist_stat, 6),
        "yaw_change_done_deg": None,
        "dist_done_cm": None,
        "deg_per_cm_done": None,
        "yaw_change_tlm_deg": None,
        "dist_tlm_cm": None,
        "deg_per_cm_tlm": None,
        "counter_steer_result": result,
    })


def samples_from_jsonl(log_dir: Path, yaw_fault_patterns: list[str]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for path in sorted(log_dir.glob("*.jsonl")):
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for lineno, line in enumerate(lines, 1):
            if not line.lstrip().startswith("{"):
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row.get("event") not in ("stm32_motion_result", "final_blind_reverse_result"):
                if row.get("event") == "counter_steer_result":
                    sample = sample_from_counter_steer_result(path, lineno, row, yaw_fault_patterns)
                    if sample:
                        out.append(sample)
                continue
            sample = sample_from_motion_event(path, lineno, row, yaw_fault_patterns)
            if sample:
                out.append(sample)
    return out


def samples_from_summary(path: Path, yaw_fault_patterns: list[str]) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    out = []
    for row in data.get("rows", []):
        servo = int(round(float(row.get("servo"))))
        yaw_stat = to_float(row.get("yaw_change_stat_deg"))
        dist_stat = to_float(row.get("dist_stat_cm"))
        if yaw_stat is None or dist_stat is None or dist_stat <= 0.0:
            continue
        source = source_log(row.get("file"))
        yaw_ok, yaw_reason = yaw_source_review(source, yaw_fault_patterns)
        out.append(mark_curvature_use({
            "source": "c2_summary",
            "file": source,
            "source_log": source,
            "log_date": log_date_from_source(source),
            "time_unix": row.get("time_unix"),
            "yaw_source_ok": yaw_ok,
            "yaw_source_reason": yaw_reason,
            "label": row.get("label"),
            "command": row.get("command"),
            "servo": servo,
            "yaw_change_stat_deg": round(yaw_stat, 6),
            "dist_stat_cm": round(dist_stat, 6),
            "deg_per_cm_stat": round(yaw_stat / dist_stat, 6),
            "yaw_change_done_deg": row.get("yaw_change_done_deg"),
            "dist_done_cm": row.get("dist_done_cm"),
            "deg_per_cm_done": (
                None if to_float(row.get("yaw_change_done_deg")) is None
                or to_float(row.get("dist_done_cm")) in (None, 0.0)
                else round(float(row["yaw_change_done_deg"]) / float(row["dist_done_cm"]), 6)
            ),
            "yaw_change_tlm_deg": row.get("yaw_change_tlm_deg"),
            "dist_tlm_cm": row.get("dist_tlm_cm"),
            "deg_per_cm_tlm": (
                None if to_float(row.get("yaw_change_tlm_deg")) is None
                or to_float(row.get("dist_tlm_cm")) in (None, 0.0)
                else round(float(row["yaw_change_tlm_deg"]) / float(row["dist_tlm_cm"]), 6)
            ),
        }))
    return out


def aggregate(samples: list[dict[str, Any]], servos: list[int]) -> list[dict[str, Any]]:
    rows = []
    for servo in servos:
        servo_samples = [
            s for s in samples
            if int(s.get("servo")) == servo and bool(s.get("use_for_curvature", True))
        ]
        values = [float(s["deg_per_cm_stat"]) for s in servo_samples if s.get("deg_per_cm_stat") is not None]
        if values:
            deg = mean(values)
            sigma = pstdev(values) if len(values) > 1 else 0.0
            r_eff = 57.29577951308232 / abs(deg) if abs(deg) > 1e-9 else None
            direction = "right" if deg > 0.0 else "left"
        else:
            deg = None
            sigma = None
            r_eff = None
            direction = "unknown"
        rows.append({
            "ste": servo,
            "direction": direction,
            "r_eff_cm": None if r_eff is None else round(r_eff, 3),
            "deg_per_cm": None if deg is None else round(deg, 6),
            "abs_deg_per_cm": None if deg is None else round(abs(deg), 6),
            "std_deg_per_cm": None if sigma is None else round(sigma, 6),
            "n": len(values),
            "samples": servo_samples,
        })
    return rows


def build_report(samples: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    servos = [60, 75, 105, 120]
    rows = aggregate(samples, servos)
    symmetry = {}
    by_servo = {row["ste"]: row for row in rows}
    for left, right in [(60, 120), (75, 105)]:
        l = by_servo[left].get("abs_deg_per_cm")
        r = by_servo[right].get("abs_deg_per_cm")
        symmetry[f"{left}_{right}"] = None if not l or not r else round(l / r, 3)
    return {
        "schema": "parking_chassis_kinematics.v1",
        "version": "2026-06-13-audited-extracted",
        "generated_from": {
            "log_dir": str(args.log_dir),
            "summary": str(args.summary),
            "audit": str(args.audit_out),
        },
        "units": {
            "ste": "servo_deg",
            "r_eff_cm": "cm",
            "deg_per_cm": "yaw_deg_per_cm_stat_after",
        },
        "steer_curvature": rows,
        "symmetry_ratio_abs_deg_per_cm": symmetry,
        "arc_min_effective_cmd_cm": None,
        "arc_deadband_cm": 2.0,
        "move_deadband_cm": 2.0,
        "coast_after_done_cm": 1.0,
        "notes": [
            "deg_per_cm uses STAT-after yaw delta divided by STAT-after D for each command.",
            "DONE/TLM derivatives are retained per sample for diagnostics.",
            "Samples marked yaw_source_ok=false or use_for_curvature=false are retained in the audit report but excluded from steer_curvature aggregation.",
        ],
    }


def preserve_existing_fields(report: dict[str, Any], out_path: Path) -> None:
    if not out_path.exists():
        return
    try:
        existing = json.loads(out_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    for key in (
        "arc_deadband_samples",
        "arc_min_effective_cmd_cm",
        "arc_deadband_cm",
        "move_deadband_cm",
        "coast_after_done_cm",
        "servo_center_trim_ste",
    ):
        if key in existing:
            report[key] = existing[key]
    old_notes = existing.get("notes") or []
    for note in old_notes:
        if "P0.T1" in str(note) or "arc_deadband_cm" in str(note):
            if note not in report["notes"]:
                report["notes"].append(note)


def build_audit_report(samples: list[dict[str, Any]], report: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    servos = [60, 75, 105, 120]
    excluded = [s for s in samples if not s.get("use_for_curvature", True)]
    yaw_excluded = [s for s in samples if not s.get("yaw_source_ok", True)]
    by_servo = {}
    for servo in servos:
        servo_samples = [s for s in samples if int(s.get("servo")) == servo]
        used = [s for s in servo_samples if s.get("use_for_curvature", True)]
        raw_values = [float(s["deg_per_cm_stat"]) for s in servo_samples if s.get("deg_per_cm_stat") is not None]
        used_values = [float(s["deg_per_cm_stat"]) for s in used if s.get("deg_per_cm_stat") is not None]
        raw_mean = mean(raw_values) if raw_values else None
        used_mean = mean(used_values) if used_values else None
        if raw_mean is None or used_mean is None or abs(raw_mean) < 1e-9:
            change_pct = None
        else:
            change_pct = abs((used_mean - raw_mean) / raw_mean) * 100.0
        by_servo[str(servo)] = {
            "total_samples": len(servo_samples),
            "used_samples": len(used),
            "excluded_samples": len(servo_samples) - len(used),
            "raw_deg_per_cm_mean": None if raw_mean is None else round(raw_mean, 6),
            "used_deg_per_cm_mean": None if used_mean is None else round(used_mean, 6),
            "change_pct_after_exclusions": None if change_pct is None else round(change_pct, 2),
            "needs_probe": bool(change_pct is not None and change_pct > 20.0),
        }
    return {
        "schema": "parking_chassis_kinematics_audit.v1",
        "inputs": {
            "log_dir": str(args.log_dir),
            "summary": str(args.summary),
            "out": str(args.out),
            "yaw_fault_patterns": args.yaw_fault_pattern,
            "min_curvature_command_cm": MIN_CURVATURE_COMMAND_CM,
        },
        "counts": {
            "samples_total": len(samples),
            "samples_used_for_curvature": len([s for s in samples if s.get("use_for_curvature", True)]),
            "samples_excluded": len(excluded),
            "samples_excluded_yaw_fault": len(yaw_excluded),
            "counter_steer_samples_total": len([s for s in samples if s.get("source") == "counter_steer_result"]),
        },
        "by_servo": by_servo,
        "steer_curvature": report.get("steer_curvature", []),
        "excluded_samples": excluded,
        "yaw_fault_excluded_samples": yaw_excluded,
        "samples": samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--audit-out", type=Path, default=DEFAULT_AUDIT_OUT)
    parser.add_argument(
        "--yaw-fault-pattern",
        action="append",
        default=list(DEFAULT_YAW_FAULT_PATTERNS),
        help="case-insensitive source-log substring to exclude as an unsafe YAW sample; repeatable",
    )
    args = parser.parse_args()

    samples = samples_from_summary(args.summary, args.yaw_fault_pattern) + samples_from_jsonl(
        args.log_dir, args.yaw_fault_pattern)
    # Deduplicate by source file + command + stat distance/yaw. Keep both summary
    # and fresh JSONL samples when they differ because they are useful repeats.
    seen = set()
    unique = []
    for sample in samples:
        key = (
            sample.get("file"),
            sample.get("command"),
            sample.get("yaw_change_stat_deg"),
            sample.get("dist_stat_cm"),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(sample)
    report = build_report(unique, args)
    preserve_existing_fields(report, args.out)
    audit = build_audit_report(unique, report, args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.audit_out.parent.mkdir(parents=True, exist_ok=True)
    args.audit_out.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("WROTE", args.out)
    print("WROTE", args.audit_out)
    print(
        "AUDIT samples=%s used=%s excluded=%s yaw_fault_excluded=%s counter_steer=%s" % (
            audit["counts"]["samples_total"],
            audit["counts"]["samples_used_for_curvature"],
            audit["counts"]["samples_excluded"],
            audit["counts"]["samples_excluded_yaw_fault"],
            audit["counts"]["counter_steer_samples_total"],
        )
    )
    for row in report["steer_curvature"]:
        print(
            "STE=%s n=%s deg_per_cm=%s r_eff_cm=%s direction=%s" % (
                row["ste"], row["n"], row["deg_per_cm"], row["r_eff_cm"], row["direction"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
