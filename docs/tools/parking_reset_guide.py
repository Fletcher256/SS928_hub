#!/usr/bin/env python3
"""Guide manual reset to the parking calibration baseline.

Default mode is offline: compare an existing no-motion JSONL log against the
baseline dry-run window. Live board sampling requires both --execute and
--allow-risk. Live mode only runs board_parking_controller in
action_replanner/replanner-dry-run mode; it does not open STM32 or send motion.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import parking_probe_runner as probe_runner


PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"
DEFAULT_ARTIFACT_DIR = ROOT / "artifacts" / "autopark_baseline"
DEFAULT_BASELINE_LOG = DEFAULT_ARTIFACT_DIR / "parking_action_replanner_dryrun_20260612.jsonl"


def round_value(value, digits: int = 3):
    if value is None:
        return None
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return value


def build_target(args: argparse.Namespace) -> dict:
    if args.target_json:
        data = json.loads(args.target_json.read_text(encoding="utf-8"))
        if "baseline" in data:
            return data["baseline"]
        if "target" in data:
            return data["target"]
        return data
    states = probe_runner.candidate_states(args.baseline_log)
    return probe_runner.summarize_states(states, args.window_rows)


def build_current(path: Path, args: argparse.Namespace) -> dict:
    states = probe_runner.candidate_states(path)
    return probe_runner.summarize_states(states, args.window_rows)


def compare_pose(target: dict, current: dict, args: argparse.Namespace) -> dict:
    if not current.get("count"):
        return {
            "ready_for_t5": False,
            "recommended_close": False,
            "reason": "no_state_rows",
            "stable_ok": False,
            "x_ok": False,
            "heading_ok": False,
            "y_ok": False,
            "deltas": {},
            "limits": limits(args),
        }
    quality = probe_runner.reset_quality(
        target,
        current,
        args.max_x_delta_px,
        args.max_heading_delta_deg,
        args.min_stable_rows,
    )
    y_delta = float(current.get("slot_y_dist_cm", 0.0)) - float(target.get("slot_y_dist_cm", 0.0))
    y_ok = abs(y_delta) <= args.max_y_delta_cm
    deltas = {
        "slot_x_delta_px": quality["slot_x_delta_px"],
        "slot_y_dist_delta_cm": round(y_delta, 3),
        "heading_delta_deg": quality["heading_delta_deg"],
        "slot_lateral_delta_cm": round(
            float(current.get("slot_lateral_cm", 0.0)) - float(target.get("slot_lateral_cm", 0.0)),
            3,
        ),
    }
    ready_for_t5 = bool(quality["pass"])
    recommended_close = bool(ready_for_t5 and y_ok)
    return {
        "ready_for_t5": ready_for_t5,
        "recommended_close": recommended_close,
        "stable_ok": quality["stable_ok"],
        "x_ok": quality["x_ok"],
        "heading_ok": quality["heading_ok"],
        "y_ok": y_ok,
        "deltas": deltas,
        "limits": limits(args),
    }


def limits(args: argparse.Namespace) -> dict:
    return {
        "min_stable_rows": args.min_stable_rows,
        "max_x_delta_px": args.max_x_delta_px,
        "max_y_delta_cm": args.max_y_delta_cm,
        "max_heading_delta_deg": args.max_heading_delta_deg,
    }


def fmt_ok(ok: bool) -> str:
    return "OK" if ok else "ADJUST"


def signed_need(delta: float, unit: str) -> str:
    needed = -delta
    sign = "+" if needed >= 0 else ""
    return "%s%.3f %s" % (sign, needed, unit)


def guidance(target: dict, current: dict, review: dict) -> dict:
    deltas = review.get("deltas") or {}
    x_delta = float(deltas.get("slot_x_delta_px", 0.0))
    y_delta = float(deltas.get("slot_y_dist_delta_cm", 0.0))
    heading_delta = float(deltas.get("heading_delta_deg", 0.0))
    hints = {
        "x_err": "hold" if review.get("x_ok") else "change slot_x_err_px by %s" % signed_need(x_delta, "px"),
        "y_dist": "hold" if review.get("y_ok") else "change slot_y_dist_cm by %s" % signed_need(y_delta, "cm"),
        "heading": "hold" if review.get("heading_ok") else "change heading by %s" % signed_need(heading_delta, "deg"),
    }
    if not review.get("stable_ok"):
        hints["stability"] = "hold the car/camera steady until stable rows reach %s" % review["limits"]["min_stable_rows"]
    else:
        hints["stability"] = "stable"
    hints["note"] = (
        "Use the numeric deltas as the source of truth. Physical left/right depends on camera mounting; "
        "adjust a little, rerun, and keep the delta moving toward zero."
    )
    return hints


def print_status(target: dict, current: dict, review: dict, source: str) -> None:
    hints = guidance(target, current, review)
    deltas = review.get("deltas") or {}
    print("", flush=True)
    print("=== PARKING RESET GUIDE ===", flush=True)
    print("source: %s" % source, flush=True)
    print(
        "stable rows: current=%s target_min=%s [%s]" %
        (current.get("stable_count", 0), review["limits"]["min_stable_rows"], fmt_ok(review.get("stable_ok", False)))
    , flush=True)
    print(
        "slot_x_err_px: current=%s target=%s delta=%s limit=+/-%.1f [%s]" %
        (
            round_value(current.get("slot_x_err_px")),
            round_value(target.get("slot_x_err_px")),
            round_value(deltas.get("slot_x_delta_px")),
            review["limits"]["max_x_delta_px"],
            fmt_ok(review.get("x_ok", False)),
        )
    , flush=True)
    print(
        "slot_y_dist_cm: current=%s target=%s delta=%s guide_limit=+/-%.1f [%s]" %
        (
            round_value(current.get("slot_y_dist_cm")),
            round_value(target.get("slot_y_dist_cm")),
            round_value(deltas.get("slot_y_dist_delta_cm")),
            review["limits"]["max_y_delta_cm"],
            fmt_ok(review.get("y_ok", False)),
        )
    , flush=True)
    print(
        "heading_deg: current=%s target=%s delta=%s limit=+/-%.1f [%s]" %
        (
            round_value(current.get("slot_heading_err_deg")),
            round_value(target.get("slot_heading_err_deg")),
            round_value(deltas.get("heading_delta_deg")),
            review["limits"]["max_heading_delta_deg"],
            fmt_ok(review.get("heading_ok", False)),
        )
    , flush=True)
    print("phase: current=%s target=%s" % (current.get("phase_hint"), target.get("phase_hint")), flush=True)
    print("hint x: %s" % hints["x_err"], flush=True)
    print("hint y: %s" % hints["y_dist"], flush=True)
    print("hint heading: %s" % hints["heading"], flush=True)
    print("ready_for_t5_gate: %s" % ("YES" if review.get("ready_for_t5") else "NO"), flush=True)
    print("recommended_close: %s" % ("YES" if review.get("recommended_close") else "NO"), flush=True)


def report_for(target: dict, current: dict, review: dict, source: str, args: argparse.Namespace) -> dict:
    return {
        "schema": "parking_reset_guide_report.v1",
        "source": source,
        "baseline_log": str(args.baseline_log),
        "target": target,
        "current": current,
        "review": review,
        "guidance": guidance(target, current, review),
    }


def write_report(report: dict, args: argparse.Namespace) -> None:
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    if args.jsonl_out:
        args.jsonl_out.parent.mkdir(parents=True, exist_ok=True)
        with args.jsonl_out.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(report, ensure_ascii=False, separators=(",", ":")) + "\n")


def board_capture_command(args: argparse.Namespace, remote_log: str) -> str:
    return probe_runner.board_controller_command(args, remote_log, args.capture_sec, True)


def run_board_capture(args: argparse.Namespace, remote_log: str) -> subprocess.CompletedProcess:
    """Run one live no-motion capture.

    The board controller can return nonzero for useful reset states, for
    example ABORT_BY_CRITERIA/min_margin_below_floor. For reset guidance that is
    diagnostic data, so keep the process result and still try to download the
    JSONL log.
    """
    cmd = probe_runner.board_base(args) + [
        "--command-timeout", str(int(args.capture_sec + 30)),
        "--allow-risk",
        board_capture_command(args, remote_log),
    ]
    return subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        capture_output=True,
        timeout=args.capture_sec + 60.0,
    )


def print_live_command_plan(args: argparse.Namespace, remote_log: str) -> None:
    remote_cmd = board_capture_command(args, remote_log)
    local_cmd = probe_runner.board_base(args) + [
        "--command-timeout", str(int(args.capture_sec + 30)),
        "--allow-risk",
        remote_cmd,
    ]
    print("Live reset-guide board command:", flush=True)
    print(subprocess.list2cmdline(local_cmd), flush=True)
    print("Purpose: capture a no-motion action_replanner dry-run window for reset guidance.", flush=True)
    print("Risk: starts the board parking controller and writes a /tmp JSONL log, but uses --replanner-dry-run and sends no STM32 motion.", flush=True)


def maybe_sync_inputs(args: argparse.Namespace) -> None:
    if not args.sync_inputs_to_board:
        return
    probe_runner.board_put_file(args, Path(args.local_controller), args.remote_controller)
    probe_runner.board_put_file(args, args.library, args.remote_action_library)
    probe_runner.board_put_file(args, args.model, args.remote_response_model)
    probe_runner.board_put_file(args, args.success_criteria, args.remote_success_criteria)
    probe_runner.board_run_command(
        args,
        "/usr/local/bin/python3 -m py_compile %s" % probe_runner.sh_quote(args.remote_controller),
        timeout=30,
        allow_risk=True,
    )


def run_live(args: argparse.Namespace) -> int:
    if not args.allow_risk:
        raise RuntimeError("--execute requires --allow-risk")
    target = build_target(args)
    stamp = args.stamp or time.strftime("%Y%m%d_%H%M%S")
    args.artifact_dir.mkdir(parents=True, exist_ok=True)
    print_live_command_plan(args, "/tmp/parking_reset_guide_%s_001.jsonl" % stamp)
    maybe_sync_inputs(args)

    iteration = 0
    last_report = None
    try:
        while args.iterations == 0 or iteration < args.iterations:
            iteration += 1
            remote_log = "/tmp/parking_reset_guide_%s_%03d.jsonl" % (stamp, iteration)
            local_log = args.artifact_dir / ("parking_reset_guide_%s_%03d.jsonl" % (stamp, iteration))
            proc = run_board_capture(args, remote_log)
            probe_runner.download_remote_file(args, remote_log, local_log)
            current = build_current(local_log, args)
            review = compare_pose(target, current, args)
            last_report = report_for(target, current, review, str(local_log), args)
            last_report["board_exit_code"] = proc.returncode
            last_report["board_stdout_tail"] = proc.stdout[-3000:]
            last_report["board_stderr_tail"] = proc.stderr[-2000:]
            print_status(target, current, review, str(local_log))
            write_report(last_report, args)
            if args.stop_when_ready and review.get("recommended_close"):
                break
            if args.iterations == 0 or iteration < args.iterations:
                time.sleep(args.delay_sec)
    except KeyboardInterrupt:
        print("\nreset guide stopped by KeyboardInterrupt")
    if last_report:
        print(json.dumps({
            "ok": True,
            "ready_for_t5_gate": last_report["review"].get("ready_for_t5"),
            "recommended_close": last_report["review"].get("recommended_close"),
            "last_source": last_report.get("source"),
            "out": str(args.out) if args.out else "",
            "jsonl_out": str(args.jsonl_out) if args.jsonl_out else "",
        }, ensure_ascii=False))
    return 0


def run_offline(args: argparse.Namespace) -> int:
    target = build_target(args)
    if not args.current_log:
        print("Target reset pose:")
        print(json.dumps(target, indent=2, ensure_ascii=False))
        print("")
        print("Provide --current-log for offline comparison, or use --execute --allow-risk for live no-motion board sampling.")
        return 0
    current = build_current(args.current_log, args)
    review = compare_pose(target, current, args)
    report = report_for(target, current, review, str(args.current_log), args)
    print_status(target, current, review, str(args.current_log))
    write_report(report, args)
    print(json.dumps({
        "ok": True,
        "ready_for_t5_gate": review.get("ready_for_t5"),
        "recommended_close": review.get("recommended_close"),
        "out": str(args.out) if args.out else "",
        "jsonl_out": str(args.jsonl_out) if args.jsonl_out else "",
    }, ensure_ascii=False))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--current-log", type=Path, help="existing no-motion JSONL log to compare offline")
    ap.add_argument("--baseline-log", type=Path, default=DEFAULT_BASELINE_LOG)
    ap.add_argument("--target-json", type=Path, help="optional report/JSON target instead of baseline log")
    ap.add_argument("--out", type=Path, default=DEFAULT_ARTIFACT_DIR / "parking_reset_guide_latest.json")
    ap.add_argument("--jsonl-out", type=Path, default=DEFAULT_ARTIFACT_DIR / "parking_reset_guide_history.jsonl")
    ap.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    ap.add_argument("--window-rows", type=int, default=10)
    ap.add_argument("--min-stable-rows", type=int, default=10)
    ap.add_argument("--max-x-delta-px", type=float, default=5.0)
    ap.add_argument("--max-y-delta-cm", type=float, default=5.0)
    ap.add_argument("--max-heading-delta-deg", type=float, default=1.0)

    ap.add_argument("--execute", action="store_true", help="run live no-motion captures on the board")
    ap.add_argument("--allow-risk", action="store_true", help="required with --execute")
    ap.add_argument("--iterations", type=int, default=1, help="live iterations; 0 means until Ctrl-C")
    ap.add_argument("--capture-sec", type=float, default=8.0)
    ap.add_argument("--delay-sec", type=float, default=1.0)
    ap.add_argument("--stop-when-ready", action="store_true")
    ap.add_argument("--stamp", default="")

    ap.add_argument("--host", default="192.168.137.2")
    ap.add_argument("--user", default="root")
    ap.add_argument("--password", default="ebaina")
    ap.add_argument("--library", type=Path, default=ROOT / "configs" / "parking_action_library.json")
    ap.add_argument("--model", type=Path, default=ROOT / "configs" / "parking_action_response_model.json")
    ap.add_argument("--success-criteria", type=Path, default=ROOT / "configs" / "parking_success_criteria.json")
    ap.add_argument("--local-controller", default=str(TOOLS / "board_parking_controller.py"))
    ap.add_argument("--remote-controller", default=probe_runner.DEFAULT_REMOTE_DIR + "/board_parking_controller.py")
    ap.add_argument("--remote-action-library", default=probe_runner.DEFAULT_REMOTE_DIR + "/parking_action_library.json")
    ap.add_argument("--remote-response-model", default=probe_runner.DEFAULT_REMOTE_DIR + "/parking_action_response_model.json")
    ap.add_argument("--remote-success-criteria", default=probe_runner.DEFAULT_REMOTE_DIR + "/parking_success_criteria.json")
    ap.add_argument("--sync-inputs-to-board", action="store_true")
    ap.add_argument("--stable-frames", type=int, default=3)
    ap.add_argument("--vision-lost-stop-sec", type=float, default=0.5)
    args = ap.parse_args()

    try:
        if args.execute:
            return run_live(args)
        return run_offline(args)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
