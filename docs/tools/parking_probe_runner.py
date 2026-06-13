#!/usr/bin/env python3
"""Run a one-action parking calibration campaign.

T5 automation:
  1. capture a no-motion reset-quality window
  2. verify the pose is close to a baseline window
  3. execute one bounded primitive probe, only with --execute --allow-risk
  4. capture a no-motion post window
  5. merge logs and update the response model with parking_response_model_updater

The tool never creates /tmp/parking_armed. Real motion still requires the user
to create that file on the board and pass --execute --allow-risk.
"""

from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import parking_response_model_updater as updater
import parking_slot_state_analyzer as state_analyzer


PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"
BOARD_SSH = TOOLS / "board_auto_ssh.py"
DEFAULT_REMOTE_DIR = "/opt/parking/autopark"
DEFAULT_ARTIFACT_DIR = ROOT / "artifacts" / "autopark_baseline"
DEFAULT_BASELINE_LOG = DEFAULT_ARTIFACT_DIR / "parking_action_replanner_dryrun_20260612.jsonl"


def sh_quote(text: str) -> str:
    return "'" + text.replace("'", "'\"'\"'") + "'"


def run_local(cmd: list[str], timeout: float, check: bool = True) -> subprocess.CompletedProcess:
    proc = subprocess.run(cmd, cwd=str(ROOT), text=True, capture_output=True, timeout=timeout)
    if check and proc.returncode != 0:
        raise RuntimeError(
            "command failed rc=%s\nCMD: %s\nSTDOUT:\n%s\nSTDERR:\n%s" %
            (proc.returncode, " ".join(cmd), proc.stdout, proc.stderr)
        )
    return proc


def board_base(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(BOARD_SSH),
        "run",
        "--host", args.host,
        "--user", args.user,
        "--password", args.password,
    ]


def board_put_base(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(BOARD_SSH),
        "put-text",
        "--host", args.host,
        "--user", args.user,
        "--password", args.password,
    ]


def board_run_command(args: argparse.Namespace, remote_command: str, timeout: float, allow_risk: bool) -> subprocess.CompletedProcess:
    cmd = board_base(args) + ["--command-timeout", str(int(timeout))]
    if allow_risk:
        cmd.append("--allow-risk")
    cmd.append(remote_command)
    return run_local(cmd, timeout=timeout + 20.0)


def board_put_file(args: argparse.Namespace, local: Path, remote: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    cmd = board_put_base(args) + ["--allow-risk", str(local), remote]
    return run_local(cmd, timeout=timeout)


def download_remote_file(args: argparse.Namespace, remote: str, local: Path) -> None:
    try:
        import paramiko
    except ImportError as exc:
        raise RuntimeError("paramiko is required to download board logs") from exc
    local.parent.mkdir(parents=True, exist_ok=True)
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname=args.host, username=args.user, password=args.password, timeout=10)
    try:
        command = "base64 %s" % sh_quote(remote)
        _stdin, stdout, stderr = client.exec_command(command, timeout=45)
        data = stdout.read().decode("ascii", errors="ignore")
        err = stderr.read().decode("utf-8", errors="replace")
        rc = stdout.channel.recv_exit_status()
        if rc != 0:
            raise RuntimeError("base64 download failed rc=%s: %s" % (rc, err))
    finally:
        client.close()
    local.write_bytes(base64.b64decode("".join(data.split())))


def iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8-sig", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            row["_lineno"] = lineno
            yield row


def candidate_states(path: Path) -> list[dict]:
    states = []
    for row in iter_jsonl(path):
        if row.get("event") != "candidate":
            continue
        state = state_analyzer.extract_state(row, state_analyzer.default_args())
        if not state:
            continue
        states.append(state_analyzer.flatten_state(path, row.get("_lineno", 0), row, state))
    return states


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def summarize_states(states: list[dict], tail: int = 10) -> dict:
    stable = [s for s in states if s.get("stable") and s.get("stable_enough")]
    selected = (stable or states)[-tail:]
    return {
        "count": len(selected),
        "stable_count": len(stable),
        "slot_x_err_px": round(mean([float(s.get("slot_x_err_px") or 0.0) for s in selected]), 3),
        "slot_heading_err_deg": round(mean([float(s.get("slot_heading_err_deg") or 0.0) for s in selected]), 3),
        "slot_lateral_cm": round(mean([float(s.get("slot_lateral_cm") or 0.0) for s in selected]), 3),
        "slot_y_dist_cm": round(mean([float(s.get("slot_y_dist_cm") or 0.0) for s in selected]), 3),
        "min_margin_px": round(mean([float(s.get("min_margin_px") or 0.0) for s in selected]), 3),
        "phase_hint": selected[-1].get("phase_hint") if selected else "unknown",
        "linenos": [s.get("lineno") for s in selected],
    }


def reset_quality(baseline: dict, current: dict, max_x_delta: float, max_heading_delta: float, min_stable_rows: int) -> dict:
    dx = current["slot_x_err_px"] - baseline["slot_x_err_px"]
    dh = current["slot_heading_err_deg"] - baseline["slot_heading_err_deg"]
    stable_ok = current["stable_count"] >= min_stable_rows
    x_ok = abs(dx) <= max_x_delta
    heading_ok = abs(dh) <= max_heading_delta
    return {
        "pass": bool(stable_ok and x_ok and heading_ok),
        "stable_ok": stable_ok,
        "x_ok": x_ok,
        "heading_ok": heading_ok,
        "slot_x_delta_px": round(dx, 3),
        "heading_delta_deg": round(dh, 3),
        "limits": {
            "max_x_delta_px": max_x_delta,
            "max_heading_delta_deg": max_heading_delta,
            "min_stable_rows": min_stable_rows,
        },
    }


def board_controller_command(args: argparse.Namespace, remote_log: str, duration_sec: float, no_motion: bool) -> str:
    base = [
        "/usr/local/bin/python3",
        args.remote_controller,
    ]
    if no_motion:
        parts = base + [
            "--strategy", "action_replanner",
            "--replanner-dry-run",
            "--duration-sec", "%.1f" % duration_sec,
            "--stable-frames", str(args.stable_frames),
            "--pixel-vision-lost-stop-sec", "%.3f" % args.vision_lost_stop_sec,
            "--action-library-json", args.remote_action_library,
            "--response-model-json", args.remote_response_model,
            "--success-criteria-json", args.remote_success_criteria,
            "--log-jsonl", remote_log,
        ]
    else:
        parts = base + [
            "--strategy", "primitive_probe",
            "--primitive-command", args.primitive_command,
            "--primitive-max-command-abs-d-cm", "%.1f" % args.primitive_max_command_abs_d_cm,
            "--arm",
            "--target-wait-sec", "%.1f" % args.target_wait_sec,
            "--settle-sec", "%.1f" % args.settle_sec,
            "--move-read-sec", "%.1f" % args.move_read_sec,
            "--stable-frames", str(args.stable_frames),
            "--pixel-vision-lost-stop-sec", "%.3f" % args.vision_lost_stop_sec,
            "--max-motion-steps", "1",
            "--max-total-cm", "%.1f" % args.max_total_cm,
            "--log-stm32-detail",
            "--pre-steer-settle-sec", "%.1f" % args.pre_steer_settle_sec,
            "--log-jsonl", remote_log,
        ]
    return " ".join(sh_quote(p) if (" " in p or "=" in p) else p for p in parts)


def merge_logs(paths: list[Path], out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as dst:
        for path in paths:
            with path.open("r", encoding="utf-8-sig", errors="replace") as src:
                for line in src:
                    if line.strip():
                        dst.write(line.rstrip("\n") + "\n")


def action_id_for_command(library_path: Path, command: str) -> str:
    library = json.loads(library_path.read_text(encoding="utf-8"))
    canonical = updater.canonical_command(command)
    mapping = updater.command_to_action_id(library)
    return mapping.get(canonical, "unknown")


def latest_record_for_action(model: dict, action_id: str) -> dict | None:
    records = [r for r in model.get("records", []) if r.get("action_id") == action_id]
    if not records:
        return None
    records.sort(key=lambda r: (r.get("n", 0), json.dumps(r.get("bucket", {}), sort_keys=True)), reverse=True)
    return records[0]


def print_command_plan(args: argparse.Namespace, names: dict[str, str]) -> None:
    reset_cmd = board_controller_command(args, names["remote_reset"], args.reset_duration_sec, True)
    probe_cmd = board_controller_command(args, names["remote_probe"], 0.0, False)
    post_cmd = board_controller_command(args, names["remote_post"], args.post_duration_sec, True)
    reset_local = board_base(args) + ["--command-timeout", str(int(args.reset_duration_sec + 20)), "--allow-risk", reset_cmd]
    probe_local = board_base(args) + ["--command-timeout", str(int(args.probe_timeout_sec)), "--allow-risk", probe_cmd]
    post_local = board_base(args) + ["--command-timeout", str(int(args.post_duration_sec + 20)), "--allow-risk", post_cmd]
    print("T5 command plan:")
    print("1. reset no-motion capture:")
    print("   " + subprocess.list2cmdline(reset_local))
    print("2. real primitive probe:")
    print("   " + subprocess.list2cmdline(probe_local))
    print("3. post no-motion capture:")
    print("   " + subprocess.list2cmdline(post_local))


def run_campaign(args: argparse.Namespace) -> dict:
    stamp = args.stamp or time.strftime("%Y%m%d_%H%M%S")
    safe_action = args.action_label or action_id_for_command(args.library, args.primitive_command)
    prefix = "parking_probe_%s_%s" % (safe_action, stamp)
    artifact_dir = args.artifact_dir
    artifact_dir.mkdir(parents=True, exist_ok=True)
    names = {
        "remote_reset": "/tmp/%s_reset.jsonl" % prefix,
        "remote_probe": "/tmp/%s_motion.jsonl" % prefix,
        "remote_post": "/tmp/%s_post.jsonl" % prefix,
        "local_reset": artifact_dir / ("%s_reset.jsonl" % prefix),
        "local_probe": artifact_dir / ("%s_motion.jsonl" % prefix),
        "local_post": artifact_dir / ("%s_post.jsonl" % prefix),
        "local_combined": artifact_dir / ("%s_combined.jsonl" % prefix),
        "report": artifact_dir / ("%s_report.json" % prefix),
    }
    print_command_plan(args, {k: str(v) for k, v in names.items()})
    if not args.execute:
        return {"executed": False, "reason": "plan_only", "paths": {k: str(v) for k, v in names.items()}}
    if not args.allow_risk:
        raise RuntimeError("--execute requires --allow-risk")

    if args.sync_inputs_to_board:
        board_put_file(args, Path(args.local_controller), args.remote_controller)
        board_put_file(args, args.library, args.remote_action_library)
        board_put_file(args, args.model, args.remote_response_model)
        board_put_file(args, args.success_criteria, args.remote_success_criteria)
        board_run_command(
            args,
            "/usr/local/bin/python3 -m py_compile %s" % sh_quote(args.remote_controller),
            timeout=30,
            allow_risk=True,
        )

    baseline_states = candidate_states(args.baseline_log)
    baseline = summarize_states(baseline_states, args.window_rows)

    reset_cmd = board_controller_command(args, names["remote_reset"], args.reset_duration_sec, True)
    reset_proc = board_run_command(args, reset_cmd, timeout=args.reset_duration_sec + 30.0, allow_risk=True)
    download_remote_file(args, names["remote_reset"], names["local_reset"])
    current = summarize_states(candidate_states(names["local_reset"]), args.window_rows)
    quality = reset_quality(baseline, current, args.max_reset_x_delta_px, args.max_reset_heading_delta_deg, args.min_reset_stable_rows)
    if not quality["pass"]:
        report = {
            "schema": "parking_probe_runner_report.v1",
            "executed": True,
            "probe_executed": False,
            "reason": "reset_quality_failed",
            "baseline": baseline,
            "reset": current,
            "reset_quality": quality,
            "reset_stdout_tail": reset_proc.stdout[-4000:],
            "paths": {k: str(v) for k, v in names.items()},
        }
        names["report"].write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        return report

    if args.require_arm_file:
        arm = board_run_command(
            args,
            "sh -lc '[ -e %s ] && echo ARM_FILE_PRESENT || echo ARM_FILE_MISSING'" % sh_quote(args.arm_file),
            timeout=20,
            allow_risk=False,
        )
        if "ARM_FILE_PRESENT" not in arm.stdout:
            report = {
                "schema": "parking_probe_runner_report.v1",
                "executed": True,
                "probe_executed": False,
                "reason": "arm_file_missing",
                "arm_file": args.arm_file,
                "baseline": baseline,
                "reset": current,
                "reset_quality": quality,
                "paths": {k: str(v) for k, v in names.items()},
            }
            names["report"].write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
            return report

    probe_cmd = board_controller_command(args, names["remote_probe"], 0.0, False)
    probe_proc = board_run_command(args, probe_cmd, timeout=args.probe_timeout_sec, allow_risk=True)
    download_remote_file(args, names["remote_probe"], names["local_probe"])

    post_cmd = board_controller_command(args, names["remote_post"], args.post_duration_sec, True)
    post_proc = board_run_command(args, post_cmd, timeout=args.post_duration_sec + 30.0, allow_risk=True)
    download_remote_file(args, names["remote_post"], names["local_post"])

    merge_logs([names["local_reset"], names["local_probe"], names["local_post"]], names["local_combined"])
    updated_model = updater.update_model(
        [names["local_combined"]],
        args.model,
        args.library,
        args.window_rows,
        args.window_rows,
    )
    args.model.write_text(json.dumps(updated_model, indent=2, ensure_ascii=False), encoding="utf-8")
    action_id = action_id_for_command(args.library, args.primitive_command)
    latest = latest_record_for_action(updated_model, action_id)

    if args.sync_model_to_board:
        board_put_file(args, args.model, args.remote_response_model)

    report = {
        "schema": "parking_probe_runner_report.v1",
        "executed": True,
        "probe_executed": True,
        "primitive_command": args.primitive_command,
        "action_id": action_id,
        "baseline": baseline,
        "reset": current,
        "reset_quality": quality,
        "updated_from": updated_model.get("updated_from", []),
        "latest_record": latest,
        "probe_stdout_tail": probe_proc.stdout[-5000:],
        "post_stdout_tail": post_proc.stdout[-3000:],
        "paths": {k: str(v) for k, v in names.items()},
        "model": str(args.model),
        "synced_model_to_board": bool(args.sync_model_to_board),
    }
    names["report"].write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.137.2")
    ap.add_argument("--user", default="root")
    ap.add_argument("--password", default="ebaina")
    ap.add_argument("--execute", action="store_true", help="actually run board reset/probe/post workflow")
    ap.add_argument("--allow-risk", action="store_true", help="required with --execute; allows real primitive probe motion")
    ap.add_argument("--primitive-command", default="ARC D=-6.0 STE=120 V=1")
    ap.add_argument("--action-label", default="")
    ap.add_argument("--stamp", default="")
    ap.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    ap.add_argument("--baseline-log", type=Path, default=DEFAULT_BASELINE_LOG)
    ap.add_argument("--library", type=Path, default=ROOT / "configs" / "parking_action_library.json")
    ap.add_argument("--model", type=Path, default=ROOT / "configs" / "parking_action_response_model.json")
    ap.add_argument("--success-criteria", type=Path, default=ROOT / "configs" / "parking_success_criteria.json")
    ap.add_argument("--local-controller", default=str(TOOLS / "board_parking_controller.py"))
    ap.add_argument("--remote-controller", default=DEFAULT_REMOTE_DIR + "/board_parking_controller.py")
    ap.add_argument("--remote-action-library", default=DEFAULT_REMOTE_DIR + "/parking_action_library.json")
    ap.add_argument("--remote-response-model", default=DEFAULT_REMOTE_DIR + "/parking_action_response_model.json")
    ap.add_argument("--remote-success-criteria", default=DEFAULT_REMOTE_DIR + "/parking_success_criteria.json")
    ap.add_argument("--sync-inputs-to-board", action="store_true")
    ap.add_argument("--sync-model-to-board", action="store_true")
    ap.add_argument("--require-arm-file", action="store_true", default=True)
    ap.add_argument("--arm-file", default="/tmp/parking_armed")
    ap.add_argument("--reset-duration-sec", type=float, default=8.0)
    ap.add_argument("--post-duration-sec", type=float, default=8.0)
    ap.add_argument("--probe-timeout-sec", type=float, default=70.0)
    ap.add_argument("--window-rows", type=int, default=10)
    ap.add_argument("--min-reset-stable-rows", type=int, default=10)
    ap.add_argument("--max-reset-x-delta-px", type=float, default=5.0)
    ap.add_argument("--max-reset-heading-delta-deg", type=float, default=1.0)
    ap.add_argument("--primitive-max-command-abs-d-cm", type=float, default=8.0)
    ap.add_argument("--target-wait-sec", type=float, default=1.0)
    ap.add_argument("--settle-sec", type=float, default=0.5)
    ap.add_argument("--move-read-sec", type=float, default=8.0)
    ap.add_argument("--stable-frames", type=int, default=3)
    ap.add_argument("--vision-lost-stop-sec", type=float, default=0.5)
    ap.add_argument("--max-total-cm", type=float, default=8.0)
    ap.add_argument("--pre-steer-settle-sec", type=float, default=0.5)
    args = ap.parse_args()

    try:
        report = run_campaign(args)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps({
        "ok": True,
        "executed": report.get("executed"),
        "probe_executed": report.get("probe_executed", False),
        "reason": report.get("reason", ""),
        "reset_quality": report.get("reset_quality"),
        "updated_from": report.get("updated_from", []),
        "latest_record": report.get("latest_record"),
        "paths": report.get("paths"),
        "model": report.get("model"),
    }, ensure_ascii=False))
    return 0 if report.get("reason") not in ("reset_quality_failed", "arm_file_missing") else 2


if __name__ == "__main__":
    raise SystemExit(main())
