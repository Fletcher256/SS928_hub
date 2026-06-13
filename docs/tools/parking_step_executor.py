#!/usr/bin/env python3
"""Parking step executor (Windows bridges VM planner <-> board STM32).

Modes:
  (default)  PREVIEW   : read planner next_step, print it, send nothing.
  --send     SINGLE    : send exactly one current candidate (with --allow-motion).
  --auto     CLOSED LOOP: repeatedly read -> stability-gate -> send -> settle ->
             re-read, until aligned / target lost / a safety cap trips / abort.

Closed-loop safety (always on in --auto):
  - requires --allow-motion (motion gate) AND --auto (arm).
  - each step is a small closed-loop MOVE/ARC that auto-stops at the board.
  - re-reads perception between steps; only acts on a fresh + stable target.
  - hard caps: --max-steps and --max-total-cm.
  - rejects any single step whose ground distance exceeds --max-step-cm.
  - stops cleanly on planner 'aligned' or target loss (e.g. close-range slot loss).
  - Ctrl-C (or any error) sends STOP to the board before exiting.

Network note: VM (planner) and board (STM32) need not share a LAN; this tool runs
on Windows which can reach both (VM via NAT IP, board via its current IP).
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import paramiko


ROOT = Path(__file__).resolve().parent
STM32_SEND = ROOT / "stm32_send.py"
PYTHON = sys.executable


def vm_read_path_cm(host: str, user: str, password: str, topic: str, timeout: float) -> str:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password, timeout=20,
                   banner_timeout=20, auth_timeout=20)
    try:
        cmd = (
            "bash -lc 'source /opt/ros/humble/setup.bash "
            "&& source ~/parking_ws/install/setup.bash "
            f"&& timeout 10 ros2 topic echo --once --full-length {topic} std_msgs/msg/String 2>/dev/null'"
        )
        _stdin, stdout, _stderr = client.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode("utf-8", errors="replace")
        stdout.channel.recv_exit_status()
        return out
    finally:
        client.close()


def find_str(text: str, key: str) -> str | None:
    m = re.search(rf'"{key}":"([^"]*)"', text)
    return m.group(1) if m else None


def find_num(text: str, key: str) -> float | None:
    m = re.search(rf'"{key}":(-?\d+(?:\.\d+)?)', text)
    return float(m.group(1)) if m else None


def read_step(args) -> dict:
    raw = vm_read_path_cm(args.vm_host, args.vm_user, args.vm_pass, args.topic, args.timeout)
    step = {
        "ok": "schema_version" in raw,
        "status": find_str(raw, "status"),
        "input_fresh": '"input_fresh":true' in raw,
        "direction": find_str(raw, "direction"),
        "candidate": find_str(raw, "stm32_candidate_cmd"),
        "distance_cm": find_num(raw, "distance_cm"),
        "cmd_d": find_num(raw, "stm32_command_distance_cm"),
        "servo": find_num(raw, "stm32_servo_deg"),
        "steer_hint": find_num(raw, "steering_hint_deg"),
        "lon": find_num(raw, "longitudinal_remaining_cm"),
        "lat": find_num(raw, "lateral_error_cm"),
        "head": find_num(raw, "heading_error_deg"),
    }
    return step


def classify(step: dict) -> str:
    if not step["ok"]:
        return "no_data"
    if step["status"] == "aligned":
        return "aligned"
    if (not step["input_fresh"]) or step["status"] in {"waiting_for_target", "no_target_pose", "invalid_target"}:
        return "no_target"
    if step["status"] == "planning" and step["candidate"] and step["candidate"] not in {"", "STOP"}:
        return "actionable"
    return "not_actionable"


def print_step(step: dict) -> None:
    print("=== PARKING NEXT STEP (from VM planner) ===")
    print(f"  planner status : {step['status']}   input_fresh={step['input_fresh']}")
    if step["lon"] is not None:
        print(f"  errors         : longitudinal_remaining={step['lon']}cm  lateral={step['lat']}cm  heading={step['head']}deg")
    print(f"  next direction : {step['direction']}")
    print(f"  ground step    : {step['distance_cm']} cm   (servo {step['servo']}, steering_hint {step['steer_hint']} deg)")
    print(f"  STM32 candidate: {step['candidate']}   (command D={step['cmd_d']}, deadband-compensated)")


def send_candidate(candidate: str, board_host: str, allow_motion: bool) -> int:
    send_cmd = [PYTHON, str(STM32_SEND), "--host", board_host, "--cmd", candidate]
    if allow_motion:
        send_cmd.append("--allow-motion")
    return subprocess.run(send_cmd).returncode


def send_stop(board_host: str) -> None:
    print(f">>> Sending STOP to board {board_host}")
    subprocess.run([PYTHON, str(STM32_SEND), "--host", board_host, "--cmd", "STOP"])


def consistent(a: dict, b: dict, tol_cm: float) -> bool:
    for key in ("lon", "lat"):
        if a[key] is None or b[key] is None:
            return False
        if abs(a[key] - b[key]) > tol_cm:
            return False
    return True


def read_stat(board_host: str) -> dict | None:
    """Read STM32 STAT and parse IMU yaw (deg, CCW+) and odometry distance D (cm)."""
    r = subprocess.run(
        [PYTHON, str(STM32_SEND), "--host", board_host, "--cmd", "STAT", "--read-sec", "3"],
        capture_output=True, text=True,
    )
    m = re.search(r"YAW=(-?\d+(?:\.\d+)?).*?\bD=(-?\d+(?:\.\d+)?)", r.stdout)
    if not m:
        return None
    return {"yaw": float(m.group(1)), "d": float(m.group(2))}


def predict_slot(anchor: dict, cur_yaw: float, ds: float) -> tuple[float, float]:
    """Dead-reckon the target (lon,lat) in the CURRENT vehicle frame, from the last
    camera anchor, using IMU yaw delta + COMMANDED (calibrated) ground distance ds.

    Vehicle frame: +x = reverse direction (toward slot), +y = left, yaw CCW+.
    The car reversed ds (>0) along +x with mean heading dpsi/2 and the frame rotated
    by dpsi (CCW+); a fixed point transforms as P' = Rot(-dpsi) * (P - displacement).
    ds = sum of commanded ground distances since the anchor (calibrated |D|-2), which
    is more trustworthy than the STM32 odometry D (murky reset semantics + wheel slip).
    """
    dpsi = math.radians(cur_yaw - anchor["yaw"])
    dx = ds * math.cos(dpsi / 2.0)
    dy = ds * math.sin(dpsi / 2.0)
    qx = anchor["lon"] - dx
    qy = anchor["lat"] - dy
    c, s = math.cos(-dpsi), math.sin(-dpsi)
    return (c * qx - s * qy, s * qx + c * qy)


def pursuit_command(lon: float, lat: float, args) -> tuple[str, float, int]:
    """Reverse pure-pursuit command for a target at (lon,lat) in the vehicle frame.
    Mirrors the planner's law so blind (dead-reckoned) steps steer the same way."""
    lookahead = max(args.pp_min_lookahead_cm, math.hypot(lon, lat))
    curvature = 2.0 * lat / (lookahead * lookahead)
    delta = math.degrees(math.atan(args.wheelbase_cm * curvature))
    steer = max(-args.max_steer_deg, min(args.max_steer_deg, delta)) * args.steering_sign
    servo = max(args.servo_min_deg, min(args.servo_max_deg, args.servo_center_deg + steer))
    servo_i = int(round(servo))
    step = min(args.max_step_cm, lon) if lon > 0 else args.max_step_cm
    cmd_d = -(round(step, 1) + args.deadband_cm)
    if abs(servo - args.servo_center_deg) <= args.steering_deadzone_deg:
        cmd = f"MOVE D={cmd_d:.1f} V={args.gear}"
    else:
        cmd = f"ARC D={cmd_d:.1f} STE={servo_i} V={args.gear}"
    return cmd, step, servo_i


def acquire_target(args, wait_sec: float):
    """Poll the planner until a stable, actionable target appears or wait_sec elapses.

    Tolerant of intermittent detection dropouts (the slot detection is flaky at an
    angle / close range): single no_target frames do not abort; we keep polling and
    return the first actionable read, preferring one confirmed stable across two reads.

    Returns (step, state): state in {'aligned', 'ok', 'ok_unconfirmed', 'lost'}.
    """
    deadline = time.monotonic() + wait_sec
    first_valid = None
    while True:
        step = read_step(args)
        cls = classify(step)
        if cls == "aligned":
            return step, "aligned"
        if cls == "actionable":
            if first_valid is None:
                first_valid = step
            elif consistent(first_valid, step, args.stable_tol_cm):
                return step, "ok"
            else:
                first_valid = step
        if time.monotonic() >= deadline:
            break
        time.sleep(0.2)
    return (first_valid, "ok_unconfirmed") if first_valid is not None else (None, "lost")


def run_auto(args) -> int:
    print("=== AUTO CLOSED-LOOP PARKING (vision + commanded-motion dead-reckoning) ===")
    print(f"  caps: max_steps={args.max_steps}, max_total_cm={args.max_total_cm}, max_step_cm={args.max_step_cm}")
    print(f"  vision lost -> blind straight finish <= {args.max_blind_cm}cm, ONLY if aligned "
          f"(|lat|<={args.align_lat_cm}cm, |head|<={args.align_head_deg}deg)")
    print("  Ctrl-C aborts and sends STOP. Each step is a closed-loop auto-stopping move.")
    print()

    steps = 0
    total_cm = 0.0
    committed_head = None
    committed_lat = None
    committed_lon = None
    # most recent VISION fix + how far we have blind-coasted since it
    vis_lon = None
    vis_lat = None
    vis_head = None
    blind_cm = 0.0
    # best (smallest) errors seen, to catch slow steady divergence
    best_lat = None
    best_head = None
    # dead-reckoning anchor: target (lon,lat) + IMU yaw at last vision fix; ds_anchor =
    # commanded ground distance accumulated since that anchor.
    anchor = None
    ds_anchor = 0.0
    try:
        while True:
            step, state = acquire_target(args, args.target_wait_sec)
            if state == "aligned":
                print("AUTO_STOP=ALIGNED: planner reports within tolerance. Parking loop complete.")
                return 0
            if state == "lost":
                if vis_lon is None:
                    print("AUTO_STOP=NO_TARGET: never acquired a slot (position it in the rear camera).")
                    send_stop(args.board_host)
                    return 0

                # ---- preferred: IMU(yaw) + odometry(distance) dead-reckoning ----
                if args.dead_reckon and anchor is not None:
                    cur = read_stat(args.board_host)
                    if cur is None:
                        send_stop(args.board_host)
                        print("AUTO_STOP=NO_STAT: could not read STM32 STAT for dead-reckoning.", file=sys.stderr)
                        return 0
                    lon_p, lat_p = predict_slot(anchor, cur["yaw"], ds_anchor)
                    print(f"--- VISION LOST -> DEAD-RECKON: predicted lon={lon_p:.1f} lat={lat_p:.1f}cm "
                          f"(IMU yaw {cur['yaw']} vs anchor {anchor['yaw']}; commanded ds={round(ds_anchor, 1)}cm)")
                    if lon_p <= args.align_lon_tol_cm:
                        send_stop(args.board_host)
                        print(f"AUTO_STOP=PARKED: dead-reckoned remaining ~{lon_p:.1f}cm <= tol. Parking complete.")
                        return 0
                    if blind_cm >= args.max_blind_cm:
                        send_stop(args.board_host)
                        print(f"AUTO_STOP=DEADRECKON_CAP: dead-reckoned {round(blind_cm, 1)}cm (cap {args.max_blind_cm}); "
                              f"~{lon_p:.1f}cm may remain. STOP.")
                        return 0
                    cmd, step_g, _servo = pursuit_command(lon_p, lat_p, args)
                    print(f">>> dead-reckon step {round(step_g, 1)}cm: {cmd}")
                    rc = send_candidate(cmd, args.board_host, allow_motion=True)
                    if rc != 0:
                        send_stop(args.board_host)
                        print(f"AUTO_STOP=SEND_FAILED rc={rc}. STOP sent.", file=sys.stderr)
                        return rc
                    blind_cm += step_g
                    ds_anchor += step_g
                    steps += 1
                    total_cm += step_g
                    time.sleep(args.settle_sec)
                    continue

                # ---- fallback: straight commanded-motion finish (no IMU), aligned-only ----
                pred_remaining = vis_lon - blind_cm
                aligned_ok = (vis_lat is not None and vis_head is not None
                              and abs(vis_lat) <= args.align_lat_cm and abs(vis_head) <= args.align_head_deg)
                print(f"--- VISION LOST: predicted_remaining~={round(pred_remaining, 1)}cm "
                      f"(last vision lon={vis_lon}, blind-coasted={round(blind_cm, 1)}cm, aligned={aligned_ok})")
                if pred_remaining <= args.align_lon_tol_cm:
                    print(f"AUTO_STOP=PARKED: predicted remaining ~{round(pred_remaining, 1)}cm <= tol. "
                          f"Parking complete (dead-reckoned finish).")
                    return 0
                if not aligned_ok:
                    send_stop(args.board_host)
                    print(f"AUTO_STOP=LOST_WHILE_OFFSET: slot lost with lat={vis_lat}/head={vis_head}; "
                          f"blind steering is unsafe. STOP. Reposition / improve detection.", file=sys.stderr)
                    return 0
                if blind_cm >= args.max_blind_cm:
                    send_stop(args.board_host)
                    print(f"AUTO_STOP=BLIND_CAP: coasted {round(blind_cm, 1)}cm blind (cap {args.max_blind_cm}); "
                          f"~{round(pred_remaining, 1)}cm may remain. STOP.")
                    return 0
                creep = min(args.max_step_cm, pred_remaining)
                cmd = f"MOVE D={-(round(creep, 1) + args.deadband_cm):.1f} V={args.gear}"
                print(f">>> blind coast {round(creep, 1)}cm straight (aligned near-range finish): {cmd}")
                rc = send_candidate(cmd, args.board_host, allow_motion=True)
                if rc != 0:
                    send_stop(args.board_host)
                    print(f"AUTO_STOP=SEND_FAILED rc={rc}. STOP sent.", file=sys.stderr)
                    return rc
                blind_cm += creep
                steps += 1
                total_cm += creep
                time.sleep(args.settle_sec)
                continue

            # ---- vision available: trust it (full steering) ----
            print(f"--- VISION ({state}): lon={step['lon']} lat={step['lat']} "
                  f"head={step['head']} cand={step['candidate']}")

            # divergence guard: only on LATERAL (the objective) vs the best seen.
            # Heading is NOT guarded: steering during reverse intentionally changes
            # heading to translate laterally, so heading growth is expected, not failure.
            cur_head = abs(step["head"]) if step["head"] is not None else 0.0
            cur_lat = abs(step["lat"]) if step["lat"] is not None else 0.0
            if best_lat is not None and cur_lat > best_lat + args.lateral_div_cm:
                send_stop(args.board_host)
                print(f"AUTO_STOP=DIVERGING: lateral grew vs best ({best_lat:.1f}->{cur_lat:.1f}cm). STOP. "
                      f"Steering not reducing lateral for this pose.", file=sys.stderr)
                return 5
            if step["lon"] is not None and committed_lon is not None and step["lon"] > committed_lon + args.lateral_div_cm:
                send_stop(args.board_host)
                print(f"AUTO_STOP=WRONG_WAY: longitudinal grew {committed_lon:.1f}->{step['lon']:.1f}cm. STOP sent.", file=sys.stderr)
                return 5
            best_lat = cur_lat if best_lat is None else min(best_lat, cur_lat)
            best_head = cur_head if best_head is None else min(best_head, cur_head)

            dist = step["distance_cm"] or 0.0
            if dist <= 0 or dist > args.max_step_cm:
                send_stop(args.board_host)
                print(f"AUTO_STOP=BAD_STEP: ground step {dist}cm out of (0,{args.max_step_cm}]. STOP sent.", file=sys.stderr)
                return 4
            if steps >= args.max_steps:
                print(f"AUTO_STOP=MAX_STEPS reached ({args.max_steps}).")
                return 0
            if total_cm + dist > args.max_total_cm:
                print(f"AUTO_STOP=MAX_TOTAL_CM would exceed {args.max_total_cm}cm (committed {total_cm}cm).")
                return 0

            # anchor dead-reckoning to this (pre-step) vision fix: slot pose + IMU yaw
            if args.dead_reckon:
                anc = read_stat(args.board_host)
                if anc is not None and step["lon"] is not None:
                    anchor = {"lon": step["lon"], "lat": step["lat"], "yaw": anc["yaw"]}
                    ds_anchor = 0.0

            print(f">>> step {steps + 1}: sending {step['candidate']}  (commits {dist}cm ground)")
            rc = send_candidate(step["candidate"], args.board_host, allow_motion=True)
            if rc != 0:
                send_stop(args.board_host)
                print(f"AUTO_STOP=SEND_FAILED rc={rc}. STOP sent.", file=sys.stderr)
                return rc

            steps += 1
            total_cm += dist
            committed_head = abs(step["head"]) if step["head"] is not None else 0.0
            committed_lat = abs(step["lat"]) if step["lat"] is not None else 0.0
            committed_lon = step["lon"]
            # refresh the vision fix used for dead-reckoning if vision later drops
            vis_lon = step["lon"]
            vis_lat = step["lat"]
            vis_head = step["head"]
            blind_cm = 0.0
            ds_anchor += dist  # commanded ground distance since the dead-reckon anchor
            print(f"    committed: steps={steps}, total_ground_cm~={round(total_cm, 1)}; settling {args.settle_sec}s")
            time.sleep(args.settle_sec)
    except KeyboardInterrupt:
        print("\n!!! ABORT (Ctrl-C): sending STOP.", file=sys.stderr)
        send_stop(args.board_host)
        return 130


def run_single(args) -> int:
    step = read_step(args)
    if not step["ok"]:
        print("STEP=NO_PLANNER_DATA: could not read a fresh planner message from the VM.", file=sys.stderr)
        return 3
    print_step(step)
    print()
    cls = classify(step)
    if cls != "actionable":
        if cls == "aligned":
            print("STEP=ALIGNED: planner says target reached / within tolerance.")
        elif cls == "no_target":
            print("STEP=NO_TARGET: planner has no fresh slot/target. Nothing to send.")
        else:
            print(f"STEP=NOT_ACTIONABLE: status={step['status']}, candidate={step['candidate']!r}.")
        return 0

    if not args.send:
        print("STEP=PREVIEW_ONLY. To execute this one step, re-run with: --send --allow-motion")
        return 0

    print(f">>> SENDING to board {args.board_host}: {step['candidate']}")
    rc = send_candidate(step["candidate"], args.board_host, args.allow_motion)
    if rc != 0:
        print(f"STEP=SEND_FAILED rc={rc} (needs --allow-motion, or sender rejected the command).", file=sys.stderr)
        return rc
    print()
    print(">>> Re-reading STAT after the step:")
    subprocess.run([PYTHON, str(STM32_SEND), "--host", args.board_host, "--cmd", "STAT"])
    print("\nSTEP=DONE. Re-read perception (run again) for the next step.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vm-host", default="192.168.247.129")
    parser.add_argument("--vm-user", default="ebaina")
    parser.add_argument("--vm-pass", default="ebaina")
    parser.add_argument("--board-host", default="172.20.10.2")
    parser.add_argument("--topic", default="/parking/planner/path_cm")
    parser.add_argument("--send", action="store_true", help="SINGLE mode: send one current candidate.")
    parser.add_argument("--auto", action="store_true", help="CLOSED-LOOP mode: auto-advance steps.")
    parser.add_argument("--allow-motion", action="store_true", help="Required to actually move (MOVE/ARC).")
    parser.add_argument("--timeout", type=float, default=30.0)
    # auto-mode safety caps
    parser.add_argument("--max-steps", type=int, default=8)
    parser.add_argument("--max-total-cm", type=float, default=40.0)
    parser.add_argument("--max-step-cm", type=float, default=10.0)
    parser.add_argument("--stable-tol-cm", type=float, default=3.0)
    parser.add_argument("--heading-div-deg", type=float, default=4.0,
                        help="Auto-abort if heading error grows by more than this over a step.")
    parser.add_argument("--lateral-div-cm", type=float, default=3.0,
                        help="Auto-abort if lateral error (or longitudinal) grows by more than this over a step.")
    parser.add_argument("--settle-sec", type=float, default=0.6)
    parser.add_argument("--target-wait-sec", type=float, default=8.0,
                        help="Per-step window to (re)acquire a stable target despite flaky detection.")
    # vision-lost dead-reckoning (commanded-motion) finish
    parser.add_argument("--max-blind-cm", type=float, default=14.0,
                        help="Max total straight blind-coast distance after vision loss.")
    parser.add_argument("--align-lat-cm", type=float, default=2.0,
                        help="Blind finish only if last-vision |lateral| <= this.")
    parser.add_argument("--align-head-deg", type=float, default=4.0,
                        help="Blind finish only if last-vision |heading| <= this.")
    parser.add_argument("--align-lon-tol-cm", type=float, default=2.0,
                        help="Predicted remaining <= this counts as parked.")
    parser.add_argument("--deadband-cm", type=float, default=2.0,
                        help="Distance deadband for blind MOVE command (D = -(ground+deadband)).")
    parser.add_argument("--gear", type=int, default=1)
    # IMU+odometry dead-reckoning (blind finish through the vision gap)
    parser.add_argument("--dead-reckon", action="store_true",
                        help="On vision loss, use IMU yaw + odometry distance to keep parking (not just straight).")
    parser.add_argument("--wheelbase-cm", type=float, default=14.0)
    parser.add_argument("--pp-min-lookahead-cm", type=float, default=10.0)
    parser.add_argument("--max-steer-deg", type=float, default=25.0)
    parser.add_argument("--steering-sign", type=float, default=1.0)
    parser.add_argument("--servo-center-deg", type=float, default=90.0)
    parser.add_argument("--servo-min-deg", type=float, default=45.0)
    parser.add_argument("--servo-max-deg", type=float, default=135.0)
    parser.add_argument("--steering-deadzone-deg", type=float, default=2.0)
    parser.add_argument("--near-range-cm", type=float, default=15.0,
                        help="Below this last-known remaining, target loss is treated as close-range slot loss.")
    args = parser.parse_args()

    if args.auto:
        if not args.allow_motion:
            print("AUTO mode needs --allow-motion to arm. Re-run with: --auto --allow-motion", file=sys.stderr)
            return 4
        return run_auto(args)
    return run_single(args)


if __name__ == "__main__":
    raise SystemExit(main())
