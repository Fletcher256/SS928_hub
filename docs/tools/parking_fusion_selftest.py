#!/usr/bin/env python3
"""Self-test for tools/parking_fusion.py.

Uses only historical text samples; does not connect to board, VM, camera, YOLO,
or STM32.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from parking_fusion import (  # noqa: E402
    PoseFuser,
    load_chassis_signs,
    parse_stm32_line,
    vision_anchor_from_slot_state,
)


def assert_eq(actual, expected, label):
    if actual != expected:
        raise AssertionError("%s: expected %r, got %r" % (label, expected, actual))


def assert_near(actual, expected, tol, label):
    if abs(actual - expected) > tol:
        raise AssertionError("%s: expected %.6f +/- %.6f, got %.6f" % (label, expected, tol, actual))


def main() -> int:
    signs = load_chassis_signs(ROOT / "configs" / "chassis_signs.json")
    assert_eq(signs.yaw_cw_positive, True, "yaw_cw_positive")
    assert_eq(signs.odom_d_reverse_negative, False, "odom_d_reverse_negative")
    assert_eq(signs.vision_lateral_left_negative, False, "vision_lateral_left_negative")

    tlm = parse_stm32_line("TLM 10 YAW=1.4 X=0.6 Y=-3.5 D=3.6 V=-6.5 ANG=120.0 IMU=OK")
    assert_eq(tlm["type"], "tlm", "tlm type")
    assert_eq(tlm["n"], 10, "tlm n")
    assert_near(tlm["yaw"], 1.4, 0.001, "tlm yaw")
    assert_eq(tlm["imu"], "OK", "tlm imu")

    done = parse_stm32_line("DONE 8521 ARC X=0.7 Y=-4.0 D=4.0 YAW=1.5")
    assert_eq(done["type"], "done", "done type")
    assert_eq(done["seq"], 8521, "done seq")
    assert_eq(done["cmd"], "ARC", "done cmd")
    assert_near(done["d"], 4.0, 0.001, "done d")

    stat = parse_stm32_line(
        "STAT 8523 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 "
        "YAW=1.5 X=0.8 Y=-4.4 D=4.5 VEL=0.0 DROP=0 IMU=OK"
    )
    assert_eq(stat["type"], "stat", "stat type")
    assert_eq(stat["mode"], "IDLE", "stat mode")
    assert_eq(stat["drop"], 0, "stat drop")

    slot_state = {
        "ground_estimate": {
            "slot_lateral_cm": -3.949,
            "slot_y_dist_cm": 48.331,
            "slot_axis_heading_deg": 2.0,
        }
    }
    anchor = vision_anchor_from_slot_state(slot_state)
    assert_near(anchor["x_s_cm"], -3.949, 0.001, "anchor x")
    assert_near(anchor["y_s_cm"], -48.331, 0.001, "anchor y")

    fuser = PoseFuser(signs)
    fuser.anchor_vision(slot_state)
    fuser.ingest_tlm(parse_stm32_line("TLM 0 YAW=-0.1 X=0.0 Y=0.0 D=0.0 V=0.0 ANG=120.0 IMU=OK"))
    snap = fuser.ingest_tlm(parse_stm32_line("TLM 10 YAW=1.5 X=0.7 Y=-4.0 D=4.0 V=-6.5 ANG=120.0 IMU=OK"))
    assert_near(snap["dphi_deg"], 1.6, 0.001, "fuser dphi")
    assert_near(snap["ds_cm"], 4.0, 0.001, "fuser ds")
    if not (snap["y_s_cm"] > -48.331):
        raise AssertionError("reverse progress should move y_s toward the slot entrance")

    print(json.dumps({"ok": True, "snapshot": snap}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
