# Autopark Path Template Planner - 2026-06-13

## Purpose

`path_template_planner` is the first full-path planning strategy for the board-side
parking controller. It replaces pure per-frame steering binding with ranked
multi-command path templates.

The planner still replans after every visible motion step. The difference is that
each decision is made by comparing complete candidate paths, not by directly
mapping one pixel error to one servo angle.

## Runtime Logic

1. Read YOLO parking-slot geometry on the board from UDP `127.0.0.1:24580`.
2. Compute scale-independent corridor features:
   - `slot_x_err_norm`
   - `slot_entry_x_err_norm`
   - `min_margin_norm`
   - heading error
   - closeness
3. Generate candidate paths:
   - `straight_entry`
   - `steer_high_then_straight`
   - `steer_low_then_straight`
   - `steer_high_hold`
   - `steer_low_hold`
4. Score each path using desired steering side, arc count, total distance, and
   normalized side margin.
5. Log the full selected path and all candidates.
6. Execute only the first command, then re-observe and replan.

## Blind Reverse Policy

IMU and odometry may be used after vision loss, but only as bounded dead-reckon:

- Vision must have been stable first.
- A safe anchor must exist.
- STM32 `STAT` must return yaw/odometry.
- Existing step and total-distance caps still apply.
- If there is no prior target/anchor, the controller stops.

This keeps blind motion as a short continuation of an already planned path, not
as a primary perception source.

## Direction Convention

Current empirical path sign:

- Slot appears left in image: prefer `STE=112` first.
- Slot appears right in image: prefer `STE=68` first.
- Slot centered within deadband: prefer straight reverse.

This matches the latest observed run where the user reported the `STE>90`
correction was visually moving in the correct direction.

## Files

Local:

```text
tools/board_parking_controller.py
tools/_run_path_template_planner_dryrun.sh
tools/_run_path_template_planner_once.sh
```

Board:

```text
/opt/parking/autopark/board_parking_controller.py
/opt/parking/autopark/_run_path_template_planner_dryrun.sh
```

## Validation

Completed:

```text
.venv\Scripts\python -m py_compile tools\board_parking_controller.py
board: python3 -m py_compile /opt/parking/autopark/board_parking_controller.py
```

Offline direction check using historical slot geometry:

```text
slot_left_in_image  -> selected steer_high_hold, first ARC D=-6.0 STE=112 V=1
slot_centered       -> selected straight_entry, first MOVE D=-6.0 V=1
slot_right_in_image -> selected steer_low_then_straight, first ARC D=-6.0 STE=68 V=1
```

Live board dry-run result:

```text
YOLO and UDP tee were running.
The latest YOLO frames had count=0, so the planner had no live slot target.
Dry-run therefore logged NO_TARGET and did not produce a path.
```

Next step when YOLO detects the slot again:

```sh
sh /opt/parking/autopark/_run_path_template_planner_dryrun.sh
```

Only after reviewing dry-run output should the real-motion script be executed.
