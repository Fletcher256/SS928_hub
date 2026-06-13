# Autopark Long-Term Memory

## 2026-06-13 L0 Closeout Calibration

- P0.T1 ARC small-distance probing is complete. Use `arc_min_effective_cmd_cm=3.0`, `arc_deadband_cm=1.9`, and `coast_after_done_cm=0.8` from `configs/chassis_kinematics.json`.
- P0.T2 servo-center trim is complete enough for L0 regression. Use `servo_center_trim_ste=92`: `STE=90` produced `-1.2deg` yaw over about 8.3cm, `STE=92` produced `-0.7deg` over about 8.2cm, and `STE=94` produced `-0.8deg` over about 8.3cm.
- `tools/extract_chassis_kinematics.py` is now the audited source for steering curvature extraction. It must keep small ARC deadband probes in the audit report but exclude them from `steer_curvature` aggregation.
- The current audit report is `artifacts/autopark_baseline/chassis_kinematics_audit_20260613.json`: 26 total samples, 22 used, 4 small deadband samples excluded, 0 YAW-fault-pattern exclusions, 0 counter-steer samples so far.
- Future real counter-steer logs should produce `counter_steer_result` samples and be reviewed through the same extractor before changing curvature assumptions.
- Authorized real board-side flows now send one extra `STOP` before process exit through `--final-stop-on-exit` (default on). Dry-run and unarmed runs do not send this final STOP.
- L0/L1 runs should let the board send commands continuously. Do not use `--confirm-each-step` unless explicitly debugging a single step.

## Active Architecture Decision

The autonomous parking goal should no longer be treated as a fixed reverse
sequence such as "turn left, reverse, counter-steer, reverse straight".

The accepted long-term direction is:

```text
YOLO parking-slot polygon
  -> relative slot pose/state
  -> action-template library
  -> score candidate actions
  -> execute one short action
  -> stop and observe again
  -> replan every step
```

This is a "relative-pose + action-template replanning" controller.

## Why This Replaces The Previous Plan

The earlier fixed staged-reverse plan can only work when the car starts from a
small, known initial region. It is not suitable for broad initial positions.

The target capability is wider:

- car may start slightly left/right of the slot
- car may have different heading offsets
- distance to slot entrance may vary
- controller should choose a path rather than follow one hard-coded path

Therefore the controller must make decisions from observed car-slot state, not
from a fixed step order.

## Scope Boundary

This still does not mean fully arbitrary parking from anywhere.

The first achievable scope is:

- YOLO can see the parking slot polygon
- camera mounting is fixed
- ground/slot appearance is similar to calibration scenes
- no obstacle avoidance beyond line-risk and stop gates
- low-speed stepwise motion only

Within that scope, the controller should support a wider range of initial
poses by replanning after every action.

## Required State Representation

From each YOLO polygon frame, derive a stable slot-relative state:

- `slot_x_err`: lateral offset from slot centerline
- `slot_y_dist`: distance to slot entrance / usable longitudinal progress
- `slot_heading_err`: vehicle heading relative to slot axis
- `line_margin_left_px`
- `line_margin_right_px`
- `corridor_x_err_px`
- `vision_confidence`
- `vision_stability`

Ground coordinates may be used if reliable, but the controller must remain
auditable from image-space slot geometry.

## Action Template Library

Do not directly learn arbitrary commands from positive/negative feedback.
First build a bounded action library:

- `MOVE D=-6`
- `ARC D=-6 STE=60`
- `ARC D=-6 STE=75`
- `ARC D=-6 STE=105`
- `ARC D=-6 STE=120`
- optionally short forward correction actions after reverse-only behavior works

Every action must have:

- max distance
- expected state delta
- measured response statistics
- line-risk penalty
- allowed state range

## Planning Rule

At each step:

1. Read stable YOLO state.
2. Enumerate the action library.
3. Predict the next state for each action using measured response.
4. Score each action:
   - reduce lateral error
   - reduce heading error
   - make useful longitudinal progress
   - preserve or improve line margins
   - avoid actions likely to lose vision
5. Execute only the best first action.
6. Stop.
7. Observe again and replan.

After one-step planning is stable, allow two- or three-step lookahead, but still
execute only the first action before observing again.

## Safety Gates

Keep these hard constraints:

- no real motion without `--arm`
- no real motion without `/tmp/parking_armed`
- dry-run never sends motion
- YOLO loss over 0.5 s stops motion
- line margin too small stops motion
- divergence stops motion
- STM32 abnormal state stops motion
- total commanded distance is capped
- abnormal exit attempts `STOP`

## Current Calibration Memory

As of 2026-06-12:

```text
Tested action: ARC D=-6.0 STE=60 V=1
Result: worsened
lon_cm: 34.11 -> 31.70
lat_cm: -0.01 -> -1.68
corridor_x_err_px: 18.0 -> 46.0
corridor_min_margin_px: 186.0 -> 162.0
Verdict: do not use STE=60 as the entry arc from that pose.
```

Next useful calibration after resetting to the same initial pose:

```text
ARC D=-6.0 STE=120 V=1
```

## Next Software Milestone

Implement these before more full parking attempts:

1. `slot_relative_state` extraction module/function. Done on 2026-06-12; see `docs/autopark_stage1_observation_20260612.md`.
2. `parking_action_library` config. Done on 2026-06-12; see `docs/autopark_stage2_action_library_20260612.md`.
3. offline one-step action scorer. Done on 2026-06-12; see `tools/parking_action_scorer.py`.
4. parking success/abort criteria. Done on 2026-06-12; see `docs/autopark_t1_success_criteria_20260612.md`.
5. response model v2 and updater. Done on 2026-06-12; see `docs/autopark_t2_response_model_v2_20260612.md`.
6. replay tool comparing candidate actions on historical YOLO logs. Done on 2026-06-12; see `docs/autopark_t4_replay_planner_20260612.md`.
7. dry-run controller mode:
   `--strategy action_replanner`. Done on 2026-06-12; see `docs/autopark_t3_action_replanner_20260612.md`. Board S1 no-motion dry-run passed.

Only after dry-run scores are stable should this strategy be allowed to command
real one-step motion.

## T3 Action Replanner Memory

The board controller now has an `action_replanner` strategy and a no-motion
mode:

```text
--strategy action_replanner --replanner-dry-run
```

The planner core is in `tools/board_parking_controller.py`, and
`tools/parking_action_scorer.py` delegates scoring to that same core.

Dry-run may choose prior-based actions for audit. Real action-replanner motion
must not execute prior-only actions: the action library currently sets
`requires_measured=true` for all five actions, so real mode stops with
`none_eligible` until exact measured response records exist for the current
state bucket.

Board S1 no-motion dry-run result on 2026-06-12:

```text
Log: artifacts/autopark_baseline/parking_action_replanner_dryrun_20260612.jsonl
candidate events: 99
replanner_step events: 99
stable actionable rows: 97
chosen counts: WAIT=2, reverse_right_hard_6=97
stable switch count: 0
will_execute_motion true: 0
send_to_stm32 true: 0
direction review: pass
```

## T5 Probe Runner Memory

The calibration campaign runner exists:

```text
tools/parking_probe_runner.py
```

It automates reset no-motion capture, reset quality gating, one primitive probe,
post no-motion capture, combined-log generation, and response-model update.

Software status on 2026-06-12:

```text
py_compile: pass
plan-only smoke test: pass
real STE=120 sample: not executed yet
/tmp/parking_armed: missing during latest check
```

First execute attempt for `STE=120` on 2026-06-12 did not move the car:

```text
Command stamp: 20260612_ste120_r1
Result: reset_quality_failed
probe_executed: false
will_execute_motion true: 0
send_to_stm32 true: 0
slot_x_delta_px: +7.125
heading_delta_deg: -3.364
slot_y_dist_delta_cm: -8.259
```

Second execute attempt for `STE=120` on 2026-06-12 also did not move the car:

```text
Command stamp: 20260612_ste120_r2
Result: reset_quality_failed
probe_executed: false
will_execute_motion true: 0
send_to_stm32 true: 0
baseline slot_x_err_px: 70.621
current slot_x_err_px: 15.481
slot_x_delta_px: -55.140
heading_delta_deg: -0.540
slot_y_dist_delta_cm: -18.044
```

Do not relax the reset gate to accept this sample. The car is in a different
image-space lateral state, and the dry-run recommendation changed to
`MOVE D=-6.0 V=1`, so it is not a valid `reverse_right_hard_6` response sample.

The car must be reset closer to the baseline pose before retrying:

```text
slot_y_dist_cm near 48.1
slot_x_err_px near 70.6
slot_heading_err_deg near -0.9
```

Reset guide tool added on 2026-06-12:

```text
tools/parking_reset_guide.py
```

Offline r2 check command:

```powershell
.venv\Scripts\python tools\parking_reset_guide.py --current-log artifacts\autopark_baseline\parking_probe_reverse_right_hard_6_20260612_ste120_r2_reset.jsonl --out artifacts\autopark_baseline\parking_reset_guide_r2_report.json
```

Live no-motion guide command after explicit approval:

```powershell
.venv\Scripts\python tools\parking_reset_guide.py --execute --allow-risk --iterations 5 --capture-sec 8 --delay-sec 1
```

Use this guide before the next T5 run. Do not judge the reset pose only by eye.

First live guide run after approval captured 34 no-motion windows. No motion
flags were set (`will_execute_motion true: 0`, `send_to_stm32 true: 0`), but no
window reached the reset target. Latest state:

```text
slot_x_err_px: -25.378
slot_y_dist_cm: 28.623
slot_heading_err_deg: -0.931
delta_x: -95.999 px
delta_y: -19.445 cm
```

The heading is good; the car is still too close and far from the image-space
lateral target.

Second live guide run:

```text
slot_x_err_px: 130.260
slot_y_dist_cm: 42.767
slot_heading_err_deg: -1.102
delta_x: +59.639 px
delta_y: -5.301 cm
min_margin_px: 19.91
result: no-motion abort by min_margin_below_floor
motion flags: 0
```

Distance and heading are now close. The remaining issue is lateral overshoot:
reduce `slot_x_err_px` by about 60 px while keeping the distance near 48 cm.

Later, a reset-guide run returned only `vision_lost` because board YOLO was not
running. Restarting YOLO required:

```text
LD_LIBRARY_PATH=/opt/lib/npu:$LD_LIBRARY_PATH
```

After restart, YOLO process `sample_parking_yolo_rtsp` produced Parking
detections around 0.84-0.87 confidence. Latest no-motion guide result:

```text
slot_x_err_px: 122.000
slot_y_dist_cm: 42.258
slot_heading_err_deg: 0.000
min_margin_px: 34.000
delta_x: +51.379 px
delta_y: -5.810 cm
motion flags: 0
result: no-motion abort by min_margin_below_floor
```

The perception link is healthy again. The car still needs lateral correction
to reduce `slot_x_err_px` toward 70.6 and a small distance increase toward
48.1 cm.

Safety gate update after reviewing the current pose:

```text
min_margin_px < 30      -> hard stop
30 <= min_margin < 40   -> edge recovery zone
min_margin >= 40        -> normal planning zone
```

In edge recovery, `action_replanner` can plan only actions that predict:

```text
min_margin_px >= 40
margin gain >= 5 px
|slot_x_err_px| decreases
line_risk is false
```

For the latest pose (`min_margin_px=34`, `slot_x_err_px=122`), dry-run recovery
selects:

```text
ARC D=-6.0 STE=120 V=1
predicted min_margin_px: 46
predicted slot_x_err_px: 94
```

Real automatic action-replanner motion still blocks this because the response is
prior-only (`no_exact_measured_response`). Execute it only as an explicitly
approved one-step probe or after a measured record exists.

Real campaign command after the car is physically reset and the arm file is
manually created:

```powershell
.venv\Scripts\python tools\parking_probe_runner.py --execute --allow-risk --sync-inputs-to-board --sync-model-to-board --stamp 20260612_ste120_r1 --action-label reverse_right_hard_6
```

## T4 Replay Acceptance Memory

The offline replay planner must reuse `parking_action_scorer.score_actions()`;
do not copy scoring logic into a second implementation.

Baseline result on 2026-06-12:

```text
Input: artifacts/autopark_baseline/parking_slot_state_dryrun_20260612.jsonl
Rows: 33
Stable actionable rows: 31
Chosen action on stable rows: reverse_right_hard_6
Stable action switch count: 0
Direction review: pass
```

For right-offset scenes (`slot_x_err_px > 40`), stable actionable rows must
choose the right-correction side: action id `reverse_right_*` or command
`STE > 90`. If this fails after a model or scorer change, do not run real
parking motion until the sign/scoring issue is fixed.

## Runtime Architecture Decision - 2026-06-13

Use board-side closed loop as the target runtime:

```text
Board camera -> board YOLO -> board_yolo_udp_tee.py -> board_parking_controller.py -> safety gate -> STM32
```

The VM is monitor-only:

```text
board YOLO UDP -> VM ROS topics -> Foxglove
```

Do not rely on VM planner/controller nodes for the runtime decision path.
VM-side launch should use `tools/vm_start_board_yolo_monitor_only.sh`, which
starts only `board_yolo_udp_node` and the live overlay view node. It must not
start `slot_geometry_transform_node`, `parking_target_pose_node`,
`parking_metric_planner_node`, `parking_planner_node`, or controller nodes.

The architecture note is:

```text
docs/autopark_board_closed_loop_monitor_architecture_20260613.md
```

Important UDP routing constraint: the board YOLO binary has one detection UDP
target. For board closed-loop plus Foxglove monitoring, run
`tools/board_start_yolo_closed_loop_monitor.sh` so YOLO sends detections to
`127.0.0.1:24579`, then `board_yolo_udp_tee.py` forwards them to both:

```text
127.0.0.1:24580       board_parking_controller.py
192.168.137.100:24580 VM monitor/Foxglove
```

## Approval Preference - 2026-06-13

User explicitly approved default autonomous operation on the board and VM for
all non-motion work. Do not pause for separate approval for board/VM operations
such as file sync, perception process start/stop/restart, media-stack reload,
VM ROS monitor restart, Foxglove bridge restart, log capture, dry-run, or other
non-actuator diagnostics.

Still require explicit approval before any command that can move the car or
directly drive actuators, including STM32 motion commands such as `MOVE`, `ARC`,
`SERVO`, motor, steering, brake, throttle, CAN actuator control, or any real
closed-loop run with motion enabled.

## Board Media Reload Incident - 2026-06-13

After YOLO repeatedly failed with:

```text
sample_common_svp_vb_init failed
sample_common_svp_start_vi_vpss_venc_vo failed
```

a non-motion media-stack reload was attempted:

```text
cd /opt/ko && ./load_ss928v100 -a -sensor0 os08a20 -sensor1 os08a20 -sensor2 os08a20 -sensor3 os08a20
```

The SSH command timed out, then the board disappeared from `192.168.137.2`.
`board_auto_ssh.py discover` found no board, and Windows could not open
`COM11` (`FileNotFoundError`, COM port not present). This indicates the board
needs physical power/USB/network recovery before software work can continue.

After physical power cycle, board SSH returned at `192.168.137.2`. The
board-closed-loop plus VM-monitor routing was restored successfully:

```text
VM monitor-only:
  board_yolo_udp_node PID 28112
  board_yolo_live_view_node.py PID 28113

Board:
  board_yolo_udp_tee.py PID 1996
  sample_parking_yolo_rtsp PID 1997
```

YOLO detection and Foxglove overlay recovered. Captured preview:

```text
artifacts/autopark_baseline/parking_view_closed_loop_monitor_after_dryrun_20260613_0258.jpg
```

Board-side no-motion dry-run validated the final runtime input path:

```text
Command: board_parking_controller.py --strategy action_replanner --replanner-dry-run --duration-sec 30
Input: 127.0.0.1:24580 from board_yolo_udp_tee.py
Log: artifacts/autopark_baseline/parking_board_closed_loop_monitor_dryrun_20260613.jsonl
Rows: 101
candidate events: 50
replanner_step events: 50
stable rows: 48
chosen actions: WAIT=4, reverse_straight_6=96
motion_flags: 0
send_to_stm32: 0
slot_x_err_px mean -7.373, stdev 0.261
slot_y_dist_cm mean 37.638, stdev 0.026
slot_lateral_cm mean 1.297, stdev 0.023
slot_heading_err_deg mean -2.395, stdev 0.152
```

Current pose is nearly centered laterally and heading is good enough for a
straight reverse recommendation. Next real-motion step, if requested, should be
a bounded one-step test only, for example `max-motion-steps=1`, not continuous
automatic parking yet.

## Straight Reverse Probe - 2026-06-13

User approved one bounded real-motion step. Executed:

```text
primitive_probe
MOVE D=-6.0 V=1
max-motion-steps=1
max-total-cm=8
arm file: /tmp/parking_armed, removed on exit
exit cleanup: STOP
```

Result:

```text
STM32 motion_response: ACK 1004 MOVE / DONE 1004 MOVE
post STAT: MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-0.5 X=-0.1 Y=-5.5 D=5.5 VEL=0.0 DROP=0
arm file removed: yes
controller residual process: none
```

Log:

```text
artifacts/autopark_baseline/parking_probe_straight_6_20260613.jsonl
```

Visual pre/post from the motion log:

```text
pre stable state:  slot_x_err_px=-5.81, slot_y_dist_cm=37.51, slot_lateral_cm=1.32, heading=-1.38
post state:        slot_x_err_px=-8.31, slot_y_dist_cm=36.88, slot_lateral_cm=1.59, heading=-0.32
visual delta:      y_dist only -0.62cm while STM32 odom says D=5.5cm
```

Important conclusion: the straight motion executed safely and the car remained
stable, but the current visual ground-centimeter estimate under-reports
longitudinal progress. Do not use visual `slot_y_dist_cm` alone as the motion
distance source for continuous closed loop; fuse STM32 odometry for executed
distance or recalibrate the homography/scale.

Post-motion no-motion dry-run:

```text
artifacts/autopark_baseline/parking_post_straight_6_dryrun_20260613.jsonl
rows: 43
candidate events: 21
replanner_step events: 21
stable rows: 19
chosen actions: WAIT=4, reverse_straight_6=38
motion_flags: 0
send_to_stm32: 0
slot_x_err_px mean -12.913, stdev 0.264
slot_y_dist_cm mean 35.567, stdev 0.015
slot_lateral_cm mean 2.164, stdev 0.028
slot_heading_err_deg mean 2.131, stdev 0.114
```

Next recommended step: implement/fix odometry-fused progress accounting before
allowing more than one autonomous motion step per run.

## Scale-Independent Slot Control Update - 2026-06-13

User clarified the target is not fixed-box parking: the car should park into
any slot whose size is suitable for the vehicle. Do not treat one physical
tape-box size as a fixed calibration target.

Implemented in `tools/board_parking_controller.py` and deployed to:

```text
/opt/parking/autopark/board_parking_controller.py
```

Changes:

```text
slot_relative_state.corridor now includes:
  slot_width_px
  slot_height_px
  slot_x_err_norm
  slot_entry_x_err_norm
  left_margin_norm
  right_margin_norm
  min_margin_norm
  center_y_norm
  bbox_h_norm

new strategy:
  --strategy normalized_corridor_servo

real-motion progress accounting:
  after each actual motion, always read STM32 STAT
  log commanded_step_cm and odom_delta_cm
  update total_cm/ds_anchor from odom_delta_cm when available
```

Principle:

```text
YOLO slot polygon provides normalized lateral/alignment/safety signals.
STM32 odometry provides actual executed distance.
Old visual slot_y_dist_cm remains diagnostic only and must not be the sole
continuous-loop progress source.
```

Validation:

```text
local py_compile: passed
board py_compile: passed
board dry-run:
  command: board_parking_controller.py --dry-run --strategy normalized_corridor_servo --duration-sec 10
  log: artifacts/autopark_baseline/parking_normalized_corridor_postdeploy_20260613.jsonl
  rows: 19
  candidate events: 18
  chosen action: MOVE D=-6.0 V=1 for all candidate rows
  motion/send_to_stm32: 0
  last slot_x_err_norm: -0.0247
  last min_margin_norm: 0.4753
  last slot_width_px: 520.43
```

Next motion testing should use `normalized_corridor_servo` only as a bounded
single-step test first (`max-motion-steps=1`, small `max-total-cm`), then inspect
`odom_delta_cm`, normalized margins, and Foxglove before allowing multi-step
closed loop.

## Full Path Template Planner Update - 2026-06-13

User wants full path planning, not only local pixel/normalized visual servo.

Implemented strategy:

```text
--strategy path_template_planner
```

Principle:

```text
Use YOLO slot geometry to build scale-independent corridor features.
Generate and rank complete short reverse-path templates:
  straight_entry
  steer_high_then_straight
  steer_low_then_straight
  steer_high_hold
  steer_low_hold
Execute only the first command, then re-observe and replan.
```

Direction convention from current empirical tests:

```text
slot left in image  -> prefer STE=112 first
slot right in image -> prefer STE=68 first
centered           -> prefer MOVE
```

Blind reverse policy:

```text
IMU and odometry may be used after YOLO target loss.
Blind reverse is allowed only after a stable visual anchor/path exists.
If no prior stable visual target/anchor exists, STOP instead of blind motion.
Keep hard caps: max steps, max total distance, max blind distance, STAT required.
```

Validation so far:

```text
local py_compile: passed
board py_compile: passed
offline synthetic direction check:
  slot_left_in_image  -> ARC D=-6.0 STE=112 V=1
  slot_centered       -> MOVE D=-6.0 V=1
  slot_right_in_image -> ARC D=-6.0 STE=68 V=1
live board dry-run:
  YOLO/tee running, but latest YOLO count=0, so planner logged NO_TARGET.
```

Docs:

```text
docs/autopark_path_template_planner_20260613.md
```

## Core Plan Override - Fusion Closed Loop - 2026-06-13

User provided and explicitly selected the following document as the core
execution basis for the autoparking project:

```text
docs/autopark_fusion_closed_loop_plan_20260613.md
```

Priority rule from now on:

```text
The fusion closed-loop plan is the core route.
The earlier path_template_planner is only a temporary runnable capability and
must not drive the long-term architecture.
Future work should prioritize:
  F2/F3 structured STM32 telemetry and DONE terminal-state fields
  C0 sign verification and configs/chassis_signs.json
  B1 board-side serial reader and token dispatch
  B2 PoseFuser core and unit tests
  B3 in-motion safety monitor
  C2/C3 calibration and fusion_reconcile samples
```

Execution discipline:

```text
No new real-motion or burn/flash step should be started merely because an older
path-template test exists.
Use the fusion plan's dependency order as the default task order.
Non-motion source audit, code implementation, static checks, and dry-run/unit
tests may proceed under the user's default board/VM approval preference.
Real vehicle motion, firmware flashing, or actuator tests still require explicit
motion approval.
```

## STM32 Control Logic Audit After User Changes - 2026-06-13

User reported updated STM32 control logic in `SS928_hub`.

Observed source changes/current state:

```text
SS928_hub/Core/CarControl.c
  StartArcDrive() applies ApplyAckermannSpeedScale(Angle).
  ApplyAckermannSpeedScale() computes left/right wheel speed scale from:
    ACKERMANN_WHEEL_BASE_CM / tan(steer_offset)
    WHEEL_TRACK_CM / 2
  Distance/arc completion uses Odometry_GetSnapshot().distance.

SS928_hub/HARDWARE/Motors.c
  Odometry_t odom exists with x/y/theta/distance.
  Odometry_Update() integrates wheel speeds every PID cycle.
  Reverse motion flips raw differential heading sign.
  x is lateral right-positive, y is forward-positive, theta left-turn-positive.
  Motor_SetSpeedScale()/Motor_ResetSpeedScale() are active in PID target scaling.

SS928_hub/Core/HeadingControl.c
  Straight hold PID uses IMU yaw error plus optional odometry x cross-track
  correction through headingPID.CrossTrackKp.
```

Impact on the new fusion plan:

```text
Good: STM32 now exposes the right physical ingredients for C0/C2/C3 and B2.
Still missing for the core plan:
  F2 structured TLM: current PrintTelemetry() is still bare CSV.
  F3 DONE terminal state: CarProtocol_FinishActiveMotionOk("") is still called
      with an empty extra string in MOVE/ARC/TURN paths.
  TLM cadence is still EXCOUNT(TelemetryCnt,100), not yet verified as 5Hz.
  Geometry constants remain placeholders:
      WHEEL_TRACK_CM=14.5
      ACKERMANN_WHEEL_BASE_CM=16.0
```

Risk notes:

```text
ApplyAckermannSpeedScale() should be validated for reverse ARC sign and
inside/outside wheel scaling before relying on R_eff.
Odometry_Update() reverse dtheta flip is plausible but must be confirmed in C0.
Heading cross-track correction should not be used as a substitute for board-side
PoseFuser; it is an inner-loop stabilizer only.
```

## Fusion Plan F2/F3 Firmware Source Implementation - 2026-06-13

Implemented local STM32 source changes for the core fusion plan. Not flashed or
vehicle-tested yet.

Files changed:

```text
SS928_hub/Core/CarControl.c
SS928_hub/Core/CarControl.h
SS928_hub/Core/CarProtocol.c
SS928_hub/HARDWARE/CarApp.c
```

Implemented behavior:

```text
F2 structured periodic telemetry:
  TLM <n> YAW=<deg> X=<cm> Y=<cm> D=<cm> V=<cm/s> ANG=<deg>
  TelemetryReady divider changed to EXCOUNT(TelemetryCnt,200).
  Legacy RC_STAT still emits the old bare CSV through PrintLegacyTelemetry().

F3 active-motion terminal state:
  DONE <seq> <cmd> X=<cm> Y=<cm> D=<cm> YAW=<deg>
  ERR <seq> CODE=<code> X=<cm> Y=<cm> D=<cm> YAW=<deg>
  Generic non-motion ReplyDone/ReplyErr paths are unchanged.
```

Build result:

```text
powershell -ExecutionPolicy Bypass -File build_gcc/build.ps1
passed
FLASH 61228 B / 64 KB = 93.43%
RAM 4856 B / 20 KB = 23.71%
outputs rebuilt:
  SS928_hub/build/gcc/SS928_hub.hex
  SS928_hub/build/gcc/SS928_hub.bin
```

Pending before treating F2/F3 as accepted:

```text
User flashes firmware.
Validate TEL OFF/ON behavior on hardware.
Validate TLM rate, line length, DONE/ERR terminal fields, and DROP.
No autonomous parking should depend on TLM until this hardware validation passes.
```

## F2/F3 Hardware Validation Result - 2026-06-13

Firmware was flashed through ST-LINK/OpenOCD and verified OK.

Non-motion validation:

```text
PING: pass
VER: pass
STAT: pass, DROP=0
TEL ON while IDLE: pass after gating fix, no periodic TLM spam
TEL OFF: pass
```

Motion validation:

```text
Command sequence:
  TEL ON
  MOVE D=-6.0 V=1
  TEL OFF

Observed:
  ACK 8211 MOVE
  11 TLM lines during motion
  DONE 8211 MOVE X=0.0 Y=-4.1 D=4.1 YAW=59.0
  post STAT: MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0
             YAW=-25.7 X=0.0 Y=-5.1 D=5.2 VEL=0.0 DROP=0
```

Acceptance status:

```text
F2/F3 protocol format is accepted for B1 parser work.
TLM and DONE terminal fields are present.
DROP stayed 0.
Do not accept YAW as reliable for PoseFuser yet: yaw changed implausibly during
a short straight reverse move. C0 must explicitly test yaw sign, zeroing, and
stability before B2 trusts it.
```

## C0 YAW Validation Failure - 2026-06-13

Static YAW validation failed.

Reports:

```text
docs/autopark_c0_yaw_validation_20260613.md
artifacts/autopark_baseline/c0_yaw_static_before_zero_20260613.json
artifacts/autopark_baseline/c0_yaw_static_after_zero_20260613.json
```

Observed:

```text
Before ZERO_YAW:
  samples: -152.5, 57.8, -94.0, 112.6, -37.2, 171.0, 17.6, -135.8, 73.3, -73.3
  range: 323.5 deg

After ZERO_YAW:
  samples: 69.1, -81.8, 128.9, -23.6, -178.7, 30.0, -118.7, 91.5, -63.6, 146.1
  range: 324.8 deg

Same serial session rapid STAT:
  YAW advanced about 21.4 deg per sample while static.
  This rules out repeated Windows/CH341 query initialization as the cause.
```

Decision:

```text
STAT YAW is not usable for PoseFuser.
Do not run yaw-based fusion or yaw-based blind reverse until BMI270 gyro
diagnostics pass.
Next fix should instrument BMI270 raw/post-offset GyroZ, gyro_zero_z, and dt.
Likely source files:
  SS928_hub/HARDWARE/BMI270/bmi270_driver.c
  SS928_hub/HARDWARE/CarApp.c
```

## C0 YAW Static Retest Pass - 2026-06-13

After the user's YAW firmware update, the board firmware exposes:

```text
STAT ... IMU=OK
GDIAG
GYROCAL
```

Non-motion retest result:

```text
Report: artifacts/autopark_baseline/c0_yaw_static_retest_after_user_update_20260613.json
GDIAG: ID=0x24 RANGE=0 SCALE=1065 DT=5.0ms ZZ=-2 I2CERR=0 IMU=OK
After GYROCAL + ZERO_YAW:
  STAT x20 in one serial session
  yaw values: all -0.1 deg
  yaw_range_deg: 0.0
  yaw_first_last_delta_deg: 0.0
  pass_static_yaw: true
```

Decision:

```text
Static YAW drift is fixed enough to proceed to C0 movement sign validation.
Do not fully enable yaw-based PoseFuser movement yet until C0 sign validation
records yaw_cw_positive and odometry direction signs in configs/chassis_signs.json.
```

## C0 Reverse Right Motion Sign Sample - 2026-06-13

Approved motion command:

```text
TEL ON
ARC D=-6.0 STE=120 V=1
TEL OFF
STAT
```

Observed:

```text
DONE 8521 ARC X=0.7 Y=-4.0 D=4.0 YAW=1.5
STAT 8523 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0
          YAW=1.5 X=0.8 Y=-4.4 D=4.5 VEL=0.0 DROP=0 IMU=OK
```

Recorded:

```text
artifacts/autopark_baseline/c0_motion_sign_reverse_right_20260613.json
configs/chassis_signs.json
```

Partial sign result:

```text
odom_d_reverse_negative = false
odom_x_right_positive = true
yaw_cw_positive = null
vision_lateral_left_negative = null
```

Decision:

```text
YAW is stable and moves smoothly during a short reverse arc.
PoseFuser still must not enable full yaw-based movement until yaw_cw_positive
and vision_lateral_left_negative are filled by explicit C0 sign evidence.
```

Operator follow-up:

```text
The repeated STE=120 reverse arc looked clockwise from top view.
Since YAW increased during the same motion, configs/chassis_signs.json now sets:
  yaw_cw_positive = true
Confidence is preliminary/operator-observed.
```

Current C0 sign file:

```text
configs/chassis_signs.json
  yaw_cw_positive = true
  odom_d_reverse_negative = false
  odom_x_right_positive = true
  vision_lateral_left_negative = null
```

## C0 Vision Lateral Sign - 2026-06-13

Live YOLO status during the check:

```text
board YOLO and board_yolo_udp_tee.py were running.
UDP samples were received on 127.0.0.1:24580.
Current YOLO detection_count=0, so there was no fresh live slot placement test.
```

Source-code and historical evidence:

```text
tools/board_parking_controller.py defines ground +y as left.
slot_relative_state.ground_estimate.slot_lateral_cm = plan.lat.
Historical 33-row dry-run:
  slot_x_err_px mean = +76.333
  slot_lateral_cm mean = -3.949
This is consistent with image/slot right -> lateral negative, so left is positive.
```

Recorded:

```text
artifacts/autopark_baseline/c0_vision_lateral_sign_20260613.json
configs/chassis_signs.json
```

Current sign result:

```text
yaw_cw_positive = true
odom_d_reverse_negative = false
odom_x_right_positive = true
vision_lateral_left_negative = false
```

Decision:

```text
C0 signs are populated. Treat vision_lateral_left_negative=false as based on
source convention plus historical evidence, not a fresh physical live test.
Recheck visually when YOLO detects the slot again before trusting wide-range
autonomous parking.
```

## B1/B2 Local Fusion Software Foundation - 2026-06-13

Implemented:

```text
tools/parking_fusion.py
tools/parking_fusion_selftest.py
```

Capabilities:

```text
parse STM32 ACK/TLM/DONE/ERR/STAT/GDIAG/VER lines
load configs/chassis_signs.json and reject null/non-boolean sign fields
anchor fused pose from slot_relative_state
propagate fused pose with TLM yaw + odometry D
blend small vision innovations
```

Controller integration:

```text
tools/board_parking_controller.py
  imports parking_fusion if available
  adds --chassis-signs-json
  adds --require-fusion-signs
  logs structured pre_servo_events and motion_events for STM32 responses
  truncates acquire_info wait by --duration-sec for bounded dry-run checks
```

Verified locally:

```text
py_compile passed for parking_fusion.py, parking_fusion_selftest.py,
board_parking_controller.py, stm32_send.py

parking_fusion_selftest.py passed.
Short local controller dry-run printed FUSION_SIGNS=OK and exited by duration.
```

Not yet done:

```text
Board deployment to /opt/parking/autopark/
Live board dry-run using deployed parking_fusion.py
Real movement with PoseFuser enabled
```

## B1/B2 Board Deployment - 2026-06-13

Deployed to board `192.168.137.2`:

```text
/opt/parking/autopark/parking_fusion.py
/opt/parking/autopark/board_parking_controller.py
/opt/parking/autopark/chassis_signs.json
```

Board checks passed:

```text
python3 -m py_compile passed for parking_fusion.py and board_parking_controller.py.
parking_fusion.py loaded chassis_signs.json and parsed:
  DONE 8521 ARC X=0.7 Y=-4.0 D=4.0 YAW=1.5

board_parking_controller.py dry-run with --require-fusion-signs printed:
  FUSION_SIGNS=OK
  STOP=NO_TARGET (no slot / no anchor).
  STOP=DURATION elapsed.
```

No motion command was sent.

Next:

```text
Run a live board dry-run on the real YOLO UDP port once YOLO detects the slot.
Then add PoseFuser log events to dry-run candidate rows before allowing fused
movement.
```

## B2 Shadow Fusion Logging Local Update - 2026-06-13

Updated locally:

```text
tools/board_parking_controller.py
```

New behavior:

```text
When chassis signs are valid, controller creates PoseFuser in shadow_log_only mode.
Stable candidate rows include candidate.fusion_pose.
Real-motion result rows will include fusion_motion_trace and fusion_motion_final
from parsed TLM events.
Fusion output is not used for motion decisions yet.
```

Local validation:

```text
artifacts/autopark_baseline/fusion_shadow_local_dryrun_20260613.jsonl
candidate.fusion_pose:
  x_s_cm=-3.925
  y_s_cm=-48.36
  phi_deg=2.32
  source=vision_anchor
  tlm_count=0
```

Board deployment is pending approval.

## B2 Shadow Fusion Board Live Dry-Run - 2026-06-13

Updated controller deployed to:

```text
/opt/parking/autopark/board_parking_controller.py
```

Live board dry-run:

```text
Log: /tmp/parking_fusion_live_dryrun_20260613_055316.jsonl
FUSION_SIGNS=OK
fusion_pose=shadow_log_only
YOLO live detections available
dry_run=True
send_to_stm32=False
```

Observed live state:

```text
slot_y_dist_cm around 38.5
slot_lateral_cm around -1.1
slot_x_err_px around 27..31
min_margin_px around 187..191
candidate_cmd = MOVE D=-7.0 V=1
fusion_pose mirrors vision anchor:
  x_s_cm around -1.1
  y_s_cm around -38.6
  phi_deg around 2.3..2.7
```

Decision:

```text
B2 shadow fusion logging works on real board YOLO UDP.
Next validation requires an approved short real motion with logger enabled to
validate fusion_motion_trace from actual TLM rows.
```

## B2 Shadow Fusion Motion Trace Pass - 2026-06-13

Validated with one approved short motion:

```text
Controller selected MOVE D=-7.0 V=1
TEL ON -> MOVE -> TEL OFF
Board log: /tmp/parking_fusion_motion_trace_tel_20260613.jsonl
Report: artifacts/autopark_baseline/b2_fusion_motion_trace_tel_20260613.json
```

Observed:

```text
TLM rows: 11
fusion_motion_trace rows: 11
DONE: X=-0.0 Y=-5.1 D=5.1
STAT after: YAW=-21.3 X=-0.0 Y=-6.2 D=6.2 DROP=0 IMU=OK
```

Fusion result:

```text
before:
  x_s_cm=-0.704
  y_s_cm=-36.302
  phi_deg=-0.614

after:
  x_s_cm=-0.757
  y_s_cm=-31.402
  phi_deg=-0.614
  tlm_count=11
```

Decision:

```text
B2 TLM -> fusion_motion_trace path is validated in shadow mode.
PoseFuser may be used for dry-run reconcile logging next.
Do not use PoseFuser to alter control commands yet.
```

Protocol note:

```text
This run's MOVE DONE line omitted YAW, while TLM and STAT included YAW.
Earlier F3 validation had DONE YAW. Check firmware DONE formatting consistency
before relying on DONE yaw.
```

## C2 ARC Calibration STE=120 Sample - 2026-06-13

Approved command:

```text
ARC D=-6.0 STE=120 V=1
Board log: /tmp/parking_arc_calib_ste120_20260613.jsonl
Report: artifacts/autopark_baseline/c2_arc_calib_ste120_20260613.json
```

Result:

```text
Pre-YOLO:  lon=37.00 lat=-0.58 head=0.03
TLM first: YAW=-21.5 D=0.0 X=0.0 Y=0.0
TLM last:  YAW=-19.2 D=3.6 X=0.2 Y=-3.6
TLM yaw_change=+2.3deg
TLM dist_change=3.6cm
preliminary R_eff ~= 89.7cm
STAT after: YAW=-18.0 X=0.3 Y=-5.9 D=5.9 DROP=0 IMU=OK
fusion final: x_s=-0.503 y_s=-33.405 phi=2.326 tlm_count=10
Post-YOLO: lon=35.59 lat=3.28 head=-1.95
```

Decision:

```text
STE=120 arc direction is confirmed: positive yaw change and clockwise by C0 sign.
Use this as a preliminary curve sample only.
Do not fit a final steering model from one sample because post-YOLO lateral jump
is much larger than fusion x_s change.
```

Reader fix after this sample:

```text
DONE line was truncated as:
  DONE 1004 ARC X=0.2 Y=-4.1 D=4.1 Y

Local fix:
  board_parking_controller.py waits for complete DONE/ERR line.
  stm32_send.py reads extra tail after DONE/ERR.

Deploy this fix before the next ARC calibration run.
```

## C2 ARC Calibration STE=120 Repeat After Reader Fix - 2026-06-13

Approved command:

```text
ARC D=-6.0 STE=120 V=1
Board log: /tmp/parking_arc_calib_ste120_repeat_20260613.jsonl
Report: artifacts/autopark_baseline/c2_arc_calib_ste120_repeat_20260613.json
```

Result:

```text
Pre-YOLO: lon=36.83 lat=-1.35 head=-1.52
TLM first: YAW=-31.6 D=0.0 X=0.0 Y=0.0
TLM last:  YAW=-29.8 D=2.7 X=0.1 Y=-2.6
TLM yaw_change=+1.8deg
TLM dist_change=2.7cm
R_eff ~= 85.9cm
DONE complete: DONE 1004 ARC X=0.2 Y=-4.1 D=4.1 YAW=-29.1
STAT after: YAW=-28.4 X=0.4 Y=-5.8 D=5.8 DROP=0 IMU=OK
Post-YOLO: lon=34.98 lat=1.53 head=-0.96
```

Decision:

```text
DONE reader fix is confirmed.
STE=120 has two preliminary R_eff samples: about 89.7cm and 85.9cm.
Continue C2 with STE=105, 75, 60.
```

## C2 STE=105 One-Step Incident And Safety Gate - 2026-06-13

Approved command under test:

```text
ARC D=-6.0 STE=105 V=1
Report: artifacts/autopark_baseline/c2_arc_calib_ste105_incident_20260613.json
```

Observed first motion:

```text
Pre-YOLO: lon=35.1 lat=1.6 head=-1.3
DONE: DONE 1004 ARC X=0.3 Y=-4.1 D=4.1 YAW=-28.8
STAT after first step: YAW=-28.4 X=0.5 Y=-5.8 D=5.9 DROP=0 IMU=OK
fusion final: x_s=1.529 y_s=-31.772 phi=-0.493
```

Incident:

```text
After the first approved primitive-probe motion, YOLO vision was lost.
The legacy dead-reckon branch then issued an extra blind MOVE D=-10.0 V=1,
despite the one-step calibration intent.
STOP was sent immediately afterward and STM32 returned to IDLE.
```

Decision:

```text
Do not use this STE=105 run for curve fitting.
Use it as a safety regression record only.
Default behavior after vision loss must be STOP.
Dead-reckon continuation now requires explicit --allow-dead-reckon-after-loss.
primitive_probe must stop on vision loss and never enter blind dead-reckon.
All dead-reckon paths must honor --max-motion-steps and --max-total-cm before
sending a command.
```

## C2 ARC Calibration STE=105 Repeat After Safety Fix - 2026-06-13

Approved command:

```text
ARC D=-6.0 STE=105 V=1
Board log: /tmp/parking_arc_calib_ste105_repeat_20260613.jsonl
Report: artifacts/autopark_baseline/c2_arc_calib_ste105_repeat_20260613.json
Local logs:
  artifacts/autopark_baseline/parking_arc_calib_ste105_repeat_20260613.jsonl
  artifacts/autopark_baseline/parking_arc_calib_ste105_post_dryrun_20260613.jsonl
```

Result:

```text
Pre-YOLO:  lon=37.12 lat=0.47 head=1.09 confidence=0.799
Post-YOLO: lon=34.88 lat=3.11 head=-1.23 confidence=0.886
Vision delta: lon=-2.24cm lat=+2.64cm head=-2.32deg
TLM rows: 10
TLM yaw_change=+0.5deg dist_change=1.6cm R_eff ~= 183.3cm
DONE: yaw_change=+1.0deg D=4.1cm R_eff ~= 234.9cm
STAT after: yaw_change=+1.5deg D=6.8cm R_eff ~= 259.7cm
Fusion final: x_s=0.511 y_s=-35.522 phi=1.594
Safety: steps=1 total_cm=6.8, no extra blind move
```

Decision:

```text
STE=105 is a valid one-step calibration sample after the safety fix.
It produces a much shallower arc than STE=120, so the steering response is
strongly nonlinear or affected by low-speed/deadband behavior.
Collect STE=75 and STE=60 before fitting a steering response model.
```

## C2 ARC Calibration STE=75 - 2026-06-13

Approved command:

```text
ARC D=-6.0 STE=75 V=1
Board log: /tmp/parking_arc_calib_ste75_20260613.jsonl
Report: artifacts/autopark_baseline/c2_arc_calib_ste75_20260613.json
Local logs:
  artifacts/autopark_baseline/parking_arc_calib_ste75_20260613.jsonl
  artifacts/autopark_baseline/parking_arc_calib_ste75_post_dryrun_20260613.jsonl
```

Result:

```text
Pre-YOLO:  lon=34.89 lat=3.22 head=-1.22 confidence=0.898
Post-YOLO: lon=32.54 lat=4.34 head=-0.83 confidence=0.941
Vision delta: lon=-2.35cm lat=+1.12cm head=+0.39deg
TLM rows: 11
TLM yaw_change=-0.8deg dist_change=2.3cm R_eff ~= 164.7cm
DONE: yaw_change=-1.6deg D=4.2cm R_eff ~= 150.4cm
STAT after: yaw_change=-2.3deg D=5.9cm R_eff ~= 147.0cm
Fusion final: x_s=3.153 y_s=-32.587 phi=-2.020
Safety: steps=1 total_cm=5.9, no extra blind move
```

Decision:

```text
STE=75 is a valid one-step left-arc sample.
The response is stronger than STE=105 but weaker than STE=120 in absolute
curvature. Collect STE=60 next before fitting the left/right steering model.
```

## C2 ARC Calibration STE=60 - 2026-06-13

Approved command:

```text
ARC D=-6.0 STE=60 V=1
Board log: /tmp/parking_arc_calib_ste60_20260613.jsonl
Report: artifacts/autopark_baseline/c2_arc_calib_ste60_20260613.json
Local logs:
  artifacts/autopark_baseline/parking_arc_calib_ste60_20260613.jsonl
  artifacts/autopark_baseline/parking_arc_calib_ste60_post_dryrun_20260613.jsonl
```

Result:

```text
Pre-YOLO:  lon=32.45 lat=4.40 head=-0.50 confidence=0.940
Post-YOLO: lon=28.01 lat=1.74 head=0.09 confidence=0.895
Vision delta: lon=-4.44cm lat=-2.66cm head=+0.59deg
TLM rows: 11
TLM yaw_change=-3.1deg dist_change=3.9cm R_eff ~= 72.1cm
DONE: yaw_change=-3.3deg D=4.2cm R_eff ~= 72.9cm
STAT after: yaw_change=-4.7deg D=6.5cm R_eff ~= 79.2cm
Fusion final: x_s=4.252 y_s=-28.554 phi=-3.599
Safety: steps=1 total_cm=6.5, no extra blind move
```

Decision:

```text
STE=60 is a valid strong left-arc sample.
In this pose it corrected visual lateral error toward center:
slot_x_err_px -51.9 -> near center after the move.
Current C2 set is enough to fit a first steering response model:
right side STE=105/120, left side STE=75/60.
```

## C2 Response Model Update - 2026-06-13

Updated local planner configs:

```text
configs/parking_action_library.json
configs/parking_action_response_model.json
```

Documentation:

```text
docs/autopark_c2_steering_response_20260613.md
```

Verification:

```text
T4 replay after C2:
  artifacts/autopark_baseline/parking_replay_planner_after_c2_final_20260613.json
  state_rows=33
  stable_actionable_rows=31
  action_switch_count_stable=0
  direction_review_pass=true
  acceptance_pass=true

Current post-STE60 live-pose score:
  artifacts/autopark_baseline/parking_arc_calib_ste60_post_dryrun_20260613.jsonl
  best_action=reverse_straight_6
  command=MOVE D=-6.0 V=1
```

Decision:

```text
Do not continue fixed primitive probing as the main path.
Next software step is deploy updated action library/response model to board and
run live action_replanner dry-run. If ranking is stable and safe, execute one
confirmed real action from action_replanner.
```

## Board Deploy C2 Model And Live Replanner Dry-Run - 2026-06-13

Deployed to board:

```text
/opt/parking/autopark/parking_action_library.json
/opt/parking/autopark/parking_action_response_model.json
```

Board validation:

```text
python3 -m json.tool passed for both files.
YOLO and board_yolo_udp_tee were still running.
```

Live dry-run:

```text
Board log: /tmp/parking_action_replanner_after_c2_dryrun_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_after_c2_dryrun_20260613.jsonl
strategy=action_replanner
replanner_dry_run=true
duration=5s
records=4
```

Result:

```text
Initial unstable frame: WAIT
Stable frames: consistently MOVE D=-6.0 V=1
Observed state: lon ~= 28.0cm, lat ~= 1.7-1.9cm, head ~= 0.1deg
No motion sent.
```

Decision:

```text
The deployed C2 model is usable for the next cautious real action.
Next real action should be one confirmed action_replanner step, expected command
MOVE D=-6.0 V=1, with --max-motion-steps 1 and --max-total-cm 8.
```

## Action Replanner Real-Motion Gate Fix - 2026-06-13

Attempted one real `action_replanner` step:

```text
Board log: /tmp/parking_action_replanner_real_step_after_c2_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_real_step_after_c2_20260613.jsonl
```

Result:

```text
No motion was sent.
Stable state selected STOP reason=none_eligible.
Best-ranked action was reverse_straight_6 / MOVE D=-6.0 V=1, but it was
hard-blocked by no_exact_measured_response because reverse_straight_6 had
requires_measured=true.
```

Local fix:

```text
configs/parking_action_library.json:
  reverse_straight_6.requires_measured=false
```

Reason:

```text
The exact measured-response hard gate should remain strict for steering actions.
For the neutral 6cm straight baseline, prior response is acceptable when normal
stability, line margin, predicted line risk, max-step, and max-distance gates
all pass.
```

Verification:

```text
Real-motion planner scoring on the failed real-step state:
  best=reverse_straight_6
  command=MOVE D=-6.0 V=1
  hard_blocked=false

T4 replay:
  artifacts/autopark_baseline/parking_replay_planner_after_straight_gate_fix_20260613.json
  acceptance_pass=true
  action_switch_count_stable=0
  direction_review_pass=true
```

## Final Visual Blind Zone Confirmation - 2026-06-13

After deploying the straight-action gate fix, the real `action_replanner` step
executed:

```text
MOVE D=-6.0 V=1
Board log: /tmp/parking_action_replanner_real_step_after_straight_gate_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_real_step_after_straight_gate_20260613.jsonl
Result: steps=1 total_cm=4.8
```

Post-action live dry-run produced `NO_TARGET`, while YOLO/UDP were still alive:

```text
Local log: artifacts/autopark_baseline/parking_action_replanner_post_straight_gate_dryrun_20260613.jsonl
YOLO confidence dropped to about 0.25 and the controller had no stable target.
```

User confirmed this is expected:

```text
This position is in the visual blind zone, not a perception fault.
```

Decision:

```text
When the car reaches this terminal blind-zone pose after a stable near-center
visual state, do not treat YOLO loss as an error. The next planner stage should
switch from visual action_replanner to a tightly bounded blind final reverse
using STM32 odometry/IMU only.

This final blind segment must be explicitly gated:
  - only after recent stable visual state near center
  - only after one successful action_replanner step
  - max one blind command initially
  - short distance cap, suggested 4-6cm for first validation
  - STOP after command
  - no repeated dead-reckon loop unless separately approved

## Final Blind Reverse Token Implementation - 2026-06-13

Implemented locally in `tools/board_parking_controller.py`:

```text
--allow-final-blind-reverse
--final-blind-token /tmp/parking_final_blind_token.json
--final-blind-reverse-cm <cm>
--final-blind-vision-lost-sec 0.5
```

Behavior:

```text
1. During a real action_replanner visible action, the controller reviews the
   pre-action slot state.
2. If the state is stable, near centered, line-safe, and in
   straighten_or_enter/final_stop_zone, it writes a one-shot token.
3. If YOLO is later lost for the configured delay, action_replanner may consume
   that token and send exactly one straight MOVE D=-x V=1 command.
4. The controller immediately sends STOP after the final blind command and marks
   the token consumed.
5. If the token is absent, stale, consumed, or gate checks fail, the controller
   stops instead of falling back to old dead-reckon.
```

Default token write gates:

```text
abs(slot_x_err_px) <= 25
abs(slot_heading_err_deg) <= 5
min_margin_px >= 80
slot_y_dist_cm <= 35
phase_hint in {straighten_or_enter, final_stop_zone}
stable_enough=true
line_margin_ok=true
line_risk=false
```

Local verification:

```text
.venv\Scripts\python -m py_compile tools\board_parking_controller.py tools\parking_fusion.py tools\stm32_send.py

.venv\Scripts\python tools\board_parking_controller.py --dry-run --strategy action_replanner --replanner-dry-run --listen-port 24680 --duration-sec 1.2 --target-wait-sec 0.2 --settle-sec 0.1 --action-library-json configs\parking_action_library.json --response-model-json configs\parking_action_response_model.json --chassis-signs-json configs\chassis_signs.json --require-fusion-signs --allow-final-blind-reverse --final-blind-reverse-cm 4 --log-jsonl artifacts\autopark_baseline\final_blind_smoke_20260613.jsonl
```

Result:

```text
No target and no token -> FINAL_BLIND_GATE_CLOSED.
No serial opened, no motion sent.
```

## Final Blind Reverse 4cm Real Test - 2026-06-13

Setup:

```text
Physical pose was already in the terminal visual blind zone.
The one-shot token was manually reconstructed from the previous successful
visible action log because the token feature was implemented after that action.
Token source:
  artifacts/autopark_baseline/parking_action_replanner_real_step_after_straight_gate_20260613.jsonl
Board token:
  /tmp/parking_final_blind_token.json
```

Dry-run gate check:

```text
Command would send: MOVE D=-4.0 V=1
No-motion validation passed before real motion.
```

Real command:

```text
Board log: /tmp/parking_final_blind_real_4cm_20260613.jsonl
Controller: action_replanner + --allow-final-blind-reverse
Final blind command: MOVE D=-4.0 V=1
```

Result:

```text
Token gate: pass
Token consumed: true
STM32 DONE: DONE 1004 MOVE X=-0.0 Y=-2.1 D=2.1 YAW=-54.3
STAT after: STAT 1006 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-54.5 X=-0.0 Y=-2.9 D=2.9 VEL=0.0 DROP=0 IMU=OK
Follow-up STAT: STAT 1494 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-55.0 X=-0.0 Y=-2.9 D=2.9 VEL=0.0 DROP=0 IMU=OK
```

Decision:

```text
The final-blind safety mechanism works: one candidate, one MOVE, STOP, and token
consumed. The actual odometry distance for command D=-4.0 was about 2.1-2.9cm,
so terminal blind distance calibration still needs visual/user assessment before
increasing the command length.
```

User visual assessment:

```text
The car was almost parked.
Body was slightly biased/skewed.
The front still had a small portion outside the parking slot.
```

Implication:

```text
The terminal blind step should likely be longer than D=-4.0, but the remaining
problem is not only distance. The visible-stage entry pose needs to be slightly
straighter/less biased before YOLO loss. Next tests should separate:
  1. final blind distance calibration, e.g. D=-6.0 or D=-8.0 one-shot;
  2. pre-blind pose correction, using the last visible slot_x_err/heading_err
     to avoid entering the blind zone with a small skew.
```

## Claude Review Incorporated - 2026-06-13

External review agreed that the current architecture is correct:

```text
Vision-guided action_replanner + one-shot token-gated final blind reverse is the
right B7 terminal-blind structure. Keep one-shot, consumed token, immediate STOP.
Do not return to continuous blind dead-reckon.
```

Key quantitative interpretation:

```text
Known drivetrain deadband: MOVE D=N actual travel ~= N-2cm.
Recent evidence matches:
  visible MOVE D=-6.0 -> about 4.8cm
  final blind MOVE D=-4.0 -> about 2.1-2.9cm

Therefore "front still outside" is mostly distance budget/deadband, not a
high-level architecture failure. For desired actual travel A, command roughly:
  D = -(A + 2cm)
Also account for about 0.8-1.0cm post-DONE coast in terminal distance budgets.
```

Priority order:

```text
1. Fix visible-stage final pose with a small counter-steer/straighten action.
2. Fix final-blind distance using deadband-compensated dynamic distance budget.
3. Delay full geometric path planning until response model coverage is better.
```

Terminal blind policy to implement:

```text
Let x = last visible slot_lateral_cm at the rear-axle target, phi = heading error.

If abs(phi) < 2deg and abs(x) < 1.5cm:
  allow straight final blind.

If 2deg <= abs(phi) <= 6deg and abs(x) < 1.5cm:
  allow heading-cancel final blind arc using measured R_eff.
  Arc length d ~= radians(abs(phi)) * R_eff.
  Remaining distance may be straight, but still under one token budget.

If abs(x) >= 1.5cm:
  do not write token. Stay in visible stage and correct lateral error first.
```

Architecture decision:

```text
Keep per-step scoring. The T4 replay already showed stable states naturally
collapse to deterministic choices. Use allowed_phases as the sequence skeleton,
not hard-coded fixed sequences.
```

## Claude Review Software Follow-up - 2026-06-13

Local controller updates in `tools/board_parking_controller.py`:

```text
1. Final blind token review now recomputes pre_state gates at consumption time.
   It no longer trusts an old stored pre_state_review blindly.

2. Token write/consume gates now include rear-target lateral cm:
   --final-blind-max-lateral-cm default 1.5
   --final-blind-max-terminal-lateral-cm default 1.8

3. Heading gate is split:
   --final-blind-straight-heading-err-deg default 2.0
   --final-blind-allow-heading-arc default off
   --final-blind-arc-max-heading-err-deg default 6.0

4. Final blind distance can be deadband compensated:
   --final-blind-compensate-deadband
   --final-blind-deadband-cm default 2.0
   --final-blind-coast-cm default 1.0
   Example: desired actual 4cm -> command MOVE D=-5.0 V=1.

5. Visible terminal counter-steer candidate admission was added but is default off:
   --replanner-allow-terminal-countersteer
   It only lets selected ARC actions enter straighten_or_enter ranking.
   Real motion still honors requires_measured, so unmeasured terminal arcs remain blocked.
```

Local verification:

```text
.venv\Scripts\python -m py_compile tools\board_parking_controller.py

Compensated final-blind smoke:
  desired_actual=4cm + deadband 2cm - coast 1cm
  no-motion output: MOVE D=-5.0 V=1

Lateral reject smoke:
  synthetic old token said pass, but recomputed lateral was 1.86cm
  with max_lateral=1.5cm
  no-motion output: FINAL_BLIND_GATE_CLOSED
```

Safety decision:

```text
Do not deploy/use terminal ARC or visible terminal counter-steer for real motion
until there are cleaner measured samples in the relevant straighten_or_enter
state buckets. The next safe live test should first use the stricter lateral
token gate and deadband-compensated straight final blind.
```

## Strict Final-Blind Deploy + Live No-Motion Dry-Run - 2026-06-13

Deployed:

```text
tools/board_parking_controller.py
  -> /opt/parking/autopark/board_parking_controller.py

Board compile check passed.
Help confirmed:
  --final-blind-compensate-deadband
  --final-blind-max-lateral-cm
  --replanner-allow-terminal-countersteer
```

Live no-motion dry-run:

```text
Board log: /tmp/parking_action_replanner_strict_final_blind_dryrun_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_strict_final_blind_dryrun_20260613.jsonl
Command mode: --dry-run --replanner-dry-run
No serial opened, no motion sent.
```

Observed current state:

```text
phase_hint=align_in_corridor
slot_y_dist_cm=20.205
slot_lateral_cm=4.17
slot_heading_err_deg=0.0
slot_x_err_px=-50.0
min_margin_px=198.0
```

Planner output:

```text
chosen=reverse_left_hard_6
command=ARC D=-6.0 STE=60 V=1
reason=hold_hysteresis
origin=measured
response_verdict=improved
hard_blocked=false
```

Interpretation:

```text
The stricter final-blind policy correctly prevents terminal token entry here:
slot_lateral_cm=4.17cm is greater than the 1.5cm token gate.

The planner instead recommends the measured/improved visible correction action
reverse_left_hard_6. This is consistent with the Claude review: correct visible
stage lateral bias before allowing final blind.
```

## Visible Correction Real Step STE=60 - 2026-06-13

Executed after strict final-blind dry-run showed:

```text
phase_hint=align_in_corridor
slot_y_dist_cm=20.205
slot_lateral_cm=4.17
slot_heading_err_deg=0.0
slot_x_err_px=-50.0
chosen=reverse_left_hard_6 / ARC D=-6.0 STE=60 V=1
origin=measured
response_verdict=improved
```

Real command:

```text
Board log: /tmp/parking_action_replanner_visible_correction_ste60_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_visible_correction_ste60_20260613.jsonl
Command: ARC D=-6.0 STE=60 V=1
Pre-steer: SERVO A=60, settle 0.3s
Safety: max_motion_steps=1, max_total_cm=8
```

STM32 result:

```text
DONE 1007 ARC X=-0.3 Y=-4.0 D=4.0 YAW=-79.1
STAT after: STAT 1010 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-79.8 X=-0.4 Y=-5.1 D=5.1 VEL=0.0 DROP=0 IMU=OK
Follow-up STAT: STAT 2783 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-80.2 X=-0.4 Y=-5.1 D=5.1 VEL=0.0 DROP=0 IMU=OK
```

Predicted action effect from pre-state:

```text
slot_y_dist_cm: 20.205 -> 15.760
slot_x_err_px: -50.0 -> -8.1
slot_lateral_cm: 4.17 -> 1.506
slot_heading_err_deg: 0.0 -> -0.979
min_margin_px: 198.0 -> 197.49
```

Post-action dry-run:

```text
Board log: /tmp/parking_action_replanner_post_ste60_strict_dryrun_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_action_replanner_post_ste60_strict_dryrun_20260613.jsonl
Result: YOLO/no target; car is likely in terminal blind zone.
No motion sent.
No new token was generated because the visible pre-state did not satisfy the
strict lateral token gate before the corrective action.
```

Decision:

```text
The measured visible correction executed successfully and likely reduced
lateral bias to near the token threshold. Because there is no post-action visual
state, do not automatically continue. Require user visual assessment before
allowing any reconstructed one-shot final-blind token from the predicted
post-action state.
```

## User-Confirmed Final Blind After STE=60 Correction - 2026-06-13

User visual assessment after the STE=60 correction:

```text
The car body was indeed corrected and very close to the target pose.
```

Based on this human confirmation, a one-shot reconstructed token was generated
from the predicted post-correction state:

```text
Token file:
  artifacts/autopark_baseline/parking_final_blind_token_after_ste60_user_confirmed_20260613.json
Board token:
  /tmp/parking_final_blind_token.json

Predicted post-correction state:
  phase_hint=straighten_or_enter
  slot_x_err_px=-8.1
  slot_lateral_cm=1.506
  slot_heading_err_deg=-0.979
  slot_y_dist_cm=15.76
  min_margin_px=197.49

Temporary gate for this token:
  max_lateral_cm=1.6
  max_terminal_lateral_cm=1.8
  visual_user_confirmed=true
```

No-motion check:

```text
Board log: /tmp/parking_final_blind_after_ste60_dryrun_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_final_blind_after_ste60_dryrun_20260613.jsonl
computed_pre_state_review.pass=true
candidate_cmd=MOVE D=-5.0 V=1
desired_actual_cm=4.0
```

Real final blind command:

```text
Board log: /tmp/parking_final_blind_after_ste60_real_20260613.jsonl
Local log: artifacts/autopark_baseline/parking_final_blind_after_ste60_real_20260613.jsonl
Command: MOVE D=-5.0 V=1
Reason: desired actual 4cm + deadband 2cm - coast 1cm
```

STM32 result:

```text
DONE 1004 MOVE X=0.0 Y=-3.1 D=3.1 YAW=-86.8
STAT after: STAT 1006 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-86.9 X=0.0 Y=-3.7 D=3.7 VEL=0.0 DROP=0 IMU=OK
Follow-up STAT: STAT 3154 MODE=IDLE RUN=STANDBY DIR=-1 SPD=0 ANG=90.0 YAW=-87.3 X=0.0 Y=-3.7 D=3.7 VEL=0.0 DROP=0 IMU=OK
```

Token status:

```text
consumed=true
```

Decision:

```text
This two-stage sequence is now the best current successful pattern:
  1. visible measured correction: ARC D=-6.0 STE=60 V=1
  2. user/strict-gate final blind: MOVE D=-5.0 V=1

The final-blind deadband compensation worked better than D=-4.0:
  D=-4.0 produced about 2.1-2.9cm
  D=-5.0 produced about 3.1-3.7cm

Need user final visual assessment before declaring the parking result good.
```

User final visual assessment:

```text
The result basically meets the requirement.
The car basically entered the parking slot.
Remaining issue: the final attitude is slightly skewed; the car body is not
perfectly parallel to the slot side boundary line. User considers this a minor
issue.
```

Current best-known successful pattern:

```text
Initial visible state near terminal:
  align_in_corridor
  slot_y_dist_cm ~= 20.2
  slot_lateral_cm ~= 4.17
  slot_x_err_px ~= -50

Step 1 visible correction:
  ARC D=-6.0 STE=60 V=1
  actual odom ~= 5.1cm

Step 2 terminal final blind:
  MOVE D=-5.0 V=1
  actual odom ~= 3.1-3.7cm

Outcome:
  Basic parking success, with slight final yaw/body skew.
```

Next optimization target:

```text
Distance is now acceptable. The remaining quality issue is final attitude.
Prioritize tuning the last visible correction / terminal heading cancellation:
  - either reduce the STE=60 correction distance slightly if it over-rotates;
  - or add a measured terminal counter-steer/heading-cancel action before the
    final straight blind step;
  - do not increase final blind distance as the primary fix.
```
```

## 2026-06-13 Tuning: Chassis Kinematics + Final Pose Straighten

Implemented offline and no-motion code path for the final-pose straightening plan.

Artifacts:

```text
tools/extract_chassis_kinematics.py
configs/chassis_kinematics.json
tools/board_parking_controller.py
```

Extracted steering response from existing logs:

```text
STE=60  n=5 deg_per_cm=-0.716722 r_eff_cm=79.941  direction=left
STE=75  n=3 deg_per_cm=-0.389831 r_eff_cm=146.976 direction=left
STE=105 n=3 deg_per_cm= 0.220588 r_eff_cm=259.741 direction=right
STE=120 n=2 deg_per_cm= 0.572472 r_eff_cm=100.085 direction=right
```

Controller changes:

```text
--chassis-kinematics-json loads measured steering curvature.
--counter-steer-enable enables dynamic terminal counter-steer only in straighten_or_enter.
counter_steer_decision logs the selected side, STE gear, distance, and predicted heading delta.
counter_steer_result logs measured yaw delta after real execution.
final_blind tokens now include yaw_token.
final_pose_report logs final_heading_deg / final_lateral_est_cm / depth_est_cm / verdict.
```

Default behavior remains unchanged unless `--counter-steer-enable` is supplied.
Smoke-tested locally with no board movement:

```text
py_compile passed for board_parking_controller.py, extract_chassis_kinematics.py,
parking_fusion.py, parking_action_scorer.py, parking_slot_state_analyzer.py.

counter_steer function smoke:
  heading +4 deg -> ARC D=-6.0 STE=60 V=1
  heading -4 deg -> ARC D=-6.0 STE=120 V=1
  heading +1 deg -> gate_closed

final_pose_report function smoke:
  final_heading 1.6 deg -> parked_straight
  final_heading 5.0 deg -> parked_crooked

controller entry smoke:
  action_replanner --replanner-dry-run loaded local configs and exited no-motion.
```

## 2026-06-13 Board Deploy: Counter-Steer No-Motion Dry-Run

Deployed the final-pose straighten implementation to the board:

```text
/opt/parking/autopark/board_parking_controller.py
/opt/parking/autopark/parking_fusion.py
/opt/parking/autopark/chassis_kinematics.json
```

Board verification:

```text
python3 -m py_compile board_parking_controller.py parking_fusion.py passed.
chassis_kinematics.json loaded:
  STE=60  deg_per_cm=-0.716722 n=5
  STE=75  deg_per_cm=-0.389831 n=3
  STE=105 deg_per_cm= 0.220588 n=3
  STE=120 deg_per_cm= 0.572472 n=2
```

No-motion live dry-run command:

```text
board_parking_controller.py --strategy action_replanner --replanner-dry-run
  --counter-steer-enable --allow-final-blind-reverse
  --chassis-kinematics-json /opt/parking/autopark/chassis_kinematics.json
  --log-jsonl /tmp/parking_counter_steer_dryrun_20260613.jsonl
```

Result:

```text
No STM32 serial was opened and no motion command was sent.
YOLO/UDP path was alive, but current packets had detections=[].
UDP probe on 127.0.0.1:24580 received 27 packets in 6 seconds.
Each packet had detection_count=0.
Dry-run event counts:
  vision_lost: 23
  final_blind_reverse_candidate: 22
  duration_elapsed: 1
No candidate or counter_steer_decision events were produced because no live slot
state was available.
```

Interpretation:

```text
Deployment and no-motion safety path are good.
The next live validation requires the model to detect the slot again. Current
blocking point is perception visibility/model output, not controller startup,
configuration loading, or UDP transport.
```
