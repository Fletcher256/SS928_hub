# T5 Probe Runner Automation - 2026-06-12

## Purpose

T5 adds a PC-side automation tool for one-action calibration campaigns:

```text
manual physical reset
  -> no-motion reset quality capture
  -> reset quality gate
  -> one primitive real probe
  -> no-motion post capture
  -> merged calibration log
  -> response model update
```

The first calibration target remains:

```text
ARC D=-6.0 STE=120 V=1
```

## Implemented

New tool:

```text
tools/parking_probe_runner.py
```

Default behavior is plan-only. It prints the exact board commands and does not
connect to motion unless both flags are present:

```text
--execute --allow-risk
```

The runner does not create `/tmp/parking_armed`. Real motion still requires the
operator to create that file manually on the board before running the campaign.

## Workflow

The tool performs these steps when executed:

1. Optionally sync local controller/configs to the board.
2. Capture reset quality with:
   `--strategy action_replanner --replanner-dry-run`.
3. Compare the latest stable reset window against the baseline log:
   `artifacts/autopark_baseline/parking_action_replanner_dryrun_20260612.jsonl`.
4. Abort before motion if:
   - stable reset rows `< 10`
   - `abs(slot_x_err_delta_px) > 5`
   - `abs(heading_delta_deg) > 1`
   - `/tmp/parking_armed` is missing
5. Execute exactly one primitive probe:
   `ARC D=-6.0 STE=120 V=1`.
6. Capture post-state with another no-motion dry-run.
7. Merge reset/probe/post logs into one combined log.
8. Run `parking_response_model_updater.py` logic and update:
   `configs/parking_action_response_model.json`.
9. Optionally sync the updated response model back to the board.

## Plan-Only Check

Command run locally:

```powershell
.venv\Scripts\python tools\parking_probe_runner.py --stamp 20260612_plancheck --action-label reverse_right_hard_6
```

Result:

```text
py_compile: passed
plan-only: passed
executed: false
probe_executed: false
```

The generated plan contains three board commands:

```text
reset no-motion capture
real primitive probe
post no-motion capture
```

## Current Board Arm State

Read-only check:

```text
/tmp/parking_armed: missing
```

Therefore the real probe cannot run yet. This is expected and safe: the runner
will refuse before motion unless the arm file exists.

## First STE=120 Attempt

Command:

```powershell
.venv\Scripts\python tools\parking_probe_runner.py --execute --allow-risk --sync-inputs-to-board --sync-model-to-board --stamp 20260612_ste120_r1 --action-label reverse_right_hard_6
```

Result:

```text
executed: true
probe_executed: false
reason: reset_quality_failed
will_execute_motion true: 0
send_to_stm32 true: 0
```

Reset quality failed before the real primitive probe:

```text
baseline slot_x_err_px: 70.621
current  slot_x_err_px: 77.746
delta: +7.125 px
limit: +/-5 px

baseline slot_heading_err_deg: -0.913
current  slot_heading_err_deg: -4.277
delta: -3.364 deg
limit: +/-1 deg

baseline slot_y_dist_cm: 48.068
current  slot_y_dist_cm: 39.809
delta: -8.259 cm
```

Artifacts:

```text
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_reset.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_report.json
```

No response-model record was added for `reverse_right_hard_6`.

## Second STE=120 Attempt

Command:

```powershell
.venv\Scripts\python tools\parking_probe_runner.py --execute --allow-risk --sync-inputs-to-board --sync-model-to-board --stamp 20260612_ste120_r2 --action-label reverse_right_hard_6
```

Result:

```text
executed: true
probe_executed: false
reason: reset_quality_failed
will_execute_motion true: 0
send_to_stm32 true: 0
```

Reset quality failed before the real primitive probe:

```text
baseline slot_x_err_px: 70.621
current  slot_x_err_px: 15.481
delta: -55.140 px
limit: +/-5 px

baseline slot_heading_err_deg: -0.913
current  slot_heading_err_deg: -1.453
delta: -0.540 deg
limit: +/-1 deg

baseline slot_y_dist_cm: 48.068
current  slot_y_dist_cm: 30.024
delta: -18.044 cm
```

The heading gate passed, but the image-space lateral state was far from the
baseline. The reset dry-run also recommended `MOVE D=-6.0 V=1` instead of the
target `reverse_right_hard_6`, so this was not the same calibration bucket.

Artifacts:

```text
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r2_reset.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r2_report.json
```

No response-model record was added for `reverse_right_hard_6`.

Before retrying, reset the car closer to the prior baseline:

```text
slot_y_dist_cm near 48.1
slot_x_err_px near 70.6
slot_heading_err_deg near -0.9
```

Use the reset guide instead of judging this by eye:

```powershell
.venv\Scripts\python tools\parking_reset_guide.py `
  --execute `
  --allow-risk `
  --iterations 5 `
  --capture-sec 8 `
  --delay-sec 1
```

See `docs/autopark_reset_guide_20260612.md`.

## Real Campaign Command

After physically resetting the car to the taped start pose and manually creating
`/tmp/parking_armed`, run:

```powershell
.venv\Scripts\python tools\parking_probe_runner.py `
  --execute `
  --allow-risk `
  --sync-inputs-to-board `
  --sync-model-to-board `
  --stamp 20260612_ste120_r1 `
  --action-label reverse_right_hard_6
```

Expected outputs:

```text
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_reset.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_motion.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_post.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_combined.jsonl
artifacts/autopark_baseline/parking_probe_reverse_right_hard_6_20260612_ste120_r1_report.json
```

Acceptance for the real campaign:

```text
reset quality gate passes
one STM32 motion event exists
updated_from contains reverse_right_hard_6
response model contains a measured record for ARC D=-6.0 STE=120 V=1
updated model is synced back to the board if --sync-model-to-board is used
```

## Safety Notes

The real probe command reverses the car by a bounded single step:

```text
ARC D=-6.0 STE=120 V=1
```

The board controller is invoked with:

```text
--max-motion-steps 1
--max-total-cm 8
--primitive-max-command-abs-d-cm 8
--log-stm32-detail
--pre-steer-settle-sec 0.5
```

Do not run the real campaign unless the car is physically positioned in the
reset jig and there is enough clearance for the one-step reverse arc.
