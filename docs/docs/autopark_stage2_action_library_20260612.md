# Autopark Stage 2 Action Library - 2026-06-12

## Goal

Stage 2 builds the software side of the action-template replanning architecture:

```text
slot_relative_state -> bounded action templates -> one-step score ranking
```

This stage does not command motion. It ranks candidate actions offline so the
next real test can be chosen deliberately.

## Implemented Files

- `configs/parking_action_library.json`
  - Defines bounded reverse actions and scoring weights.

- `configs/parking_action_response_model.json`
  - Stores measured response records.
  - Currently includes the failed `STE=60` probe.

- `tools/parking_action_scorer.py`
  - Reads JSONL logs or slot-state CSV rows.
  - Predicts one-step result for each action.
  - Scores and ranks actions.
  - Marks predictions as `measured` or `prior`.

## Action Library

Current bounded reverse actions:

```text
reverse_straight_6     MOVE D=-6.0 V=1
reverse_left_hard_6    ARC D=-6.0 STE=60 V=1
reverse_left_soft_6    ARC D=-6.0 STE=75 V=1
reverse_right_soft_6   ARC D=-6.0 STE=105 V=1
reverse_right_hard_6   ARC D=-6.0 STE=120 V=1
```

Each action has:

- command
- distance
- servo
- allowed phases
- predicted delta
- prior confidence
- notes

## Measured Response

Only one real response is currently measured:

```text
reverse_left_hard_6
ARC D=-6.0 STE=60 V=1
delta slot_x_err_px: +28.0
delta slot_lateral_cm: -1.67
delta min_margin_px: -24.0
verdict: worsened
```

Therefore `STE=60` should not be used as the entry arc from that pose.

All other actions are still priors and must be calibrated before promotion.

## Scoring Logic

The scorer penalizes:

- large absolute `slot_x_err_px`
- large absolute `slot_heading_err_deg`
- large absolute `slot_lateral_cm`
- low or shrinking `min_margin_px`
- line risk
- phase mismatch
- uncalibrated/low-confidence actions
- large steering offset

It rewards:

- useful reduction of `slot_y_dist_cm`

The result is a ranking, not permission to move.

## Validation Run

Input:

```text
artifacts/autopark_baseline/parking_slot_state_dryrun_20260612.jsonl
```

Command:

```powershell
.venv\Scripts\python tools\parking_action_scorer.py artifacts\autopark_baseline\parking_slot_state_dryrun_20260612.jsonl --tail 5 --out artifacts\autopark_baseline\parking_action_scores_stage2_20260612.json
```

Result:

```text
state_rows = 33
selected_state_rows = 5
top_action_counts = reverse_right_hard_6:5
```

Latest ranked actions:

```text
1. reverse_right_hard_6   ARC D=-6.0 STE=120 V=1   origin=prior     confidence=0.25
2. reverse_right_soft_6   ARC D=-6.0 STE=105 V=1   origin=prior     confidence=0.25
3. reverse_straight_6     MOVE D=-6.0 V=1          origin=prior     confidence=0.35
4. reverse_left_soft_6    ARC D=-6.0 STE=75 V=1    origin=prior     confidence=0.25
5. reverse_left_hard_6    ARC D=-6.0 STE=60 V=1    origin=measured  confidence=0.90
```

Interpretation:

```text
The software now recommends STE=120 as the next action to calibrate,
but it is still a prior, not a verified control action.
```

## Next Step

Reset the car to the same initial pose and run:

```text
ARC D=-6.0 STE=120 V=1
```

Then update `configs/parking_action_response_model.json` with the measured
delta. If `STE=120` improves `slot_x_err_px`, `slot_lateral_cm`, and
`min_margin_px`, it can become the preferred entry action for similar states.

## Not Done Yet

The following are not implemented in Stage 2:

- real-time `--strategy action_replanner`
- multi-step lookahead
- automatic execution of scorer output
- forward correction actions
- adaptive response model fitting

Those belong to later stages after more real calibration data exists.
