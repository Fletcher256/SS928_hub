# T4 Replay Planner Report - 2026-06-12

## Purpose

T4 adds a PC-side offline replay planner:

```text
historical slot_relative_state rows
  -> parking_action_scorer.score_actions()
  -> per-row ranking
  -> gated chosen action
  -> stability and direction acceptance checks
```

It does not connect to the board, VM, YOLO process, camera, STM32, serial port,
or any actuator. It only reads historical logs/configs and writes replay
artifacts.

## Tool

New tool:

```text
tools/parking_replay_planner.py
```

Baseline command:

```powershell
.venv\Scripts\python tools\parking_replay_planner.py `
  artifacts\autopark_baseline\parking_slot_state_dryrun_20260612.jsonl `
  --library configs\parking_action_library.json `
  --responses configs\parking_action_response_model.json `
  --out artifacts\autopark_baseline\parking_replay_planner_20260612.json `
  --csv artifacts\autopark_baseline\parking_replay_planner_20260612.csv `
  --max-switches 2
```

CSV compatibility command:

```powershell
.venv\Scripts\python tools\parking_replay_planner.py `
  artifacts\autopark_baseline\slot_state_dryrun_rows_20260612.csv `
  --library configs\parking_action_library.json `
  --responses configs\parking_action_response_model.json
```

## Output Schema

JSON report:

- `schema`: `parking_replay_planner_report.v1`
- `inputs`: state logs, action library, response model
- `counts`: state rows, stable rows, wait rows, stop rows
- `top_action_counts`: chosen action counts, including `WAIT`/`STOP`
- `stable_top_action_counts`: chosen action counts for stable actionable rows
- `action_switch_count_stable`: stable actionable action switch count
- `chosen_switch_count_all`: switch count including `WAIT`/`STOP`
- `direction_review`: right-offset steering direction audit
- `acceptance`: row-count, switch-count, and direction pass/fail
- `rows`: per-row `pre_state`, `ranking`, and `chosen`

CSV fields:

```text
source_file,lineno,stable,stable_enough,line_risk,phase_hint,
slot_x_err_px,slot_y_dist_cm,min_margin_px,
chosen_action_id,chosen_command,chosen_reason,
best_cost,best_origin,best_confidence,top3
```

## Baseline Result

Input:

```text
artifacts/autopark_baseline/parking_slot_state_dryrun_20260612.jsonl
```

Result:

```json
{
  "state_rows": 33,
  "stable_rows": 31,
  "stable_actionable_rows": 31,
  "wait_rows": 2,
  "stop_rows": 0,
  "top_action_counts": {"WAIT": 2, "reverse_right_hard_6": 31},
  "stable_top_action_counts": {"reverse_right_hard_6": 31},
  "action_switch_count_stable": 0,
  "chosen_switch_count_all": 1,
  "direction_review_pass": true,
  "acceptance_pass": true
}
```

Artifacts:

```text
artifacts/autopark_baseline/parking_replay_planner_20260612.json
artifacts/autopark_baseline/parking_replay_planner_20260612.csv
```

## Direction Review

For rows with `slot_x_err_px > 40`, the expected steering side is right
correction. T4 defines right correction as either:

- `chosen_action_id` starts with `reverse_right_`
- or parsed command servo `STE > 90`

The baseline replay had:

```text
right_offset_rows: 33
checked_rows: 31
skipped_wait_or_stop_rows: 2
wrong_direction_rows: 0
```

The first two rows were skipped because the state was not stable enough, so the
gated decision was `WAIT`. All stable actionable rows selected:

```text
reverse_right_hard_6 -> ARC D=-6.0 STE=120 V=1
```

## Validation

Static compile:

```powershell
.venv\Scripts\python -m py_compile tools\parking_replay_planner.py tools\parking_action_scorer.py tools\parking_slot_state_analyzer.py
```

Result: passed.

Replay acceptance:

```text
33 rows replayed
stable action switch count: 0 <= 2
direction review: pass
acceptance: pass
```

CSV input compatibility: passed with the same 33-row result.

## Follow-up Use

T3 should align its board-side `action_replanner` JSONL step output with this
shape:

```text
pre_state -> ranking -> chosen -> gates -> post_state/delta/verdict
```

Before any real action-replanner motion, S1 dry-run should produce a log that
can be replayed by this tool and pass the same stability and direction checks.
