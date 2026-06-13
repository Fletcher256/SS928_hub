# T3 Action Replanner Implementation - 2026-06-12

## Purpose

T3 adds the board-side action-template replanner strategy:

```text
YOLO slot polygon
  -> slot_relative_state
  -> action library + response model
  -> ranking
  -> gated chosen action
  -> replanner_step JSONL
```

The first supported validation mode is no-motion S1 dry-run. Real action motion
is still blocked unless all normal motion gates are satisfied.

## Implemented

Updated:

```text
tools/board_parking_controller.py
tools/parking_action_scorer.py
configs/parking_action_library.json
```

New controller strategy:

```text
--strategy action_replanner
```

New CLI flags:

```text
--replanner-dry-run
--confirm-each-step
--action-library-json PATH
--response-model-json PATH
--replanner-switch-penalty FLOAT
--replanner-hold-margin FLOAT
```

Planner core now lives inside `board_parking_controller.py` and is reused by
`parking_action_scorer.score_actions()`, so T3 board-side ranking and T4
offline replay use the same scoring implementation.

## Safety Behavior

No-motion modes:

```text
--dry-run
--strategy action_replanner --replanner-dry-run
```

In no-motion mode the controller does not open serial, send `SERVO`, send
`MOVE/ARC`, or send `STOP`.

Real motion still requires:

```text
--arm
/tmp/parking_armed
not --dry-run
not --replanner-dry-run
chosen action is MOVE or ARC
stable slot state
no STOP/WAIT gate
no cap/lateral stop
```

For action-replanner real motion, actions with `requires_measured=true` are
hard-blocked unless the current state bucket has an exact measured response.
The current action library marks all five existing actions as
`requires_measured=true`, so T3 cannot automatically execute uncalibrated
prior-only actions.

## JSONL Events

The controller still logs `candidate` events with `slot_relative_state` so the
existing analyzers keep working.

For `--strategy action_replanner`, it also logs `replanner_step`:

```json
{
  "event": "replanner_step",
  "step": 1,
  "pre_state": {},
  "ranking": [],
  "chosen": {},
  "gates": {},
  "stm32": {"sent": "", "ack": "", "done": "", "pwm_stat": "", "stat_after": ""},
  "post_state": {},
  "delta": {},
  "verdict": "continue",
  "totals": {"steps_done": 0, "total_cm": 0.0}
}
```

## Local Validation

Static compile:

```powershell
.venv\Scripts\python -m py_compile tools\board_parking_controller.py tools\parking_action_scorer.py tools\parking_replay_planner.py tools\parking_slot_state_analyzer.py
```

Result: passed.

T4 replay after sharing board planner core:

```text
state_rows: 33
stable_actionable_rows: 31
stable_top_action_counts: reverse_right_hard_6=31
action_switch_count_stable: 0
direction_review_pass: true
acceptance_pass: true
```

Direct function smoke test on the first stable row of
`parking_slot_state_dryrun_20260612.jsonl`:

```text
dry-run chosen: reverse_right_hard_6 -> ARC D=-6.0 STE=120 V=1
real-motion chosen: STOP, reason none_eligible
real-motion block reason: no_exact_measured_response
```

This is intentional: dry-run can audit prior-based recommendations, but real
action-replanner motion is blocked until calibration adds exact measured
response records for the current bucket.

## Board S1 Dry-Run

```bash
/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy action_replanner \
  --replanner-dry-run \
  --duration-sec 60 \
  --stable-frames 3 \
  --pixel-vision-lost-stop-sec 0.5 \
  --action-library-json /opt/parking/autopark/parking_action_library.json \
  --response-model-json /opt/parking/autopark/parking_action_response_model.json \
  --success-criteria-json /opt/parking/autopark/parking_success_criteria.json \
  --log-jsonl /tmp/parking_action_replanner_dryrun_20260612.jsonl
```

Executed on the board after syncing:

```text
/opt/parking/autopark/board_parking_controller.py
/opt/parking/autopark/parking_action_library.json
/opt/parking/autopark/parking_action_response_model.json
/opt/parking/autopark/parking_success_criteria.json
```

Board-side compile:

```text
/usr/local/bin/python3 -m py_compile /opt/parking/autopark/board_parking_controller.py
```

Result: passed.

S1 result:

```text
duration: 60 seconds
candidate events: 99
replanner_step events: 99
stable actionable rows: 97
chosen counts: WAIT=2, reverse_right_hard_6=97
stable chosen action switch count: 0
right-offset direction review: pass
will_execute_motion true: 0
send_to_stm32 true: 0
```

Downloaded board log:

```text
artifacts/autopark_baseline/parking_action_replanner_dryrun_20260612.jsonl
```

T4 replay artifacts:

```text
artifacts/autopark_baseline/parking_action_replanner_dryrun_t4_20260612.json
artifacts/autopark_baseline/parking_action_replanner_dryrun_t4_20260612.csv
```

S1 dry-run passed. This validates the no-motion realtime planner loop, not real
vehicle motion. The next real-world step remains calibration of measured action
responses, especially `ARC D=-6.0 STE=120 V=1`.
