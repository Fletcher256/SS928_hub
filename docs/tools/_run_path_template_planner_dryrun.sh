#!/bin/sh
set -eu

LOG=/tmp/parking_path_template_planner_dryrun_20260613.jsonl
: > "$LOG"

/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy path_template_planner \
  --dry-run \
  --duration-sec 18 \
  --target-wait-sec 1 \
  --settle-sec 0.4 \
  --stable-frames 3 \
  --max-motion-steps 6 \
  --max-total-cm 40 \
  --path-step-cm 6 \
  --path-max-commands 6 \
  --path-arc-steer-high 112 \
  --path-arc-steer-low 68 \
  --path-x-deadband-norm 0.05 \
  --path-template-min-margin-norm 0.10 \
  --log-jsonl "$LOG"

echo "PATH_TEMPLATE_DRYRUN_LOG $LOG"
