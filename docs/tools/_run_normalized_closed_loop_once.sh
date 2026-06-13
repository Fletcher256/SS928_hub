#!/bin/sh
set -eu

LOG=/tmp/parking_normalized_closed_loop_once_20260613.jsonl
ARM=/tmp/parking_armed

cleanup() {
  rm -f "$ARM"
  /usr/local/bin/python3 -c "import sys; sys.path.insert(0,'/opt/parking/autopark'); import board_parking_controller as b; print(b.send_cmd('STOP',2).strip())" || true
}
trap cleanup EXIT INT TERM

touch "$ARM"

/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy normalized_corridor_servo \
  --arm \
  --target-wait-sec 1 \
  --settle-sec 0.6 \
  --move-read-sec 8 \
  --stable-frames 3 \
  --pixel-vision-lost-stop-sec 0.5 \
  --max-motion-steps 5 \
  --max-total-cm 32 \
  --log-stm32-detail \
  --normalized-approach-d-cm 8 \
  --normalized-align-d-cm 6 \
  --normalized-enter-d-cm 6 \
  --normalized-min-command-abs-d-cm 5 \
  --normalized-x-tolerance 0.05 \
  --normalized-min-margin 0.12 \
  --normalized-kx 130 \
  --normalized-entry-kx 55 \
  --normalized-ka 0.45 \
  --normalized-min-steer-offset-deg 22 \
  --normalized-align-max-steer-offset-deg 32 \
  --normalized-enter-max-steer-offset-deg 22 \
  --success-criteria-json /opt/parking/autopark/parking_success_criteria.json \
  --log-jsonl "$LOG"

echo "CLOSED_LOOP_LOG $LOG"
