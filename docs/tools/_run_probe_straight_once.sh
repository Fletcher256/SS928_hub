#!/bin/sh
set -eu

LOG=/tmp/parking_probe_straight_6_20260613.jsonl
ARM=/tmp/parking_armed

cleanup() {
  rm -f "$ARM"
  /usr/local/bin/python3 -c "import sys; sys.path.insert(0,'/opt/parking/autopark'); import board_parking_controller as b; print(b.send_cmd('STOP',2).strip())" || true
}
trap cleanup EXIT INT TERM

touch "$ARM"

/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy primitive_probe \
  --primitive-command 'MOVE D=-6.0 V=1' \
  --primitive-max-command-abs-d-cm 6 \
  --arm \
  --target-wait-sec 1 \
  --settle-sec 0.5 \
  --move-read-sec 8 \
  --stable-frames 3 \
  --pixel-vision-lost-stop-sec 0.5 \
  --max-motion-steps 1 \
  --max-total-cm 8 \
  --log-stm32-detail \
  --success-criteria-json /opt/parking/autopark/parking_success_criteria.json \
  --log-jsonl "$LOG"

echo "PROBE_LOG $LOG"
