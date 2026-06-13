#!/bin/sh
set -u

touch /tmp/parking_armed
/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy primitive_probe \
  --primitive-command 'ARC D=-6.0 STE=60 V=1' \
  --primitive-max-command-abs-d-cm 8 \
  --arm \
  --target-wait-sec 1 \
  --settle-sec 0.5 \
  --move-read-sec 8 \
  --stable-frames 3 \
  --pixel-vision-lost-stop-sec 0.5 \
  --max-motion-steps 1 \
  --max-total-cm 8 \
  --log-stm32-detail \
  --pre-steer-settle-sec 0.5 \
  --log-jsonl /tmp/parking_probe_left_20260612.jsonl
rc=$?
rm -f /tmp/parking_armed
/usr/local/bin/python3 -c "import sys; sys.path.insert(0,'/opt/parking/autopark'); import board_parking_controller as b; print(b.send_cmd('STOP',2).strip()); print(b.read_stat()['raw']); print(b.query_pwm_stat())"
exit "$rc"
