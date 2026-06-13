#!/bin/sh
set -eu

LOG=/tmp/parking_path_template_planner_once_20260613.jsonl
ARM=/tmp/parking_armed
: > "$LOG"

cleanup() {
  rm -f "$ARM"
  /usr/local/bin/python3 - <<'PY' >/dev/null 2>&1 || true
import importlib.util
spec = importlib.util.spec_from_file_location("bpc", "/opt/parking/autopark/board_parking_controller.py")
bpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bpc)
bpc.serial_setup()
bpc.stop()
PY
}
trap cleanup EXIT INT TERM

touch "$ARM"

/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy path_template_planner \
  --arm \
  --target-wait-sec 1 \
  --settle-sec 0.6 \
  --move-read-sec 8 \
  --pre-steer-settle-sec 0.25 \
  --log-stm32-detail \
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

echo "PATH_TEMPLATE_REALRUN_LOG $LOG"
