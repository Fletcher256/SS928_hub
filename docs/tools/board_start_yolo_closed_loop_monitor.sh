#!/bin/sh
set -eu

APP_DIR=${APP_DIR:-/opt/sample/parking_yolo_seg_safe}
AUTOPARK_DIR=${AUTOPARK_DIR:-/opt/parking/autopark}
BIN=${BIN:-./sample_parking_yolo_rtsp}
LOG=${LOG:-/tmp/parking_yolo_closed_loop_monitor.log}
PID_FILE=${PID_FILE:-/tmp/parking_yolo_closed_loop_monitor.pid}
TEE_LOG=${TEE_LOG:-/tmp/parking_yolo_udp_tee.log}
TEE_PID_FILE=${TEE_PID_FILE:-/tmp/parking_yolo_udp_tee.pid}

TEE_HOST=${TEE_HOST:-127.0.0.1}
TEE_PORT=${TEE_PORT:-24579}
LOCAL_CONTROLLER_HOST=${LOCAL_CONTROLLER_HOST:-127.0.0.1}
LOCAL_CONTROLLER_PORT=${LOCAL_CONTROLLER_PORT:-24580}
VM_HOST=${VM_HOST:-192.168.137.100}
VM_DET_PORT=${VM_DET_PORT:-24580}
VM_IMAGE_PORT=${VM_IMAGE_PORT:-24581}

for pid_file in "$PID_FILE" "$TEE_PID_FILE"; do
  if [ -s "$pid_file" ]; then
    old="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -n "$old" ]; then
      kill -INT "$old" 2>/dev/null || true
    fi
  fi
done

for pattern in sample_camera_rtsp sample_parking_yolo sample_parking_yolo_rtsp board_yolo_udp_tee.py; do
  for pid in $(ps | grep "$pattern" | grep -v grep | awk '{print $1}'); do
    kill -INT "$pid" 2>/dev/null || true
  done
done
sleep 2
for pattern in sample_camera_rtsp sample_parking_yolo sample_parking_yolo_rtsp board_yolo_udp_tee.py; do
  for pid in $(ps | grep "$pattern" | grep -v grep | awk '{print $1}'); do
    kill -TERM "$pid" 2>/dev/null || true
  done
done

nohup /usr/local/bin/python3 "$AUTOPARK_DIR/board_yolo_udp_tee.py" \
  --listen-host "$TEE_HOST" \
  --listen-port "$TEE_PORT" \
  --target "${LOCAL_CONTROLLER_HOST}:${LOCAL_CONTROLLER_PORT}" \
  --target "${VM_HOST}:${VM_DET_PORT}" \
  > "$TEE_LOG" 2>&1 &
tee_pid=$!
echo "$tee_pid" > "$TEE_PID_FILE"

cd "$APP_DIR"
export LD_LIBRARY_PATH=/opt/lib/npu:/opt/lib:${LD_LIBRARY_PATH:-}
export PARKING_YOLO_UDP_HOST="$TEE_HOST"
export PARKING_YOLO_UDP_PORT="$TEE_PORT"
export PARKING_YOLO_IMAGE_UDP_HOST="$VM_HOST"
export PARKING_YOLO_IMAGE_UDP_PORT="$VM_IMAGE_PORT"
export PARKING_YOLO_IMAGE_STRIDE="${PARKING_YOLO_IMAGE_STRIDE:-30}"
export PARKING_YOLO_RUN_FOREVER="${PARKING_YOLO_RUN_FOREVER:-1}"
export PARKING_YOLO_LOWLIGHT_AE="${PARKING_YOLO_LOWLIGHT_AE:-1}"
export PARKING_YOLO_AE_COMPENSATION="${PARKING_YOLO_AE_COMPENSATION:-96}"
export PARKING_YOLO_AE_MIN_EXP_US="${PARKING_YOLO_AE_MIN_EXP_US:-0}"
export PARKING_YOLO_AE_MAX_EXP_US="${PARKING_YOLO_AE_MAX_EXP_US:-944036}"
export PARKING_YOLO_ROTATE180="${PARKING_YOLO_ROTATE180:-1}"
export PARKING_YOLO_SWAP_UV="${PARKING_YOLO_SWAP_UV:-1}"
export PARKING_YOLO_CONFIDENCE_THRESHOLD="${PARKING_YOLO_CONFIDENCE_THRESHOLD:-0.25}"

nohup "$BIN" > "$LOG" 2>&1 &
yolo_pid=$!
echo "$yolo_pid" > "$PID_FILE"

echo "BOARD_YOLO_CLOSED_LOOP_MONITOR_TEE_PID $tee_pid"
echo "BOARD_YOLO_CLOSED_LOOP_MONITOR_TEE_LOG $TEE_LOG"
echo "BOARD_YOLO_CLOSED_LOOP_MONITOR_PID $yolo_pid"
echo "BOARD_YOLO_CLOSED_LOOP_MONITOR_LOG $LOG"
echo "BOARD_YOLO_DETECTION_TEE ${TEE_HOST}:${TEE_PORT}"
echo "BOARD_CONTROLLER_DETECTION ${LOCAL_CONTROLLER_HOST}:${LOCAL_CONTROLLER_PORT}"
echo "VM_MONITOR_DETECTION ${VM_HOST}:${VM_DET_PORT}"
echo "VM_MONITOR_IMAGE ${VM_HOST}:${VM_IMAGE_PORT}"
