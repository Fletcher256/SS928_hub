#!/bin/sh
set -eu

APP_DIR=/opt/sample/parking_yolo_seg_safe
BIN=${BIN:-./sample_parking_yolo_rtsp}
LOG=/tmp/parking_yolo_seg_safe_live.log
PID_FILE=/tmp/parking_yolo_seg_safe_live.pid
VM_HOST=${VM_HOST:-192.168.137.100}

for pattern in sample_camera_rtsp sample_parking_yolo; do
  for pid in $(ps | grep "$pattern" | grep -v grep | awk '{print $1}'); do
    kill -INT "$pid" 2>/dev/null || true
  done
done
sleep 2
for pattern in sample_camera_rtsp sample_parking_yolo; do
  for pid in $(ps | grep "$pattern" | grep -v grep | awk '{print $1}'); do
    kill -TERM "$pid" 2>/dev/null || true
  done
done

cd "$APP_DIR"
: > "$LOG"
export LD_LIBRARY_PATH=/opt/lib/npu:/opt/lib:${LD_LIBRARY_PATH:-}
export PARKING_YOLO_UDP_HOST="$VM_HOST"
export PARKING_YOLO_UDP_PORT=24580
export PARKING_YOLO_IMAGE_UDP_HOST="$VM_HOST"
export PARKING_YOLO_IMAGE_UDP_PORT=24581
export PARKING_YOLO_IMAGE_STRIDE=30
export PARKING_YOLO_RUN_FOREVER=1
export PARKING_YOLO_LOWLIGHT_AE=1
export PARKING_YOLO_AE_COMPENSATION=96
export PARKING_YOLO_AE_MIN_EXP_US=0
export PARKING_YOLO_AE_MAX_EXP_US=944036
export PARKING_YOLO_ROTATE180=1
export PARKING_YOLO_SWAP_UV=1
export PARKING_YOLO_CONFIDENCE_THRESHOLD=0.25

nohup "$BIN" > "$LOG" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "BOARD_PARKING_YOLO_LIVE_PID $pid"
echo "BOARD_PARKING_YOLO_LIVE_LOG $LOG"
echo "BOARD_PARKING_YOLO_LIVE_BIN $BIN"
echo "BOARD_PARKING_YOLO_DET_UDP ${VM_HOST}:24580"
echo "BOARD_PARKING_YOLO_IMAGE_UDP ${VM_HOST}:24581"
