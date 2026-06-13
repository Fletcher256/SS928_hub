#!/bin/sh
set -eu

APP_DIR=/opt/sample/parking_yolo_seg_safe
LOG=/tmp/parking_yolo_seg_safe_live.log
PID_FILE=/tmp/parking_yolo_seg_safe_live.pid
VM_HOST=${PARKING_YOLO_UDP_HOST:-192.168.137.100}
VM_PORT=${PARKING_YOLO_UDP_PORT:-24580}

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
export LD_LIBRARY_PATH=/opt/lib/npu:/opt/lib:${LD_LIBRARY_PATH:-}
export PARKING_YOLO_UDP_HOST="$VM_HOST"
export PARKING_YOLO_UDP_PORT="$VM_PORT"

nohup sh -c 'tail -f /dev/null | ./sample_parking_yolo' > "$LOG" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "BOARD_PARKING_YOLO_SAFE_PID $pid"
echo "BOARD_PARKING_YOLO_SAFE_LOG $LOG"
echo "BOARD_PARKING_YOLO_SAFE_UDP ${VM_HOST}:${VM_PORT}"
