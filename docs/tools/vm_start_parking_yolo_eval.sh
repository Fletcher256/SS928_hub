#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=${STATE_DIR:-/tmp/parking_yolo_eval}
RECORD_ROOT=${RECORD_ROOT:-/home/ebaina/parking_sensor_records/yolo_parking_eval}
RTSP_URL=${RTSP_URL:-rtsp://172.20.10.2:554/live0}
MODEL_PATH=${MODEL_PATH:-/home/ebaina/parking_models/best.onnx}
RUN_ID=$(date +%Y%m%d_%H%M%S)
RECORD_DIR="$RECORD_ROOT/run_$RUN_ID"
LOG="$STATE_DIR/parking_ros.log"
PID_FILE="$STATE_DIR/parking_ros.pid"

mkdir -p "$STATE_DIR" "$RECORD_DIR"
echo "$RECORD_DIR" > "$STATE_DIR/record_dir"

if [ -s "$PID_FILE" ]; then
  old="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [ -n "$old" ] && [ -d "/proc/$old" ]; then
    echo "VM_PARKING_YOLO_ALREADY_RUNNING $old"
    echo "VM_RECORD_DIR $RECORD_DIR"
    echo "VM_LOG $LOG"
    exit 0
  fi
fi

nohup setsid bash -lc "
source /opt/ros/humble/setup.bash
source ~/parking_ws/install/setup.bash
exec ros2 launch parking_bridge parking.launch.py \
  record_dir:=$RECORD_DIR \
  rtsp_url:=$RTSP_URL \
  enable_dtof:=false \
  camera_backend:=ffmpeg_mjpeg \
  camera_ffmpeg_low_delay:=true \
  camera_scale:=0.5 \
  camera_rotate:=rotate180 \
  publish_camera_raw:=false \
  camera_jpeg_quality:=90 \
  camera_publish_stride:=1 \
  camera_record_stride:=10 \
  publish_yolo_input:=true \
  yolo_input_topic:=/parking/camera/yolo_input_jpeg \
  yolo_camera_input_width:=1280 \
  yolo_camera_roi_bottom_fraction:=0.86 \
  yolo_camera_clahe_clip_limit:=2.0 \
  yolo_camera_sharpen_amount:=0.35 \
  yolo_camera_jpeg_quality:=96 \
  enable_vision_preprocess:=true \
  enable_yolo_person:=false \
  enable_parking_yolo:=true \
  enable_parking_planner:=true \
  parking_yolo_model_path:=$MODEL_PATH \
  parking_yolo_class_names:=Parking \
  parking_yolo_empty_class_names:=__none_empty__ \
  parking_yolo_occupied_class_names:=__none_occupied__ \
  parking_yolo_process_stride:=3 \
  parking_yolo_confidence_threshold:=0.35 \
  parking_yolo_nms_threshold:=0.45 \
  enable_stm32:=false
" > "$LOG" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"
echo "VM_PARKING_YOLO_PID $pid"
echo "VM_RECORD_DIR $RECORD_DIR"
echo "VM_LOG $LOG"
