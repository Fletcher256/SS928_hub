#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/parking_ws/install/setup.bash"

LOG=/tmp/board_yolo_live_view.log
nohup python3 /tmp/board_yolo_live_view_node.py \
  --ros-args \
  -p image_udp_port:=24581 \
  -p detections_topic:=/parking/yolo/parking_detections \
  -p view_topic:=/parking/yolo/parking_view \
  -p jpeg_quality:=80 \
  >"$LOG" 2>&1 &

echo "BOARD_YOLO_LIVE_VIEW_PID $!"
echo "BOARD_YOLO_LIVE_VIEW_LOG $LOG"
echo "BOARD_YOLO_LIVE_VIEW_TOPIC /parking/yolo/parking_view"
