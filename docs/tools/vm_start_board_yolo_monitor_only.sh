#!/usr/bin/env bash
set -eo pipefail

STATE_DIR=${STATE_DIR:-/tmp/parking_board_yolo_monitor_only}
LOG_DIR="$STATE_DIR/logs"
PID_FILE="$STATE_DIR/pids"
IMAGE_UDP_PORT=${IMAGE_UDP_PORT:-24581}
DET_UDP_PORT=${DET_UDP_PORT:-24580}
VIEW_TOPIC=${VIEW_TOPIC:-/parking/yolo/parking_view}
DETECTIONS_TOPIC=${DETECTIONS_TOPIC:-/parking/yolo/parking_detections}
LIVE_VIEW_NODE=${LIVE_VIEW_NODE:-/tmp/board_yolo_live_view_node.py}

mkdir -p "$LOG_DIR"

if [ -s "$PID_FILE" ]; then
  while read -r old; do
    [ -n "$old" ] || continue
    kill -INT "$old" 2>/dev/null || true
  done < "$PID_FILE"
  sleep 1
fi

for child in $(ps -eo pid,args | awk '/board_yolo_udp_node|board_yolo_live_view_node.py|board_yolo_rtsp_view_node|board_yolo_view_node|slot_geometry_transform_node|parking_target_pose_node|parking_metric_planner_node|parking_controller_dry_run_node|parking_planner_node/ && !/awk/ && !/vm_start_board_yolo_monitor_only/ {print $1}'); do
  kill -INT "$child" 2>/dev/null || true
done
sleep 2
for child in $(ps -eo pid,args | awk '/board_yolo_udp_node|board_yolo_live_view_node.py|board_yolo_rtsp_view_node|board_yolo_view_node|slot_geometry_transform_node|parking_target_pose_node|parking_metric_planner_node|parking_controller_dry_run_node|parking_planner_node/ && !/awk/ && !/vm_start_board_yolo_monitor_only/ {print $1}'); do
  kill -TERM "$child" 2>/dev/null || true
done

: > "$PID_FILE"

start_node() {
  local name="$1"
  shift
  nohup setsid bash -lc "set +u; source /opt/ros/humble/setup.bash; source ~/parking_ws/install/setup.bash; set -u; exec $*" \
    > "$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  echo "$pid" >> "$PID_FILE"
  echo "${name}_PID $pid"
}

start_node board_yolo_udp \
  ros2 run parking_bridge board_yolo_udp_node --ros-args \
    -p listen_host:=0.0.0.0 \
    -p listen_port:="$DET_UDP_PORT" \
    -p detections_topic:="$DETECTIONS_TOPIC" \
    -p state_topic:=/parking/perception/state

if [ -f "$LIVE_VIEW_NODE" ]; then
  start_node board_yolo_live_view \
    python3 "$LIVE_VIEW_NODE" --ros-args \
      -p image_udp_port:="$IMAGE_UDP_PORT" \
      -p detections_topic:="$DETECTIONS_TOPIC" \
      -p view_topic:="$VIEW_TOPIC" \
      -p jpeg_quality:=80
else
  echo "BOARD_YOLO_LIVE_VIEW_NODE_MISSING $LIVE_VIEW_NODE"
  echo "Upload artifacts/milestones/board_om_yolo_live_success_20260609_1215/board_yolo_live_view_node.py to $LIVE_VIEW_NODE"
fi

echo "BOARD_YOLO_MONITOR_ONLY_LOG_DIR $LOG_DIR"
echo "BOARD_YOLO_MONITOR_ONLY_DETECTIONS_TOPIC $DETECTIONS_TOPIC"
echo "BOARD_YOLO_MONITOR_ONLY_VIEW_TOPIC $VIEW_TOPIC"
echo "BOARD_YOLO_MONITOR_ONLY_NO_PLANNER true"
echo "BOARD_YOLO_MONITOR_ONLY_NO_CONTROLLER true"
