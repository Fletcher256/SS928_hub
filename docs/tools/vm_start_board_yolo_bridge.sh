#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=${STATE_DIR:-/tmp/parking_board_yolo}
LOG_DIR="$STATE_DIR/logs"
PID_FILE="$STATE_DIR/pids"
mkdir -p "$LOG_DIR"

if [ -s "$PID_FILE" ]; then
  while read -r old; do
    [ -n "$old" ] || continue
    kill -INT "$old" 2>/dev/null || true
  done < "$PID_FILE"
  sleep 1
fi

for child in $(ps -eo pid,args | awk '/parking_bridge.*parking.launch.py|board_yolo_udp_node|parking_yolo_node|parking_planner_node|parking_controller_dry_run_node/ && !/awk/ && !/vm_start_board_yolo_bridge/ {print $1}'); do
  kill -INT "$child" 2>/dev/null || true
done
sleep 2
for child in $(ps -eo pid,args | awk '/parking_bridge.*parking.launch.py|board_yolo_udp_node|parking_yolo_node|parking_planner_node|parking_controller_dry_run_node/ && !/awk/ && !/vm_start_board_yolo_bridge/ {print $1}'); do
  kill -TERM "$child" 2>/dev/null || true
done

: > "$PID_FILE"

start_node() {
  local name="$1"
  shift
  nohup setsid bash -lc "source /opt/ros/humble/setup.bash && source ~/parking_ws/install/setup.bash && exec $*" \
    > "$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  echo "$pid" >> "$PID_FILE"
  echo "${name}_PID $pid"
}

start_node board_yolo_udp \
  ros2 run parking_bridge board_yolo_udp_node --ros-args \
    -p listen_host:=0.0.0.0 \
    -p listen_port:=24580 \
    -p detections_topic:=/parking/yolo/parking_detections \
    -p state_topic:=/parking/perception/state

start_node board_yolo_view \
  ros2 run parking_bridge board_yolo_view_node --ros-args \
    -p detections_topic:=/parking/yolo/parking_detections \
    -p view_topic:=/parking/yolo/parking_view

start_node parking_planner \
  ros2 run parking_bridge parking_planner_node --ros-args \
    -p fallback_to_pixel_candidates:=false \
    -p stale_after_sec:=1.5

start_node parking_controller_dry_run \
  ros2 run parking_bridge parking_controller_dry_run_node

echo "BOARD_YOLO_BRIDGE_LOG_DIR $LOG_DIR"
