#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=${STATE_DIR:-/tmp/parking_board_yolo_vision_only}
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

for child in $(ps -eo pid,args | awk '/board_yolo_udp_node|board_yolo_view_node|board_yolo_rtsp_view_node|slot_geometry_transform_node|parking_target_pose_node|parking_metric_planner_node|parking_yolo_node/ && !/awk/ && !/vm_start_board_yolo_vision_only/ {print $1}'); do
  kill -INT "$child" 2>/dev/null || true
done
sleep 2
for child in $(ps -eo pid,args | awk '/board_yolo_udp_node|board_yolo_view_node|board_yolo_rtsp_view_node|slot_geometry_transform_node|parking_target_pose_node|parking_metric_planner_node|parking_yolo_node/ && !/awk/ && !/vm_start_board_yolo_vision_only/ {print $1}'); do
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

start_node board_yolo_rtsp_view \
  ros2 run parking_bridge board_yolo_rtsp_view_node --ros-args \
    -p rtsp_url:=rtsp://192.168.137.2:554/live0 \
    -p detections_topic:=/parking/yolo/parking_detections \
    -p view_topic:=/parking/yolo/parking_view \
    -p output_width:=1280 \
    -p publish_fps:=10.0 \
    -p jpeg_quality:=65 \
    -p rotate180:=true

start_node slot_geometry_transform \
  ros2 run parking_bridge slot_geometry_transform_node --ros-args \
    -p detections_topic:=/parking/yolo/parking_detections \
    -p slot_geometry_topic:=/parking/slot_geometry \
    -p state_topic:=/parking/slot_geometry_state \
    -p calibration_file:=/home/ebaina/parking_calibration/slot_homography_rear_axle.json \
    -p vehicle_frame_id:=vehicle_rear_axle_cm

start_node parking_target_pose \
  ros2 run parking_bridge parking_target_pose_node --ros-args \
    -p slot_geometry_topic:=/parking/slot_geometry \
    -p target_pose_topic:=/parking/target_pose \
    -p state_topic:=/parking/target_pose_state \
    -p rear_axle_to_vehicle_center_cm:=11.0 \
    -p approach_distance_cm:=18.0

start_node parking_metric_planner \
  ros2 run parking_bridge parking_metric_planner_node --ros-args \
    -p target_pose_topic:=/parking/target_pose \
    -p path_topic:=/parking/planner/path_cm \
    -p state_topic:=/parking/planner/path_cm_state \
    -p rear_axle_to_vehicle_center_cm:=11.0 \
    -p step_cm:=8.0 \
    -p command_distance_deadband_cm:=2.0 \
    -p command_speed_gear:=1

echo "BOARD_YOLO_VISION_ONLY_LOG_DIR $LOG_DIR"
