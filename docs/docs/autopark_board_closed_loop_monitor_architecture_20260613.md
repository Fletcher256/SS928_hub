# Board Closed-Loop + VM Monitor Architecture - 2026-06-13

## Decision

Current implementation direction:

```text
Board:
  OS08A20 camera
  -> board YOLO
  -> board YOLO UDP tee
  -> board_parking_controller.py
  -> safety gate
  -> STM32

VM:
  receive copied board YOLO detection UDP
  receive board YOLO image UDP
  publish Foxglove visualization only
```

The VM must not be part of the runtime control loop. It is a read-only monitor
for camera image, detection overlay, logs, and operator observation.

## Board Runtime Boundary

The board owns:

```text
/opt/sample/parking_yolo_seg_safe/sample_parking_yolo_rtsp
/opt/parking/autopark/board_parking_controller.py
/opt/parking/autopark/parking_action_library.json
/opt/parking/autopark/parking_action_response_model.json
/opt/parking/autopark/parking_success_criteria.json
STM32 serial device: /dev/ttyUSB0
```

The closed-loop controller listens to local YOLO detection UDP:

```text
127.0.0.1:24580
```

Because the board YOLO binary has a single detection UDP target, the runtime
uses a board-side tee:

```text
YOLO detection UDP -> 127.0.0.1:24579
board_yolo_udp_tee.py:
  -> 127.0.0.1:24580       board_parking_controller.py
  -> 192.168.137.100:24580 VM monitor
```

Real motion remains gated by:

```text
--arm
/tmp/parking_armed exists
not --dry-run
not --replanner-dry-run
stable vision
valid STM32 status
valid command bounds
```

## VM Monitor Boundary

Use the monitor-only launcher:

```sh
bash /tmp/vm_start_board_yolo_monitor_only.sh
```

Local source file:

```text
tools/vm_start_board_yolo_monitor_only.sh
```

It starts only:

```text
board_yolo_udp_node
board_yolo_live_view_node.py
```

It intentionally does not start:

```text
slot_geometry_transform_node
parking_target_pose_node
parking_metric_planner_node
parking_planner_node
parking_controller_dry_run_node
STM32 bridge/controller
```

Foxglove topics:

```text
/parking/yolo/parking_detections
/parking/yolo/parking_view
/parking/perception/state
```

## First Safe Bring-Up Sequence

1. Start board YOLO with local controller UDP and VM monitor UDP:

```text
tools/board_start_yolo_closed_loop_monitor.sh
```

Board-side routing:

```text
PARKING_YOLO_UDP_HOST=127.0.0.1
PARKING_YOLO_UDP_PORT=24579
board_yolo_udp_tee.py forwards to 127.0.0.1:24580 and 192.168.137.100:24580
PARKING_YOLO_IMAGE_UDP_HOST=192.168.137.100
PARKING_YOLO_IMAGE_UDP_PORT=24581
```

2. Start VM monitor-only and Foxglove bridge.

3. Run board controller dry-run:

```sh
/usr/local/bin/python3 /opt/parking/autopark/board_parking_controller.py \
  --strategy action_replanner \
  --replanner-dry-run \
  --duration-sec 30 \
  --stable-frames 3 \
  --pixel-vision-lost-stop-sec 0.5 \
  --action-library-json /opt/parking/autopark/parking_action_library.json \
  --response-model-json /opt/parking/autopark/parking_action_response_model.json \
  --success-criteria-json /opt/parking/autopark/parking_success_criteria.json \
  --log-jsonl /tmp/parking_board_closed_loop_dryrun.jsonl
```

4. Only after dry-run looks correct, run one bounded real-motion test with
   `--max-motion-steps 1` and `--max-total-cm` capped.

## Current Caveat

The action-replanner currently blocks real motion for actions that do not have
an exact measured response in the current state bucket. This is intentional.
For immediate board-side real motion, use either:

```text
--strategy primitive_probe
```

for one measured short action, or explicitly add/promote measured response
records before allowing `--strategy action_replanner` to execute automatically.
