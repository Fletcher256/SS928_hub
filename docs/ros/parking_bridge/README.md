# parking_bridge

ROS2 Humble package for the verified Euler Pi / SS928 sensor link:

- OS08A20 camera: `rtsp://<board_ip>:554/live0`
- SS-LD-AS01 dToF: UDP `2368`, `4873` byte packet, `40x30` depth frame
- Board runtime: `/opt/sample/official_dtof/sample_dtof_rtsp_stable 7 <udp_destination_ip>`
- Optional STM32 USB serial receive-only diagnostics: board UDP forwarder to VM
  UDP `24680`, disabled by default for the perception-only parking bring-up

The implementation follows the official dToF ROS UDP sample structure in
`vendor/dtof_sensor_driver-master/sample/ubuntu_pc/dtof_ros_demo_udp`, but is
adapted to the current single dToF + RTSP camera case7 chain.

## Topics

- `/parking/camera/image_raw` (`sensor_msgs/Image`, `bgr8`; publisher exists, high-bandwidth publishing can be disabled)
- `/parking/camera/image_jpeg` (`sensor_msgs/CompressedImage`)
- `/parking/camera/yolo_input_jpeg` (`sensor_msgs/CompressedImage`, camera frame optimized for YOLO input; separate from Foxglove preview)
- `/parking/dtof/raw_packet` (`std_msgs/UInt8MultiArray`)
- `/parking/dtof/depth` (`sensor_msgs/Image`, `16UC1`, millimetres)
- `/parking/dtof/confidence` (`sensor_msgs/Image`, `mono8`)
- `/parking/dtof/camera_info` (`sensor_msgs/CameraInfo`)
- `/parking/dtof/points` (`sensor_msgs/PointCloud2`, optional; disabled by default for the low-bandwidth Foxglove path)
- `/parking/dtof/depth_color` (`sensor_msgs/CompressedImage`, pseudo-color depth; near is red, far is blue, invalid is black)
- `/parking/dtof/obstacle_view` (`sensor_msgs/CompressedImage`, pseudo-color depth plus five obstacle distance blocks)
- `/parking/dtof/obstacle_blocks` (`std_msgs/String`, JSON five-zone distance summary)
- `/parking/sensors/health` (`std_msgs/String`, JSON)
- `/parking/sensors/sync_pair` (`std_msgs/String`, JSON)
- `/parking/vision/line_debug` (`sensor_msgs/CompressedImage`, pixel-only ROI/color/edge/Hough debug view)
- `/parking/parking_slot_candidates` (`std_msgs/String`, JSON pixel line candidates; uncalibrated)
- `/parking/yolo/parking_view` (`sensor_msgs/CompressedImage`, YOLO parking-slot overlay)
- `/parking/yolo/parking_detections` (`std_msgs/String`, JSON YOLO parking-slot detections and slot candidates)
- `/parking/planner/path` (`std_msgs/String`, JSON dry-run pixel guidance path, YOLO-preferred)
- `/parking/controller/dry_run_cmd` (`std_msgs/String`, JSON simulated steering/speed hints; never sent to STM32)
- `/parking/planner/state` (`std_msgs/String`, JSON planner state)
- `/parking/yolo/person_view` (`sensor_msgs/CompressedImage`, YOLO person detection overlay)
- `/parking/yolo/person_detections` (`std_msgs/String`, JSON COCO `person` detections)
- `/parking/perception/state` (`std_msgs/String`, JSON perception-only state, `motion_enabled=false`)
- `/parking/stm32/raw` (`std_msgs/UInt8MultiArray`, optional)
- `/parking/stm32/metadata` (`std_msgs/String`, JSON, optional)
- `/parking/stm32/health` (`std_msgs/String`, JSON, optional)

## Recording Layout

Default root: `/home/ebaina/parking_sensor_records/session_YYYYmmdd_HHMMSS`

- `session_metadata.json`: run configuration
- `camera_frames/*.jpg`: decoded camera frames
- `camera_frames.jsonl`: camera frame index
- `dtof_packets.bin`: raw 4873-byte dToF UDP packets concatenated
- `dtof_packets.jsonl`: offsets into `dtof_packets.bin`
- `dtof_metadata.jsonl`: parsed dToF header/depth statistics
- `dtof_obstacle_blocks.jsonl`: five-zone obstacle distance summaries
- `dtof_depth_npy/*.npy`: parsed `30x40` depth arrays in millimetres
- `dtof_preview/*.png`: dToF heatmaps
- `preview/*.jpg`: camera + dToF preview panels
- `health.jsonl`: live health snapshots
- `sync_pairs.jsonl`: nearest-neighbour camera/dToF timestamp pairs
- `stm32_session_*/stm32_serial_raw.bin`: optional board-forwarded STM32 serial bytes
- `stm32_session_*/stm32_serial_chunks.jsonl`: optional chunk metadata and VM timestamps
- `stm32_session_*/stm32_health.jsonl`: optional STM32 bridge health snapshots
- `stm32_session_*/stm32_protocol_analysis.json`: optional ASCII/binary protocol-shape
  analysis for the recorded STM32 serial stream

## Run

On the VM:

```bash
source /opt/ros/humble/setup.bash
cd ~/parking_ws
colcon build --packages-select parking_bridge
source install/setup.bash
ros2 launch parking_bridge parking.launch.py enable_stm32:=false
```

For an STM32-only VM receiver test:

```bash
ros2 launch parking_bridge stm32.launch.py stm32_udp_port:=24680
```

The STM32 receiver publishes rolling protocol-shape analysis inside
`/parking/stm32/health`. The same analysis is written to
`stm32_protocol_analysis.json` at the end of a recorded session, and can be run
offline with:

```bash
python3 tools/stm32_serial_analyze.py stm32_serial_raw.bin
```

The board must already be running `sample_dtof_rtsp case7` for live input.

The normal YOLO input topic is `/parking/camera/yolo_input_jpeg`, not the
lower-bandwidth preview topic. It is derived from the camera frame with optional
ROI crop, resize, CLAHE contrast enhancement, gamma correction, and light
sharpening. This keeps Foxglove preview settings separate from the image used
for model inference/training.

YOLO person detection is enabled by default when the ONNX model exists on the VM:

```bash
ros2 launch parking_bridge parking.launch.py \
  publish_yolo_input:=true \
  yolo_input_topic:=/parking/camera/yolo_input_jpeg \
  yolo_camera_input_width:=1280 \
  yolo_camera_clahe_clip_limit:=2.0 \
  yolo_camera_sharpen_amount:=0.35 \
  yolo_model_path:=/home/ebaina/parking_models/yolov8n.onnx \
  yolo_confidence_threshold:=0.50 \
  yolo_process_stride:=3
```

The YOLO node subscribes to `/parking/camera/yolo_input_jpeg`, publishes
`/parking/yolo/person_view` and `/parking/yolo/person_detections`, and keeps
`motion_enabled=false`. It does not expose any actuator interface.

The project parking path should use a parking-slot YOLO model rather than the
older COCO person demo:

```bash
ros2 launch parking_bridge parking.launch.py \
  publish_yolo_input:=true \
  enable_parking_yolo:=true \
  enable_parking_planner:=true \
  parking_yolo_model_path:=/home/ebaina/parking_models/parking_slot.onnx \
  parking_yolo_class_names:=empty,occupied \
  parking_yolo_confidence_threshold:=0.35 \
  parking_yolo_process_stride:=3
```

If the temporary model uses a different class order, pass it explicitly, for
example `parking_yolo_class_names:=car,empty_space,occupied_space`. The planner
prefers `/parking/yolo/parking_detections` and only falls back to
`/parking/parking_slot_candidates` when fresh YOLO slots are unavailable. It
publishes only `/parking/planner/path` and `/parking/controller/dry_run_cmd`;
`commanded_speed_cm_s` remains `0.0`, `actuator_control_allowed=false`, and no
STM32 serial output is opened.

For STM32 serial input, the board must also run the receive-only UDP forwarder:

```bash
python3 /tmp/board_stm32_usb_serial_udp_bridge.py \
  --vm-ip <vm_ip> \
  --udp-port 24680 \
  --baud 9600 \
  --bind-generic
```

That helper reads the board USB serial device and sends UDP datagrams in this
format: `STM32USB1 <json-header>\n<raw-bytes>`. It never writes bytes to the
STM32 serial port.

From the Windows host workspace, the repeatable STM32-only operating path is:

```powershell
.venv\Scripts\python tools\stm32_link_manager.py start --allow-risk
.venv\Scripts\python tools\stm32_link_manager.py health
.venv\Scripts\python tools\stm32_link_manager.py stop --allow-risk
.venv\Scripts\python tools\stm32_link_manager.py latest-analysis
```

The board-side CH340/CH341 driver helper records its state at
`/tmp/stm32_usb_serial_driver_status.json`; the forwarded STM32 metadata and
health include `serial_driver` and `serial_driver_mode`. Current fallback mode
is expected to appear as `generic` / `generic_fallback` until a matching
`4.19.90` `ch341` driver or kernel image is available.

From the Windows host workspace, the current repeatable perception path is the
auto-adapt manager. It discovers the board through COM11/SSH, discovers VM
addresses, chooses direct dToF UDP when possible, and otherwise falls back to a
Windows UDP forwarder:

```powershell
.venv\Scripts\python tools\perception_link_manager.py discover
.venv\Scripts\python tools\perception_link_manager.py adapt --allow-risk
.venv\Scripts\python tools\perception_link_manager.py health
.venv\Scripts\python tools\perception_link_manager.py latest-session
.venv\Scripts\python tools\perception_link_manager.py stop --allow-risk
```

The latest verified perception-only route is an iPhone-hotspot host-forwarded
layout:

```text
board_ip: 172.20.10.2
vm_ssh_ip: 192.168.247.129
host_forward_ip: 172.20.10.10
camera RTSP: rtsp://172.20.10.2:554/live0
dToF UDP: board -> 172.20.10.10:2368 -> 192.168.247.129:2368
Foxglove: ws://192.168.247.129:8765
acceptance report: D:\parking_board_agent\artifacts\perception_link_acceptance\perception_acceptance_20260601_122633.json
```

The latest verified COM11 plus Ethernet route is:

```text
board_ip: 192.168.137.2
vm_ssh_ip: 192.168.247.129
vm_board_subnet_ip: 192.168.137.100
camera RTSP: rtsp://192.168.137.2:554/live0
dToF UDP: board -> 192.168.137.100:2368
route mode: direct_to_vm
```

The latest 10-minute acceptance report is:

```text
D:\parking_board_agent\artifacts\perception_link_acceptance\perception_acceptance_20260601_045710.json
```

The earlier integrated short acceptance that includes the RTSP
quality/latency gate is:

```text
D:\parking_board_agent\artifacts\perception_link_acceptance\perception_acceptance_20260601_043953.json
```

Run the camera receiver comparison from the Windows workspace with:

```powershell
.venv\Scripts\python tools\rtsp_quality_latency_audit.py --seconds 8
```

Latest standalone result:

```text
RTSP_QUALITY_LATENCY_AUDIT PASS
report: D:\parking_board_agent\artifacts\rtsp_quality_latency_audit\rtsp_quality_latency_20260601_043500.json
stream: h264 3840x2160, nominal 30fps
selected mode: ffmpeg_tcp_lowdelay
ffmpeg_tcp_default: first_frame=1.220s, fps=30.125, bad_decode=0, flat=0, grayish=0
ffmpeg_tcp_lowdelay: first_frame=1.164s, fps=27.125, bad_decode=0, flat=0, grayish=0
gstreamer_tcp_lowdelay: rejected candidate
```

The production path currently uses the FFmpeg camera backend. GStreamer remains
an experiment path rather than the selected stable receiver.

Pixel-only visual preprocessing is enabled by default in
`parking.launch.py`. It does not require camera intrinsics or fixed sensor
mounting and does not publish motion commands. It provides ROI, brightness,
white/yellow color mask, Canny edge, and Hough line debug outputs for later
parking-line work.

Camera intrinsic calibration preparation is available from the host and VM:

```powershell
.venv\Scripts\python tools\camera_calibration_tool.py capture --help
.venv\Scripts\python tools\camera_calibration_tool.py calibrate --help
.venv\Scripts\python tools\camera_calibration_tool.py calibrate --pattern charuco --help
```

The same helper is uploaded by the acceptance runner to
`/tmp/camera_calibration_tool.py` on the VM. It supports chessboard and
Charuco capture/calibration output to ROS `camera_info.yaml`; formal
calibration should wait until the camera and dToF are fixed on the vehicle.

The older repeatable pure perception Wi-Fi path is:

```powershell
.venv\Scripts\python tools\wifi_sensor_suite_manager.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 start --allow-risk
.venv\Scripts\python tools\wifi_sensor_suite_manager.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 health
.venv\Scripts\python tools\wifi_sensor_suite_manager.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 stop --allow-risk
.venv\Scripts\python tools\wifi_sensor_suite_manager.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 latest-session
```

The latest verified pure Wi-Fi perception run used
`/home/ebaina/parking_sensor_records/sensor_suite_wifi/run_20260531_020207`.
At the last health check it had recorded `2359` camera frames, `1168` dToF
metadata rows, and `3041` sync pairs. The host UDP forwarder had one active
rule, `172.20.10.8:2368 -> 192.168.247.129:2368`, with `0` errors.

Current dToF health intentionally separates packet transport from depth
validity. The latest run has `transport_ok=True` and `depth_ok=True`: packets
arrive and parse as official `4873` byte / `40x30` frames, and the depth image
is no longer flat. The earlier flat `2mm` symptom is fixed by using the
`sample_dtof_rtsp_keepattr` board binary.

When the board Ethernet cable is unplugged and the board plus Windows host are
on the same Wi-Fi/hotspot, use the adapted host-forwarded path:

```powershell
.venv\Scripts\python tools\wifi_live_preview_control.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 --camera-backend ffmpeg_mjpeg --camera-scale 0.25 --preview-stride 3 start
.venv\Scripts\python tools\wifi_live_preview_control.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 status
.venv\Scripts\python tools\wifi_live_preview_control.py --vm-host 192.168.247.129 --board-host 172.20.10.2 --host-forward-ip 172.20.10.8 stop
```

The low-latency path uses `camera_backend=ffmpeg_mjpeg`, disables raw camera
publishing by default, and keeps dToF point cloud publishing disabled unless
`publish_pointcloud:=true` or `--publish-pointcloud` is explicitly requested.
Foxglove should use `/parking/dtof/depth_color`, `/parking/dtof/obstacle_view`,
and `/parking/dtof/obstacle_blocks` as the normal dToF view. The compressed
camera topic and dToF topics use ROS2 `sensor_data` QoS. Historical rosbag2
smoke test with point cloud publishing enabled:

```text
/home/ebaina/parking_sensor_records/rosbag_smoke/bag_20260531_020306
duration: 5.699 s
messages: 1144
/parking/camera/image_jpeg: 441
/parking/dtof/depth: 102
/parking/dtof/points: 102
/parking/sensors/sync_pair: 493
```

The verified Wi-Fi topology uses board `172.20.10.2`, Windows WLAN IP
`172.20.10.8`, VM `192.168.247.129`, and RTSP
`rtsp://172.20.10.2:554/live0`. See
`docs/perception_phase1_phase2_status.md` for the current evidence, rosbag2
smoke result, and Foxglove/RViz2 notes.

Current perception-only acceptance audit from the Windows workspace:

```powershell
.venv\Scripts\python tools\perception_phase12_status.py
```

Latest verified result:

```text
PERCEPTION_PHASE12_STATUS PASS
report: D:\parking_board_agent\artifacts\perception_phase12_status\status_20260531_024359.json
```

Foxglove bridge is controlled through a no-install wrapper:

```powershell
.venv\Scripts\python tools\foxglove_bridge_control.py --vm-host 192.168.247.129 status
.venv\Scripts\python tools\foxglove_bridge_control.py --vm-host 192.168.247.129 start
.venv\Scripts\python tools\foxglove_bridge_control.py --vm-host 192.168.247.129 stop
```

If `foxglove_bridge` is missing, the wrapper prints
`RECOMMENDED_PACKAGE ros-humble-foxglove-bridge` and does not install anything.
When available, connect Foxglove Studio/browser to `ws://192.168.247.129:8765`.
For dToF viewing, prefer an Image panel on `/parking/dtof/obstacle_view`.
Use `/parking/dtof/depth_color` for the plain pseudo-color depth map and
`/parking/dtof/obstacle_blocks` for the raw five-zone distance JSON.
The bridge wrapper advertises only the low-bandwidth viewing topics by default
so stale Foxglove layouts cannot keep subscribing to point clouds/raw packets.

When the official bridge is missing, the workspace also provides a no-install
Foxglove WebSocket v1 compatible endpoint. It reads the live record directory
written by `parking_bridge` and publishes camera, dToF preview, dToF point
cloud, health, and metadata channels:

```powershell
.venv\Scripts\python tools\foxglove_lite_control.py --vm-host 192.168.247.129 start
.venv\Scripts\python tools\foxglove_lite_control.py --vm-host 192.168.247.129 status
.venv\Scripts\python tools\foxglove_lite_control.py --vm-host 192.168.247.129 stop
```

Connect Foxglove Studio to:

```text
ws://192.168.247.129:8765
```

Probe without Foxglove Studio:

```powershell
.venv\Scripts\python tools\foxglove_lite_probe.py --url ws://192.168.247.129:8765 --listen-sec 12 --require-all
```

Local browser dashboard:

```text
D:\parking_board_agent\tools\foxglove_lite_dashboard.html
```

Render a dashboard PNG from the live WebSocket stream:

```powershell
.venv\Scripts\python tools\foxglove_lite_visual_check.py --host 192.168.247.129
```

Verify rosbag2 playback without interfering with live topics:

```powershell
.venv\Scripts\python tools\vm_ssh_run.py --host 192.168.247.129 --timeout 60 put-text --allow-risk tools\vm_rosbag_replay_check.sh /tmp/vm_rosbag_replay_check.sh
.venv\Scripts\python tools\vm_ssh_run.py --host 192.168.247.129 --timeout 90 run "bash /tmp/vm_rosbag_replay_check.sh"
```

For post-reboot, post-USB-replug, or migration validation:

```powershell
.venv\Scripts\python tools\post_replug_validation.py
```

The latest verified post-replug validation report is
`D:\parking_board_agent\artifacts\post_replug_validation\post_replug_20260530_180414.json`.
It was run after a user-confirmed physical board reboot, recorded `36096` STM32
raw bytes through the VM ROS2 check path, and found no forbidden control
process.

For a read-only audit only:

```powershell
.venv\Scripts\python tools\parking_link_audit.py
```

For the current camera+dToF phase-1/phase-2 objective:

```powershell
.venv\Scripts\python tools\perception_phase12_status.py
```

The older `tools\parking_goal_status.py` covers the previous STM32/CH341 route
and is not the acceptance gate for this perception-only goal. See
`docs/perception_link_runbook.md` for the complete operating procedure.

Do not use chassis, MCU, CAN, serial actuator, motor, steering, brake, or
throttle commands with this package.
