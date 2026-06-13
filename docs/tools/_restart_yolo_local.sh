#!/bin/sh
# Repoint the board YOLO UDP to localhost so the on-board controller can receive it.
pkill -f sample_parking_yolo_rtsp 2>/dev/null
sleep 2
cd /opt/sample/parking_yolo_seg_safe || exit 1
: > /tmp/parking_yolo_rtsp_live.log
export LD_LIBRARY_PATH=/opt/lib/npu:/opt/lib
export PARKING_YOLO_RTSP=1
export PARKING_YOLO_UDP_HOST=127.0.0.1
export PARKING_YOLO_UDP_PORT=24580
export PARKING_YOLO_IMAGE_STRIDE=0
export PARKING_YOLO_RUN_FOREVER=1
export PARKING_YOLO_LOWLIGHT_AE=1
export PARKING_YOLO_AE_COMPENSATION=96
export PARKING_YOLO_AE_MIN_EXP_US=0
export PARKING_YOLO_AE_MAX_EXP_US=944036
export PARKING_YOLO_ROTATE180=1
export PARKING_YOLO_SWAP_UV=1
export PARKING_YOLO_CONFIDENCE_THRESHOLD=0.25
nohup ./sample_parking_yolo_rtsp >/tmp/parking_yolo_rtsp_live.log 2>&1 &
echo YOLO_LOCAL_PID $!
