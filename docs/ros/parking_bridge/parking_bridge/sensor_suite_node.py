#!/usr/bin/env python3
"""ROS2 receiver/recorder for OS08A20 RTSP video and SS-LD-AS01 dToF UDP."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import json
import math
import os
import socket
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Header, String, UInt8MultiArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .dtof_packet import HEIGHT, PACKET_SIZE, PIXELS, WIDTH, DtofFrame, parse_packet


POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="depth_mm", offset=12, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 16
OBSTACLE_ZONE_SPECS = [
    ("far_left", "FL", 0.0, 0.2),
    ("left", "L", 0.2, 0.4),
    ("center", "C", 0.4, 0.6),
    ("right", "R", 0.6, 0.8),
    ("far_right", "FR", 0.8, 1.0),
]


def now_ns() -> int:
    return time.time_ns()


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def json_line(handle, payload: dict[str, Any]) -> None:
    handle.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
    handle.flush()


def image_msg(frame: np.ndarray, stamp, frame_id: str, encoding: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    if frame.ndim == 2:
        channels = 1
    else:
        channels = int(frame.shape[2])
    msg.step = int(frame.shape[1] * frame.dtype.itemsize * channels)
    msg.data = frame.tobytes()
    return msg


def compressed_jpeg_msg(frame: np.ndarray, stamp, frame_id: str, quality: int) -> CompressedImage | None:
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, int(quality)))],
    )
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


def ffmpeg_mjpeg_qscale(jpeg_quality: int) -> int:
    quality = max(1, min(100, int(jpeg_quality)))
    return max(2, min(31, round(31 - (quality * 29 / 100))))


class SensorSuiteNode(Node):
    def __init__(self) -> None:
        super().__init__("parking_sensor_suite")

        self.declare_parameter("rtsp_url", "rtsp://192.168.137.2:554/live0")
        self.declare_parameter("dtof_bind_ip", "0.0.0.0")
        self.declare_parameter("dtof_port", 2368)
        self.declare_parameter("enable_dtof", True)
        self.declare_parameter("record_dir", str(Path.home() / "parking_sensor_records"))
        self.declare_parameter("enable_recording", True)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("visualize_window", False)
        self.declare_parameter("camera_frame_id", "os08a20_camera")
        self.declare_parameter("dtof_frame_id", "ss_ld_as01_dtof")
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("dtof_to_camera_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("dtof_to_camera_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_backend", "ffmpeg_mjpeg")
        self.declare_parameter("camera_ffmpeg_low_delay", False)
        self.declare_parameter("camera_scale", 1.0)
        self.declare_parameter("camera_rotate", "none")
        self.declare_parameter("camera_jpeg_quality", 85)
        self.declare_parameter("publish_camera_raw", True)
        self.declare_parameter("camera_publish_stride", 1)
        self.declare_parameter("publish_yolo_input", False)
        self.declare_parameter("yolo_input_topic", "/parking/camera/yolo_input_jpeg")
        self.declare_parameter("yolo_input_publish_stride", 1)
        self.declare_parameter("yolo_input_width", 1280)
        self.declare_parameter("yolo_roi_top_fraction", 0.0)
        self.declare_parameter("yolo_roi_bottom_fraction", 1.0)
        self.declare_parameter("yolo_roi_left_fraction", 0.0)
        self.declare_parameter("yolo_roi_right_fraction", 1.0)
        self.declare_parameter("yolo_clahe_clip_limit", 2.0)
        self.declare_parameter("yolo_clahe_tile_grid", 8)
        self.declare_parameter("yolo_sharpen_amount", 0.35)
        self.declare_parameter("yolo_gamma", 1.0)
        self.declare_parameter("yolo_jpeg_quality", 96)
        self.declare_parameter("camera_drop_flat_frames", False)
        self.declare_parameter("camera_flat_luma_std_threshold", 6.0)
        self.declare_parameter("camera_flat_color_delta_threshold", 4.0)
        self.declare_parameter("camera_flat_reconnect_threshold", 12)
        self.declare_parameter("camera_record_stride", 1)
        self.declare_parameter("dtof_depth_record_stride", 1)
        self.declare_parameter("preview_stride", 15)
        self.declare_parameter("sync_slop_ms", 700.0)
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("min_valid_depth_mm", 20)
        self.declare_parameter("min_valid_depth_pixels", 20)
        self.declare_parameter("max_depth_mm", 10000)
        self.declare_parameter("publish_pointcloud", False)
        self.declare_parameter("dtof_visual_publish_stride", 2)
        self.declare_parameter("dtof_visual_jpeg_quality", 80)
        self.declare_parameter("dtof_visual_width", 480)
        self.declare_parameter("dtof_visual_height", 360)
        self.declare_parameter("dtof_visual_min_mm", 50)
        self.declare_parameter("dtof_visual_max_mm", 4000)
        self.declare_parameter("dtof_obstacle_near_mm", 500)
        self.declare_parameter("dtof_obstacle_warn_mm", 1200)
        self.declare_parameter("dtof_obstacle_noise_floor_mm", 250)
        self.declare_parameter("dtof_obstacle_distance_percentile", 25.0)
        self.declare_parameter("dtof_obstacle_min_support_pixels", 16)
        self.declare_parameter("dtof_obstacle_min_support_ratio", 0.20)

        self.rtsp_url = str(self.get_parameter("rtsp_url").value)
        self.dtof_bind_ip = str(self.get_parameter("dtof_bind_ip").value)
        self.dtof_port = int(self.get_parameter("dtof_port").value)
        self.enable_dtof = as_bool(self.get_parameter("enable_dtof").value)
        self.record_root = Path(str(self.get_parameter("record_dir").value)).expanduser()
        self.enable_recording = as_bool(self.get_parameter("enable_recording").value)
        self.enable_visualization = as_bool(self.get_parameter("enable_visualization").value)
        self.visualize_window = as_bool(self.get_parameter("visualize_window").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.dtof_frame_id = str(self.get_parameter("dtof_frame_id").value)
        self.publish_static_tf = as_bool(self.get_parameter("publish_static_tf").value)
        self.dtof_to_camera_xyz = self._float_list_param("dtof_to_camera_xyz", 3)
        self.dtof_to_camera_rpy = self._float_list_param("dtof_to_camera_rpy", 3)
        self.camera_backend = str(self.get_parameter("camera_backend").value).strip().lower()
        self.camera_ffmpeg_low_delay = as_bool(self.get_parameter("camera_ffmpeg_low_delay").value)
        self.camera_scale = float(self.get_parameter("camera_scale").value)
        self.camera_rotate = str(self.get_parameter("camera_rotate").value).strip().lower()
        self.camera_jpeg_quality = int(self.get_parameter("camera_jpeg_quality").value)
        self.publish_camera_raw = as_bool(self.get_parameter("publish_camera_raw").value)
        self.camera_publish_stride = max(1, int(self.get_parameter("camera_publish_stride").value))
        self.publish_yolo_input = as_bool(self.get_parameter("publish_yolo_input").value)
        self.yolo_input_topic = str(self.get_parameter("yolo_input_topic").value)
        self.yolo_input_publish_stride = max(1, int(self.get_parameter("yolo_input_publish_stride").value))
        self.yolo_input_width = max(0, int(self.get_parameter("yolo_input_width").value))
        self.yolo_roi_top_fraction = min(0.99, max(0.0, float(self.get_parameter("yolo_roi_top_fraction").value)))
        self.yolo_roi_bottom_fraction = min(1.0, max(0.01, float(self.get_parameter("yolo_roi_bottom_fraction").value)))
        self.yolo_roi_left_fraction = min(0.99, max(0.0, float(self.get_parameter("yolo_roi_left_fraction").value)))
        self.yolo_roi_right_fraction = min(1.0, max(0.01, float(self.get_parameter("yolo_roi_right_fraction").value)))
        if self.yolo_roi_bottom_fraction <= self.yolo_roi_top_fraction:
            self.yolo_roi_top_fraction = 0.0
            self.yolo_roi_bottom_fraction = 1.0
        if self.yolo_roi_right_fraction <= self.yolo_roi_left_fraction:
            self.yolo_roi_left_fraction = 0.0
            self.yolo_roi_right_fraction = 1.0
        self.yolo_clahe_clip_limit = max(0.0, float(self.get_parameter("yolo_clahe_clip_limit").value))
        self.yolo_clahe_tile_grid = max(2, int(self.get_parameter("yolo_clahe_tile_grid").value))
        self.yolo_sharpen_amount = max(0.0, float(self.get_parameter("yolo_sharpen_amount").value))
        self.yolo_gamma = max(0.1, min(5.0, float(self.get_parameter("yolo_gamma").value)))
        self.yolo_jpeg_quality = int(self.get_parameter("yolo_jpeg_quality").value)
        self.camera_drop_flat_frames = as_bool(self.get_parameter("camera_drop_flat_frames").value)
        self.camera_flat_luma_std_threshold = float(self.get_parameter("camera_flat_luma_std_threshold").value)
        self.camera_flat_color_delta_threshold = float(self.get_parameter("camera_flat_color_delta_threshold").value)
        self.camera_flat_reconnect_threshold = max(1, int(self.get_parameter("camera_flat_reconnect_threshold").value))
        self.camera_record_stride = max(1, int(self.get_parameter("camera_record_stride").value))
        self.dtof_depth_record_stride = max(1, int(self.get_parameter("dtof_depth_record_stride").value))
        self.preview_stride = max(1, int(self.get_parameter("preview_stride").value))
        self.sync_slop_ns = int(float(self.get_parameter("sync_slop_ms").value) * 1_000_000.0)
        self.min_valid_depth_mm = int(self.get_parameter("min_valid_depth_mm").value)
        self.min_valid_depth_pixels = int(self.get_parameter("min_valid_depth_pixels").value)
        self.max_depth_mm = int(self.get_parameter("max_depth_mm").value)
        self.publish_pointcloud = as_bool(self.get_parameter("publish_pointcloud").value)
        self.dtof_visual_publish_stride = max(1, int(self.get_parameter("dtof_visual_publish_stride").value))
        self.dtof_visual_jpeg_quality = int(self.get_parameter("dtof_visual_jpeg_quality").value)
        self.dtof_visual_width = max(WIDTH, int(self.get_parameter("dtof_visual_width").value))
        self.dtof_visual_height = max(HEIGHT, int(self.get_parameter("dtof_visual_height").value))
        self.dtof_visual_min_mm = max(self.min_valid_depth_mm, int(self.get_parameter("dtof_visual_min_mm").value))
        self.dtof_visual_max_mm = max(
            self.dtof_visual_min_mm + 1,
            int(self.get_parameter("dtof_visual_max_mm").value),
        )
        self.dtof_obstacle_near_mm = max(self.min_valid_depth_mm, int(self.get_parameter("dtof_obstacle_near_mm").value))
        self.dtof_obstacle_warn_mm = max(
            self.dtof_obstacle_near_mm + 1,
            int(self.get_parameter("dtof_obstacle_warn_mm").value),
        )
        self.dtof_obstacle_noise_floor_mm = max(
            self.min_valid_depth_mm,
            int(self.get_parameter("dtof_obstacle_noise_floor_mm").value),
        )
        self.dtof_obstacle_distance_percentile = max(
            1.0,
            min(50.0, float(self.get_parameter("dtof_obstacle_distance_percentile").value)),
        )
        self.dtof_obstacle_min_support_pixels = max(
            1,
            int(self.get_parameter("dtof_obstacle_min_support_pixels").value),
        )
        self.dtof_obstacle_min_support_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter("dtof_obstacle_min_support_ratio").value)),
        )

        self.camera_pub = self.create_publisher(Image, "/parking/camera/image_raw", qos_profile_sensor_data)
        self.camera_jpeg_pub = self.create_publisher(CompressedImage, "/parking/camera/image_jpeg", qos_profile_sensor_data)
        self.yolo_input_pub = (
            self.create_publisher(CompressedImage, self.yolo_input_topic, qos_profile_sensor_data)
            if self.publish_yolo_input
            else None
        )
        self.dtof_raw_pub = self.create_publisher(UInt8MultiArray, "/parking/dtof/raw_packet", qos_profile_sensor_data)
        self.dtof_depth_pub = self.create_publisher(Image, "/parking/dtof/depth", qos_profile_sensor_data)
        self.dtof_conf_pub = self.create_publisher(Image, "/parking/dtof/confidence", qos_profile_sensor_data)
        self.dtof_info_pub = self.create_publisher(CameraInfo, "/parking/dtof/camera_info", qos_profile_sensor_data)
        self.dtof_points_pub = self.create_publisher(PointCloud2, "/parking/dtof/points", qos_profile_sensor_data)
        self.dtof_depth_color_pub = self.create_publisher(
            CompressedImage,
            "/parking/dtof/depth_color",
            qos_profile_sensor_data,
        )
        self.dtof_obstacle_view_pub = self.create_publisher(
            CompressedImage,
            "/parking/dtof/obstacle_view",
            qos_profile_sensor_data,
        )
        self.dtof_obstacle_blocks_pub = self.create_publisher(String, "/parking/dtof/obstacle_blocks", 10)
        self.health_pub = self.create_publisher(String, "/parking/sensors/health", 10)
        self.sync_pub = self.create_publisher(String, "/parking/sensors/sync_pair", 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_static_tf:
            self._publish_static_tf()

        self.start_ns = now_ns()
        self.running = True
        self.lock = threading.Lock()

        self.camera_count = 0
        self.camera_flat_drop_count = 0
        self.dtof_count = 0
        self.bad_dtof_count = 0
        self.camera_reconnects = 0
        self.last_camera_ns: int | None = None
        self.last_dtof_ns: int | None = None
        self.last_camera_seq = 0
        self.last_dtof_seq = 0
        self.last_camera_shape: tuple[int, int] | None = None
        self.last_camera_flat: dict[str, Any] | None = None
        self.last_camera_frame: np.ndarray | None = None
        self.last_dtof_depth: np.ndarray | None = None
        self.last_dtof_meta: dict[str, Any] | None = None
        self.last_sync_camera_seq = -1
        self.last_sync_dtof_seq = -1
        self.camera_times: deque[int] = deque(maxlen=120)
        self.dtof_times: deque[int] = deque(maxlen=120)

        self.session_dir: Path | None = None
        self.dtof_bin = None
        self.dtof_index = None
        self.dtof_meta = None
        self.dtof_obstacle_index = None
        self.camera_index = None
        self.health_index = None
        self.sync_index = None
        self.dtof_bin_offset = 0
        self._setup_recording()

        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.dtof_thread = None
        if self.enable_dtof:
            self.dtof_thread = threading.Thread(target=self._dtof_loop, daemon=True)
        self.camera_thread.start()
        if self.dtof_thread is not None:
            self.dtof_thread.start()

        period = float(self.get_parameter("status_period_sec").value)
        self.status_timer = self.create_timer(period, self._publish_health)

        self.get_logger().info(
            "parking_sensor_suite started: "
            f"rtsp={self.rtsp_url}, dtof_enabled={self.enable_dtof}, dtof_udp={self.dtof_bind_ip}:{self.dtof_port}, "
            f"camera_backend={self.camera_backend}, publish_camera_raw={self.publish_camera_raw}, "
            f"recording={self.enable_recording}, session={self.session_dir}"
        )

    def destroy_node(self) -> bool:
        self.running = False
        self.camera_thread.join(timeout=3.0)
        if self.dtof_thread is not None:
            self.dtof_thread.join(timeout=3.0)
        self._close_recording()
        if self.visualize_window:
            cv2.destroyAllWindows()
        return super().destroy_node()

    def _setup_recording(self) -> None:
        if not self.enable_recording:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.record_root / f"session_{stamp}"
        ensure_dir(self.session_dir)
        ensure_dir(self.session_dir / "camera_frames")
        ensure_dir(self.session_dir / "dtof_depth_npy")
        ensure_dir(self.session_dir / "dtof_preview")
        ensure_dir(self.session_dir / "preview")

        self.dtof_bin = (self.session_dir / "dtof_packets.bin").open("ab")
        self.dtof_index = (self.session_dir / "dtof_packets.jsonl").open("a", encoding="utf-8")
        self.dtof_meta = (self.session_dir / "dtof_metadata.jsonl").open("a", encoding="utf-8")
        self.dtof_obstacle_index = (self.session_dir / "dtof_obstacle_blocks.jsonl").open("a", encoding="utf-8")
        self.camera_index = (self.session_dir / "camera_frames.jsonl").open("a", encoding="utf-8")
        self.health_index = (self.session_dir / "health.jsonl").open("a", encoding="utf-8")
        self.sync_index = (self.session_dir / "sync_pairs.jsonl").open("a", encoding="utf-8")

        metadata = {
            "created_time_ns": self.start_ns,
            "rtsp_url": self.rtsp_url,
            "enable_dtof": self.enable_dtof,
            "dtof_port": self.dtof_port,
            "dtof_packet_size": PACKET_SIZE,
            "dtof_width": WIDTH,
            "dtof_height": HEIGHT,
            "min_valid_depth_mm": self.min_valid_depth_mm,
            "min_valid_depth_pixels": self.min_valid_depth_pixels,
            "camera_backend": self.camera_backend,
            "camera_ffmpeg_low_delay": self.camera_ffmpeg_low_delay,
            "publish_camera_raw": self.publish_camera_raw,
            "camera_publish_stride": self.camera_publish_stride,
            "camera_rotate": self.camera_rotate,
            "camera_drop_flat_frames": self.camera_drop_flat_frames,
            "camera_flat_luma_std_threshold": self.camera_flat_luma_std_threshold,
            "camera_flat_color_delta_threshold": self.camera_flat_color_delta_threshold,
            "camera_flat_reconnect_threshold": self.camera_flat_reconnect_threshold,
            "camera_jpeg_quality": self.camera_jpeg_quality,
            "camera_scale": self.camera_scale,
            "publish_yolo_input": self.publish_yolo_input,
            "yolo_input_topic": self.yolo_input_topic,
            "yolo_input_publish_stride": self.yolo_input_publish_stride,
            "yolo_input_width": self.yolo_input_width,
            "yolo_roi": {
                "top_fraction": self.yolo_roi_top_fraction,
                "bottom_fraction": self.yolo_roi_bottom_fraction,
                "left_fraction": self.yolo_roi_left_fraction,
                "right_fraction": self.yolo_roi_right_fraction,
            },
            "yolo_clahe_clip_limit": self.yolo_clahe_clip_limit,
            "yolo_clahe_tile_grid": self.yolo_clahe_tile_grid,
            "yolo_sharpen_amount": self.yolo_sharpen_amount,
            "yolo_gamma": self.yolo_gamma,
            "yolo_jpeg_quality": self.yolo_jpeg_quality,
            "camera_frame_id": self.camera_frame_id,
            "dtof_frame_id": self.dtof_frame_id,
            "dtof_to_camera_xyz": self.dtof_to_camera_xyz,
            "dtof_to_camera_rpy": self.dtof_to_camera_rpy,
            "dtof_visual_publish_stride": self.dtof_visual_publish_stride,
            "dtof_visual_jpeg_quality": self.dtof_visual_jpeg_quality,
            "dtof_visual_size": [self.dtof_visual_width, self.dtof_visual_height],
            "dtof_visual_min_mm": self.dtof_visual_min_mm,
            "dtof_visual_max_mm": self.dtof_visual_max_mm,
            "dtof_obstacle_near_mm": self.dtof_obstacle_near_mm,
            "dtof_obstacle_warn_mm": self.dtof_obstacle_warn_mm,
            "dtof_obstacle_noise_floor_mm": self.dtof_obstacle_noise_floor_mm,
            "dtof_obstacle_distance_percentile": self.dtof_obstacle_distance_percentile,
            "dtof_obstacle_min_support_pixels": self.dtof_obstacle_min_support_pixels,
            "dtof_obstacle_min_support_ratio": self.dtof_obstacle_min_support_ratio,
                "official_reference": {
                    "dtof_ros_demo": "vendor/dtof_sensor_driver-master/sample/ubuntu_pc/dtof_ros_demo_udp",
                    "board_sample": (
                        "/opt/sample/camera_only/sample_camera_rtsp case8"
                        if not self.enable_dtof
                        else "/opt/sample/official_dtof/sample_dtof_rtsp case7"
                    ),
                },
            }
        (self.session_dir / "session_metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _close_recording(self) -> None:
        for handle in [
            self.dtof_bin,
            self.dtof_index,
            self.dtof_meta,
            self.dtof_obstacle_index,
            self.camera_index,
            self.health_index,
            self.sync_index,
        ]:
            if handle:
                handle.close()

    def _camera_loop(self) -> None:
        if self.camera_backend == "ffmpeg_mjpeg":
            self._camera_loop_ffmpeg_mjpeg()
            return
        self._camera_loop_opencv()

    def _camera_loop_opencv(self) -> None:
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|stimeout;2000000|max_delay;500000",
        )
        while self.running:
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                with self.lock:
                    self.camera_reconnects += 1
                self.get_logger().warn(f"camera open failed: {self.rtsp_url}")
                time.sleep(2.0)
                continue

            self.get_logger().info(f"camera RTSP connected: {self.rtsp_url}")
            flat_streak = 0
            while self.running:
                ok, frame = cap.read()
                recv_ns = now_ns()
                if not ok or frame is None:
                    self.get_logger().warn("camera frame read failed, reconnecting")
                    break
                if self.camera_scale != 1.0:
                    h, w = frame.shape[:2]
                    frame = cv2.resize(
                        frame,
                        (max(1, int(w * self.camera_scale)), max(1, int(h * self.camera_scale))),
                        interpolation=cv2.INTER_AREA,
                    )
                frame = self._rotate_camera_frame(frame)
                try:
                    accepted = self._handle_camera_frame(frame, recv_ns)
                    if accepted:
                        flat_streak = 0
                    else:
                        flat_streak += 1
                        if flat_streak >= self.camera_flat_reconnect_threshold:
                            self.get_logger().warn("camera flat-frame streak hit threshold, reconnecting")
                            break
                except Exception:
                    if not self.running or not rclpy.ok():
                        break
                    raise

            cap.release()
            with self.lock:
                self.camera_reconnects += 1
            time.sleep(1.0)

    def _camera_loop_ffmpeg_mjpeg(self) -> None:
        vf_filters = []
        if self.camera_scale > 0.0 and self.camera_scale != 1.0:
            vf_filters.append(f"scale=trunc(iw*{self.camera_scale}/2)*2:trunc(ih*{self.camera_scale}/2)*2")
        vf_filter_args = ["-vf", ",".join(vf_filters)] if vf_filters else []
        latency_flags = []
        if self.camera_ffmpeg_low_delay:
            latency_flags = [
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
            ]

        while self.running:
            cmd = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "warning",
                "-rtsp_transport",
                "tcp",
                *latency_flags,
                "-i",
                self.rtsp_url,
                "-an",
                *vf_filter_args,
                "-q:v",
                str(ffmpeg_mjpeg_qscale(self.camera_jpeg_quality)),
                "-vsync",
                "0",
                "-f",
                "image2pipe",
                "-vcodec",
                "mjpeg",
                "pipe:1",
            ]
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0,
                )
            except OSError as exc:
                with self.lock:
                    self.camera_reconnects += 1
                self.get_logger().warn(f"camera ffmpeg start failed: {exc}")
                time.sleep(2.0)
                continue

            self.get_logger().info("camera RTSP connected through ffmpeg_mjpeg")
            buffer = bytearray()
            flat_streak = 0
            reconnect_requested = False
            try:
                assert proc.stdout is not None
                while self.running:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        if proc.poll() is not None:
                            break
                        time.sleep(0.01)
                        continue
                    buffer.extend(chunk)
                    while True:
                        start = buffer.find(b"\xff\xd8")
                        if start < 0:
                            if len(buffer) > 1_000_000:
                                del buffer[:-2]
                            break
                        end = buffer.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            if start:
                                del buffer[:start]
                            break
                        jpeg_bytes = bytes(buffer[start : end + 2])
                        del buffer[: end + 2]
                        recv_ns = now_ns()
                        frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is None:
                            continue
                        try:
                            accepted = self._handle_camera_frame(frame, recv_ns, encoded_jpeg=jpeg_bytes)
                            if accepted:
                                flat_streak = 0
                            else:
                                flat_streak += 1
                                if flat_streak >= self.camera_flat_reconnect_threshold:
                                    self.get_logger().warn("camera flat-frame streak hit threshold, reconnecting")
                                    reconnect_requested = True
                                    break
                        except Exception:
                            if not self.running or not rclpy.ok():
                                break
                            raise
                    if reconnect_requested:
                        break
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                with self.lock:
                    self.camera_reconnects += 1
                time.sleep(1.0)

    def _rotate_camera_frame(self, frame: np.ndarray) -> np.ndarray:
        if self.camera_rotate in {"rotate180", "180"}:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if self.camera_rotate in {"90cw", "cw", "clockwise"}:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if self.camera_rotate in {"90ccw", "ccw", "counterclockwise"}:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def _handle_camera_frame(self, frame: np.ndarray, recv_ns: int, encoded_jpeg: bytes | None = None) -> bool:
        if not self.running or not rclpy.ok():
            return False
        flat, flat_meta = self._flat_camera_frame(frame)
        if self.camera_drop_flat_frames and flat:
            with self.lock:
                self.camera_flat_drop_count += 1
                self.last_camera_flat = {
                    "recv_time_ns": recv_ns,
                    "dropped": True,
                    **flat_meta,
                }
            self.get_logger().warn(
                "dropping flat camera frame "
                f"luma_std={flat_meta['luma_std']:.2f} color_delta={flat_meta['color_delta']:.2f}",
                throttle_duration_sec=2.0,
            )
            return False
        with self.lock:
            self.camera_count += 1
            seq = self.camera_count
            self.last_camera_seq = seq
            self.last_camera_ns = recv_ns
            self.last_camera_shape = (int(frame.shape[1]), int(frame.shape[0]))
            self.last_camera_frame = frame.copy()
            self.camera_times.append(recv_ns)

        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame_id
        publish_this_frame = seq % self.camera_publish_stride == 0
        if publish_this_frame and self.publish_camera_raw:
            msg = image_msg(frame, stamp, self.camera_frame_id, "bgr8")
            self.camera_pub.publish(msg)
            header = msg.header

        ok = False
        jpeg_bytes = b""
        if encoded_jpeg is None:
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.camera_jpeg_quality))],
            )
            jpeg_bytes = encoded.tobytes() if ok else b""
        else:
            ok = True
            jpeg_bytes = encoded_jpeg
        if publish_this_frame and ok:
            comp = CompressedImage()
            comp.header = header
            comp.format = "jpeg"
            comp.data = jpeg_bytes
            self.camera_jpeg_pub.publish(comp)

        if self.publish_yolo_input and seq % self.yolo_input_publish_stride == 0:
            self._publish_yolo_input(frame, stamp, seq)

        if self.enable_recording and self.session_dir and seq % self.camera_record_stride == 0 and ok:
            rel = Path("camera_frames") / f"camera_{seq:08d}.jpg"
            path = self.session_dir / rel
            path.write_bytes(jpeg_bytes)
            json_line(
                self.camera_index,
                {
                    "seq": seq,
                    "recv_time_ns": recv_ns,
                    "ros_stamp_sec": stamp.sec,
                    "ros_stamp_nanosec": stamp.nanosec,
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                    "encoding": "jpeg",
                    "path": str(rel).replace("\\", "/"),
                },
            )

        if self.enable_dtof:
            self._try_sync("camera")
        return True

    def _publish_yolo_input(self, frame: np.ndarray, stamp, seq: int) -> None:
        if self.yolo_input_pub is None or self.yolo_input_pub.get_subscription_count() <= 0:
            return
        yolo_frame = self._make_yolo_input_frame(frame)
        msg = compressed_jpeg_msg(yolo_frame, stamp, self.camera_frame_id, self.yolo_jpeg_quality)
        if msg is None:
            return
        roi = {
            "top": round(self.yolo_roi_top_fraction, 4),
            "bottom": round(self.yolo_roi_bottom_fraction, 4),
            "left": round(self.yolo_roi_left_fraction, 4),
            "right": round(self.yolo_roi_right_fraction, 4),
        }
        msg.format = (
            "jpeg; "
            f"source=/parking/camera/image_jpeg; seq={seq}; "
            f"optimized_for=yolo; roi={json.dumps(roi, separators=(',', ':'))}; "
            f"clahe={self.yolo_clahe_clip_limit:.2f}; sharpen={self.yolo_sharpen_amount:.2f}; "
            f"gamma={self.yolo_gamma:.3f}"
        )
        self.yolo_input_pub.publish(msg)

    def _make_yolo_input_frame(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        y0 = int(round(h * self.yolo_roi_top_fraction))
        y1 = int(round(h * self.yolo_roi_bottom_fraction))
        x0 = int(round(w * self.yolo_roi_left_fraction))
        x1 = int(round(w * self.yolo_roi_right_fraction))
        y0 = max(0, min(h - 1, y0))
        y1 = max(y0 + 1, min(h, y1))
        x0 = max(0, min(w - 1, x0))
        x1 = max(x0 + 1, min(w, x1))
        work = frame[y0:y1, x0:x1].copy()

        if self.yolo_input_width > 0 and work.shape[1] != self.yolo_input_width:
            scale = float(self.yolo_input_width) / float(work.shape[1])
            target_h = max(1, int(round(work.shape[0] * scale)))
            interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
            work = cv2.resize(work, (self.yolo_input_width, target_h), interpolation=interpolation)

        if self.yolo_clahe_clip_limit > 0.0:
            lab = cv2.cvtColor(work, cv2.COLOR_BGR2LAB)
            l_chan, a_chan, b_chan = cv2.split(lab)
            clahe = cv2.createCLAHE(
                clipLimit=self.yolo_clahe_clip_limit,
                tileGridSize=(self.yolo_clahe_tile_grid, self.yolo_clahe_tile_grid),
            )
            l_chan = clahe.apply(l_chan)
            work = cv2.cvtColor(cv2.merge((l_chan, a_chan, b_chan)), cv2.COLOR_LAB2BGR)

        if abs(self.yolo_gamma - 1.0) > 0.01:
            inv_gamma = 1.0 / self.yolo_gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255.0 for i in range(256)], dtype=np.uint8)
            work = cv2.LUT(work, table)

        if self.yolo_sharpen_amount > 0.0:
            blurred = cv2.GaussianBlur(work, (0, 0), sigmaX=1.0, sigmaY=1.0)
            work = cv2.addWeighted(work, 1.0 + self.yolo_sharpen_amount, blurred, -self.yolo_sharpen_amount, 0.0)

        return work

    def _flat_camera_frame(self, frame: np.ndarray) -> tuple[bool, dict[str, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        luma_mean = float(gray.mean())
        luma_std = float(gray.std())
        b = frame[:, :, 0].astype(np.int16)
        g = frame[:, :, 1].astype(np.int16)
        r = frame[:, :, 2].astype(np.int16)
        color_delta = float(np.mean(np.abs(b - g) + np.abs(g - r) + np.abs(b - r)) / 3.0)
        flat = (
            luma_std < self.camera_flat_luma_std_threshold
            and color_delta < self.camera_flat_color_delta_threshold
        )
        return flat, {
            "luma_mean": luma_mean,
            "luma_std": luma_std,
            "color_delta": color_delta,
        }

    def _float_list_param(self, name: str, expected_len: int) -> list[float]:
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)):
            return [0.0] * expected_len
        result = [float(item) for item in value[:expected_len]]
        while len(result) < expected_len:
            result.append(0.0)
        return result

    def _publish_static_tf(self) -> None:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.camera_frame_id
        tf.child_frame_id = self.dtof_frame_id
        tf.transform.translation.x = self.dtof_to_camera_xyz[0]
        tf.transform.translation.y = self.dtof_to_camera_xyz[1]
        tf.transform.translation.z = self.dtof_to_camera_xyz[2]
        qx, qy, qz, qw = self._quat_from_rpy(*self.dtof_to_camera_rpy)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _dtof_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.dtof_bind_ip, self.dtof_port))
        sock.settimeout(1.0)
        self.get_logger().info(f"dToF UDP listening on {self.dtof_bind_ip}:{self.dtof_port}")
        while self.running:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            recv_ns = now_ns()
            try:
                frame = parse_packet(data, recv_ns, addr)
            except Exception as exc:
                with self.lock:
                    self.bad_dtof_count += 1
                self.get_logger().warn(f"bad dToF packet: {exc}", throttle_duration_sec=5.0)
                continue
            try:
                self._handle_dtof_frame(frame)
            except Exception:
                if not self.running or not rclpy.ok():
                    break
                raise
        sock.close()

    def _handle_dtof_frame(self, frame: DtofFrame) -> None:
        if not self.running or not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()

        raw_msg = UInt8MultiArray()
        raw_msg.data = list(frame.raw)
        self.dtof_raw_pub.publish(raw_msg)

        depth_u16 = np.clip(frame.depth_mm, 0, 65535).astype(np.uint16)
        conf_u8 = frame.confidence.astype(np.uint8)
        self.dtof_depth_pub.publish(image_msg(depth_u16, stamp, self.dtof_frame_id, "16UC1"))
        self.dtof_conf_pub.publish(image_msg(conf_u8, stamp, self.dtof_frame_id, "mono8"))
        self.dtof_info_pub.publish(self._camera_info(frame, stamp))

        if self.publish_pointcloud:
            points = self._point_cloud(frame)
            if points is not None:
                self.dtof_points_pub.publish(points)

        meta = frame.metadata()
        valid_depth_mask = (frame.depth_mm >= self.min_valid_depth_mm) & (frame.depth_mm <= self.max_depth_mm)
        obstacle_payload = self._obstacle_summary(frame.depth_mm, valid_depth_mask, frame.recv_time_ns)
        meta["min_valid_depth_mm"] = self.min_valid_depth_mm
        meta["depth_valid_pixels"] = int(valid_depth_mask.sum())
        meta["depth_flat"] = bool(meta.get("depth_unique_count", 0) <= 1)
        meta["depth_ok"] = bool(meta["depth_valid_pixels"] >= self.min_valid_depth_pixels and not meta["depth_flat"])
        meta["ros_stamp_sec"] = stamp.sec
        meta["ros_stamp_nanosec"] = stamp.nanosec

        with self.lock:
            self.dtof_count += 1
            seq = self.dtof_count
            self.last_dtof_seq = seq
            self.last_dtof_ns = frame.recv_time_ns
            self.last_dtof_depth = frame.depth_mm.copy()
            self.last_dtof_meta = meta
            self.dtof_times.append(frame.recv_time_ns)

        obstacle_payload["seq"] = seq
        obstacle_payload["ros_stamp_sec"] = stamp.sec
        obstacle_payload["ros_stamp_nanosec"] = stamp.nanosec
        meta["obstacle_nearest_mm"] = obstacle_payload["nearest_mm"]
        meta["obstacle_nearest_zone"] = obstacle_payload["nearest_zone"]
        meta["obstacle_state"] = obstacle_payload["state"]
        self._publish_obstacle_blocks(obstacle_payload)
        if seq % self.dtof_visual_publish_stride == 0:
            self._publish_dtof_visuals(frame.depth_mm, obstacle_payload, stamp)

        if self.enable_recording and self.session_dir:
            offset = self.dtof_bin_offset
            self.dtof_bin.write(frame.raw)
            self.dtof_bin.flush()
            self.dtof_bin_offset += len(frame.raw)
            index_payload = {
                "seq": seq,
                "recv_time_ns": frame.recv_time_ns,
                "offset": offset,
                "length": len(frame.raw),
                "source_ip": frame.source_ip,
                "source_port": frame.source_port,
            }
            json_line(self.dtof_index, index_payload)
            meta["seq"] = seq
            json_line(self.dtof_meta, meta)
            if self.dtof_obstacle_index:
                json_line(self.dtof_obstacle_index, obstacle_payload)
            if seq % self.dtof_depth_record_stride == 0:
                rel = Path("dtof_depth_npy") / f"dtof_depth_{seq:08d}.npy"
                np.save(self.session_dir / rel, frame.depth_mm)
                if self.enable_visualization:
                    self._save_dtof_preview(frame.depth_mm, seq)

        if self.enable_visualization and seq % self.preview_stride == 0:
            self._save_combined_preview(seq)

        self._try_sync("dtof")

    def _camera_info(self, frame: DtofFrame, stamp) -> CameraInfo:
        fx, fy, cx, cy = self._dtof_intrinsics(frame)
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.dtof_frame_id
        msg.width = WIDTH
        msg.height = HEIGHT
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _dtof_intrinsics(self, frame: DtofFrame) -> tuple[float, float, float, float]:
        fx = float(frame.calibration[0]) if len(frame.calibration) > 0 else 0.0
        fy = float(frame.calibration[1]) if len(frame.calibration) > 1 else 0.0
        cx = float(frame.calibration[2]) if len(frame.calibration) > 2 else WIDTH / 2.0
        cy = float(frame.calibration[3]) if len(frame.calibration) > 3 else HEIGHT / 2.0
        if not math.isfinite(fx) or fx <= 1.0:
            fx = 36.0
        if not math.isfinite(fy) or fy <= 1.0:
            fy = 36.0
        if not math.isfinite(cx) or cx <= 0.0:
            cx = WIDTH / 2.0
        if not math.isfinite(cy) or cy <= 0.0:
            cy = HEIGHT / 2.0
        return fx, fy, cx, cy

    def _point_cloud(self, frame: DtofFrame) -> PointCloud2 | None:
        fx, fy, cx, cy = self._dtof_intrinsics(frame)
        depth = frame.depth_mm.astype(np.float32)
        uu, vv = np.meshgrid(np.arange(WIDTH, dtype=np.float32), np.arange(HEIGHT, dtype=np.float32))
        z_mm = depth
        valid = (z_mm >= float(self.min_valid_depth_mm)) & (z_mm <= float(self.max_depth_mm))
        z_m = z_mm / 1000.0
        x = (uu - cx) * z_m / fx
        y = (vv - cy) * z_m / fy
        flat_valid = valid.reshape(-1)
        if not np.any(flat_valid):
            return None
        points = np.column_stack((
            x.reshape(-1)[flat_valid],
            y.reshape(-1)[flat_valid],
            z_m.reshape(-1)[flat_valid],
            z_mm.reshape(-1)[flat_valid],
        )).astype(np.float32, copy=False)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.dtof_frame_id
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * int(points.shape[0])
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg

    def _obstacle_summary(self, depth: np.ndarray, valid_mask: np.ndarray, recv_time_ns: int) -> dict[str, Any]:
        zones: list[dict[str, Any]] = []
        nearest_distance: int | None = None
        nearest_zone: str | None = None
        nearest_label: str | None = None
        total_valid = 0
        any_near = False
        any_warn = False
        any_clear = False
        for name, label, start_ratio, end_ratio in OBSTACLE_ZONE_SPECS:
            start_col = max(0, min(WIDTH - 1, int(round(WIDTH * start_ratio))))
            end_col = max(start_col + 1, min(WIDTH, int(round(WIDTH * end_ratio))))
            zone_depth = depth[:, start_col:end_col]
            zone_valid_raw = valid_mask[:, start_col:end_col]
            raw_values = zone_depth[zone_valid_raw].astype(np.float32)
            zone_valid = zone_valid_raw & (zone_depth >= self.dtof_obstacle_noise_floor_mm)
            values = zone_depth[zone_valid].astype(np.float32)
            total_valid += int(values.size)
            support_threshold = max(
                self.dtof_obstacle_min_support_pixels,
                int(math.ceil(float(max(1, values.size)) * self.dtof_obstacle_min_support_ratio)),
            )
            if values.size:
                min_mm = int(np.min(values))
                median_mm = int(np.median(values))
                p10_mm = int(np.percentile(values, 10.0)) if values.size >= 4 else min_mm
                p25_mm = int(np.percentile(values, 25.0)) if values.size >= 4 else min_mm
                robust_mm = (
                    int(np.percentile(values, self.dtof_obstacle_distance_percentile))
                    if values.size >= 4
                    else min_mm
                )
                near_support_pixels = int(np.count_nonzero(values <= self.dtof_obstacle_near_mm))
                warn_support_pixels = int(np.count_nonzero(values <= self.dtof_obstacle_warn_mm))
                if near_support_pixels >= support_threshold and robust_mm <= self.dtof_obstacle_near_mm:
                    state = "near"
                    distance_mm = robust_mm
                    any_near = True
                elif warn_support_pixels >= support_threshold and robust_mm <= self.dtof_obstacle_warn_mm:
                    state = "warn"
                    distance_mm = robust_mm
                    any_warn = True
                else:
                    state = "clear"
                    distance_mm = median_mm
                    any_clear = True
                if nearest_distance is None or distance_mm < nearest_distance:
                    nearest_distance = distance_mm
                    nearest_zone = name
                    nearest_label = label
            else:
                min_mm = None
                median_mm = None
                p10_mm = None
                p25_mm = None
                robust_mm = None
                distance_mm = None
                near_support_pixels = 0
                warn_support_pixels = 0
                state = "unknown"
            raw_min_mm = int(np.min(raw_values)) if raw_values.size else None
            raw_p10_mm = int(np.percentile(raw_values, 10.0)) if raw_values.size >= 4 else raw_min_mm
            zones.append({
                "name": name,
                "label": label,
                "columns": [start_col, end_col],
                "valid_pixels": int(values.size),
                "valid_ratio": float(values.size) / float(zone_valid.size),
                "raw_valid_pixels": int(raw_values.size),
                "raw_valid_ratio": float(raw_values.size) / float(zone_valid_raw.size),
                "raw_min_mm": raw_min_mm,
                "raw_p10_mm": raw_p10_mm,
                "min_mm": min_mm,
                "p10_mm": p10_mm,
                "p25_mm": p25_mm,
                "median_mm": median_mm,
                "robust_mm": robust_mm,
                "distance_mm": distance_mm,
                "near_support_pixels": near_support_pixels,
                "warn_support_pixels": warn_support_pixels,
                "support_threshold_pixels": support_threshold,
                "state": state,
            })

        if any_near:
            overall_state = "near"
        elif any_warn:
            overall_state = "warn"
        elif any_clear:
            overall_state = "clear"
        else:
            overall_state = "unknown"
        return {
            "recv_time_ns": recv_time_ns,
            "frame_id": self.dtof_frame_id,
            "width": WIDTH,
            "height": HEIGHT,
            "valid_pixels": total_valid,
            "near_threshold_mm": self.dtof_obstacle_near_mm,
            "warn_threshold_mm": self.dtof_obstacle_warn_mm,
            "noise_floor_mm": self.dtof_obstacle_noise_floor_mm,
            "distance_percentile": self.dtof_obstacle_distance_percentile,
            "min_support_pixels": self.dtof_obstacle_min_support_pixels,
            "min_support_ratio": self.dtof_obstacle_min_support_ratio,
            "nearest_mm": nearest_distance,
            "nearest_zone": nearest_zone,
            "nearest_label": nearest_label,
            "state": overall_state,
            "zones": zones,
        }

    def _obstacle_state(self, distance_mm: int | None) -> str:
        if distance_mm is None:
            return "unknown"
        if distance_mm <= self.dtof_obstacle_near_mm:
            return "near"
        if distance_mm <= self.dtof_obstacle_warn_mm:
            return "warn"
        return "clear"

    def _publish_obstacle_blocks(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.dtof_obstacle_blocks_pub.publish(msg)

    def _publish_dtof_visuals(self, depth: np.ndarray, obstacle_payload: dict[str, Any], stamp) -> None:
        heat = cv2.resize(
            self._depth_heatmap(depth),
            (self.dtof_visual_width, self.dtof_visual_height),
            interpolation=cv2.INTER_NEAREST,
        )
        depth_msg = compressed_jpeg_msg(heat, stamp, self.dtof_frame_id, self.dtof_visual_jpeg_quality)
        if depth_msg:
            self.dtof_depth_color_pub.publish(depth_msg)

        obstacle_view = self._obstacle_view(heat, obstacle_payload)
        view_msg = compressed_jpeg_msg(obstacle_view, stamp, self.dtof_frame_id, self.dtof_visual_jpeg_quality)
        if view_msg:
            self.dtof_obstacle_view_pub.publish(view_msg)

    def _obstacle_view(self, heat: np.ndarray, payload: dict[str, Any]) -> np.ndarray:
        image_h, image_w = heat.shape[:2]
        blocks_h = 112
        canvas = np.full((image_h + blocks_h, image_w, 3), (22, 22, 22), dtype=np.uint8)
        canvas[:image_h, :image_w] = heat

        for _, _, start_ratio, _ in OBSTACLE_ZONE_SPECS[1:]:
            x = int(round(image_w * start_ratio))
            cv2.line(canvas, (x, 0), (x, image_h - 1), (230, 230, 230), 1)

        nearest_label = payload.get("nearest_label") or "--"
        nearest_mm = payload.get("nearest_mm")
        if nearest_mm is None:
            headline = "nearest --"
        else:
            headline = f"nearest {nearest_label} {nearest_mm / 1000.0:.2f}m"
        self._draw_text(canvas, headline, (10, 26), 0.72, (255, 255, 255), 2)
        scale_line = f"near<={self.dtof_obstacle_near_mm / 1000.0:.2f}m warn<={self.dtof_obstacle_warn_mm / 1000.0:.2f}m"
        self._draw_text(canvas, scale_line, (10, 52), 0.5, (245, 245, 245), 1)
        support_line = (
            f"support>={self.dtof_obstacle_min_support_pixels}px "
            f"noise>={self.dtof_obstacle_noise_floor_mm / 1000.0:.2f}m"
        )
        self._draw_text(canvas, support_line, (10, 74), 0.45, (235, 235, 235), 1)

        zones = payload.get("zones", [])
        margin = 8
        gap = 6
        block_top = image_h + 12
        block_w = max(1, (image_w - margin * 2 - gap * (len(zones) - 1)) // max(1, len(zones)))
        for index, zone in enumerate(zones):
            x0 = margin + index * (block_w + gap)
            x1 = min(image_w - margin, x0 + block_w)
            color = self._state_color(zone.get("state"))
            cv2.rectangle(canvas, (x0, block_top), (x1, block_top + 78), color, -1)
            cv2.rectangle(canvas, (x0, block_top), (x1, block_top + 78), (245, 245, 245), 1)
            label = str(zone.get("label") or "--")
            distance = zone.get("distance_mm")
            distance_text = "--" if distance is None else f"{float(distance) / 1000.0:.2f}m"
            support_text = f"{int(zone.get('near_support_pixels') or 0)}/{int(zone.get('support_threshold_pixels') or 0)}"
            self._draw_text(canvas, label, (x0 + 8, block_top + 27), 0.65, (255, 255, 255), 2)
            self._draw_text(canvas, distance_text, (x0 + 8, block_top + 58), 0.54, (255, 255, 255), 1)
            self._draw_text(canvas, support_text, (x0 + 8, block_top + 76), 0.42, (255, 255, 255), 1)
        return canvas

    def _state_color(self, state: str | None) -> tuple[int, int, int]:
        if state == "near":
            return (35, 35, 230)
        if state == "warn":
            return (0, 165, 255)
        if state == "clear":
            return (70, 170, 70)
        return (75, 75, 75)

    @staticmethod
    def _draw_text(
        image: np.ndarray,
        text: str,
        org: tuple[int, int],
        scale: float,
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    def _save_dtof_preview(self, depth: np.ndarray, seq: int) -> None:
        if not self.session_dir:
            return
        vis = self._depth_heatmap(depth)
        cv2.imwrite(str(self.session_dir / "dtof_preview" / f"dtof_{seq:08d}.png"), vis)

    def _save_combined_preview(self, seq: int) -> None:
        if not self.session_dir:
            return
        with self.lock:
            cam = None if self.last_camera_frame is None else self.last_camera_frame.copy()
            depth = None if self.last_dtof_depth is None else self.last_dtof_depth.copy()
        if depth is None:
            return
        heat = self._depth_heatmap(depth)
        heat = cv2.resize(heat, (320, 240), interpolation=cv2.INTER_NEAREST)
        if cam is None:
            combined = heat
        else:
            cam_small = cv2.resize(cam, (320, 240), interpolation=cv2.INTER_AREA)
            combined = np.hstack([cam_small, heat])
        cv2.imwrite(str(self.session_dir / "preview" / f"sync_preview_{seq:08d}.jpg"), combined)
        if self.visualize_window:
            cv2.imshow("parking sensor preview", combined)
            cv2.waitKey(1)

    def _depth_heatmap(self, depth: np.ndarray) -> np.ndarray:
        depth_f = depth.astype(np.float32)
        valid = (depth_f >= float(self.min_valid_depth_mm)) & (depth_f <= float(self.max_depth_mm))
        normalized = np.zeros(depth_f.shape, dtype=np.uint8)
        if np.any(valid):
            lo = float(self.dtof_visual_min_mm)
            hi = float(self.dtof_visual_max_mm)
            clipped = np.clip(depth_f[valid], lo, hi)
            normalized[valid] = np.clip(255.0 * (hi - clipped) / (hi - lo), 1.0, 255.0).astype(np.uint8)
        heat = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        heat[~valid] = (0, 0, 0)
        return heat

    def _try_sync(self, source: str) -> None:
        with self.lock:
            cam_ns = self.last_camera_ns
            dtof_ns = self.last_dtof_ns
            cam_seq = self.last_camera_seq
            dtof_seq = self.last_dtof_seq
            if cam_ns is None or dtof_ns is None:
                return
            if cam_seq == self.last_sync_camera_seq and dtof_seq == self.last_sync_dtof_seq:
                return
            delta_ns = cam_ns - dtof_ns
            if abs(delta_ns) > self.sync_slop_ns:
                return
            self.last_sync_camera_seq = cam_seq
            self.last_sync_dtof_seq = dtof_seq

        payload = {
            "source": source,
            "camera_seq": cam_seq,
            "dtof_seq": dtof_seq,
            "camera_time_ns": cam_ns,
            "dtof_time_ns": dtof_ns,
            "delta_ms": delta_ns / 1_000_000.0,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.sync_pub.publish(msg)
        if self.enable_recording and self.sync_index:
            json_line(self.sync_index, payload)

    def _rate(self, times: deque[int]) -> float:
        if len(times) < 2:
            return 0.0
        elapsed = (times[-1] - times[0]) / 1_000_000_000.0
        return (len(times) - 1) / elapsed if elapsed > 0.0 else 0.0

    def _publish_health(self) -> None:
        if not self.running or not rclpy.ok():
            return
        current_ns = now_ns()
        with self.lock:
            camera_age = None if self.last_camera_ns is None else (current_ns - self.last_camera_ns) / 1e9
            dtof_age = None if self.last_dtof_ns is None else (current_ns - self.last_dtof_ns) / 1e9
            dtof_transport_ok = dtof_age is not None and dtof_age < 2.0
            dtof_depth_ok = bool(self.last_dtof_meta.get("depth_ok")) if self.last_dtof_meta else False
            dtof_ok = (dtof_transport_ok and dtof_depth_ok) if self.enable_dtof else True
            payload = {
                "time_ns": current_ns,
                "uptime_sec": (current_ns - self.start_ns) / 1e9,
                "camera": {
                    "frames": self.camera_count,
                    "fps": self._rate(self.camera_times),
                    "age_sec": camera_age,
                    "ok": camera_age is not None and camera_age < 2.0,
                    "shape": self.last_camera_shape,
                    "reconnects": self.camera_reconnects,
                    "flat_dropped": self.camera_flat_drop_count,
                    "last_flat": self.last_camera_flat,
                },
                "dtof": {
                    "enabled": self.enable_dtof,
                    "packets": self.dtof_count,
                    "fps": self._rate(self.dtof_times),
                    "age_sec": dtof_age,
                    "transport_ok": dtof_transport_ok,
                    "depth_ok": dtof_depth_ok,
                    "ok": dtof_ok,
                    "bad_packets": self.bad_dtof_count,
                    "packet_size": PACKET_SIZE,
                    "shape": [WIDTH, HEIGHT],
                    "last": self.last_dtof_meta,
                },
                "recording": {
                    "enabled": self.enable_recording,
                    "session_dir": str(self.session_dir) if self.session_dir else None,
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.health_pub.publish(msg)
        if self.enable_recording and self.health_index:
            json_line(self.health_index, payload)
        self.get_logger().info(
            "health "
            f"camera={payload['camera']['ok']} {payload['camera']['fps']:.1f}fps "
            f"dtof_enabled={self.enable_dtof} dtof_transport={payload['dtof']['transport_ok']} "
            f"dtof_depth={payload['dtof']['depth_ok']} {payload['dtof']['fps']:.1f}fps "
            f"bad_dtof={self.bad_dtof_count}",
            throttle_duration_sec=2.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorSuiteNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
