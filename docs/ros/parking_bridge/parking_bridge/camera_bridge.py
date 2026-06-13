#!/usr/bin/env python3
"""
Camera ROS2 Bridge Node

Connects to the board's TCP H265 stream (camera_tcp_server.py),
decodes frames with cv2 (built-in FFmpeg), and publishes:
  /camera/image_raw   (sensor_msgs/Image, bgr8)

cv2.VideoCapture uses the FFmpeg TCP protocol to open
tcp://host:port directly — no local FIFO or extra ffmpeg binary needed.

Run standalone:
  source /opt/ros/humble/setup.bash
  python3 camera_bridge.py

Or via launch file (board_host / board_port parameters).
"""
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

BOARD_HOST_DEFAULT = '192.168.137.2'
BOARD_PORT_DEFAULT = 5000
RECONNECT_DELAY    = 3.0   # seconds between reconnect attempts


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter('board_host', BOARD_HOST_DEFAULT)
        self.declare_parameter('board_port', BOARD_PORT_DEFAULT)
        self.declare_parameter('frame_id',   'camera')
        # scale: resize factor applied to each frame before publishing.
        # 1.0 = full 8MP (3840x2160), 0.5 = 1920x1080, 0.25 = 960x540.
        self.declare_parameter('scale', 1.0)

        self._host     = self.get_parameter('board_host').value
        self._port     = self.get_parameter('board_port').value
        self._frame_id = self.get_parameter('frame_id').value
        self._scale    = float(self.get_parameter('scale').value)
        self._running  = True

        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'Camera bridge → tcp://{self._host}:{self._port}  scale={self._scale}'
        )

    def destroy_node(self):
        self._running = False
        super().destroy_node()

    # ------------------------------------------------------------------
    # Streaming loop
    # ------------------------------------------------------------------

    def _stream_loop(self):
        url = f'tcp://{self._host}:{self._port}'
        while self._running:
            cap = self._open_capture(url)
            if cap is None:
                time.sleep(RECONNECT_DELAY)
                continue

            self.get_logger().info('Stream open — decoding frames')
            frame_count = 0
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn('Frame read failed — reconnecting')
                    break
                frame_count += 1
                if frame_count % 30 == 1:
                    h, w = frame.shape[:2]
                    self.get_logger().info(f'Frame {frame_count}: {w}x{h}', throttle_duration_sec=10.0)
                self._publish(frame)

            cap.release()
            if self._running:
                self.get_logger().info(f'Reconnecting in {RECONNECT_DELAY}s…')
                time.sleep(RECONNECT_DELAY)

    def _open_capture(self, url: str):
        self.get_logger().info(f'Opening {url}')
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().warn(f'cv2 failed to open {url}')
            return None
        return cap

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _publish(self, frame: np.ndarray):
        if self._scale != 1.0:
            h, w = frame.shape[:2]
            nw = int(w * self._scale)
            nh = int(h * self._scale)
            frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = 'bgr8'
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
