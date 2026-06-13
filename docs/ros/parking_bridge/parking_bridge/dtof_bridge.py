#!/usr/bin/env python3
"""
dToF UDP Bridge Node
Receives dToF packets from SS928 board (sample_dtof) and publishes:
  - /dtof/points     (sensor_msgs/PointCloud2)
  - /dtof/depth      (sensor_msgs/Image, 32FC1 in mm)
  - /dtof/info       (sensor_msgs/CameraInfo)

Packet format (pragma pack(1)):
  TofUdpPacketHead (73 bytes):
    checkSum   int16
    seqNum     int16
    startPixel uint32
    pixelNumber int16
    timestampSec  uint32
    timestampNSec uint32
    TofAdditionInfo (55 bytes):
      width      int16
      height     int16
      frameRate  int16
      version    uint8
      reserved   float32[12]  (fx,fy,cx,cy,k1,k2,p1,p2,k3,...)
  TofUdpPacketData (4800 bytes):
    TofPixelContent[1200]:
      depth      int16   (mm)
      confidence uint8
      flag       uint8
Total = 4873 bytes
"""

import socket
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header

# ---- Packet layout constants ----
PIXELS_H = 30
PIXELS_W = 40
PIXELS_TOTAL = PIXELS_H * PIXELS_W  # 1200

# struct formats (little-endian, packed)
HEAD_FMT = '<hh Ih II hhh B 12f'
HEAD_SIZE = struct.calcsize(HEAD_FMT)   # should be 73
PIXEL_FMT = '<hBB'
PIXEL_SIZE = struct.calcsize(PIXEL_FMT)  # 4
PACKET_SIZE = HEAD_SIZE + PIXELS_TOTAL * PIXEL_SIZE  # 4873

# PointCloud2 field layout (xyz float32 + depth float32)
PC2_FIELDS = [
    PointField(name='x',     offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',     offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',     offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='depth', offset=12, datatype=PointField.FLOAT32, count=1),
]
PC2_POINT_STEP = 16  # 4 × float32


def _build_time(sec: int, nsec: int) -> Time:
    t = Time()
    t.sec = sec
    t.nanosec = nsec
    return t


class DtofBridge(Node):
    def __init__(self):
        super().__init__('dtof_bridge')

        # Parameters
        self.declare_parameter('udp_port', 2368)
        self.declare_parameter('frame_id', 'dtof')
        self.declare_parameter('max_depth_mm', 5000)

        self._port = self.get_parameter('udp_port').value
        self._frame_id = self.get_parameter('frame_id').value
        self._max_depth = self.get_parameter('max_depth_mm').value

        # Publishers
        self._pub_pc2   = self.create_publisher(PointCloud2, '/dtof/points', 10)
        self._pub_depth = self.create_publisher(Image,       '/dtof/depth',  10)
        self._pub_info  = self.create_publisher(CameraInfo,  '/dtof/info',   10)

        # UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', self._port))
        self._sock.settimeout(1.0)

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'dToF bridge listening on UDP port {self._port}, '
            f'expecting {PACKET_SIZE}-byte packets'
        )

    def destroy_node(self):
        self._running = False
        self._sock.close()
        self._thread.join(timeout=2.0)
        super().destroy_node()

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < PACKET_SIZE:
                self.get_logger().warn(
                    f'Short packet: {len(data)} bytes from {addr}', throttle_duration_sec=5.0
                )
                continue

            try:
                self._process_packet(data)
            except Exception as e:
                self.get_logger().error(f'Packet parse error: {e}')

    def _process_packet(self, data: bytes):
        # ---- Parse header ----
        hf = struct.unpack_from(HEAD_FMT, data, 0)
        check_sum  = hf[0]
        seq_num    = hf[1]
        start_pix  = hf[2]
        pix_num    = hf[3]
        ts_sec     = hf[4]
        ts_nsec    = hf[5]
        width      = hf[6]   # should be 40
        height     = hf[7]   # should be 30
        frame_rate = hf[8]
        version    = hf[9]
        reserved   = hf[10:22]  # 12 floats

        fx = reserved[0]
        fy = reserved[1]
        cx = reserved[2]
        cy = reserved[3]

        stamp = _build_time(ts_sec, ts_nsec)
        hdr   = Header(stamp=stamp, frame_id=self._frame_id)

        # ---- Parse pixel data ----
        pix_offset = HEAD_SIZE
        depths      = np.zeros(PIXELS_TOTAL, dtype=np.float32)
        confidences = np.zeros(PIXELS_TOTAL, dtype=np.uint8)
        flags       = np.zeros(PIXELS_TOTAL, dtype=np.uint8)

        for i in range(PIXELS_TOTAL):
            d, c, f = struct.unpack_from(PIXEL_FMT, data, pix_offset + i * PIXEL_SIZE)
            depths[i]      = float(d)
            confidences[i] = c
            flags[i]       = f

        depths = depths.reshape((PIXELS_H, PIXELS_W))

        # ---- Publish CameraInfo ----
        self._publish_info(hdr, fx, fy, cx, cy)

        # ---- Publish depth image (32FC1, mm) ----
        self._publish_depth(hdr, depths)

        # ---- Publish PointCloud2 ----
        if fx > 0 and fy > 0:
            self._publish_pc2(hdr, depths, fx, fy, cx, cy)

    def _publish_info(self, hdr, fx, fy, cx, cy):
        msg = CameraInfo()
        msg.header = hdr
        msg.width  = PIXELS_W
        msg.height = PIXELS_H
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.distortion_model = 'plumb_bob'
        self._pub_info.publish(msg)

    def _publish_depth(self, hdr, depths: np.ndarray):
        msg = Image()
        msg.header   = hdr
        msg.width    = PIXELS_W
        msg.height   = PIXELS_H
        msg.encoding = '32FC1'
        msg.step     = PIXELS_W * 4
        msg.data     = depths.astype(np.float32).tobytes()
        self._pub_depth.publish(msg)

    def _publish_pc2(self, hdr, depths: np.ndarray, fx, fy, cx, cy):
        points = np.zeros((PIXELS_TOTAL, 4), dtype=np.float32)
        idx = 0
        for v in range(PIXELS_H):
            for u in range(PIXELS_W):
                z = depths[v, u]
                if z <= 0 or z > self._max_depth:
                    z = float('nan')
                x = (u - cx) * z / fx if not np.isnan(z) else float('nan')
                y = (v - cy) * z / fy if not np.isnan(z) else float('nan')
                points[idx] = [x / 1000.0, y / 1000.0, z / 1000.0, z]  # xyz in metres, depth in mm
                idx += 1

        msg = PointCloud2()
        msg.header      = hdr
        msg.height      = 1
        msg.width       = PIXELS_TOTAL
        msg.fields      = PC2_FIELDS
        msg.is_bigendian = False
        msg.point_step  = PC2_POINT_STEP
        msg.row_step    = PC2_POINT_STEP * PIXELS_TOTAL
        msg.is_dense    = False
        msg.data        = points.tobytes()
        self._pub_pc2.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DtofBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
