"""Parser for the official GS1860/dToF UDP packet used by sample_dtof."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Any

import numpy as np


WIDTH = 40
HEIGHT = 30
PIXELS = WIDTH * HEIGHT

# Official struct from dtof_sensor_driver sample:
#   short checkSum
#   short seqNum
#   unsigned int startPixel
#   short pixelNumber
#   unsigned int timestampSeconds
#   unsigned int timestampNanoSeconds
#   short width
#   short height
#   short frameRate
#   uint8 version
#   float reserved[12]
HEADER_FMT = "<hhIhIIhhhB12f"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
PIXEL_DTYPE = np.dtype([("depth", "<i2"), ("confidence", "u1"), ("flag", "u1")])
PACKET_SIZE = HEADER_SIZE + PIXELS * PIXEL_DTYPE.itemsize

WIDTH_OFFSET = 18
HEIGHT_OFFSET = 20


@dataclass(frozen=True)
class DtofFrame:
    raw: bytes
    recv_time_ns: int
    source_ip: str
    source_port: int
    checksum: int
    seq_num: int
    start_pixel: int
    pixel_number: int
    device_time_sec: int
    device_time_nsec: int
    width: int
    height: int
    frame_rate: int
    version: int
    calibration: tuple[float, ...]
    depth_mm: np.ndarray
    confidence: np.ndarray
    flags: np.ndarray

    @property
    def is_expected_shape(self) -> bool:
        return self.width == WIDTH and self.height == HEIGHT and self.pixel_number == PIXELS

    def metadata(self) -> dict[str, Any]:
        valid = self.depth_mm[self.depth_mm > 0]
        depth_unique = np.unique(self.depth_mm)
        confidence_nonzero = self.confidence[self.confidence > 0]
        return {
            "recv_time_ns": self.recv_time_ns,
            "source_ip": self.source_ip,
            "source_port": self.source_port,
            "checksum": self.checksum,
            "seq_num": self.seq_num,
            "start_pixel": self.start_pixel,
            "pixel_number": self.pixel_number,
            "device_time_sec": self.device_time_sec,
            "device_time_nsec": self.device_time_nsec,
            "width": self.width,
            "height": self.height,
            "frame_rate": self.frame_rate,
            "version": self.version,
            "packet_size": len(self.raw),
            "expected_packet_size": PACKET_SIZE,
            "expected_shape": self.is_expected_shape,
            "depth_min_mm": int(valid.min()) if valid.size else None,
            "depth_max_mm": int(valid.max()) if valid.size else None,
            "depth_mean_mm": float(valid.mean()) if valid.size else None,
            "valid_pixels": int(valid.size),
            "depth_unique_count": int(depth_unique.size),
            "depth_nonzero_pixels": int(valid.size),
            "depth_gt20mm_pixels": int((self.depth_mm > 20).sum()),
            "confidence_nonzero_pixels": int(confidence_nonzero.size),
            "calibration": list(self.calibration),
        }


def parse_packet(data: bytes, recv_time_ns: int, source: tuple[str, int]) -> DtofFrame:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"unexpected dToF packet size {len(data)}, expected {PACKET_SIZE}")

    fields = struct.unpack_from(HEADER_FMT, data, 0)
    pixels = np.frombuffer(data, dtype=PIXEL_DTYPE, count=PIXELS, offset=HEADER_SIZE)

    depth = pixels["depth"].reshape((HEIGHT, WIDTH)).copy()
    confidence = pixels["confidence"].reshape((HEIGHT, WIDTH)).copy()
    flags = pixels["flag"].reshape((HEIGHT, WIDTH)).copy()

    return DtofFrame(
        raw=bytes(data),
        recv_time_ns=recv_time_ns,
        source_ip=source[0],
        source_port=source[1],
        checksum=fields[0],
        seq_num=fields[1],
        start_pixel=fields[2],
        pixel_number=fields[3],
        device_time_sec=fields[4],
        device_time_nsec=fields[5],
        width=fields[6],
        height=fields[7],
        frame_rate=fields[8],
        version=fields[9],
        calibration=tuple(float(x) for x in fields[10:22]),
        depth_mm=depth,
        confidence=confidence,
        flags=flags,
    )


def make_synthetic_packet(seq: int = 1, depth_mm: int = 1200) -> bytes:
    """Build one valid packet for parser and ROS node smoke tests."""
    header = struct.pack(
        HEADER_FMT,
        0,
        seq,
        0,
        PIXELS,
        0,
        0,
        WIDTH,
        HEIGHT,
        30,
        1,
        36.0,
        36.0,
        WIDTH / 2.0,
        HEIGHT / 2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    pixel = struct.pack("<hBB", depth_mm, 255, 0)
    return header + pixel * PIXELS
