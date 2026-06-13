#!/usr/bin/env python3
"""ROS2 UDP receiver for board-forwarded STM32 USB serial data."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import json
import socket
import threading
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

from parking_bridge.stm32_protocol import analyze_bytes


MAGIC = b"STM32USB1 "


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


def parse_datagram(packet: bytes) -> tuple[dict[str, Any], bytes]:
    if not packet.startswith(MAGIC):
        raise ValueError("bad magic")
    header_end = packet.find(b"\n", len(MAGIC))
    if header_end < 0:
        raise ValueError("missing header delimiter")
    header = json.loads(packet[len(MAGIC):header_end].decode("utf-8"))
    return header, packet[header_end + 1:]


class Stm32UdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("parking_stm32_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 24680)
        self.declare_parameter("record_dir", str(Path.home() / "parking_sensor_records"))
        self.declare_parameter("enable_recording", True)
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("stale_after_sec", 2.0)
        self.declare_parameter("analysis_sample_bytes", 8192)

        self.bind_ip = str(self.get_parameter("bind_ip").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.record_root = Path(str(self.get_parameter("record_dir").value)).expanduser()
        self.enable_recording = as_bool(self.get_parameter("enable_recording").value)
        self.stale_after_sec = float(self.get_parameter("stale_after_sec").value)
        self.analysis_sample_bytes = int(self.get_parameter("analysis_sample_bytes").value)

        self.raw_pub = self.create_publisher(UInt8MultiArray, "/parking/stm32/raw", 10)
        self.metadata_pub = self.create_publisher(String, "/parking/stm32/metadata", 10)
        self.health_pub = self.create_publisher(String, "/parking/stm32/health", 10)

        self.start_ns = now_ns()
        self.running = True
        self.lock = threading.Lock()

        self.chunk_count = 0
        self.health_datagrams = 0
        self.bad_datagrams = 0
        self.total_bytes = 0
        self.last_chunk_ns: int | None = None
        self.last_source: tuple[str, int] | None = None
        self.last_metadata: dict[str, Any] | None = None
        self.chunk_times: deque[int] = deque(maxlen=120)
        self.analysis_sample = bytearray()

        self.session_dir: Path | None = None
        self.raw_file = None
        self.chunk_index = None
        self.health_index = None
        self.raw_offset = 0
        self._setup_recording()

        self.thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.thread.start()

        period = float(self.get_parameter("status_period_sec").value)
        self.status_timer = self.create_timer(period, self._publish_health)
        self.get_logger().info(
            f"STM32 UDP bridge listening on {self.bind_ip}:{self.udp_port}, "
            f"recording={self.enable_recording}, session={self.session_dir}"
        )

    def destroy_node(self) -> bool:
        self.running = False
        self.thread.join(timeout=2.0)
        self._write_final_analysis()
        for handle in (self.raw_file, self.chunk_index, self.health_index):
            if handle:
                handle.close()
        return super().destroy_node()

    def _setup_recording(self) -> None:
        if not self.enable_recording:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.record_root / f"stm32_session_{stamp}"
        ensure_dir(self.session_dir)
        self.raw_file = (self.session_dir / "stm32_serial_raw.bin").open("ab")
        self.chunk_index = (self.session_dir / "stm32_serial_chunks.jsonl").open("a", encoding="utf-8")
        self.health_index = (self.session_dir / "stm32_health.jsonl").open("a", encoding="utf-8")
        metadata = {
            "created_time_ns": self.start_ns,
            "udp_port": self.udp_port,
            "format": "STM32USB1 <json-header>\\n<raw-bytes>",
            "source": "board_stm32_usb_serial_udp_bridge.py",
            "analysis_sample_bytes": self.analysis_sample_bytes,
        }
        (self.session_dir / "session_metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def _udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.udp_port))
        sock.settimeout(1.0)
        while self.running:
            try:
                packet, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            recv_ns = now_ns()
            try:
                header, data = parse_datagram(packet)
            except Exception as exc:
                with self.lock:
                    self.bad_datagrams += 1
                self.get_logger().warn(f"bad STM32 UDP datagram: {exc}", throttle_duration_sec=5.0)
                continue

            header["vm_recv_time_ns"] = recv_ns
            header["source_ip"] = addr[0]
            header["source_port"] = addr[1]
            if not self.running or not rclpy.ok():
                break
            try:
                if header.get("type") == "serial_chunk":
                    self._handle_chunk(header, data)
                elif header.get("type") == "health":
                    self._handle_remote_health(header)
                else:
                    with self.lock:
                        self.bad_datagrams += 1
            except Exception:
                if not self.running or not rclpy.ok():
                    break
                raise
        sock.close()

    def _handle_chunk(self, header: dict[str, Any], data: bytes) -> None:
        if not self.running or not rclpy.ok():
            return
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.raw_pub.publish(msg)

        meta_msg = String()
        meta_msg.data = json.dumps(header, ensure_ascii=False, separators=(",", ":"))
        self.metadata_pub.publish(meta_msg)

        with self.lock:
            self.chunk_count += 1
            self.total_bytes += len(data)
            self.last_chunk_ns = int(header["vm_recv_time_ns"])
            self.last_source = (str(header["source_ip"]), int(header["source_port"]))
            self.last_metadata = dict(header)
            self.chunk_times.append(self.last_chunk_ns)
            self.analysis_sample.extend(data)
            overflow = len(self.analysis_sample) - self.analysis_sample_bytes
            if overflow > 0:
                del self.analysis_sample[:overflow]
            offset = self.raw_offset
            self.raw_offset += len(data)

        if self.enable_recording and self.raw_file and self.chunk_index:
            self.raw_file.write(data)
            self.raw_file.flush()
            record = dict(header)
            record["offset"] = offset
            record["length"] = len(data)
            json_line(self.chunk_index, record)

    def _handle_remote_health(self, header: dict[str, Any]) -> None:
        with self.lock:
            self.health_datagrams += 1
            self.last_source = (str(header["source_ip"]), int(header["source_port"]))
            self.last_metadata = dict(header)
        if self.enable_recording and self.health_index:
            json_line(self.health_index, header)

    def _rate(self, times: deque[int]) -> float:
        if len(times) < 2:
            return 0.0
        elapsed = (times[-1] - times[0]) / 1_000_000_000.0
        return (len(times) - 1) / elapsed if elapsed > 0.0 else 0.0

    def _analysis_snapshot(self) -> dict[str, Any]:
        with self.lock:
            sample = bytes(self.analysis_sample)
            total_bytes = self.total_bytes
        analysis = analyze_bytes(sample, sample_limit=self.analysis_sample_bytes)
        analysis["stream_total_bytes"] = total_bytes
        return analysis

    def _write_final_analysis(self) -> None:
        if not self.enable_recording or not self.session_dir:
            return
        if self.raw_file:
            self.raw_file.flush()
        raw_path = self.session_dir / "stm32_serial_raw.bin"
        if raw_path.exists():
            data = raw_path.read_bytes()
            analysis = analyze_bytes(data, sample_limit=self.analysis_sample_bytes)
        else:
            analysis = self._analysis_snapshot()
        analysis["final"] = True
        (self.session_dir / "stm32_protocol_analysis.json").write_text(
            json.dumps(analysis, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def _publish_health(self) -> None:
        if not self.running or not rclpy.ok():
            return
        current_ns = now_ns()
        with self.lock:
            sample = bytes(self.analysis_sample)
            age = None if self.last_chunk_ns is None else (current_ns - self.last_chunk_ns) / 1e9
            payload = {
                "time_ns": current_ns,
                "uptime_sec": (current_ns - self.start_ns) / 1e9,
                "ok": age is not None and age < self.stale_after_sec,
                "chunks": self.chunk_count,
                "bytes": self.total_bytes,
                "chunk_rate_hz": self._rate(self.chunk_times),
                "last_chunk_age_sec": age,
                "bad_datagrams": self.bad_datagrams,
                "remote_health_datagrams": self.health_datagrams,
                "last_source": self.last_source,
                "last": self.last_metadata,
                "analysis": analyze_bytes(sample, sample_limit=self.analysis_sample_bytes),
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
            f"stm32 health ok={payload['ok']} chunks={payload['chunks']} "
            f"bytes={payload['bytes']} bad={payload['bad_datagrams']}",
            throttle_duration_sec=2.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32UdpBridge()
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
