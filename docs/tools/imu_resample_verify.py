#!/usr/bin/env python3
"""IMU底盘重采样验证 — 直接连接小车,采集纯IMU数据并验证打表效果.

不依赖任何历史.json数据。直接通过串口(或SSH隧道)向STM32发送校准命令,
收集STAT响应,从纯IMU yaw计算曲率/死区/中位等参数,独立验证打表方案。

连接方式:
  A. 直接串口 (Windows推荐):  --port COM7
  B. SSH隧道 (Linux SBC):     --host 172.20.10.2

用法:
  # 扫描串口,确认连接
  python imu_resample_verify.py --scan

  # 单角度探针
  python imu_resample_verify.py --port COM7 --ste 60,75,92,105,120

  # 全量B2曲率扫描 (STE=55~125步长5°)
  python imu_resample_verify.py --port COM7 --scan-b2

  # B1中位精扫 (STE=86~98步长1°)
  python imu_resample_verify.py --port COM7 --scan-b1

  # 仅预览命令,不实际发送 (安全检查)
  python imu_resample_verify.py --port COM7 --scan-b2 --dry-run
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean, pstdev
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

# ═══════════════════════════════════════════════════════════════════════════════
# IMU 误差模型 (BMI270, Kalman q=0.5, r=0.1)
# ═══════════════════════════════════════════════════════════════════════════════

IMU_DRIFT_RATE_DPS = 0.007
IMU_ARW = 0.758          # °/√h
IMU_BI = 4.52            # °/h
IMU_BI_TAU = 102.4       # s


def imu_yaw_sigma(duration_s: float) -> float:
    """1σ IMU yaw误差(°) for given duration."""
    s_arw = IMU_ARW * math.sqrt(duration_s / 3600.0)
    if duration_s < IMU_BI_TAU:
        s_bi = IMU_BI * (duration_s / 3600.0)
    else:
        s_bi = IMU_BI * (IMU_BI_TAU / 3600.0) * math.sqrt(duration_s / IMU_BI_TAU)
    s_drift = IMU_DRIFT_RATE_DPS * duration_s
    return math.sqrt(s_arw**2 + s_bi**2 + s_drift**2)


def imu_snr(yaw_deg: float, duration_s: float) -> dict:
    s = imu_yaw_sigma(duration_s)
    if s < 1e-9:
        return {"snr": float("inf"), "sigma": 0.0, "grade": "A"}
    snr = abs(yaw_deg) / s
    grade = "A" if snr > 30 else ("B" if snr > 15 else ("C" if snr > 10 else "D"))
    return {"snr": round(snr, 1), "sigma": round(s, 4), "grade": grade}


# ═══════════════════════════════════════════════════════════════════════════════
# 串口扫描
# ═══════════════════════════════════════════════════════════════════════════════

def scan_serial_ports() -> list[dict]:
    """列出系统所有串口."""
    try:
        import serial.tools.list_ports
    except ImportError:
        print("需要 pyserial: pip install pyserial", file=sys.stderr)
        return []

    ports = []
    for item in serial.tools.list_ports.comports():
        ports.append({
            "device": item.device,
            "description": item.description or "",
            "hwid": item.hwid or "",
        })
    return ports


def detect_stm32_port() -> str | None:
    """自动检测STM32串口 (CH340直连 或 JDY-24M蓝牙)."""
    ports = scan_serial_ports()
    # 优先级1: CH340/CH341 USB转串口
    for p in ports:
        hwid = p["hwid"].lower()
        desc = p["description"].lower()
        if any(k in hwid for k in ("ch340", "ch341", "1a86", "7523")):
            return p["device"]
        if any(k in desc for k in ("ch340", "ch341", "usb-serial ch")):
            return p["device"]
    # 优先级2: JDY-24M 蓝牙串口 (BTHENUM\{00001101...})
    for p in ports:
        hwid = p["hwid"].lower()
        if "bthenum" in hwid and "00001101" in hwid:
            return p["device"]
    # 优先级3: 任何蓝牙串口
    for p in ports:
        if "bthenum" in p["hwid"].lower():
            return p["device"]
    # 回退
    return ports[0]["device"] if ports else None


# ═══════════════════════════════════════════════════════════════════════════════
# STAT响应解析
# ═══════════════════════════════════════════════════════════════════════════════

STAT_RE = re.compile(
    r"STAT\s+(?:(?P<seq>\d+)\s+)?"
    r"MODE=(?P<mode>\S+)\s+"
    r"RUN=(?P<run>\S+)\s+"
    r"DIR=(?P<dir>-?\d+)\s+"
    r"SPD=(?P<spd>\d+)\s+"
    r"ANG=(?P<ang>[-\d.]+)\s+"
    r"YAW=(?P<yaw>[-\d.]+)\s+"
    r"X=(?P<x>[-\d.]+)\s+"
    r"Y=(?P<y>[-\d.]+)\s+"
    r"D=(?P<d>[-\d.]+)\s+"
    r"VEL=(?P<vel>[-\d.]+)\s+"
    r"DROP=(?P<drop>\d+)"
)

DONE_RE = re.compile(r"DONE\s+\d+\s+(ARC|MOVE|TURN)", re.IGNORECASE)
ERR_RE = re.compile(r"ERR\s", re.IGNORECASE)


def parse_stat(line: str) -> dict | None:
    m = STAT_RE.search(line.replace("\r", "").strip())
    if not m:
        return None
    return {
        "seq": int(m.group("seq")) if m.group("seq") else None,
        "mode": m.group("mode"), "run": m.group("run"),
        "dir": int(m.group("dir")), "spd": int(m.group("spd")),
        "ang": float(m.group("ang")), "yaw": float(m.group("yaw")),
        "x": float(m.group("x")), "y": float(m.group("y")),
        "d": float(m.group("d")), "vel": float(m.group("vel")),
        "drop": int(m.group("drop")),
    }


def is_done(line: str) -> bool:
    return bool(DONE_RE.search(line))


def is_err(line: str) -> bool:
    return bool(ERR_RE.search(line))


# ═══════════════════════════════════════════════════════════════════════════════
# 串口会话管理
# ═══════════════════════════════════════════════════════════════════════════════

class SerialSession:
    """直接串口连接STM32."""

    def __init__(self, port: str, baud: int = 9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self._rx_buf: list[str] = []

    def open(self):
        import serial
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.1,
                                 write_timeout=1.0)
        time.sleep(0.4)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def send(self, seq: int, command: str):
        """发送V2协议命令: @<seq> <command>\r"""
        msg = f"@{seq} {command}\r"
        self.ser.write(msg.encode("utf-8"))

    def read_until_done(self, timeout_s: float = 15.0) -> list[str]:
        """读取响应直到DONE/ERR或超时. 返回所有行."""
        lines: list[str] = []
        deadline = time.monotonic() + timeout_s
        done_seen = False
        while time.monotonic() < deadline:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if raw:
                try:
                    text = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                if text:
                    lines.append(text)
                    if is_done(text) or is_err(text):
                        done_seen = True
            if done_seen:
                # 再读200ms收集STAT after
                extra_deadline = time.monotonic() + 0.3
                while time.monotonic() < extra_deadline:
                    try:
                        raw = self.ser.readline()
                    except Exception:
                        break
                    if raw:
                        try:
                            text = raw.decode("utf-8", errors="replace").strip()
                        except Exception:
                            continue
                        if text:
                            lines.append(text)
                break
        return lines


class SSHSession:
    """SSH隧道连接: Python -> SSH -> Linux SBC -> /dev/ttyUSB0 -> STM32."""

    BOARD_SCRIPT = """#!/bin/sh
set -u
CMD="${1:-STAT}"; SEQ="${2:-1}"; READ_SEC="${3:-15}"
TTY=/dev/ttyUSB0; VID=1a86; PID=7523
INIT=/opt/parking/stm32_uart/ch341_user_init
node=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ -f "$d/idProduct" ] || continue
  if [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ]; then
    node=$(basename "$d"); break
  fi
done
[ -n "$node" ] || { echo "FAIL no_ch341"; exit 2; }
[ -e "$TTY" ] || { echo "FAIL no_tty"; exit 3; }
BUS=$(cat "/sys/bus/usb/devices/$node/busnum")
DEV=$(cat "/sys/bus/usb/devices/$node/devnum")
USBDEV=$(printf "/dev/bus/usb/%03d/%03d" "$BUS" "$DEV")
[ -x "$INIT" ] && "$INIT" "$USBDEV" >/dev/null 2>&1
stty -F "$TTY" 9600 cs8 -cstopb -parenb -ixon -ixoff -crtscts -hupcl clocal cread raw -echo min 0 time 1
e=$(( $(date +%s) + 1 ))
while [ "$(date +%s)" -lt "$e" ]; do dd if="$TTY" bs=1 count=256 2>/dev/null >/dev/null; done
printf '@%s %s\\r' "$SEQ" "$CMD" > "$TTY"
OUT=/tmp/sp.bin; : > "$OUT"
e=$(( $(date +%s) + $READ_SEC ))
while [ "$(date +%s)" -lt "$e" ]; do
  dd if="$TTY" bs=1 count=256 2>/dev/null >> "$OUT"
  if grep -Eq '(^|\\r|\\n)(DONE|ERR) ' "$OUT" 2>/dev/null; then
    sleep 0.2; dd if="$TTY" bs=1 count=256 2>/dev/null >> "$OUT"; break
  fi
done
echo "---RESP---"; tr -cd '\\11\\12\\15\\40-\\176' < "$OUT"; echo; echo "---END---"
"""

    def __init__(self, host: str, user: str = "root", password: str = "ebaina"):
        self.host = host
        self.user = user
        self.password = password
        self.client = None

    def open(self):
        import paramiko
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(self.host, username=self.user, password=self.password,
                            timeout=20, banner_timeout=20, auth_timeout=20)

    def close(self):
        if self.client:
            self.client.close()
        self.client = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def send_and_recv(self, seq: int, command: str, timeout_s: float = 15.0) -> list[str]:
        data = self.BOARD_SCRIPT.replace("\r\n", "\n").encode("utf-8")
        b64 = base64.b64encode(data).decode("ascii")
        _stdin, stdout, stderr = self.client.exec_command(
            f"echo '{b64}' | base64 -d > /tmp/sp.sh && sh /tmp/sp.sh '{command}' {seq} {int(timeout_s)}",
            timeout=timeout_s + 30)
        out = stdout.read().decode("utf-8", errors="replace")
        # 提取 RESP ... END 之间的内容
        if "---RESP---" in out and "---END---" in out:
            start = out.index("---RESP---") + len("---RESP---")
            end = out.index("---END---")
            raw = out[start:end]
        else:
            raw = out
        return [ln.strip() for ln in raw.splitlines() if ln.strip()]


# ═══════════════════════════════════════════════════════════════════════════════
# 核心探针逻辑
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class ProbeResult:
    """单次探针的完整数据."""
    command: str
    ste: int | None          # 舵机角度 (从命令解析)
    seq: int
    timestamp: str
    # STAT before
    yaw_before: float | None
    d_before: float | None
    x_before: float | None
    y_before: float | None
    # STAT after
    yaw_after: float | None
    d_after: float | None
    x_after: float | None
    y_after: float | None
    # 计算值 (纯IMU)
    delta_yaw_deg: float | None
    delta_d_cm: float | None
    delta_x_cm: float | None
    delta_y_cm: float | None
    deg_per_cm: float | None
    # IMU质量评估
    duration_est_s: float
    imu_sigma_deg: float
    imu_snr: float
    imu_grade: str
    # 状态
    done_received: bool
    err_received: bool
    err_detail: str
    raw_response_lines: int

    # 可选: DONE事件数据
    done_d_cm: float | None = None
    done_yaw_deg: float | None = None


def estimate_duration(dist_cm: float, command: str) -> float:
    """估算测量持续时间."""
    if "TURN" in command.upper():
        return max(dist_cm / 48.0, 3.0)
    if "MOVE" in command.upper():
        return max(dist_cm / 5.0, 1.0)
    if dist_cm <= 2.0:
        return 1.0
    return max(dist_cm / 2.5, 1.5)


def extract_ste(command: str) -> int | None:
    """从命令中提取舵机角度."""
    m = re.search(r"STE=(\d+)", command)
    return int(m.group(1)) if m else None


def execute_one_probe(
    session,           # SerialSession or SSHSession
    command: str,
    seq: int,
    timeout_s: float = 15.0,
    dry_run: bool = False,
) -> ProbeResult:
    """执行一次探针: 发送命令 -> 等待DONE -> 收集STAT. 返回结构化结果."""
    ste = extract_ste(command)
    ts = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    if dry_run:
        return ProbeResult(
            command=command, ste=ste, seq=seq, timestamp=ts,
            yaw_before=None, d_before=None, x_before=None, y_before=None,
            yaw_after=None, d_after=None, x_after=None, y_after=None,
            delta_yaw_deg=None, delta_d_cm=None, delta_x_cm=None, delta_y_cm=None,
            deg_per_cm=None,
            duration_est_s=0, imu_sigma_deg=0, imu_snr=0, imu_grade="?",
            done_received=False, err_received=False, err_detail="",
            raw_response_lines=0,
        )

    # 发送命令并收集响应
    if isinstance(session, SSHSession):
        lines = session.send_and_recv(seq, command, timeout_s)
    else:
        # 先读空缓冲区
        try:
            session.ser.reset_input_buffer()
        except Exception:
            pass
        session.send(seq, command)
        lines = session.read_until_done(timeout_s)

    # 解析响应
    stats = []
    done_data = {}
    err_detail = ""
    done_received = False
    err_received = False

    for line in lines:
        s = parse_stat(line)
        if s:
            stats.append(s)
            continue
        if is_done(line):
            done_received = True
            # 尝试解析 DONE D=...
            for part in line.split():
                if "=" in part:
                    k, v = part.split("=", 1)
                    try:
                        done_data[k.lower()] = float(v)
                    except ValueError:
                        done_data[k.lower()] = v
            continue
        if is_err(line):
            err_received = True
            err_detail = line[:120]
            continue

    # 提取 before/after
    if len(stats) >= 2:
        stat_before = stats[0]
        stat_after = stats[-1]
    elif len(stats) == 1:
        stat_before = stats[0]
        stat_after = stats[0]
    else:
        stat_before = None
        stat_after = None

    # 计算deltas
    yaw_before = stat_before["yaw"] if stat_before else None
    d_before = stat_before["d"] if stat_before else None
    yaw_after = stat_after["yaw"] if stat_after else None
    d_after = stat_after["d"] if stat_after else None

    delta_yaw = None
    delta_d = None
    deg_per_cm = None
    if yaw_before is not None and yaw_after is not None:
        delta_yaw = round(yaw_after - yaw_before, 6)
    if d_before is not None and d_after is not None:
        delta_d = round(d_after - d_before, 6)
    if delta_yaw is not None and delta_d is not None and abs(delta_d) > 0.005:
        deg_per_cm = round(delta_yaw / delta_d, 6)

    # IMU质量
    dur = estimate_duration(abs(delta_d or 2.0), command)
    imu_q = imu_snr(delta_yaw or 0.0, dur)

    return ProbeResult(
        command=command, ste=ste, seq=seq, timestamp=ts,
        yaw_before=yaw_before, d_before=d_before,
        x_before=stat_before["x"] if stat_before else None,
        y_before=stat_before["y"] if stat_before else None,
        yaw_after=yaw_after, d_after=d_after,
        x_after=stat_after["x"] if stat_after else None,
        y_after=stat_after["y"] if stat_after else None,
        delta_yaw_deg=delta_yaw, delta_d_cm=delta_d,
        delta_x_cm=round(stat_after["x"] - stat_before["x"], 6)
            if stat_before and stat_after and stat_before is not stat_after else None,
        delta_y_cm=round(stat_after["y"] - stat_before["y"], 6)
            if stat_before and stat_after and stat_before is not stat_after else None,
        deg_per_cm=deg_per_cm,
        duration_est_s=round(dur, 1),
        imu_sigma_deg=round(imu_q["sigma"], 4),
        imu_snr=round(imu_q["snr"], 1),
        imu_grade=imu_q["grade"],
        done_received=done_received,
        err_received=err_received,
        err_detail=err_detail,
        raw_response_lines=len(lines),
        done_d_cm=done_data.get("d"),
        done_yaw_deg=done_data.get("yaw"),
    )


# ═══════════════════════════════════════════════════════════════════════════════
# 探针序列构建
# ═══════════════════════════════════════════════════════════════════════════════

def build_probe_list(
    ste_list: list[int] | None = None,
    scan_b1: bool = False,
    scan_b2: bool = False,
    d_cm: float = 6.0,
    near_center_d_cm: float = 8.0,
    v_gear: int = 1,
) -> list[str]:
    """构建探针命令列表."""
    commands: list[str] = []
    ste_set: set[int] = set()

    if scan_b1:
        for s in range(86, 99):
            ste_set.add(s)

    if scan_b2:
        for s in list(range(55, 86, 5)) + list(range(95, 130, 5)):
            ste_set.add(s)

    if ste_list:
        for s in ste_list:
            ste_set.add(s)

    for ste in sorted(ste_set):
        d = near_center_d_cm if (80 <= ste <= 100) else d_cm
        commands.append(f"ARC D=-{d:.1f} STE={ste} V={v_gear}")

    return commands


# ═══════════════════════════════════════════════════════════════════════════════
# 报告输出
# ═══════════════════════════════════════════════════════════════════════════════

def grade_mark(grade: str) -> str:
    return {"A": "[A]", "B": "[B]", "C": "[C]", "D": "[D]"}.get(grade, "[?]")


def print_probe_table(results: list[ProbeResult]) -> None:
    """打印探针结果表."""
    print(f"\n{'STE':>4s} {'Command':<25s} {'ΔYaw°':>8s} {'ΔD cm':>7s} "
          f"{'deg/cm':>9s} {'σ°':>7s} {'SNR':>7s} {'Grd':>4s} Status")
    print(f"{'─'*4} {'─'*25} {'─'*8} {'─'*7} {'─'*9} {'─'*7} {'─'*7} {'─'*4} {'─'*12}")

    for r in results:
        if r.dry_run if hasattr(r, 'dry_run') else False:
            continue
        ste_str = f"{r.ste:4d}" if r.ste is not None else "  ? "
        dyaw_str = f"{r.delta_yaw_deg:+.2f}" if r.delta_yaw_deg is not None else "N/A"
        dd_str = f"{r.delta_d_cm:.1f}" if r.delta_d_cm is not None else "N/A"
        dpc_str = f"{r.deg_per_cm:+.4f}" if r.deg_per_cm is not None else "N/A"

        if r.err_received:
            status = "❌ ERR"
        elif not r.done_received:
            status = "⏱ TIMEOUT"
        elif r.imu_grade in ("A", "B"):
            status = "✅"
        elif r.imu_grade == "C":
            status = "⚠️ 边缘"
        else:
            status = "❌ SNR不足"

        print(f"  {ste_str} {r.command:<25s} {dyaw_str:>8s} {dd_str:>7s} "
              f"{dpc_str:>9s} {r.imu_sigma_deg:7.4f} {r.imu_snr:7.1f} "
              f"{grade_mark(r.imu_grade):>4s} {status}")


def print_summary(results: list[ProbeResult]) -> None:
    """打印汇总分析."""
    valid = [r for r in results if r.delta_yaw_deg is not None
             and r.delta_d_cm is not None and r.done_received and not r.err_received]

    if not valid:
        print("\n⚠️ 无有效探针数据,无法生成汇总。")
        return

    print(f"\n{'='*72}")
    print(f"  IMU重采样汇总")
    print(f"{'='*72}")
    print(f"  总探针: {len(results)}  有效: {len(valid)}   "
          f"超时/错误: {len(results) - len(valid)}")

    # 按STE分组
    by_ste: dict[int, list[ProbeResult]] = {}
    for r in valid:
        if r.ste is not None:
            by_ste.setdefault(r.ste, []).append(r)

    # 每档统计
    print(f"\n  {'STE':>4s} {'n':>3s} {'deg/cm mean':>12s} {'±std':>9s} "
          f"{'SNR avg':>8s} {'方向':>4s} {'评定':<20s}")
    print(f"  {'─'*4} {'─'*3} {'─'*12} {'─'*9} {'─'*8} {'─'*4} {'─'*20}")

    for ste in sorted(by_ste.keys()):
        items = by_ste[ste]
        n = len(items)
        dpc_vals = [r.deg_per_cm for r in items if r.deg_per_cm is not None]
        snr_vals = [r.imu_snr for r in items]

        dpc_mean = mean(dpc_vals) if dpc_vals else 0.0
        dpc_std = pstdev(dpc_vals) if len(dpc_vals) > 1 else 0.0
        snr_avg = mean(snr_vals) if snr_vals else 0.0
        direction = "左转" if dpc_mean < 0 else "右转"

        # 评定
        cv = abs(dpc_std / dpc_mean) if abs(dpc_mean) > 1e-9 else 0.0
        if n >= 4 and cv < 0.05 and snr_avg > 15:
            grade = "✅ PASS — 高置信度"
        elif n >= 2 and cv < 0.10 and snr_avg > 10:
            grade = "✅ PASS — 可用"
        elif n >= 2:
            grade = f"⚠️ 补充采样 (CV={cv*100:.1f}%)"
        else:
            grade = "⚠️ 单样本,需重复"

        print(f"  {ste:4d} {n:3d} {dpc_mean:+12.6f} {dpc_std:9.6f} "
              f"{snr_avg:8.1f} {direction:>4s} {grade:<20s}")

    # 不对称性
    print(f"\n  ── 不对称性分析 (左侧|deg/cm| / 右侧|deg/cm|) ──")
    pairs = [(55, 125), (60, 120), (65, 115), (70, 110), (75, 105), (80, 100), (85, 95)]
    for left, right in pairs:
        l_items = by_ste.get(left, [])
        r_items = by_ste.get(right, [])
        if l_items and r_items:
            l_abs = abs(mean([r.deg_per_cm for r in l_items if r.deg_per_cm]))
            r_abs = abs(mean([r.deg_per_cm for r in r_items if r.deg_per_cm]))
            if r_abs > 0.001:
                ratio = l_abs / r_abs
                flag = "⚠️ 不对称" if ratio < 0.5 or ratio > 2.0 else ""
                print(f"    STE={left}/{right}: {ratio:.3f} {flag}")
            else:
                print(f"    STE={left}/{right}: ratio=∞ (右侧deg/cm≈0)")

    # 中位分析 (如果扫描了B1区域)
    near_center = {ste: items for ste, items in by_ste.items() if 80 <= ste <= 100}
    if near_center:
        print(f"\n  ── 中位分析 ──")
        best_ste = min(near_center.items(),
                       key=lambda x: abs(mean([r.deg_per_cm for r in x[1] if r.deg_per_cm])))
        best_dpc = mean([r.deg_per_cm for r in best_ste[1] if r.deg_per_cm])
        print(f"  最优中位: STE={best_ste[0]} (deg/cm={best_dpc:+.4f}, "
              f"即 |deg/cm|={abs(best_dpc):.4f})")

        # 抛物线拟合 (粗略)
        stes = sorted(near_center.keys())
        dpcs = [abs(mean([r.deg_per_cm for r in near_center[s] if r.deg_per_cm]))
                for s in stes]
        min_idx = dpcs.index(min(dpcs))
        print(f"  |deg/cm|最小点: STE={stes[min_idx]} (扫描范围 {stes[0]}~{stes[-1]})")

    # IMU评级分布
    all_grades = [r.imu_grade for r in valid]
    print(f"\n  ── IMU质量 ──")
    print(f"  SNR范围: {min(r.imu_snr for r in valid):.0f} ~ {max(r.imu_snr for r in valid):.0f}")
    print(f"  Grade: A={all_grades.count('A')} B={all_grades.count('B')} "
          f"C={all_grades.count('C')} D={all_grades.count('D')}")

    bad = sum(1 for r in valid if r.imu_grade in ("C", "D"))
    if bad == 0:
        print(f"\n  ✅ 所有探针IMU SNR评级A/B,数据可信。")
    else:
        print(f"\n  ⚠️ {bad}个探针IMU SNR偏低,建议增大命令距离或增加重复。")


# ═══════════════════════════════════════════════════════════════════════════════
# JSONL日志
# ═══════════════════════════════════════════════════════════════════════════════

def probe_to_dict(r: ProbeResult, probe_index: int) -> dict:
    """将ProbeResult转为可JSON序列化的dict."""
    return {
        "probe_index": probe_index,
        "timestamp": r.timestamp,
        "command": r.command,
        "ste": r.ste,
        "seq": r.seq,
        "stat_before": {
            "yaw": r.yaw_before, "d": r.d_before,
            "x": r.x_before, "y": r.y_before,
        },
        "stat_after": {
            "yaw": r.yaw_after, "d": r.d_after,
            "x": r.x_after, "y": r.y_after,
        },
        "deltas": {
            "yaw_deg": r.delta_yaw_deg,
            "d_cm": r.delta_d_cm,
            "x_cm": r.delta_x_cm,
            "y_cm": r.delta_y_cm,
            "deg_per_cm": r.deg_per_cm,
        },
        "imu_quality": {
            "duration_est_s": r.duration_est_s,
            "sigma_deg": r.imu_sigma_deg,
            "snr": r.imu_snr,
            "grade": r.imu_grade,
        },
        "done": {
            "received": r.done_received,
            "d_cm": r.done_d_cm,
            "yaw_deg": r.done_yaw_deg,
        },
        "err": r.err_detail if r.err_received else None,
        "raw_response_lines": r.raw_response_lines,
    }


def save_jsonl(results: list[ProbeResult], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        for i, r in enumerate(results, 1):
            d = probe_to_dict(r, i)
            json.dump(d, fh, ensure_ascii=False)
            fh.write("\n")
    print(f"\n探针日志已保存: {path} ({len(results)} 条记录)")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> int:
    parser = argparse.ArgumentParser(
        description="IMU底盘重采样验证 — 直接连接小车采集纯IMU数据,验证打表效果.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # 连接方式
    conn = parser.add_mutually_exclusive_group()
    conn.add_argument("--port", help="串口号 (例如 COM7)")
    conn.add_argument("--host", help="SSH主机IP (Linux SBC)")
    conn.add_argument("--scan", action="store_true", help="扫描可用串口并退出")

    parser.add_argument("--baud", type=int, default=9600, help="波特率 (默认9600)")
    parser.add_argument("--user", default="root", help="SSH用户名")
    parser.add_argument("--password", default="ebaina", help="SSH密码")

    # 探针选择
    probe_group = parser.add_argument_group("探针选择")
    probe_group.add_argument("--ste", type=str,
                             help="舵机角度,逗号分隔 (例如: 60,75,92,105,120)")
    probe_group.add_argument("--scan-b1", action="store_true",
                             help="B1中位精扫 STE=86~98 步长1°")
    probe_group.add_argument("--scan-b2", action="store_true",
                             help="B2全曲率扫描 STE=55~125 步长5°")

    # 命令参数
    cmd_group = parser.add_argument_group("命令参数")
    cmd_group.add_argument("--d-cm", type=float, default=6.0,
                           help="ARC命令距离D (默认6.0cm)")
    cmd_group.add_argument("--near-d-cm", type=float, default=8.0,
                           help="近中位ARC命令距离D (默认8.0cm, 80<=STE<=100)")
    cmd_group.add_argument("--v-gear", type=int, default=1,
                           help="速度档位 (默认1)")
    cmd_group.add_argument("--timeout", type=float, default=15.0,
                           help="单探针超时/秒 (默认15)")

    # 输出
    out_group = parser.add_argument_group("输出")
    out_group.add_argument("--probe-log", type=Path,
                           default=ROOT / "data" / "imu_resample_probes.jsonl",
                           help="探针JSONL日志路径")
    out_group.add_argument("--dry-run", action="store_true",
                           help="仅预览命令,不实际发送")

    args = parser.parse_args()

    # --scan
    if args.scan:
        print("扫描串口...")
        ports = scan_serial_ports()
        if not ports:
            print("未发现串口。")
            return 1
        auto = detect_stm32_port()
        print(f"\n{'Device':<12s} {'Description':<40s} HWID")
        print(f"{'─'*12} {'─'*40} {'─'*30}")
        for p in ports:
            marker = " ← 疑似STM32" if p["device"] == auto else ""
            print(f"{p['device']:<12s} {p['description'][:40]:<40s} {p['hwid'][:30]}{marker}")
        if auto:
            print(f"\n推荐: --port {auto}")
        return 0

    # 构建探针列表
    ste_list = [int(x.strip()) for x in args.ste.split(",")] if args.ste else None
    commands = build_probe_list(
        ste_list=ste_list,
        scan_b1=args.scan_b1,
        scan_b2=args.scan_b2,
        d_cm=args.d_cm,
        near_center_d_cm=args.near_d_cm,
        v_gear=args.v_gear,
    )

    if not commands:
        print("请指定探针: --ste 60,75,92,105,120 或 --scan-b1/--scan-b2")
        print("或先扫描串口: --scan")
        return 1

    # 预览
    print(f"\n{'='*72}")
    print(f"  IMU重采样验证 — {'DRY-RUN预览' if args.dry_run else '实车模式'}")
    print(f"{'='*72}")
    print(f"  探针数: {len(commands)}")
    if args.dry_run:
        print(f"  连接: (dry-run,无需连接)")
    elif args.port:
        print(f"  连接: 串口 {args.port} @ {args.baud} baud")
    elif args.host:
        print(f"  连接: SSH {args.user}@{args.host}")
    else:
        print(f"  连接: 未指定 (请使用 --port COMx 或 --host IP)")
    print(f"\n  命令预览:")
    for i, cmd in enumerate(commands, 1):
        print(f"    {i:3d}. {cmd}")

    if args.dry_run:
        print(f"\n  DRY-RUN完成,未发送任何命令。")
        print(f"  去掉 --dry-run 即可实际执行。")
        return 0

    # 安全检查
    print(f"\n  ⚠️  车辆即将移动! 请确认:")
    print(f"    1. 车辆在平坦地面上,周围有 ≥1m 空间")
    print(f"    2. USB/蓝牙连接稳定")
    print(f"    3. 可随时 Ctrl+C 中断")
    print(f"    4. 紧急: 抬起车辆或断开电源")

    try:
        resp = input(f"\n  按 Enter 开始执行 {len(commands)} 个探针, 或输入 'q' 取消: ")
    except (KeyboardInterrupt, EOFError):
        print("\n  已取消。")
        return 0

    if resp.lower() == 'q':
        print("  已取消。")
        return 0

    # 建立连接
    print(f"\n── 连接 ──")
    try:
        if args.port:
            session = SerialSession(args.port, args.baud)
            session_type = "serial"
        else:
            session = SSHSession(args.host, args.user, args.password)
            session_type = "ssh"
        session.open()
        print(f"  ✅ 已连接 ({session_type})")
    except Exception as e:
        print(f"  ❌ 连接失败: {e}")
        return 1

    # 执行探针
    results: list[ProbeResult] = []
    seq_base = int(time.time()) % 9000 + 1000

    try:
        # Step 0: 初始化
        print(f"\n── 初始化 ──")
        r_stop = execute_one_probe(session, "STOP", seq_base, timeout_s=5.0)
        print(f"  STOP: {'✅' if not r_stop.err_received else '⚠️'}")
        time.sleep(0.3)

        seq_base += 1
        r_zero = execute_one_probe(session, "ZERO_ALL", seq_base, timeout_s=5.0)
        print(f"  ZERO_ALL: {'✅' if not r_zero.err_received else '⚠️'}")
        time.sleep(0.3)

        # Step 1~N: 探针序列
        print(f"\n── 探针执行 ({len(commands)}个) ──")
        for i, cmd in enumerate(commands, 1):
            seq = seq_base + i
            print(f"  [{i}/{len(commands)}] {cmd} ...", end=" ", flush=True)
            r = execute_one_probe(session, cmd, seq, timeout_s=args.timeout)
            results.append(r)

            if r.err_received:
                print(f"❌ ERR: {r.err_detail[:60]}")
            elif not r.done_received:
                print(f"⏱ TIMEOUT")
            else:
                dyaw = r.delta_yaw_deg or 0.0
                dd = r.delta_d_cm or 0.0
                print(f"Δyaw={dyaw:+.1f}° ΔD={dd:.1f}cm "
                      f"SNR={r.imu_snr:.0f} {grade_mark(r.imu_grade)}")

            # 探针间隔
            if i < len(commands):
                time.sleep(0.3)

        # 结束: STOP
        execute_one_probe(session, "STOP", seq_base + len(commands) + 1, timeout_s=3.0)

    except KeyboardInterrupt:
        print(f"\n\n⚠️ 用户中断! 已执行 {len(results)}/{len(commands)} 个探针。")
        try:
            execute_one_probe(session, "STOP", 9999, timeout_s=3.0)
            print("  已发送 STOP。")
        except Exception:
            pass
    finally:
        session.close()
        print(f"  连接已关闭。")

    # 输出报告
    if results:
        print_probe_table(results)

        real_results = [r for r in results
                       if r.done_received and not r.err_received
                       and r.delta_yaw_deg is not None]
        if real_results:
            print_summary(real_results)

        # 保存日志
        log_path = args.probe_log
        if not log_path.is_absolute():
            log_path = ROOT / log_path
        save_jsonl(results, log_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
