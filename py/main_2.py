#!/usr/bin/env python3
"""
STM32 Car PID Auto-Tuning Serial Tool
======================================
通过串口读取STM32小车数据(角速度/航向角/舵机角度/里程计X/Y),
自动计算并调整航向保持PID的Kp,Ki,Kd参数,通过串口发回STM32。

CLI调用示例 (Claude Code可直接执行):
  python main_2.py --monitor                              # 仅监视串口数据
  python main_2.py --auto-tune --duration 60              # 自动调谐60秒
  python main_2.py --auto-tune --target-yaw 0 --duration 120  # 以0度为目标调谐
  python main_2.py --set kp=2.5 ki=0.01 kd=0.18          # 手动设置PID参数
  python main_2.py --gui                                  # 启动GUI可视化界面

STM32数据格式 (USART3发送, 9600bps):
  GyroX_dps, Yaw_deg, ServoAngle_deg, Speed, OdomX_cm, OdomY_cm

发送到STM32的命令格式:
  @ST_KP 2.5\r\n     → 设置航向Kp
  @ST_KI 0.01\r\n    → 设置航向Ki
  @ST_KD 0.18\r\n    → 设置航向Kd
"""

import sys
import time
import json
import argparse
import threading
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

import serial
import serial.tools.list_ports

# ======================
# 默认配置
# ======================
DEFAULT_SERIAL_PORT = "COM7"
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 0.1

# 数据窗口大小 (采样点数)
WINDOW_SIZE = 200

# PID参数限幅
KP_MIN, KP_MAX = 0.1, 20.0
KI_MIN, KI_MAX = 0.001, 2.0
KD_MIN, KD_MAX = 0.01, 5.0

# 调整步长比例 (每次调整不超过当前值的此比例)
ADJUST_RATIO = 0.15


# ======================
# 数据结构
# ======================
@dataclass
class CarState:
    """单帧STM32发来的小车状态"""
    timestamp: float = 0.0
    gyro_z_dps: float = 0.0       # 角速度 (度/秒)
    yaw_deg: float = 0.0           # 航向角 (度)
    servo_angle_deg: float = 0.0   # 舵机角度 (度)
    speed: float = 0.0             # 平均速度
    odom_x_cm: float = 0.0         # 里程计X (cm)
    odom_y_cm: float = 0.0         # 里程计Y (cm)
    kp: float = 0.0                # STM32当前Kp (9字段格式)
    ki: float = 0.0                # STM32当前Ki
    kd: float = 0.0                # STM32当前Kd


@dataclass
class PIDParams:
    """PID参数"""
    kp: float = 2.5
    ki: float = 0.01
    kd: float = 0.18

    def clamp(self):
        self.kp = max(KP_MIN, min(KP_MAX, self.kp))
        self.ki = max(KI_MIN, min(KI_MAX, self.ki))
        self.kd = max(KD_MIN, min(KD_MAX, self.kd))
        return self


@dataclass
class TuningMetrics:
    """调谐分析指标"""
    mean_abs_error: float = 0.0       # 平均绝对航向误差
    oscillation_count: int = 0         # 过零次数 (振荡指标)
    steady_error: float = 0.0          # 稳态误差
    overshoot_ratio: float = 0.0       # 超调比例
    settling_time_samples: int = 0     # 稳定时间 (采样点)
    gyro_variance: float = 0.0         # 角速度方差


# ======================
# 串口通信类
# ======================
class SerialLink:
    """串口读写封装"""

    def __init__(self, port: str = DEFAULT_SERIAL_PORT, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._rx_buffer = ""
        self._lock = threading.Lock()

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=DEFAULT_TIMEOUT,
            )
            print(f"[串口] {self.port} 已打开 @ {self.baudrate}bps")
            return True
        except Exception as e:
            print(f"[串口] 打开失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[串口] {self.port} 已关闭")

    def read_line(self) -> Optional[str]:
        """非阻塞读取一行"""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            with self._lock:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting).decode("utf-8", errors="ignore")
                    self._rx_buffer += chunk
                if "\n" in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                    return line.strip()
        except Exception:
            pass
        return None

    def send_command(self, cmd: str) -> bool:
        """发送@命令到STM32, 格式: @CMD\r\n"""
        if not self.ser or not self.ser.is_open:
            print("[串口] 未打开,无法发送")
            return False
        try:
            full_cmd = f"@{cmd}\r\n"
            self.ser.write(full_cmd.encode("utf-8"))
            print(f"[串口TX] {cmd}")
            return True
        except Exception as e:
            print(f"[串口] 发送失败: {e}")
            return False

    def send_pid_params(self, kp: float, ki: float, kd: float) -> bool:
        """发送KP/KI/KD三条指令, 格式: ST_KPxxx (xxx=值*100, 3位整数零填充)"""
        ok = True
        # 值×100取整, 限幅3位 (max 9.99, 与C端3-digit解析一致)
        kp_int = max(0, min(999, int(round(kp * 100))))
        ki_int = max(0, min(999, int(round(ki * 100))))
        kd_int = max(0, min(999, int(round(kd * 100))))
        ok &= self.send_command(f"ST_KP{kp_int:03d}")
        time.sleep(0.05)
        ok &= self.send_command(f"ST_KI{ki_int:03d}")
        time.sleep(0.05)
        ok &= self.send_command(f"ST_KD{kd_int:03d}")
        return ok


# ======================
# 数据解析器
# ======================
class DataParser:
    """解析STM32串口CSV数据行"""

    @staticmethod
    def parse(line: str) -> Optional[CarState]:
        """
        解析格式 (6字段旧版): GyroX_dps, Yaw_deg, ServoAngle_deg, Speed, OdomX_cm, OdomY_cm
        解析格式 (9字段新版): GyroX_dps, Yaw_deg, ServoAngle_deg, Speed, OdomX_cm, OdomY_cm, Kp, Ki, Kd
        返回 CarState 或 None (解析失败时)
        """
        try:
            parts = line.split(",")
            if len(parts) < 6:
                return None
            state = CarState(
                timestamp=time.time(),
                gyro_z_dps=float(parts[0].strip()),
                yaw_deg=float(parts[1].strip()),
                servo_angle_deg=float(parts[2].strip()),
                speed=float(parts[3].strip()),
                odom_x_cm=float(parts[4].strip()),
                odom_y_cm=float(parts[5].strip()),
            )
            # 9字段格式: 后3个是STM32当前Kp/Ki/Kd
            if len(parts) >= 9:
                state.kp = float(parts[6].strip())
                state.ki = float(parts[7].strip())
                state.kd = float(parts[8].strip())
            return state
        except (ValueError, IndexError):
            return None


# ======================
# PID 自动调谐器
# ======================
class PIDAutoTuner:
    """
    基于实时数据的PID自动调谐器。

    算法:
    1. 收集时间窗口内的航向误差和角速度数据
    2. 分析: 振荡程度、稳态误差、超调量、响应速度
    3. 根据启发式规则调整Kp/Ki/Kd
    4. 逐步逼近最优参数

    调谐规则:
    - 振荡过多 (角速度过零频繁) → 降低Kp 或 增加Kd
    - 稳态误差大 (误差均值偏离0) → 增加Ki
    - 超调大 (误差符号翻转 + 峰值大) → 降低Kp, 增加Kd
    - 响应慢 (误差持续同号) → 增加Kp
    """

    def __init__(self, target_yaw: float = 0.0, window_size: int = WINDOW_SIZE):
        self.target_yaw = target_yaw
        self.window_size = window_size
        self.params = PIDParams()
        self.history: deque[CarState] = deque(maxlen=window_size)
        self.error_history: deque[float] = deque(maxlen=window_size)
        self.adjust_count = 0
        self.adjust_log: list[dict] = []
        self._params_synced = False

    def feed(self, state: CarState):
        """输入一帧新数据, 首次读到STM32真实Kp/Ki/Kd时自动同步"""
        if not self._params_synced and state.kp > 0:
            self.params.kp = state.kp
            self.params.ki = state.ki
            self.params.kd = state.kd
            self._params_synced = True
        self.history.append(state)
        error = self._normalize_angle(state.yaw_deg - self.target_yaw)
        self.error_history.append(error)

    def analyze(self) -> TuningMetrics:
        """分析当前数据窗口,返回调谐指标"""
        if len(self.error_history) < 10:
            return TuningMetrics()

        errors = list(self.error_history)
        gyro_vals = [s.gyro_z_dps for s in self.history]

        # 平均绝对误差
        mean_abs = sum(abs(e) for e in errors) / len(errors)

        # 振荡计数: 角速度过零次数
        zero_cross = 0
        for i in range(1, len(gyro_vals)):
            if gyro_vals[i] * gyro_vals[i - 1] < 0:
                zero_cross += 1

        # 稳态误差: 窗口末尾1/4数据的平均误差
        tail_n = max(1, len(errors) // 4)
        steady_err = sum(errors[-tail_n:]) / tail_n

        # 超调比例: 误差符号翻转次数 / 窗口长度
        sign_flips = 0
        for i in range(1, len(errors)):
            if errors[i] * errors[i - 1] < 0 and abs(errors[i]) > 0.5:
                sign_flips += 1
        overshoot = sign_flips / max(1, len(errors))

        # 角速度方差
        if len(gyro_vals) > 1:
            mean_g = sum(gyro_vals) / len(gyro_vals)
            gyro_var = sum((g - mean_g) ** 2 for g in gyro_vals) / len(gyro_vals)
        else:
            gyro_var = 0.0

        # 稳定时间: 从窗口开头到误差首次进入±死区并保持的采样数
        deadband = 1.0  # 度
        settle = 0
        in_deadband = False
        for i, e in enumerate(errors):
            if abs(e) < deadband:
                if not in_deadband:
                    in_deadband = True
                    settle = i
            else:
                in_deadband = False
                settle = 0
        settling = settle if in_deadband else len(errors)

        return TuningMetrics(
            mean_abs_error=mean_abs,
            oscillation_count=zero_cross,
            steady_error=steady_err,
            overshoot_ratio=overshoot,
            settling_time_samples=settling,
            gyro_variance=gyro_var,
        )

    def compute_adjustment(self, metrics: TuningMetrics) -> Tuple[float, float, float]:
        """
        根据指标计算Kp/Ki/Kd的调整系数。
        返回 (kp_factor, ki_factor, kd_factor), 1.0表示不变。
        """
        kp_f, ki_f, kd_f = 1.0, 1.0, 1.0

        osc = metrics.oscillation_count
        n = max(1, len(self.error_history))
        osc_rate = osc / n  # 振荡密度 (0~1)

        # 规则1: 振荡过多 → 降Kp, 升Kd
        if osc_rate > 0.3:
            kp_f = 0.85
            kd_f = 1.2
        elif osc_rate > 0.15:
            kp_f = 0.92
            kd_f = 1.1

        # 规则2: 稳态误差大 → 升Ki
        if abs(metrics.steady_error) > 3.0:
            ki_f = 1.2
        elif abs(metrics.steady_error) > 1.5:
            ki_f = 1.1
        elif abs(metrics.steady_error) < 0.3:
            ki_f = 0.95  # 误差小时微降Ki避免积分饱和

        # 规则3: 超调大 → 降Kp, 升Kd
        if metrics.overshoot_ratio > 0.2:
            kp_f = min(kp_f, 0.88)
            kd_f = max(kd_f, 1.15)

        # 规则4: 响应慢(误差大但不振荡) → 升Kp
        if metrics.mean_abs_error > 5.0 and osc_rate < 0.1:
            kp_f = max(kp_f, 1.15)

        # 规则5: 角速度方差大 → 升Kd (抑制抖动)
        if metrics.gyro_variance > 50.0:
            kd_f = max(kd_f, 1.15)

        # 限制单次调整幅度
        kp_f = max(0.7, min(1.3, kp_f))
        ki_f = max(0.7, min(1.3, ki_f))
        kd_f = max(0.7, min(1.3, kd_f))

        return kp_f, ki_f, kd_f

    def step(self) -> Optional[PIDParams]:
        """
        执行一次调谐步骤:
        1. 分析当前数据
        2. 计算调整量
        3. 如果调整量足够大则更新参数并返回新值
        4. 否则返回None (表示无需调整)
        """
        if len(self.history) < self.window_size // 2:
            return None  # 数据不足

        metrics = self.analyze()
        kp_f, ki_f, kd_f = self.compute_adjustment(metrics)

        # 检查是否需要调整 (任一系数偏离1.0超过3%)
        if abs(kp_f - 1.0) < 0.03 and abs(ki_f - 1.0) < 0.03 and abs(kd_f - 1.0) < 0.03:
            return None

        # 应用调整
        old_params = PIDParams(self.params.kp, self.params.ki, self.params.kd)
        self.params.kp *= kp_f
        self.params.ki *= ki_f
        self.params.kd *= kd_f
        self.params.clamp()

        self.adjust_count += 1
        log_entry = {
            "step": self.adjust_count,
            "time": time.time(),
            "old": asdict(old_params),
            "new": asdict(self.params),
            "factors": {"kp": round(kp_f, 4), "ki": round(ki_f, 4), "kd": round(kd_f, 4)},
            "metrics": asdict(metrics),
        }
        self.adjust_log.append(log_entry)

        # 调整后清空历史,等待系统响应新参数
        self.history.clear()
        self.error_history.clear()

        return self.params

    @staticmethod
    def _normalize_angle(angle_deg: float) -> float:
        """将角度归一化到 [-180, 180]"""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg


# ======================
# 数据记录器
# ======================
class DataLogger:
    """将接收到的数据记录到CSV文件"""

    def __init__(self, filepath: Optional[str] = None):
        self.filepath = filepath
        self._file = None

    def open(self):
        if self.filepath:
            self._file = open(self.filepath, "w", encoding="utf-8")
            self._file.write("timestamp,gyro_z_dps,yaw_deg,servo_angle_deg,speed,odom_x_cm,odom_y_cm\n")

    def log(self, state: CarState):
        if self._file:
            self._file.write(
                f"{state.timestamp:.3f},{state.gyro_z_dps:.2f},{state.yaw_deg:.2f},"
                f"{state.servo_angle_deg:.2f},{state.speed:.2f},"
                f"{state.odom_x_cm:.2f},{state.odom_y_cm:.2f}\n"
            )

    def close(self):
        if self._file:
            self._file.close()
            self._file = None


# ======================
# 监视模式
# ======================
def run_monitor(link: SerialLink, output_csv: Optional[str] = None, print_raw: bool = True):
    """监视模式: 持续读取并打印串口数据"""
    logger = DataLogger(output_csv)
    logger.open()

    print(f"\n{'='*60}")
    print(f"监视模式 - 按 Ctrl+C 退出")
    print(f"{'='*60}")
    print(f"{'时间':>10s} | {'角速度':>8s} | {'航向角':>8s} | {'舵机角':>8s} | {'速度':>8s} | {'里程X':>8s} | {'里程Y':>8s}")
    print(f"{'':>10s} | {'dps':>8s} | {'deg':>8s} | {'deg':>8s} | {'':>8s} | {'cm':>8s} | {'cm':>8s}")
    print("-" * 60)

    try:
        while True:
            line = link.read_line()
            if line:
                state = DataParser.parse(line)
                if state:
                    logger.log(state)
                    if print_raw:
                        print(
                            f"{state.timestamp % 1000:10.1f} | "
                            f"{state.gyro_z_dps:8.2f} | "
                            f"{state.yaw_deg:8.2f} | "
                            f"{state.servo_angle_deg:8.2f} | "
                            f"{state.speed:8.2f} | "
                            f"{state.odom_x_cm:8.2f} | "
                            f"{state.odom_y_cm:8.2f}"
                        )
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[监视] 用户中断")
    finally:
        logger.close()


# ======================
# 自动调谐模式
# ======================
def run_auto_tune(
    link: SerialLink,
    target_yaw: float = 0.0,
    duration: float = 60.0,
    adjust_interval: float = 3.0,
    output_csv: Optional[str] = None,
):
    """
    自动调谐模式: 持续读取数据,定期分析并调整PID参数。

    参数:
        link: 串口连接
        target_yaw: 目标航向角 (度)
        duration: 总运行时间 (秒), 0=无限
        adjust_interval: 调整间隔 (秒), 让系统在新参数下稳定后再分析
        output_csv: 可选的CSV输出路径
    """
    logger = DataLogger(output_csv)
    logger.open()

    tuner = PIDAutoTuner(target_yaw=target_yaw)

    print(f"\n{'='*60}")
    print(f"PID 自动调谐模式")
    print(f"{'='*60}")
    print(f"  目标航向角: {target_yaw}°")
    print(f"  调谐时长:   {duration}秒" if duration > 0 else "  调谐时长:   无限")
    print(f"  调整间隔:   {adjust_interval}秒")
    print(f"{'='*60}")

    # === 第1步: 进入直行模式, 激活航向PID ===
    print("[调谐] 发送 DT_STA 进入直行模式...")
    link.send_command("DT_STA")
    time.sleep(0.3)

    # === 第2步: 读取STM32当前PID参数 ===
    print("[调谐] 等待读取STM32当前PID参数...")
    for _ in range(50):
        line = link.read_line()
        if line:
            state = DataParser.parse(line)
            if state and state.kp > 0:
                tuner.params.kp = state.kp
                tuner.params.ki = state.ki
                tuner.params.kd = state.kd
                tuner._params_synced = True
                break
        time.sleep(0.02)

    print(f"  当前STM32参数: Kp={tuner.params.kp:.4f}, Ki={tuner.params.ki:.4f}, Kd={tuner.params.kd:.4f}")

    # === 第3步: 加速启动小车 ===
    print("[调谐] 发送 SR_ACC 加速启动小车...")
    link.send_command("SR_ACC")
    time.sleep(0.3)
    link.send_command("SR_ACC")  # 再加速一档, 确保有明显速度
    time.sleep(0.3)

    # 等待小车实际跑起来 (speed > 0)
    print("[调谐] 等待小车达到行驶速度...")
    moving = False
    for _ in range(100):  # 最多等2秒
        line = link.read_line()
        if line:
            state = DataParser.parse(line)
            if state and state.speed > 0:
                print(f"  小车已启动, 速度={state.speed:.1f}")
                moving = True
                break
        time.sleep(0.02)
    if not moving:
        print("  [警告] 未检测到速度, 继续调谐(可能车未响应)")

    print(f"{'='*60}\n")

    start_time = time.time()
    last_adjust_time = start_time
    frame_count = 0

    try:
        while True:
            # 检查是否超时
            elapsed = time.time() - start_time
            if duration > 0 and elapsed >= duration:
                print(f"\n[调谐] 达到设定时长 {duration}秒, 结束")
                break

            line = link.read_line()
            if line:
                state = DataParser.parse(line)
                if state:
                    frame_count += 1
                    logger.log(state)
                    tuner.feed(state)

            # 定期执行调谐分析
            if time.time() - last_adjust_time >= adjust_interval:
                result = tuner.step()
                if result:
                    print(f"\n--- PID 调整 #{tuner.adjust_count} (t={elapsed:.1f}s) ---")
                    last_entry = tuner.adjust_log[-1]
                    old = last_entry["old"]
                    new = last_entry["new"]
                    m = last_entry["metrics"]
                    print(f"  指标: |误差|均值={m['mean_abs_error']:.2f}°, "
                          f"振荡={m['oscillation_count']}次, "
                          f"稳态误差={m['steady_error']:.2f}°")
                    print(f"  Kp: {old['kp']:.4f} → {new['kp']:.4f}")
                    print(f"  Ki: {old['ki']:.4f} → {new['ki']:.4f}")
                    print(f"  Kd: {old['kd']:.4f} → {new['kd']:.4f}")

                    link.send_pid_params(result.kp, result.ki, result.kd)
                else:
                    # 即使不需要调整也打印状态
                    if frame_count > 0:
                        m = tuner.analyze()
                        print(f"[调谐 t={elapsed:.1f}s] "
                              f"|误差|均值={m.mean_abs_error:.2f}°, "
                              f"振荡密度={m.oscillation_count/max(1,len(tuner.error_history)):.2f}, "
                              f"稳态误差={m.steady_error:.2f}°  "
                              f"(参数未调整)")

                last_adjust_time = time.time()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[调谐] 用户中断")

    finally:
        # 停车
        print("[调谐] 发送 SR_PAU 停车...")
        link.send_command("SR_PAU")
        time.sleep(0.2)
        logger.close()

        # 输出调谐总结
        print(f"\n{'='*60}")
        print(f"调谐总结")
        print(f"{'='*60}")
        print(f"  总帧数:     {frame_count}")
        print(f"  调整次数:   {tuner.adjust_count}")
        print(f"  最终参数:   Kp={tuner.params.kp:.4f}, Ki={tuner.params.ki:.4f}, Kd={tuner.params.kd:.4f}")
        print(f"{'='*60}")

        # 输出JSON格式的调谐日志 (方便Claude Code解析)
        if tuner.adjust_log:
            print(f"\n[JSON日志]")
            print(json.dumps(tuner.adjust_log, indent=2, ensure_ascii=False))


# ======================
# 手动设置模式
# ======================
def run_set_params(link: SerialLink, kp: Optional[float], ki: Optional[float], kd: Optional[float]):
    """手动设置PID参数"""
    params = PIDParams()
    if kp is not None:
        params.kp = kp
    if ki is not None:
        params.ki = ki
    if kd is not None:
        params.kd = kd
    params.clamp()

    print(f"\n[手动设置] Kp={params.kp:.4f}, Ki={params.ki:.4f}, Kd={params.kd:.4f}")
    link.send_pid_params(params.kp, params.ki, params.kd)


# ======================
# GUI 模式 (保留原有可视化功能)
# ======================
def run_gui():
    """启动 PyQtGraph 可视化界面 (需要 PyQt5 和 pyqtgraph)"""
    try:
        import pyqtgraph as pg
        from PyQt5 import QtWidgets, QtCore
    except ImportError as e:
        print(f"[错误] GUI模式需要安装 PyQt5 和 pyqtgraph: pip install pyqtgraph PyQt5")
        print(f"  详细错误: {e}")
        sys.exit(1)

    BUFFER_LEN = 2048

    app = QtWidgets.QApplication(sys.argv)

    class MainWindow(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.ser: Optional[serial.Serial] = None
            self.init_ui()
            self.init_data()
            self.init_serial()

        def init_ui(self):
            self.setWindowTitle("STM32 Car Monitor + PID Auto-Tune")
            layout = QtWidgets.QVBoxLayout()

            # 图表区域 - 6通道
            self.graphics_view = pg.GraphicsLayoutWidget()

            # 角速度和航向角
            self.plot_gyro = self.graphics_view.addPlot(title="Gyro Z (角速度) / Yaw (航向角)")
            self.plot_gyro.addLegend()
            self.curve_gyro = self.plot_gyro.plot(pen="r", name="GyroZ dps")
            self.curve_yaw = self.plot_gyro.plot(pen="b", name="Yaw deg")

            self.graphics_view.nextRow()
            # 舵机角度和速度
            self.plot_servo = self.graphics_view.addPlot(title="Servo Angle / Speed")
            self.plot_servo.addLegend()
            self.curve_servo = self.plot_servo.plot(pen="g", name="Servo deg")
            self.curve_speed = self.plot_servo.plot(pen="y", name="Speed")

            self.graphics_view.nextRow()
            # 里程计轨迹
            self.plot_odom = self.graphics_view.addPlot(title="Odometry Trajectory")
            self.plot_odom.addLegend()
            self.curve_odom = self.plot_odom.plot(pen="w", name="Trajectory")

            layout.addWidget(self.graphics_view)

            # PID参数显示
            pid_group = QtWidgets.QGroupBox("PID 参数")
            pid_layout = QtWidgets.QHBoxLayout()
            self.kp_label = QtWidgets.QLabel("Kp: --")
            self.ki_label = QtWidgets.QLabel("Ki: --")
            self.kd_label = QtWidgets.QLabel("Kd: --")
            pid_layout.addWidget(self.kp_label)
            pid_layout.addWidget(self.ki_label)
            pid_layout.addWidget(self.kd_label)
            pid_group.setLayout(pid_layout)
            layout.addWidget(pid_group)

            # 串口收发控制
            serial_group = QtWidgets.QGroupBox("Serial Command")
            serial_layout = QtWidgets.QHBoxLayout()
            self.send_edit = QtWidgets.QLineEdit()
            self.send_edit.setPlaceholderText("输入命令 (如 ST_KP 2.5)")
            self.send_btn = QtWidgets.QPushButton("发送")
            self.send_btn.clicked.connect(self.send_serial_data)
            serial_layout.addWidget(self.send_edit)
            serial_layout.addWidget(self.send_btn)
            serial_group.setLayout(serial_layout)
            layout.addWidget(serial_group)

            # 状态栏
            self.status_label = QtWidgets.QLabel("Status: Initializing...")
            layout.addWidget(self.status_label)

            self.setLayout(layout)
            self.resize(900, 700)
            self.show()

        def init_data(self):
            self.gyro_data = deque(maxlen=BUFFER_LEN)
            self.yaw_data = deque(maxlen=BUFFER_LEN)
            self.servo_data = deque(maxlen=BUFFER_LEN)
            self.speed_data = deque(maxlen=BUFFER_LEN)
            self.odom_x_data = deque(maxlen=BUFFER_LEN)
            self.odom_y_data = deque(maxlen=BUFFER_LEN)

            self.timer = pg.QtCore.QTimer()
            self.timer.timeout.connect(self.update_plot)
            self.timer.start(20)

        def init_serial(self):
            try:
                self.ser = serial.Serial(
                    port=DEFAULT_SERIAL_PORT,
                    baudrate=DEFAULT_BAUDRATE,
                    timeout=DEFAULT_TIMEOUT,
                )
                self.status_label.setText(f"Serial {DEFAULT_SERIAL_PORT} opened @ {DEFAULT_BAUDRATE}bps")
                self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
                self.serial_thread.start()
            except Exception as e:
                self.status_label.setText(f"Serial error: {e}")
                self.ser = None

        def read_serial(self):
            buffer = ""
            while self.ser and self.ser.is_open:
                try:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        chunk = self.ser.read(waiting).decode("utf-8", errors="ignore")
                        buffer += chunk
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if line:
                                state = DataParser.parse(line)
                                if state:
                                    self.gyro_data.append(state.gyro_z_dps)
                                    self.yaw_data.append(state.yaw_deg)
                                    self.servo_data.append(state.servo_angle_deg)
                                    self.speed_data.append(state.speed)
                                    self.odom_x_data.append(state.odom_x_cm)
                                    self.odom_y_data.append(state.odom_y_cm)
                except Exception:
                    pass
                QtCore.QThread.msleep(5)

        def send_serial_data(self):
            if self.ser and self.ser.is_open:
                text = self.send_edit.text().strip()
                if text:
                    try:
                        self.ser.write((f"@{text}\r\n").encode())
                        print(f"[GUI TX] {text}")
                        self.send_edit.clear()
                    except Exception as e:
                        print(f"Send error: {e}")
            else:
                print("Serial port not available")

        def update_plot(self):
            if self.gyro_data:
                self.curve_gyro.setData(list(self.gyro_data))
                self.curve_yaw.setData(list(self.yaw_data))
            if self.servo_data:
                self.curve_servo.setData(list(self.servo_data))
                self.curve_speed.setData(list(self.speed_data))
            if self.odom_x_data:
                self.curve_odom.setData(list(self.odom_x_data), list(self.odom_y_data))

        def closeEvent(self, event):
            if self.ser and self.ser.is_open:
                self.ser.close()
            event.accept()

    win = MainWindow()
    sys.exit(app.exec_())


# ======================
# 命令行接口
# ======================
def parse_args():
    parser = argparse.ArgumentParser(
        description="STM32 Car PID Auto-Tuning Serial Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python main_2.py --monitor                        # 仅监视串口数据
  python main_2.py --monitor -o data.csv            # 监视并记录到CSV
  python main_2.py --auto-tune                      # 自动调谐(默认60秒)
  python main_2.py --auto-tune --target-yaw 0 --duration 120
  python main_2.py --set kp=2.5 ki=0.01 kd=0.18    # 手动设置PID
  python main_2.py --gui                            # GUI可视化界面
  python main_2.py --list-ports                     # 列出可用串口
        """,
    )

    # 操作模式 (互斥)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--monitor", action="store_true", help="监视模式: 持续读取并打印串口数据")
    mode_group.add_argument("--auto-tune", action="store_true", help="自动调谐模式: 分析数据并自动调整PID")
    mode_group.add_argument("--set", nargs="*", metavar="KEY=VAL", help="手动设置PID参数 (如: kp=2.5 ki=0.01 kd=0.18)")
    mode_group.add_argument("--gui", action="store_true", help="GUI模式: 启动PyQtGraph可视化界面")
    mode_group.add_argument("--list-ports", action="store_true", help="列出可用的串口")

    # 串口配置
    parser.add_argument("--port", "-p", default=DEFAULT_SERIAL_PORT, help=f"串口号 (默认: {DEFAULT_SERIAL_PORT})")
    parser.add_argument("--baudrate", "-b", type=int, default=DEFAULT_BAUDRATE, help=f"波特率 (默认: {DEFAULT_BAUDRATE})")

    # 自动调谐参数
    parser.add_argument("--target-yaw", type=float, default=0.0, help="目标航向角度 (默认: 0)")
    parser.add_argument("--duration", "-d", type=float, default=60.0, help="调谐持续时间/秒 (0=无限, 默认: 60)")
    parser.add_argument("--adjust-interval", type=float, default=3.0, help="调谐调整间隔/秒 (默认: 3.0)")

    # 输出
    parser.add_argument("--output", "-o", default=None, help="输出CSV文件路径")
    parser.add_argument("--quiet", "-q", action="store_true", help="静默模式 (减少输出)")

    return parser.parse_args()


def list_serial_ports():
    """列出系统可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到串口")
        return
    print(f"\n可用串口 ({len(ports)}个):")
    print("-" * 60)
    for port in ports:
        print(f"  {port.device}  -  {port.description}  [{port.hwid}]")
    print()


def main():
    args = parse_args()

    # 列出串口
    if args.list_ports:
        list_serial_ports()
        return

    # GUI模式不需要连接串口 (GUI自己管理)
    if args.gui:
        run_gui()
        return

    # 打开串口
    link = SerialLink(port=args.port, baudrate=args.baudrate)
    if not link.open():
        sys.exit(1)

    try:
        # 监视模式
        if args.monitor:
            run_monitor(link, output_csv=args.output, print_raw=not args.quiet)

        # 自动调谐模式
        elif args.auto_tune:
            run_auto_tune(
                link,
                target_yaw=args.target_yaw,
                duration=args.duration,
                adjust_interval=args.adjust_interval,
                output_csv=args.output,
            )

        # 手动设置模式
        elif args.set is not None:
            kp = ki = kd = None
            for item in args.set:
                if "=" in item:
                    key, val = item.split("=", 1)
                    try:
                        v = float(val)
                        if key.lower() == "kp":
                            kp = v
                        elif key.lower() == "ki":
                            ki = v
                        elif key.lower() == "kd":
                            kd = v
                    except ValueError:
                        print(f"[警告] 忽略无效参数: {item}")
            if kp is None and ki is None and kd is None:
                print("[错误] 未提供有效的PID参数。用法: --set kp=2.5 ki=0.01 kd=0.18")
            else:
                run_set_params(link, kp, ki, kd)



                

        # 默认: 如果没有指定模式, 进入监视模式
        else:
            print("[提示] 未指定模式, 进入监视模式 (--monitor)")
            run_monitor(link, output_csv=args.output, print_raw=not args.quiet)

    finally:
        link.close()


if __name__ == "__main__":
    main()
