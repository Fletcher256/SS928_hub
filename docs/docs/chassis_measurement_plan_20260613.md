# 底盘数据检测方案 — 2026-06-13

> 基于已验证的打表校准数据（chassis_kinematics.json: 15 PASS / 0 FAIL），
> 设计系统化的底盘参数测量流程，覆盖全部待标定参数。

---

## 检测分类体系

```
底盘检测
├── A. 几何参数检测 (Geometry)
│   ├── A1. 轮距 WHEEL_TRACK_CM
│   ├── A2. 轮周长 WHEEL_C
│   └── A3. 轴距 ACKERMANN_WHEEL_BASE_CM
│
├── B. 舵机参数检测 (Servo)
│   ├── B1. 中位精确扫描 (STE=86~98, 步长1°)
│   ├── B2. 曲率全量程扫描 (STE=55~125, 步长5°)
│   └── B3. 对称性验证 (左右对称档对比)
│
├── C. 运动参数检测 (Motion)
│   ├── C1. MOVE死区/滑行 (同ARC但针对直行)
│   ├── C2. 速度-实际速度标定 (SpeedRank→cm/s)
│   └── C3. 直行寄生偏航 (长距离MOVE偏航累积)
│
├── D. 里程计精度检测 (Odometry)
│   ├── D1. 线性精度 (实测距离 vs Odom.D)
│   ├── D2. 角精度 (IMU yaw vs Odom theta)
│   └── D3. 纯旋转X/Y漂移
│
└── E. 交叉验证 (Cross-Validation)
    ├── E1. 前进/后退对称性
    ├── E2. TLM vs STAT yaw一致性
    └── E3. 多次重复一致性
```

---

## 通用测量协议

每个检测项遵循统一的三阶段协议：

```text
Phase 1: 准备
  1. 车放置于平坦地面，标记起始位置（胶带十字线）
  2. 发 STOP → 确认 STAT MODE=IDLE
  3. 发 ZERO_ALL → 里程计+航向归零
  4. 发 TEL ON → 开启遥测流（可选，仅需中间轨迹时使用）
  5. 记录 STAT_before

Phase 2: 执行
  6. 发 <运动指令>
  7. 等待 DONE 或 ERR（上位机阻塞等待）
  8. 若需要中间轨迹：保存 TLM 流中的所有帧

Phase 3: 记录
  9. 记录 STAT_after（DONE后延迟100ms确保STAT更新）
  10. 发 TEL OFF
  11. 计算:
      Δyaw = YAW_after - YAW_before
      ΔD   = D_after - D_before
      ΔX   = X_after - X_before
      ΔY   = Y_after - Y_before
  12. 写入 JSONL 记录
```

### 单次测量JSONL记录格式

```json
{
  "event": "chassis_calibration_probe",
  "test_id": "B2_steer_65_001",
  "timestamp": "2026-06-13T12:00:00",
  "command": "ARC D=-6.0 STE=65 V=1",
  "stat_before": {"MODE": "IDLE", "YAW": 0.0, "X": 0.0, "Y": 0.0, "D": 0.0, "VEL": 0.0},
  "stat_after":  {"MODE": "IDLE", "YAW": -3.8, "X": 0.5, "Y": -5.2, "D": 5.5, "VEL": 0.0},
  "done": {"D": 4.0},
  "deltas": {
    "yaw_deg": -3.8,
    "dist_stat_cm": 5.5,
    "dist_done_cm": 4.0,
    "x_cm": 0.5,
    "y_cm": -5.2,
    "deadband_cm": 2.0,
    "coast_cm": 1.5,
    "deg_per_cm": -0.691
  },
  "tlm_frames": 12,
  "notes": ""
}
```

---

## A. 几何参数检测

### A1. 轮距标定 (WHEEL_TRACK_CM) — P0

**当前值**: 14.5cm (Motors.h:25, 标记为"需实测标定")

**原理**：里程计模型 `dtheta = (dr - dl) / WHEEL_TRACK_CM`，IMU提供独立Δyaw参考。原地旋转N圈比较两者。

**指令序列**：
```text
STOP → ZERO_ALL → TEL ON
TURN A=720 V=1     ← 旋转2圈（720°）
[等待 DONE] → TEL OFF → STAT
```

**计算公式**：
```text
WHEEL_TRACK_calibrated = WHEEL_TRACK_current × (odom_theta_total / imu_yaw)
```

- 正转2圈 × 3次 + 反转2圈 × 3次 = 6样本

**验收**: CV < 3%, IMU vs odo < 2° per 360°

---

### A2. 轮周长标定 (WHEEL_C) — P0

**当前值**: 21.04867cm (基于6.7cm胎径×π计算，未实测验证)

**原理**：标记精确100cm直线，MOVE后卷尺实测对比里程计D。

**指令序列**：
```text
地面贴胶带标记100cm → 后轮对齐起点
STOP → ZERO_ALL → MOVE D=100 V=1 → [DONE] → 卷尺量实际距离 → STAT
```

**计算公式**：
```text
WHEEL_C_calibrated = WHEEL_C_current × (actual_distance_cm / odom_D_cm)
```

- 前进100cm×5 + 后退100cm×5 = 10样本

**验收**: CV < 2%, 100cm误差 < 1cm

---

### A3. 轴距验证 (ACKERMANN_WHEEL_BASE_CM) — P1

**当前值**: 16.0cm (Motors.h:22)

**发现**：用实测R_eff反算的wheelbase在45~60cm之间，与16cm差异巨大。说明真实底盘不遵循简单Ackermann模型。

```text
STE=60:  wb = 79.9 × tan(32°) = 49.9cm
STE=75:  wb = 147.0 × tan(17°) = 44.9cm
STE=105: wb = 259.7 × tan(13°) = 60.0cm
STE=120: wb = 100.1 × tan(28°) = 53.2cm
```

**方法**：卷尺直接测量前轴中心到后轴中心距离，3次取均值。

**验收**: 物理测量值与代码16.0cm偏差 < 0.5cm

---

## B. 舵机参数检测

### B1. 中位精确扫描 — P0

**已有**: STE=90/92/94, 92最优 (|Δyaw|=0.70°)

**扩展**: 扫描 STE=86~98, 步长1°, 共13点

```text
for ste in [86, 87, ..., 98]:
    STOP → ZERO_ALL → ARC D=-8 STE={ste} V=1 → [DONE] → STAT
    计算 deg_per_cm = |Δyaw| / ΔD
```

**分析**: 绘制 STE vs |deg_per_cm| → 抛物线拟合找极小值

**验收**: 拟合极小值与当前配置(92)偏差 ≤ 1°

---

### B2. 曲率全量程扫描 — P0

**已有**: STE=[60, 75, 105, 120], 4档 (STE=120仅2样本)

**扩展**: 扫描14档, 步长5°

```text
左侧: 55, 60, 65, 70, 75, 80, 85
右侧: 95, 100, 105, 110, 115, 120, 125

每档: ARC D=-6 STE={ste} V=1 → STAT → deg_per_cm
过渡区(80/85/95/100): 用D=-8保证足够yaw变化
```

- 14档 × 2次 = 28次探针

**验收**: 曲率曲线单调递增; 相邻对称档不对称比变化 < 30%

---

### B3. 对称性验证 — P1

利用B2的数据，对关于92对称的档对 (STE_left + STE_right = 184):

```text
不对称比 = |deg_per_cm_left| / |deg_per_cm_right|
已知: 60/120=1.252, 75/105=1.767
新增: 55/125, 65/115, 70/110, 80/100, 85/95
```

**验收**: 不对称比随舵角增大单调变化（物理一致性）

---

## C. 运动参数检测

### C1. MOVE死区/滑行 — P1

```text
for D in [2, 3, 4, 5, 6]:
    STOP → ZERO_ALL → MOVE D=-{D} V=1 → [DONE] → STAT
    deadband = commanded_D - DONE.D
    coast = STAT.D - DONE.D
```

- 5个距离 × 2次 = 10次

**验收**: MOVE deadband < 2.5cm; coast与ARC coast差异 < 0.5cm

---

### C2. 速度标定 (SpeedRank→cm/s) — P1

```text
for V in [1, 2, 3, 4, 5, 6]:
    TEL ON → MOVE D=-15 V={V} → [DONE] → TEL OFF
    从TLM帧: 实际速度 = ΔD / Δt
```

**验收**: 速度与SpeedRank单调递增，无饱和

---

### C3. 直行寄生偏航 — P0

舵机在最优中位STE=92时，机械不对称的残余偏航：

```text
for D in [20, 40, 60]:
    ZERO_ALL → MOVE D=-{D} V=1 → [DONE] → STAT
    deg_per_cm = |Δyaw| / ΔD
```

- 3个距离 × 3次 = 9次

**验收**: STE=92时寄生偏航 < 0.10°/cm (100cm行驶偏航 < 10°)

---

## D. 里程计精度检测

### D1. 线性精度 — P0

```text
地面标记100cm和200cm
ZERO_ALL → MOVE D=100 V=1 → [DONE] → 卷尺量实际位移 → STAT
ZERO_ALL → MOVE D=200 V=1 → [DONE] → 卷尺量实际位移 → STAT
```

- 2距离 × 前/后 × 3次 = 12次

**验收**: 100cm误差 < 2cm (2%), 200cm误差 < 4cm (2%)

---

### D2. 角精度 (IMU vs 里程计) — P2

```text
ZERO_ALL → TEL ON → TURN A=360 V=1 → [DONE] → TEL OFF → STAT
比较: IMU yaw vs 里程计推算theta
```

> 注意：固件STAT当前不输出odom.theta。临时用X/Y推算旋转半径验证。

**验收**: IMU vs odo < 5° per 360°

---

### D3. 纯旋转漂移 — P2

```text
ZERO_ALL → TURN A=720 V=1 → [DONE] → STAT
漂移 = sqrt(X² + Y²), 期望接近0
```

**验收**: 2圈旋转后位置漂移 < 10cm

---

## E. 交叉验证

### E1. 前进/后退对称性 — P2

4个STE (60/75/105/120), 前进/后退各2次 = 16次

**验收**: 前进/后退deg_per_cm差异 < 15%

---

### E2. TLM vs STAT yaw一致性 — P2

利用已有数据检查系统偏差。已知STE=105: TLM/STAT ratio=1.416（TLM高42%）。

**验收**: ratio 0.7~1.3 (超出说明TLM噪声太大，仅依赖STAT)

---

### E3. 重复性验证 — P1

`ARC D=-6 STE=60 V=1` 重复5次

**验收**: deg_per_cm的CV < 5%

---

## 执行计划

| 阶段 | 内容 | 时间 | 探针数 |
|------|------|------|--------|
| 1. 离线 | A3物理测量 + E2数据分析 + B3对称性 | 30min | 0 |
| 2. 舵机 | B1中位扫描 + B2曲率全扫描 | 20min | ~50 |
| 3. 运动 | C1死区 + C2速度 + C3寄生偏航 | 15min | ~25 |
| 4. 里程计 | D1线性 + D3旋转漂移 | 20min | ~18 |
| 5. 交叉 | E1前后对称 + E3重复性 | 15min | ~21 |
| **合计** | | **~100min** | **~114** |

---

## 验收汇总

| # | 检测项 | 通过标准 | 优先级 |
|---|--------|----------|--------|
| A1 | 轮距 | CV<3%, IMU vs odo <2°/360° | P0 |
| A2 | 轮周长 | CV<2%, 100cm误差<1cm | P0 |
| A3 | 轴距 | 实测vs代码<0.5cm | P1 |
| B1 | 中位扫描 | 拟合极小值与92偏差≤1° | P0 |
| B2 | 曲率全扫描 | 单调递增, 14档全覆盖 | P0 |
| B3 | 对称性 | 不对称比趋势合理 | P1 |
| C1 | MOVE死区 | deadband<2.5cm, 与ARC一致 | P1 |
| C2 | 速度标定 | 单调, 无饱和 | P1 |
| C3 | 寄生偏航 | STE=92时<0.10°/cm | P0 |
| D1 | 线性精度 | 100cm误差<2% | P0 |
| D2 | 角精度 | IMU vs odo<5°/360° | P2 |
| D3 | 旋转漂移 | 2圈漂移<10cm | P2 |
| E1 | 前后对称 | 前后差异<15% | P2 |
| E2 | TLM一致性 | ratio 0.7~1.3 | P2 |
| E3 | 重复性 | CV<5% | P1 |
