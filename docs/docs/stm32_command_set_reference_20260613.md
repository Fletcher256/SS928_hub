# STM32 指令集参考手册 v2.0 — 2026-06-13

> 状态：基于固件代码完整审计，已与打表校准数据交叉验证。
> 本文档是 STM32 与上位机之间的**唯一接口契约**。

---

## 一、系统架构总览

```text
┌─────────────────────────────────────────────────────────┐
│  上位机 (board_parking_controller.py)                    │
│    │                                                     │
│    │ UART3 9600bps, 文本协议                              │
│    ▼                                                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │           STM32 CarProtocol Layer                  │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │   │
│  │  │ V2 Proto │  │ RC Proto │  │ Legacy Proto │   │   │
│  │  │(machine) │  │(simple)  │  │ (deprecated) │   │   │
│  │  └────┬─────┘  └────┬─────┘  └──────┬───────┘   │   │
│  │       └──────────────┼───────────────┘            │   │
│  │                      ▼                             │   │
│  │            CarControl (execution engine)           │   │
│  │  ┌──────────────────────────────────────────┐     │   │
│  │  │  ControlMode FSM:                        │     │   │
│  │  │  IDLE → MANUAL → STRAIGHT → DISTANCE     │     │   │
│  │  │                      → TURN_YAW          │     │   │
│  │  │                      → ARC               │     │   │
│  │  │                      → AUTO_ROUTE        │     │   │
│  │  └──────────────────────────────────────────┘     │   │
│  └────────────────────┬──────────────────────────────┘   │
│                       │                                   │
│     ┌─────────────────┼─────────────────┐                │
│     ▼                 ▼                  ▼                │
│  ┌──────┐    ┌──────────────┐    ┌──────────────┐       │
│  │PWMO  │    │   Motors     │    │ BMI270 IMU   │       │
│  │Servo │    │ L+R PID+Odom │    │ AHRS+Yaw     │       │
│  └──────┘    └──────────────┘    └──────────────┘       │
└─────────────────────────────────────────────────────────┘
```

### 关键常数映射

| 常数 | 值 | 来源 | 打表验证 |
|------|-----|------|----------|
| `ACKERMANN_CENTER_DEG` | **90.0** | CarControl.c:64 | ⚠️ 应改为 **92** |
| `ACKERMANN_WHEEL_BASE_CM` | 16.0 | Motors.h:22 | 几何参数 |
| `WHEEL_TRACK_CM` | 14.5 | Motors.h:25 | 需实测标定 |
| `DISTANCE_DONE_CM` | 2.0 | CarControl.h:18 | ✅ 实测 deadband=1.9cm |
| `TURN_DONE_DEG` | 3.0 | CarControl.h:19 | 航向完成阈值 |
| `REMOTE_TIMEOUT_MS` | 2000 | CarControl.h:11 | 手动模式超时 |
| `DISTANCE_TIMEOUT_MS` | 30000 | CarControl.h:12 | 位移动作超时 |
| `TURN_TIMEOUT_MS` | 8000 | CarControl.h:13 | 转向动作超时 |

---

## 二、V2 协议（机器接口，当前主力）

**线路格式**：`[<seq>] <CMD> [<KEY>=<VALUE> ...]`
**响应格式**：`ACK <seq> <CMD>` / `DONE <seq> <CMD> [<extra>]` / `ERR <seq> CODE=<code>`

### 2.1 状态查询类

| 指令 | 参数 | 响应 | 说明 |
|------|------|------|------|
| `PING` | — | `DONE PING PONG` | 存活检测 |
| `VER` | — | `VER FW=SS928-CTRL-2.0 BAUD=9600 PROTO=2` | 固件信息 |
| `STAT` | — | `STAT MODE=<m> RUN=<r> DIR=<d> SPD=<s> ANG=<a> YAW=<y> X=<x> Y=<y> D=<d> VEL=<v> DROP=<n>` | **核心状态帧** |
| `PWM_STAT` | — | `PWM HEALTH=<h> EN=<e> ANG=<a> PULSE=<p> PSC=<ps> ARR=<ar> CCR2=<c> CCER=<cc> RECOV=<r>` | 舵机健康诊断 |
| `GET PARAM=<name>` | HEADING/MOTOR/LIMIT | `PARAM <name> <fields>` | 读取PID/限幅参数 |

#### STAT 字段详解（15 字段，上位机解析入口）

| 字段 | 含义 | 单位 | 关键用法 |
|------|------|------|----------|
| `MODE` | 控制模式 | enum | IDLE/MANUAL/STRAIGHT/DISTANCE/TURN/ARC/AUTO |
| `RUN` | 运行状态 | enum | STANDBY/PARKING/HITTED |
| `DIR` | 行驶方向 | ±1 | +1=前进, -1=后退 |
| `SPD` | 速度等级 | raw | 0-720 (SPEEDSTEP×rank) |
| `ANG` | 当前舵角 | deg | 0-180, 中位=90(HW)/92(实测) |
| `YAW` | 上报航向 | deg | `Kalman(Yaw) - YawReportOffset` |
| `X` | 里程计X | cm | 右方为正 |
| `Y` | 里程计Y | cm | 前方为正 |
| `D` | 累计里程 | cm | ≥0，无符号 |
| `VEL` | 平均轮速 | raw | aveSpeed |
| `DROP` | 丢包计数 | count | USART3溢出诊断 |

### 2.2 控制指令

| 指令 | 参数 | 响应 | 执行模型 |
|------|------|------|----------|
| `MODE M=<mode>` | MANUAL/IDLE/STANDBY | `DONE MODE M=<mode>` | 切换控制模式 |
| `SERVO A=<deg>` | 0-180 | `DONE SERVO` | **直接设舵角**，不触发运动 |
| `STOP` | — | `DONE STOP` | = SetStandbyMode() |
| `CANCEL` | — | `DONE CANCEL` | = STOP 同义 |

### 2.3 运动指令（带序列号和DONE回调）

| 指令 | 参数 | 完成条件 | DONE响应 | 超时 |
|------|------|----------|----------|------|
| **`MOVE D=<cm> [V=<level>]`** | 距离(+前进/-后退) | `target - odom.d ≤ 2.0cm` | `DONE MOVE X=... Y=... D=... YAW=...` | 30s |
| **`TURN A=<deg> [V=<level>]`** | 相对航向角 | `|yaw_err| ≤ 3.0°` | `DONE TURN` | 8s |
| **`ARC D=<cm> STE=<deg> [V=<level>]`** | 距离+舵角 | `target - odom.d ≤ 2.0cm` | `DONE ARC X=... Y=... D=... YAW=...` | 30s |
| **`AUTO`** | — | 三段式完成 | `DONE AUTO` | 复合 |

#### 运动指令执行流程

```text
上位机                    STM32
  │                         │
  │──── ARC D=-6 STE=60 ───→│
  │                         ├─ Parse: D=-6, STE=60
  │                         ├─ SetSteeringAngle(60)
  │                         ├─ ApplyAckermannSpeedScale(60)
  │                         ├─ Odometry_Reset()
  │                         ├─ SetSpeedRank(V)
  │←── ACK ARC ─────────────│  ControlMode = CTRL_ARC
  │                         │
  │                    [每5ms UpdateControlTask]
  │                         ├─ Odometry_GetSnapshot()
  │                         ├─ if (target - odom.d) ≤ 2.0 → DONE
  │                         │
  │←── DONE ARC X= Y= D= YAW= ──│
  │                         ├─ HardStopMotion()
  │                         ├─ CenterSteering() → ANG=90
  │                         └─ ControlMode = CTRL_IDLE
```

#### 关键行为约束

| 约束 | 说明 |
|------|------|
| **ARC 是纯开环** | 设定舵角后不调整，只靠里程计判断终点 |
| **无 YAW 反馈** | ARC 不使用陀螺仪航向修正转向 |
| **DONE 时舵机回中** | `HardStopMotion()` → `CenterSteering()` → ANG=90 |
| **死区效应** | 命令 6cm → 实际 4cm(DONE)+0.8cm(滑行) ≈ 4.8cm STAT |
| **最小有效命令** | D≤3cm 时实际移动 <2cm，反打弧最小命令距离应 ≥4cm |
| **方向切换** | 同向直接执行，反向自动 `ExDirect()` + `InitAll()` |

### 2.4 归零指令

| 指令 | 效果 |
|------|------|
| `ZERO_ODOM` | 里程计 (x,y,distance) 归零 |
| `ZERO_YAW` | `YawReportOffset = New_Yaw` (STAT YAW 归零) |
| `ZERO_ALL` | 里程计 + 航向同时归零 |

### 2.5 参数配置

| 指令 | 可配置参数 |
|------|-----------|
| `SET PARAM=HEADING KP=.. KI=.. KD=.. MAXI=.. MAXOUT=.. DEAD=.. D_ALPHA=.. SMOOTH=.. CROSS=.. CROSS_EN=..` | 航向PID全部参数 |
| `SET PARAM=MOTOR KP=.. KI=.. KD=.. [RKP/RKI/RKD/LKP/LKI/LKD]` | 电机速度PID (左右可独立) |
| `SET PARAM=LIMIT STE_MIN=.. STE_MAX=.. SPEED_MAX=..` | 舵角限幅 + 速度上限 |
| `DEFAULT_CFG` | 恢复出厂默认参数 |

### 2.6 遥测

| 指令 | 效果 |
|------|------|
| `TEL ON` / `TEL 1` | 开启 100ms 周期遥测 (CSV 15字段) |
| `TEL OFF` / `TEL 0` | 关闭遥测 |

---

## 三、RC 协议（简化手动控制）

**无序列号，无DONE回调。面向人工操作/RC遥控。**

### 简单指令

| 指令 | 等效V2操作 |
|------|-----------|
| `RC_HB` | → `OK` (心跳) |
| `RC_STOP` / `AU_STOP` | → `SetStandbyMode()` |
| `RC_MAN` | → `SetManualMode()` |
| `RC_TEL0` / `RC_TEL1` | → `TEL OFF/ON` |
| `RC_STAT` | → `PrintTelemetry()` (单次CSV) |
| `RC_STR` | → `PrepareStraightHold()` (航向保持) |
| `RC_AUTO` / `AU_RUN` | → `StartAutoRoute()` (三段式固定路径) |

### 带参指令

| 指令 | 格式 | 等效V2 |
|------|------|--------|
| `RC_DST<x>` | x=距离cm | `MOVE D=x` |
| `RC_YAW<x>` | x=相对角度deg | `TURN A=x` |
| `RC_SPD<x>` | x=0~6 | `SetSpeedRank(x)` |
| `RC_STE<x>` | x=舵角deg | `SERVO A=x` |

---

## 四、Legacy 协议（已弃用，保留向后兼容）

**中文助记符，无结构化响应。**

| 指令 | 含义 |
|------|------|
| `COMMANDS[0]` | 加速 (SpeedAcc) |
| `COMMANDS[1]` | 减速 (SpeedSlowDown) |
| `COMMANDS[2]=x` | 直接设速度等级 |
| `COMMANDS[3]` | 停车 |
| `COMMANDS[4]=x` | 设舵角 (180-x 转换) |
| `COMMANDS[5]` | 正向 |
| `COMMANDS[6]` | 反向 |
| `COMMANDS[7]` | 软复位 |
| `COMMANDS[8]` | 直行保持 |
| `COMMANDS[9]` | 手动转向模式 |
| `COMMANDS[10]` | 自动泊车 |
| `COMMANDS[11]` | 碰撞状态 |
| `COMMANDS[12]` | 待机 |
| `COMMANDS[13]P/I/D` | 航向PID参数 |

---

## 五、控制模式状态机

```text
                    ┌──────────┐
          SetStandby │  STANDBY │ (舵机脱开, 电机停)
          from any ──→│  IDLE    │
                    └─────┬────┘
                          │
          ┌───────────────┼───────────────┐
          │               │               │
     MODE M=MANUAL   MOVE/TURN/ARC    RC_STR
          │          (带序列号)         │
          ▼               │               ▼
    ┌──────────┐    ┌──────────┐    ┌──────────┐
    │  MANUAL  │    │DISTANCE/ │    │ STRAIGHT │
    │          │    │TURN/ARC  │    │ (航向保持)│
    └──────────┘    │(自动完成)│    └──────────┘
                    │  ┌───┐  │
                    │  │超时│  │
                    │  └───┘  │
                    │  或DONE │
                    └────┬────┘
                         │
                    HardStopMotion()
                    → IDLE + DONE/ERR 响应
```

### 安全门

| 条件 | 触发 | 动作 |
|------|------|------|
| 手动/直行模式 2s 无新指令 | `REMOTE_TIMEOUT_MS` | SetStandbyMode |
| 位移/ARC >30s 未完成 | `DISTANCE_TIMEOUT_MS` | SetStandbyMode + ERR TIMEOUT |
| 转向 >8s 未完成 | `TURN_TIMEOUT_MS` | SetStandbyMode + ERR TIMEOUT |
| 碰撞检测 | `rS = HITTED` | SetStandbyMode |

---

## 六、运动执行层数据流

```text
命令解析 (CarProtocol)
    │
    ▼
StartArcDrive / StartDistanceDrive / StartYawTurn
    │
    ├─ ExDirect()         [如需换向] 切换方向, 复位PID+里程计
    ├─ SetSteeringAngle() [ARC]      设定舵角 (0-180°, 中位=90)
    ├─ ApplyAckermannSpeedScale()    [ARC] 内外轮速差
    ├─ Odometry_Reset()              [位移/ARC] 里程计归零
    ├─ SetSpeedRank()                [全部] 启动电机
    └─ ControlMode = CTRL_ARC/...    [全部] 进入自动模式
        │
        ▼
    [每5ms] UpdateControlTask()
        │
        ├─ Odometry_GetSnapshot() → odom.distance
        ├─ if (target - odom.d) ≤ DISTANCE_DONE_CM(2.0)
        │      → HardStopMotion()
        │      → CenterSteering()  // ANG → 90.0
        │      → CarProtocol_FinishActiveMotionOk("")
        │
        └─ [超时检测] (ControlTicks - ActionStartTick) > TIMEOUT
               → HardStopMotion()
               → CarProtocol_FinishActiveMotionErr("TIMEOUT")
```

### ApplyAckermannSpeedScale 几何模型

```c
steerOffset = Angle - 90.0f;
tanSteer = tan(steerOffset * DEG_TO_RAD);
radius = ACKERMANN_WHEEL_BASE_CM / tanSteer;   // 16.0 / tan(Δ)
halfTrack = WHEEL_TRACK_CM * 0.5f;              // 7.25
leftScale = (radius - halfTrack) / radius;
rightScale = (radius + halfTrack) / radius;
```

**打表实测对比**：

| STE | 几何 R(cm) | 实测 R_eff(cm) | Δ% |
|-----|-----------|---------------|-----|
| 60 | 27.7 | 79.9 | +188% |
| 75 | 59.7 | 147.0 | +146% |
| 105 | 59.7 | 259.7 | +335% |
| 120 | 27.7 | 100.1 | +261% |

**结论**：几何模型与实际差异 1.5~3.4 倍，`ApplyAckermannSpeedScale()` 计算的内外轮速差远大于实际需求。

---

## 七、里程计模型

```c
// Motors.c — Odometry_Update()
dl = left_pulses  * (WHEEL_C / ROT_NUM);   // 左轮行驶距离 (cm)
dr = right_pulses * (WHEEL_C / ROT_NUM);   // 右轮行驶距离 (cm)
dc = (dr + dl) / 2;                         // 中心行驶距离
dtheta = (dr - dl) / WHEEL_TRACK_CM;       // 航向变化 (rad)

odom.x     += dc * cos(odom.theta);         // 右方为正
odom.y     += dc * sin(odom.theta);         // 前方为正
odom.theta += dtheta;                       // 积分航向
odom.distance += dc;                        // 累计里程 (无符号)
```

**关键参数**：
- `WHEEL_C = 21.04867` cm (6.7cm 轮胎直径 × π)
- `ROT_NUM = 2496` 脉冲/转
- `WHEEL_TRACK_CM = 14.5` cm

---

## 八、舵机控制层

```c
// PWMO.c — 线性映射
Angle → Pulse = (Angle/180) × (2000-1000) + 1000 μs
// 90° → 1500μs (理论中位)
// 0°  → 1000μs
// 180°→ 2000μs
```

| 参数 | 值 | 说明 |
|------|-----|------|
| TIM2 PSC | 71 | 72MHz/72 = 1MHz → 1μs分辨率 |
| TIM2 ARR | 19999 | 20ms周期 (50Hz) |
| 脉宽范围 | 1000-2000μs | 标准舵机 |
| 更新死区 | 0.8° | 舵角变化 <0.8° 不更新PWM |

---

## 九、IMU/航向数据流

```text
BMI270 (200Hz)
    │
    ├─ Raw Acc/Gyro (12-byte burst)
    ├─ SW offset subtraction (gyro_zero_x/y/z)
    ├─ PT1 low-pass filter (fc=48Hz acc, 120Hz gyro)
    │
    ▼
BMI270_Get_AngleDt()
    ├─ Convert to physical: Gx/y/z_rate (rad/s), Ax/y/z (g)
    ├─ Speed-gated bias estimation (方案B):
    │     if (gyro_mag < 3°/s) AND (aveSpeed ≈ 0) for 0.5s:
    │         gyro_bias_z = EMA(gyro_bias_z, Gz_rate, α=0.001, τ≈5s)
    ├─ Yaw increment: (Gz_rate - gyro_bias_z) × dt
    ├─ Complementary filter: roll/pitch (acc_weight vs gyro)
    │
    ▼
CarApp ServiceMpuTask()
    ├─ Kalman filter: Yaw, Roll, Pitch (q=0.5, r=0.1)
    └─ → New_Yaw (STAT YAW 字段来源)
```

---

## 十、打表校准 ↔ 固件缺口汇总

基于 `chassis_kinematics.json` 验证结果（15 PASS / 0 FAIL / 4 WARN），以下缺口待融合：

| # | 缺口 | 影响 | 修改位置 |
|---|------|------|----------|
| 1 | **中位角 90→92** | 所有直行和ARC的寄生偏航减少42% | `CarControl.c:64` `ACKERMANN_CENTER_DEG` |
| 2 | **曲率查表缺失** | ARC 纯开环无预测能力，反打弧距离公式无输入 | 新增 `chassis_curvature` 查表 |
| 3 | **死区补偿缺失** | 上位机需自行为每个命令叠加 +1.9cm | 上位机侧补偿 |
| 4 | **滑行未建模** | DONE后仍有 0.8cm 惯性运动 | 上位机侧预留 |
| 5 | **ARC 最小距离无下限** | 发 D<3.0 命令几乎不动 | `StartArcDrive()` 增加下限检查 |
| 6 | **Ackermann 模型未标定** | 内外轮速差计算偏差 1.5-3.4x | `ApplyAckermannSpeedScale()` 引入实测修正 |

### 曲率查表（待融合）

```text
STE=60:  deg_per_cm = -0.717 °/cm,  R_eff =  79.9 cm  (左硬, n=5, CV=1.9%)
STE=75:  deg_per_cm = -0.390 °/cm,  R_eff = 147.0 cm  (左软, n=3)
STE=105: deg_per_cm = +0.221 °/cm,  R_eff = 259.7 cm  (右软, n=3)
STE=120: deg_per_cm = +0.572 °/cm,  R_eff = 100.1 cm  (右硬, n=2)
```

### 死区/滑行参数（已验证）

```text
arc_deadband_cm    = 1.88 ± 0.08 cm  (4样本, 一致性 PASS)
coast_after_done_cm = 0.80 ± 0.41 cm  (4样本, 方差偏大 WARN)
arc_min_effective_cmd_cm = 3.0        (D=3→1.5cm, D=4→3.3cm, PASS)
move_deadband_cm    = 2.0             (同 DISTANCE_DONE_CM)
```

### 不对称比（已验证）

```text
60/120: |deg_per_cm| 比 = 1.252  (左转比右转多25%偏航)
75/105: |deg_per_cm| 比 = 1.767  (左软比右软多77%偏航)
R_eff × deg_per_cm ≈ 1.000        (双向验证通过)
```

---

## 十一、上位机集成checklist

上位机 (`board_parking_controller.py`) 发送ARC命令前应执行：

```text
1. 有效距离 = max(|slot_correction| + arc_deadband_cm, arc_min_effective_cmd_cm)
2. 方向选择 = 查表选能消除当前误差的 STE (60/75/105/120)
3. 预测Δyaw = effective_distance × deg_per_cm(selected_ste)
4. 命令距离 = effective_distance (叠加死区，不依赖STM32补偿)
5. DONE后等待 STAT 确认 yaw 变化量
6. STAT yaw 变化量自动作为新样本写入响应模型
```
