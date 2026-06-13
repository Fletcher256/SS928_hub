# 终态摆正计划：反打弧 + 航向量化验收（Codex 执行版）- 2026-06-13

> 前置状态：第一版端到端泊车已跑通（视觉段修横向 → token 一次性盲倒），距离问题已用死区补偿解决。
> 剩余唯一主要缺陷：**终态车身不平行（约 3° 量级歪斜）**。
> 本计划目标只有一个：让同一起点的泊车从"停进但略歪"变成"停进且平行"，并把"歪不歪"从目视变成数字。

---

## 0. 根因（先读，决定了本计划为什么长这样）

当前成功流程 `ARC(修横向) → MOVE(直线盲倒)` 的几何缺陷：

```text
弧线修横向 = 把车头转向中线 → 横向误差转化为航向误差
  5.1cm 弧 @ R_eff≈87cm → Δφ ≈ 5.1/87 rad ≈ 3.4°
直线盲倒不改变航向 → 3.4° 被冻结为最终停车姿态
```

所以**缺的不是更准的横向修正，而是 S 形的后半段：反打弧**。流程应为：

```text
align_in_corridor:    ARC 修横向(现有, 已工作)
straighten_or_enter:  ARC 反打消航向(本计划新增 ← 唯一核心改动)
final_blind:          token 直线盲倒(现有, 不动)
```

终态数值目标（写死，作为本计划完成判据）：

```text
|终态航向 vs 车位轴| ≤ 2°
|终态横向| ≤ 1.5cm
深度达标(现状已满足)
同一起点 5 回合 ≥ 4 次达标
```

---

## A. 核心任务：反打弧动作（P0）

### A1. 航向响应系数补齐（半天，实车）

反打弧的距离公式需要每档 STE 的 `1/R_eff`（°/cm）。现状：STE=120 有两个好样本（R_eff≈87cm）。需补：

1. 从已有响应模型/C2 日志提取 STE=60/75/105 的 Δyaw 与 Δd（TLM 或 STAT 前后差），能算的直接算。
2. 缺样本的档位各补 1 个探针（沿用 primitive_probe + TELEM，`ARC D=-6` 一次即可）。
3. **ARC 小距离死区探测**（关键新数据）：反打弧通常很短（2~5cm），必须知道 ARC 的最小有效距离。探 `ARC D=-3 STE=120` 与 `ARC D=-4 STE=120` 各一次，记录实走（DONE D 与 STAT D）。若 D=-3 实走 <1cm，则反打弧最小命令距离定为 4。
4. 产出/更新 `configs/chassis_kinematics.json`：

```json
{"steer_curvature": [
   {"ste": 60,  "r_eff_cm": null, "deg_per_cm": null, "n": 0},
   {"ste": 75,  "r_eff_cm": null, "deg_per_cm": null, "n": 0},
   {"ste": 105, "r_eff_cm": null, "deg_per_cm": null, "n": 0},
   {"ste": 120, "r_eff_cm": 87.8, "deg_per_cm": 0.65, "n": 2}],
 "arc_min_effective_cmd_cm": null,
 "arc_deadband_cm": null,
 "move_deadband_cm": 2.0,
 "coast_after_done_cm": 1.0}
```

验收：四档 `deg_per_cm` 全部有实测值；左右对称档比值记录在案（60/120、75/105——顺带就是 Ackermann 差速是否对称生效的证据）。

### A2. `counter_steer` 动态动作进控制器（1 天，板端）

不是动作库里的固定条目，而是 `straighten_or_enter` 相位的**参数化动作**，在 `board_parking_controller.py` 内生成：

```text
输入: 最后稳定状态的 slot_heading_err_deg = φ (经符号确认), chassis_kinematics
方向: 选能消除 φ 的一侧(符号由 C0 signs + A1 实测的 Δyaw 方向决定, 代码里写成查表不要写死)
档位: |φ| ≥ 3° 用硬弧(60/120), < 3° 用软弧(75/105) —— 软弧 deg_per_cm 小, 短距离下分辨率更细
距离: d_cmd = clamp(|φ| / deg_per_cm(档位) + arc_deadband_cm, arc_min_effective_cmd_cm, 6.0)
执行: 单步, 执行后 STOP 重新观察(架构不变)
```

触发与退出条件（相位机规则）：

```text
进入 straighten: |slot_lateral| ≤ 1.5cm 且 |φ| > 2°
执行 counter_steer 一次 → 重新观察:
  |φ| ≤ 2°            → 写 final_blind_token, 进盲倒
  |φ| 改善但仍 >2°     → 允许再做一次(最多 2 次, 第二次后无论如何重评估)
  |φ| 变差             → STOP, verdict=straighten_failed, 留人工分析(说明 deg_per_cm 或符号错)
  横向被弧带出 >1.5cm  → 回 align 相位(允许一次往返, 两次往返 → STOP=oscillation)
```

JSONL 新事件：`counter_steer_decision`（含 φ、选档、d_cmd、预测 Δφ）与执行后 `counter_steer_result`（实测 Δφ、verdict）。每次执行自动成为 A1 系数的新样本（在线积累）。

验收：dry-run 喂 φ=+4°/-4°/+1° 三种状态，决策方向、档位、距离全部符合上表；`py_compile` 过；部署板端。

### A3. token 门槛随之收紧（半小时，与 A2 同改）

现在能修航向了，token 写入条件升级为：

```text
|slot_lateral| ≤ 1.5cm  且  |slot_heading_err_deg| ≤ 2°  且  min_margin 达标
```

2° < |φ| ≤ 6° 不再直接写 token，而是走 A2 反打。|φ| > 6° → STOP 人工（说明上游相位没干好活）。

---

## B. 终态航向量化（P0，与 A 并行，半天）

"歪不歪"不再靠目视。利用已验证的 IMU yaw（短时 Δ 可信）：

```text
写 token 时记录:  yaw_token (STAT), heading_vision_token (最后稳定视觉航向)
停车后记录:       yaw_final (STAT)
终态航向 = heading_vision_token + sign_map(yaw_final - yaw_token)
```

写入每回合 JSONL 的 `final_pose_report` 事件：`final_heading_deg / final_lateral_est_cm / depth_est_cm / verdict(parked_straight | parked_crooked | not_in)`。判 `parked_straight` 即 |final_heading| ≤ 2°。

验收：与目视对照 3 回合，数字与肉眼判断一致（歪的回合报歪，正的报正）。

---

## C. 回归测试协议（P0，A+B 完成后，1 个下午）

同一起点（贴胶带复位）连续 5 回合完整泊车：

```text
每回合记录: 初始状态 / 动作序列 / 每动作前后状态 / final_pose_report / 照片一张
通过标准:   ≥4/5 回合 parked_straight
任一回合触发安全门 → 记录后继续(安全门触发不算失败, 算系统正确自保)
3 回合连续 straighten_failed → 中止, 回 A1 查系数
```

产出 `docs/autopark_straighten_regression_2026061X.md` + 日志入 `artifacts/autopark_baseline/`。

**通过 C 即宣告：固定起点泊车（L0）完成。** 这是项目第一个可宣布的里程碑。

---

## D. 维护项（P1，穿插做，不阻塞 A-C）

1. **arm 门收口**（上次代码审查 P1 遗留，若未修）：所有运动发送收口到 `send_motion()`，内部断言 `armed`+caps；DR 与 pixel_blind_finish 路径不得绕过。
2. **融合方向位**：`parking_fusion.py` 的 ds 方向从 TLM 的 V 符号取（D 是无符号累计量，前进动作引入前必须修）。
3. **DONE 缺 YAW 协议一致性**（b2 记录过一次 DONE 无 YAW 字段）：固件侧确认四条终止路径输出一致。
4. 把本次成功 2-step 流程（ARC STE=60 修横向 + MOVE 盲倒）作为 measured 序列样本写入响应模型（带状态桶：lateral≈+4.2cm 桶）。
5. 文档记账：`autopark_long_term_memory.md` 更新当前流程图与 token 规则。

---

## E. C 通过之后的方向（预告，不在本计划内开工）

```text
L1 包络: 3×3 起点网格(横向±5cm × 航向±5°), 每格 2 回合, 产出成功矩阵
预计新需求: 网格边缘格会出现"横向修不完"→ 触发 forward_correct(L2)的立项依据
B6 几何预测器: A1 的 deg_per_cm 表就绪后, 评分器预测全面换成几何计算
Reeds-Shepp 简化版: 仅当 L1 网格 ≥7/9 通过后才考虑, 作为候选序列生成器接入现有评分架构
```

---

## 执行顺序汇总

```text
第 1 天上午  A1 系数补齐 + ARC 小距离死区探测(实车 ~6 发探针)
第 1 天下午  A2 counter_steer 实现 + dry-run 验收 / B 终态量化(并行)
第 2 天上午  A2/A3 部署板端, 单回合带反打的完整泊车 1-2 次试跑
第 2 天下午  C 回归 5 回合 → 判定 L0 里程碑
穿插        D1-D5 维护项
```
