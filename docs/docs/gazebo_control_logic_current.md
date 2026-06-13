# Gazebo-Oriented Current Vehicle Control Logic

本文整理当前小车泊车控制链路，并标出哪些部分适合直接搬到 Gazebo，哪些只是板端实车逻辑或 dry-run 诊断。

## One-Line Status

当前仓库没有真正的 Gazebo actuator 接口：没有 `/cmd_vel`、Ackermann topic、`ros2_control` joint controller 或 Gazebo plugin。现有 ROS 侧主要是感知、目标位姿、一步规划和 dry-run 候选命令；实车闭环主要在板端 `tools/board_parking_controller.py`。

因此 Gazebo 侧应该复用“状态估计 + 一步规划”，再补一个“仿真执行适配层”。

## Current ROS Control Graph

```text
camera / board YOLO / simulated slot source
  -> /parking/yolo/parking_detections
  -> slot_geometry_transform_node
  -> /parking/slot_geometry
  -> parking_target_pose_node
  -> /parking/target_pose
  -> parking_metric_planner_node
  -> /parking/planner/path_cm
  -> Gazebo adapter (not implemented yet)
  -> /cmd_vel or Ackermann command
```

另外还有一条较早的像素 dry-run 链路：

```text
/parking/yolo/parking_detections or /parking/parking_slot_candidates
  -> parking_planner_node
  -> /parking/planner/path
  -> parking_controller_dry_run_node
  -> /parking/controller/proposed_cmd
  -> /parking/controller/v2_candidate
```

这条链路只产生 JSON 和 STM32 V2 候选字符串，不发送控制命令。

## Main Components

### 1. Parking YOLO

File: `ros/parking_bridge/parking_bridge/parking_yolo_node.py`

输入：

- `/parking/camera/yolo_input_jpeg`

输出：

- `/parking/yolo/parking_detections`
- `/parking/yolo/parking_view`
- `/parking/perception/state`

核心输出是 `slot_candidates`，包含：

- `bbox`
- `center_px`
- `center_norm`
- `polygon`
- `status`: `empty` / `occupied` / `unknown`
- `confidence`

Gazebo 使用建议：

- 如果 Gazebo 已有真值车位位姿，可以跳过 YOLO，直接发布 `/parking/slot_geometry` 或 `/parking/target_pose`。
- 如果要做视觉闭环仿真，则让 Gazebo camera 图像走 YOLO 节点。

### 2. Pixel To Ground Geometry

File: `ros/parking_bridge/parking_bridge/slot_geometry_transform_node.py`

输入：

- `/parking/yolo/parking_detections`

输出：

- `/parking/slot_geometry`
- `/parking/slot_geometry_state`

作用：

- 读取 homography 标定，把 YOLO polygon 从像素坐标转成 `vehicle_rear_axle_cm` 坐标。
- 输出 `ground_geometry.center_cm`、`entrance_edge_cm`、`approach_axis_cm`、`width_cm`、`length_cm`、`yaw_ground_deg`。

限制：

- 没有有效 `/home/ebaina/parking_calibration/slot_homography_rear_axle.json` 时，只会进入 `waiting_for_calibration`。
- 对 Gazebo 来说，最稳的是先用仿真真值直接生成同格式 `/parking/slot_geometry`，不要强依赖图像 homography。

### 3. Target Pose

File: `ros/parking_bridge/parking_bridge/parking_target_pose_node.py`

输入：

- `/parking/slot_geometry`

输出：

- `/parking/target_pose`
- `/parking/target_pose_state`

作用：

- 选中一个 slot。
- 根据车位中心、入口边、车位方向和 `rear_axle_to_vehicle_center_cm` 计算目标后轴位姿。
- 给出 `target_rear_axle_pose_cm`、`approach_pose_cm`、`path_cm`。

注意：

- 该节点的 `coordinate_convention.x_cm` 字段仍写作 `forward`。
- 下游 `parking_metric_planner_node.py` 已明确把当前后视泊车场景中的 `+x_cm` 解释为“朝向车位的倒车方向”。

### 4. Metric One-Step Planner

File: `ros/parking_bridge/parking_bridge/parking_metric_planner_node.py`

输入：

- `/parking/target_pose`

输出：

- `/parking/planner/path_cm`
- `/parking/planner/path_cm_state`

这是最适合 Gazebo 复用的规划节点。

核心逻辑：

- 坐标系为 `vehicle_rear_axle_cm`。
- 每次只规划一个小步，不输出完整 open-loop 轨迹。
- 默认动作是倒车。
- 使用 reverse pure-pursuit 计算转向。
- 使用 `step_cm` 限制单步距离。
- 使用 `command_distance_deadband_cm` 补偿实车 STM32 距离死区。

输出里的关键字段：

- `status`: `waiting_for_target` / `planning` / `aligned`
- `errors.longitudinal_remaining_cm`
- `errors.lateral_error_cm`
- `errors.heading_error_deg`
- `next_step.direction`
- `next_step.distance_cm`
- `next_step.steering_hint_deg`
- `next_step.stm32_servo_deg`
- `next_step.stm32_candidate_cmd`
- `path_cm`

Gazebo 使用建议：

- 读取 `/parking/planner/path_cm`。
- 如果 `status == planning`，执行 `next_step` 对应的小步。
- 执行完后停止、等待仿真位姿更新，再重新发布/读取目标，继续下一步。
- 如果 `status == aligned`，停止。

### 5. Pixel Dry-Run Planner

File: `ros/parking_bridge/parking_bridge/parking_planner_node.py`

输入：

- `/parking/yolo/parking_detections`
- `/parking/parking_slot_candidates`

输出：

- `/parking/planner/path`
- `/parking/controller/dry_run_cmd`
- `/parking/planner/state`

作用：

- 在没有可靠地面标定时，用像素中心偏差生成归一化路径和模拟转向角。
- `commanded_speed_cm_s` 固定为 `0.0`。
- 明确 `motion_enabled=false`、`actuator_control_allowed=false`。

Gazebo 使用建议：

- 只适合作为早期视觉调试或 UI overlay。
- 不建议作为 Gazebo 主控制器，因为它没有真实尺度和车辆运动模型。

### 6. Dry-Run Candidate Controller

File: `ros/parking_bridge/parking_bridge/parking_controller_dry_run_node.py`

输入：

- `/parking/planner/path`
- 可选 `/parking/stm32/health`

输出：

- `/parking/controller/proposed_cmd`
- `/parking/controller/v2_candidate`
- `/parking/controller/state`

作用：

- 做目标稳定帧计数。
- 把像素 planner 的 `simulated_steering_deg` 转成 STM32 风格候选命令：
  - `MOVE D=-x V=1`
  - `ARC D=-x STE=<servo> V=1`
- 永远不打开串口、不发 STM32、不发 CAN、不控制电机。

Gazebo 使用建议：

- 可以临时解析 `/parking/controller/v2_candidate` 做仿真，但这不是最佳接口。
- 更推荐直接对接 metric planner 的 `/parking/planner/path_cm`。

## Board-Side Real-Car Logic

File: `tools/board_parking_controller.py`

长期方向已记录在 `docs/autopark_long_term_memory.md`：

```text
YOLO slot polygon
  -> relative slot pose/state
  -> action-template library
  -> score candidate actions
  -> execute one short action
  -> stop and observe again
  -> replan every step
```

实车侧已有 action library 和 response model：

- `configs/parking_action_library.json`
- `configs/parking_action_response_model.json`
- `configs/parking_success_criteria.json`
- `configs/chassis_kinematics.json`

核心思想：

- 不再依赖固定倒车序列。
- 从当前 slot-relative state 评估候选动作。
- 每次只执行一个短动作，然后停止、观察、重规划。
- 实车安全门包括 `--arm`、`/tmp/parking_armed`、距离上限、视觉丢失 STOP、异常退出 STOP 等。

Gazebo 使用建议：

- 不建议直接把 `board_parking_controller.py` 当 Gazebo 主控制器，因为它混合了板端 UDP、STM32 协议、安全门和实车日志逻辑。
- 可以抽取其中的 action scoring 思路，或直接复用 `configs/parking_action_library.json` 做离散动作仿真。

## Command Semantics

当前内部候选命令主要沿用 STM32 V2 字符串：

```text
MOVE D=-6.0 V=1
ARC D=-6.0 STE=60 V=1
ARC D=-6.0 STE=120 V=1
STOP
```

约定：

- `D < 0` 表示倒车。
- `V=1` 是低速档。
- `STE=90` 近似舵机中位。
- `STE` 到实际转弯方向在不同控制层有历史校准差异，Gazebo 不应硬编码实车符号；应提供 `steering_sign` 参数。

Gazebo adapter 更合理的内部命令格式：

```json
{
  "direction": "REVERSE",
  "distance_cm": 5.0,
  "steering_hint_deg": -12.0,
  "stop_after_step": true
}
```

再由 adapter 转成：

- `/cmd_vel`: `linear.x`、`angular.z`
- 或 Ackermann: speed、steering angle
- 或 Gazebo model plugin service/action

## Recommended Gazebo Integration

### Minimal Path

1. Gazebo 发布车位相对真值为 `/parking/target_pose`。
2. 运行 `parking_metric_planner_node`。
3. 新增 `gazebo_parking_adapter_node`：
   - 订阅 `/parking/planner/path_cm`
   - 当 `status=planning` 时读取 `next_step`
   - 根据 Gazebo 车辆模型发布短时 `/cmd_vel` 或 Ackermann command
   - 达到距离或超时后发布 STOP
   - 等待下一帧 target pose 后再执行下一步

优点：绕开视觉和 homography，先验证控制闭环。

### Vision-In-The-Loop Path

1. Gazebo camera 发布图像。
2. `parking_yolo_node` 检测车位。
3. `slot_geometry_transform_node` 或仿真真值转换节点生成 `/parking/slot_geometry`。
4. `parking_target_pose_node` 生成 `/parking/target_pose`。
5. `parking_metric_planner_node` 规划下一步。
6. `gazebo_parking_adapter_node` 执行动作。

优点：更接近实车感知链路。

### Action-Template Path

1. Gazebo 发布 slot-relative state。
2. 用 `configs/parking_action_library.json` 枚举动作。
3. 在 Gazebo 中预测或执行单步动作。
4. 按评分选最优动作。

优点：更贴近当前板端长期路线。
缺点：需要把 `board_parking_controller.py` 中的评分核心拆成 ROS/Gazebo 可复用模块。

## Missing Pieces For Gazebo

- `gazebo_parking_adapter_node`：把 planner JSON 转成仿真车控制 topic。
- Gazebo vehicle model 的控制接口定义：`/cmd_vel`、Ackermann 还是 `ros2_control`。
- Gazebo odom/pose 反馈接入：用于判断单步动作是否完成。
- Gazebo slot truth publisher：用于直接生成 `/parking/target_pose` 或 `/parking/slot_geometry`。
- 统一坐标约定：特别是后视泊车时 `+x` 是朝车位的倒车方向。
- 转向符号参数化：不要把实车 `STE` 符号直接写死到 Gazebo。

## Practical Recommendation

Gazebo 第一版不要接板端 `board_parking_controller.py`，也不要从像素 dry-run controller 开始。建议先接：

```text
Gazebo slot truth
  -> /parking/target_pose
  -> parking_metric_planner_node
  -> new gazebo_parking_adapter_node
  -> Gazebo vehicle control
```

这个路径最短，能最快验证“小步执行、停止、观察、重规划”的核心控制逻辑。等 Gazebo 运动闭环稳定后，再把 YOLO / homography / action-template scoring 逐层加回来。

