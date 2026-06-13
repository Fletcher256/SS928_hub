# Parking Dry-Run Controller - 2026-06-08

## Purpose

First-step controller implementation for the camera/YOLO parking pipeline.

This is dry-run only. It never opens serial, UDP send, CAN, motor, steering,
brake, or throttle interfaces. It only publishes what STM32 V2 command would
be considered after safety gates pass.

## ROS Node

```text
parking_bridge.parking_controller_dry_run_node
```

Registered executable:

```text
parking_controller_dry_run_node
```

Launch integration:

```text
enable_parking_controller_dry_run:=true
```

Default is `true` because the node is diagnostic-only.

## Inputs

```text
/parking/planner/path
/parking/stm32/health
```

`/parking/stm32/health` is optional in dry-run mode by default:

```text
parking_controller_require_stm32_health:=false
```

For stricter dry-run validation, set it to `true`.

## Outputs

```text
/parking/controller/proposed_cmd
/parking/controller/v2_candidate
/parking/controller/state
```

`/parking/controller/proposed_cmd` is a JSON payload containing:

- selected target summary
- dry-run gate states
- blocked reasons
- candidate STM32 V2 command text
- `send_to_stm32=false`
- `serial_output_enabled=false`
- `actuator_control_allowed=false`

`/parking/controller/v2_candidate` is a plain text candidate such as:

```text
@1007 ARC D=-5.0 STE=118 V=1
```

It is not sent to STM32.

## Safety Gates

The dry-run candidate is marked ready only after:

- planner output is fresh
- target slot is acquired
- target center is stable for `parking_controller_required_stable_frames`
- STM32 health is OK if `parking_controller_require_stm32_health=true`

Even when all dry-run gates pass, the node still publishes:

```text
send_to_stm32=false
motion_enabled=false
serial_output_enabled=false
actuator_control_allowed=false
```

## Current Parameters

```text
parking_controller_required_stable_frames:=5
parking_controller_max_target_center_shift_norm:=0.08
parking_controller_reverse_step_cm:=5.0
parking_controller_approach_step_cm:=3.0
parking_controller_speed_gear:=1
parking_controller_steering_sign:=1.0
```

`parking_controller_steering_sign` must be calibrated before any real motion.

## Deployment

VM build completed successfully:

```text
colcon build --packages-select parking_bridge
Summary: 1 package finished
```

Smoke test:

```text
parking_controller_dry_run started: plan=/parking/planner/path, stm32=/parking/stm32/health, proposed=/parking/controller/proposed_cmd, serial_output_enabled=false
```

The smoke test was stopped by `timeout`, so exit code `124` was expected.

## Foxglove Topics

Add Raw Messages panels for:

```text
/parking/controller/proposed_cmd
/parking/controller/state
```

Add a Raw Messages or Text panel for:

```text
/parking/controller/v2_candidate
```

## Next Step

Restart the perception/YOLO ROS launch so this new node is included, then
verify proposed commands while the car is stationary.
