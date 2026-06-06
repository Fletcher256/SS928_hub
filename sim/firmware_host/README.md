# Same-source Firmware Host Simulation

This simulator compiles `main.c` for Windows with `SS928_HOST_SIM` enabled. STM32 hardware calls are replaced by host stubs that record motor PWM, servo angle, LED state, UART replies, yaw, and odometry.

Run the default scenario:

```powershell
powershell -ExecutionPolicy Bypass -File sim\firmware_host\run.ps1
```

Build only:

```powershell
powershell -ExecutionPolicy Bypass -File sim\firmware_host\build.ps1
```

Run a specific scenario:

```powershell
build\host\ss928_firmware_host.exe sim\firmware_host\scenarios\basic_control.txt
```

Scenario commands:

- `cmd @RC_SPD2`: send a firmware text command through the same parser as USART3.
- `tick 2100`: advance firmware time without plant movement.
- `drive 4500 yaw_drift=0.8 lateral_drift=0.2`: advance time with a simple virtual car plant.
- `sense yaw=10 x=1 y=20 distance=20`: inject virtual sensor/odometry state.
- `expect mode = 0`: assert a numeric state value.
- `print`: print the current firmware snapshot.

Useful numeric values:

- Modes: `IDLE=0`, `MANUAL=1`, `STRAIGHT=2`, `DISTANCE=3`, `TURN_YAW=4`, `AUTO_ROUTE=5`.
- Auto steps: `IDLE=0`, `FORWARD1=1`, `TURN1=2`, `FORWARD2=3`.

This validates the C command parser, state machine, safety timeouts, PID steering output, distance mode, yaw turn mode, and default autonomous route without connecting the car. It does not validate real motor polarity, encoder wiring, IMU mounting direction, tire slip, or battery/load behavior.
