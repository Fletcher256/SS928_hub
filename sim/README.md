# PC Simulation

This folder contains a lightweight simulator for testing the control logic without the car hardware.

Run the default scenario:

```powershell
py sim\sim_car.py
```

Run a specific scenario:

```powershell
py sim\sim_car.py sim\scenarios\basic_control.txt
```

Open the visual simulator:

```powershell
start sim\visualizer.html
```

Supported scenario commands:

- `@COMMAND`: send the same text command used by USART3, without needing `\r\n`.
- `cmd @COMMAND`: same as above.
- `sense yaw=5 x=1 distance=20`: inject virtual sensor/odometry values.
- `tick 20`: advance simulated time without plant movement.
- `drive 1000`: advance time while using the simple built-in plant model.
- `drive 1000 yaw_drift=2 lateral_drift=0.4`: drive with artificial straight-line drift.
- `expect mode=STRAIGHT`: assert a state value.
- `expect angle<90`: assert a numeric relation.
- `print`: print current simulator state.

The simulator mirrors the firmware command parser and high-level control state machine. It does not model motor PWM, encoder electrical behavior, IMU noise, tire slip, battery sag, or servo calibration. Final motion quality still needs low-speed hardware validation.
