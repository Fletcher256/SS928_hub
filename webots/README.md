# Webots Simulation

This directory contains a Webots R2025a project for the SS928 car control logic.

Open the world:

```powershell
& "D:\Program Files\Webots\msys64\mingw64\bin\webots.exe" webots\worlds\ss928_car.wbt
```

Keyboard controls inside Webots:

- `1` / `2` / `3`: set speed rank.
- `S`: straight-hold mode.
- `D`: drive 60 cm.
- `Q`: left yaw turn 90 degrees.
- `E`: right yaw turn 90 degrees.
- `A`: run default autonomous route.
- `X`: stop.
- `R`: reset simulation controller state.
- `Space`: pause/resume controller motion.
- `+` / `-`: increase/decrease speed rank.

The controller reuses `sim/sim_car.py` for command parsing and the high-level state machine. Webots provides the 3D scene, keyboard interface, labels, and path trail.

This is a kinematic debug simulation, not a full electrical/physics model of the STM32 board, motor driver, IMU, or encoders. Final control quality still needs low-speed hardware validation.
