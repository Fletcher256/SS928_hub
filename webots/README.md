# Webots Simulation

This directory contains a Webots R2025a project for the SS928 car control logic.

Open the world:

```powershell
powershell -ExecutionPolicy Bypass -File webots\open_ss928_webots.ps1
```

The launcher opens the world in realtime mode. The car starts stopped, the camera follows `SS928_CAR`, and the `SS928 Control` robot window provides clickable controls.

Keyboard controls inside Webots:

- Use the `SS928 Control` robot window buttons for forward, reverse, turn, stop, speed, auto route, and camera reset. If the panel is hidden, right-click `SS928_CAR` and choose `Robot Window`.
- Click the 3D view once if the keyboard focus is not inside the simulator.
- Arrow keys / numpad arrow keys: drive forward, reverse, left, and right.
- `1` / `2` / `3` / `4` / `5` / `6`: set speed rank.
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
