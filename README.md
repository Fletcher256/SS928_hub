# SS928_hub

STM32F103 remote/autonomous car firmware.

## Build

The checked-in Keil project remains available as `LED_1.uvprojx`.

A GCC build entry is also provided for command-line verification:

```powershell
powershell -ExecutionPolicy Bypass -File build_gcc\build.ps1
```

The script first looks for a local xPack toolchain under `tools/gcc-arm/...`; if it is not present, it falls back to `arm-none-eabi-gcc` on `PATH`.

Build outputs are written to:

- `build/gcc/SS928_hub.elf`
- `build/gcc/SS928_hub.hex`
- `build/gcc/SS928_hub.bin`

## Serial Command Framing

Text commands are sent through USART3 as:

```text
@COMMAND\r\n
```

## Control Commands

Legacy commands are still supported:

- `SR_ACC`: increase speed rank.
- `SR_DEC`: decrease speed rank.
- `SR_SETn`: set speed rank, `n` is `0..6`.
- `SR_PAU`: stop speed output.
- `DT_1`: forward.
- `DT_0`: reverse.
- `DT_STA`: straight-hold mode.
- `DT_TUR`: manual turn mode.
- `RT_TOx`: set steering using the legacy mapping `servo = 180 - x`.
- `ST_KP/ST_KI/ST_KD`: tune heading PID.
- `ST_SB`: standby and stop.
- `ST_PK`: start the default autonomous route.
- `ST_ER`: emergency/error stop.

New remote/autonomous commands:

- `RC_MAN`: manual mode.
- `RC_STOP` or `AU_STOP`: immediate stop and standby.
- `RC_HB`: heartbeat. Use it periodically when driving manually.
- `RC_STR`: straight-hold mode using IMU heading and encoder cross-track correction.
- `RC_SPDn`: set speed rank, `n` is `0..6`.
- `RC_STEx`: set servo angle directly, `x` is `0..180`.
- `RC_DSTx`: drive straight for `x` cm. Negative values drive backward.
- `RC_YAWx`: turn by `x` degrees relative to current yaw. Positive is left.
- `RC_AUTO` or `AU_RUN`: run the default autonomous route: forward 100 cm, left 90 degrees, forward 60 cm, stop.

## Safety Behavior

- Manual and straight-hold modes stop automatically after 2 seconds without a command or heartbeat while moving.
- Distance actions stop after 30 seconds if the target is not reached.
- Yaw turn actions stop after 8 seconds if the target is not reached.
- The current hardware set does not include obstacle detection, so autonomous mode is odometry/IMU based only.
