# SS928_hub

STM32F103CB remote/autonomous car firmware.

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
- `ST_PK`: enter parking run state without starting motion.
- `ST_ER`: emergency/error stop.

New remote/autonomous commands:

- `RC_MAN`: manual mode.
- `RC_STOP` or `AU_STOP`: immediate stop and standby.
- `RC_HB`: heartbeat. Use it periodically when driving manually.
- `RC_STR`: straight-hold mode using IMU heading and encoder cross-track correction.
- `RC_SPDn`: set speed rank, `n` is `0..6`.
- `RC_STEx`: set servo angle directly, clamped to the configured safe steering window. The firmware default is `55..125`.
- `RC_DSTx`: drive straight for `x` cm. Negative values drive backward.
- `RC_YAWx`: turn by `x` degrees relative to current yaw. Positive is left.
- `RC_AUTO` or `AU_RUN`: run the default autonomous route: forward 100 cm, left 90 degrees, forward 60 cm, stop.
- `LED_BLINK ON|OFF`: flash or restore the currently selected status LED. A state
  transition (`ST_SB`, `ST_PK`, or `ST_ER`) changes the blinking colour without
  disabling the flash mode.

## Physical button protocol

The active-low PA3 button is debounced and classified on release:

- shorter than 2000 ms: `CTR_PK SHORT DUR_MS=<n>` — board launcher starts
  parking while idle; the same token safely stops an active parking controller;
- 2000 ms or longer: `CTR_REC LONG DUR_MS=<n>` — board launcher/controller
  toggles board-native H264 recording. The board enables `LED_BLINK ON` only
  after the H264 file and a fresh YOLO detection are both present.

The LED's base state remains authoritative: recording-only flashes green;
parking (`ST_PK`) changes it to flashing yellow; stopping recording restores the
current state to steady output.

## Safety Behavior

- Manual and straight-hold modes stop automatically after 2 seconds without a command or heartbeat while moving.
- Distance actions stop after 30 seconds if the target is not reached.
- Yaw turn actions stop after 8 seconds if the target is not reached.
- The current hardware set does not include obstacle detection, so autonomous mode is odometry/IMU based only.

## OLED Action Images

The OLED is treated as a 128x64 monochrome status display. Action-level visuals are dispatched through
`OLED_StateAnim_ShowAction()` and rendered only when the action changes, so the I2C display update does not
run continuously in the control loop.

Recommended LVGL Image Converter settings for custom pictures:

- Size: `128 x 64`.
- Color: 1-bit monochrome.
- Use `OLED_DrawBitmap128x64()` when the array is already in SSD1306 page order: 8 pages x 128 bytes.
- Use `OLED_DrawMonoBitmap128x64()` when the array is row-major 1-bit pixels: 64 rows x 16 bytes.
- Keep the final pixel payload at `1024` bytes per full-screen image.

This firmware does not link the full LVGL library. The small `lvgl.h` / `lvgl/lvgl.h`
compatibility headers only provide the image descriptor types and constants commonly emitted
by LVGL Image Converter. They are for bitmap resources only, not LVGL widgets, timers, styles,
or display drivers.
