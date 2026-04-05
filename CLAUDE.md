# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Standalone BNO055 IMU test for the MKS-ESP32FOC V2.0 board. This sketch validates the BNO055 wiring, initialization, and sensor-fusion output before integrating the same logic into the main `balancing_robot` project.

## Hardware

- **Board:** MKS-ESP32FOC V2.0 (ESP32-S3-VROOM)
- **IMU:** BNO055 at I²C address `0x28`
- **I²C pins:** SDA = GPIO 19 (rood), SCL = GPIO 18 (zwart), GND (geel), 3V3 (blauw)
- **Serial:** Uses `Serial0` (not `Serial`) for USB output at 115200 baud

## Architecture

The sketch uses two FreeRTOS tasks pinned to different cores — identical to the `balancing_robot` pattern:

| Task | Core | Rate | Responsibility |
|------|------|------|----------------|
| `fastTask` | Core 1 | 500 Hz (÷5 → 100 Hz IMU) | Read Euler angles + calibration status from BNO055 |
| `slowTask` | Core 0 | 10 Hz | Print telemetry to Serial |

Shared state (`gShared`) is protected by a FreeRTOS mutex. `fastTask` uses a **non-blocking** `xSemaphoreTake(xMutex, 0)` to avoid stalling the fast loop; `slowTask` uses a 5 ms timeout.

**BNO055 mode:** `IMUPLUS` (accel + gyro only, no magnetometer). Magnetometer calibration (`M`) will always read 0. Gyro and Accel should reach status 3 after a few seconds of movement.

**Init sequence:** CONFIG mode → normal power → enable external crystal (650 ms wait) → IMUPLUS mode.

## Build & Flash

Arduino CLI is available in PATH (`arduino-cli` v1.4.1). FQBN: `esp32:esp32:esp32s3`. Board typically appears on `COM3`.

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32s3 .

# Compile & upload
arduino-cli compile --fqbn esp32:esp32:esp32s3 . && arduino-cli upload --fqbn esp32:esp32:esp32s3 --port COM3 .

# Serial monitor (115200 baud, Ctrl+C to exit)
arduino-cli monitor --port COM3 --config baudrate=115200
```

To find the active port if COM3 is wrong: `arduino-cli board list`
