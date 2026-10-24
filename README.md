# AUTONOMOUS-EDGE-AI-QUADCOPTER
Unified Single-SoC Autonomous Edge-AI Quadcopter built on ESP32-S3 CAM (N16R8) and an F450 airframe. Combines a deterministic 250 Hz FreeRTOS PID flight controller on Core 1 with 95/5 Complementary IMU fusion (MPU6050), 8 MB PSRAM MJPEG FPV video streaming, and a zero-dependency HTML5 WebSocket Ground Control Station on Core 0.
# Autonomous Edge-AI Quadcopter (ESP32-S3 Single-SoC Flight Controller & Vision System)

This repository contains the presentation deck, engineering documentation, wiring schematics, and unified firmware for our **Autonomous Edge-AI Quadcopter** built on the **F450 frame** and powered by a single **ESP32-S3 WROOM-1 CAM (N16R8)**.

## Project Description
Traditional vision-enabled drones require both a flight controller and a separate companion computer (such as a Raspberry Pi), adding weight, power drain, and serial latency. This project unifies both systems onto a single dual-core ESP32-S3 microcontroller:
- **Core 1 (Hard Real-Time Control)**: Executes a deterministic 250 Hz (4 ms) flight stabilization loop featuring a 95/5 Complementary Filter, decoupled 3-axis PID controllers, and Quad-X motor mixing.
- **Core 0 (Edge Vision & Web Cockpit)**: Hosts a standalone Wi-Fi SoftAP (`AERO_DRONE_WIFI`), streams low-latency MJPEG FPV video from the OV3660 camera using 8 MB Octal PSRAM DMA, and serves a zero-dependency HTML5 touch joystick cockpit at `http://192.168.4.1`.

## Key Specifications
- **Microcontroller**: ESP32-S3 CAM (Dual-Core Xtensa LX7 @ 240 MHz, 16 MB Flash, 8 MB OPI PSRAM)
- **IMU Sensor**: MPU6050 6-DOF Gyro/Accelerometer isolated on `Wire1` (`SDA: GPIO 1`, `SCL: GPIO 2`)
- **Propulsion**: 4× A2212 1000KV BLDC Motors + 4× 30A ESCs (`GPIOs 41, 42, 39, 40`) + 1045 CW/CCW Propellers
- **Power System**: 3S 11.1 V 2200 mAh 25C LiPo Battery; logic powered via ESC 1 5 V / 2 A linear BEC
- **Thrust-to-Weight Ratio**: 3.37 : 1 (3,200 g max thrust vs. 950 g all-up weight; ~35% hover throttle)
- **Safety Failsafes**: Boot-time gyro auto-calibration, software arming lock, and 1.5 s Wi-Fi heartbeat auto-disarm

## Repository Contents
- `/presentation` — Project presentation slides (PPTX) and speaker script
- `/docs` — Full 65-feature matrix, 19-part Bill of Materials, and wiring diagrams
- `/src` — Unified ESP32-S3 PlatformIO firmware (`main.cpp` and `platformio.ini`)
