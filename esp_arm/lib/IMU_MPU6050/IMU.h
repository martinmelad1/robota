#pragma once
// IMU.h — MPU6050 heading & velocity on esp_arm
// SDA = GPIO 21, SCL = GPIO 22 (I2C default on ESP32)
//
// Exposes yaw angle (heading, degrees) computed by a
// complementary filter fusing gyro integration + accel tilt.
// Also exposes raw acceleration for optional velocity estimation.
//
// Usage:
//   1. IMU_Init()        — call once in setup() before tasks start
//   2. IMU_StartTask()   — starts 100 Hz FreeRTOS task on Core 0
//   3. IMU_GetYaw()      — thread-safe yaw snapshot (degrees, ±180)
//   4. IMU_GetAccelX/Y() — raw world-frame accel (m/s²) for vel estimate

#include <Arduino.h>

// ── Config ────────────────────────────────────────────────────
#define IMU_SDA_PIN      21
#define IMU_SCL_PIN      22
#define IMU_I2C_ADDR     0x68   // MPU6050 default (AD0 low)
#define IMU_UPDATE_HZ    100    // update rate

// Complementary filter weight: 0 = gyro only, 1 = accel only
// 0.96 = trust gyro 96% per sample, drift corrected by 4% accel
#define IMU_COMP_ALPHA   0.96f

// ── Public API ────────────────────────────────────────────────

// Initialise I2C and MPU6050 registers. Keep robot still during calibration.
void IMU_Init();

// Start the 100 Hz FreeRTOS update task (call after IMU_Init).
void IMU_StartTask();

// Thread-safe heading snapshot (degrees, 0 = forward at start, ±180)
float IMU_GetYaw();

// Thread-safe raw world-frame acceleration (m/s²)
float IMU_GetAccelX();
float IMU_GetAccelY();

// True once calibration is complete
bool IMU_IsReady();
