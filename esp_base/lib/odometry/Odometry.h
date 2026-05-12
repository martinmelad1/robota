#pragma once
#include <Arduino.h>

// ============================================================
//  ODOMETRY MODULE
//  Mecanum forward kinematics — encoder-only heading integration.
//  No IMU required. Add complementary filter later when MPU6050
//  is available by re-introducing the gyro fusion in Update().
//
//  Coordinate frame (world-frame, zeroed at reset):
//    +X  = right
//    +Y  = forward
//    +θ  = counter-clockwise (radians)
//
//  Usage:
//    1. Call Odometry_Init()    once from the OdoTask before the loop
//    2. Call Odometry_Update()  every 10 ms (100 Hz)
//    3. Call Odometry_ResetPose() when robot is at known origin
//    4. Read pose with Odometry_GetPose()
// ============================================================

// ── Robot geometry ───────────────────────────────────────────
// WHEEL_RADIUS   : metres — shaft centre to tread contact point
// LX             : metres — robot centre → left/right wheel contact (half-track)
// LY             : metres — robot centre → front/rear wheel contact (half-wheelbase)
// TICKS_PER_REV  : encoder counts per full output-shaft revolution
//                  (2x mode: A+B both on RISING → 2 × PPR)
//                  PID_Control uses 374 PPR with A+B = 748 effective.
//                  Your original code listed 1496 (4x). Adjust to match hardware.

static constexpr float ODO_WHEEL_RADIUS  = 0.097f / 2.0f; // 97 mm diameter → 48.5 mm radius
static constexpr float ODO_LX            = 0.150f;         // metres — FILL IN YOUR MEASURED VALUE
static constexpr float ODO_LY            = 0.150f;         // metres — FILL IN YOUR MEASURED VALUE
static constexpr int   ODO_TICKS_PER_REV = 748;            // 2x mode (A+B both RISING, 374 PPR)

// ── Public pose struct ───────────────────────────────────────
struct Pose {
    float x;      // metres, +right
    float y;      // metres, +forward
    float theta;  // radians, +CCW
};

// ── Public API ───────────────────────────────────────────────

// Call once before first Update. Initialises MPU6050, calibrates gyro bias.
// Keep robot still for ~1 second during calibration.
void Odometry_Init();

// Call every 10 ms (from a dedicated FreeRTOS task or timer callback).
// Reads encoder deltas + IMU gyro, integrates pose.
void Odometry_Update();

// Zero pose and encoder accumulators. Call when robot is at the
// known start position before autonomous.
void Odometry_ResetPose();

// Thread-safe pose snapshot (uses a portMUX critical section).
Pose Odometry_GetPose();
