// ============================================================
//  ODOMETRY.CPP — Encoder-only mecanum odometry
//
//  Heading is integrated purely from the encoder forward
//  kinematics (dW term). No IMU is used.
//
//  When you have the MPU6050 wired up, add it back by:
//    1. Including <Wire.h> and <MPU6050.h>
//    2. Calling mpu.initialize() + gyro bias calibration in Init()
//    3. Replacing the pure-encoder theta line in Update() with
//       the 98/2 complementary filter from the previous version.
//
//  Thread safety:
//    - Encoder ticks are shared with PID_Control.cpp via
//      extern volatile ticksFL/FR/RL/RR + tickMux.
//    - Pose is protected by poseMux for cross-task reads.
// ============================================================

#include "Odometry.h"
#include "UART_Master.h"   // for imu_yaw_deg
#include <math.h>
#include <Arduino.h>

// ── Odometry IMU Flags ───────────────────────────────────────
// ODO_USE_IMU_HEADING : Uses IMU yaw instead of encoder difference for theta.
// ODO_USE_IMU_POSITION: Double integrates IMU accel for X,Y instead of encoders.
static constexpr bool ODO_USE_IMU_HEADING  = false;
static constexpr bool ODO_USE_IMU_POSITION = false;

static float odo_imu_vel_x = 0.0f;
static float odo_imu_vel_y = 0.0f;

// ── Shared encoder state (defined in PID_Control.cpp) ────────
extern volatile long ticksFL;
extern volatile long ticksFR;
extern volatile long ticksRL;
extern volatile long ticksRR;
extern portMUX_TYPE  tickMux;

// ── Module-private state ─────────────────────────────────────
static Pose         g_pose       = {0.0f, 0.0f, 0.0f};
static portMUX_TYPE poseMux      = portMUX_INITIALIZER_UNLOCKED;

static long         prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;
static unsigned long last_odo_us = 0;

// ── Public API ───────────────────────────────────────────────

void Odometry_Init() {
    // Snapshot current ticks so the first delta is zero
    portENTER_CRITICAL(&tickMux);
    prev_FL = ticksFL;
    prev_FR = ticksFR;
    prev_RL = ticksRL;
    prev_RR = ticksRR;
    portEXIT_CRITICAL(&tickMux);

    last_odo_us = micros();
    Serial.println("[ODO] Encoder-only odometry initialised.");
}

void Odometry_Update() {
    // ── Step 1: dt in seconds ────────────────────────────────
    unsigned long now_us = micros();
    float dt = (now_us - last_odo_us) * 1e-6f;
    last_odo_us = now_us;
    if (dt <= 0.0f || dt > 0.5f) return;   // skip first call or large spikes

    // ── Step 2: Atomic tick snapshot ─────────────────────────
    long cur_FL, cur_FR, cur_RL, cur_RR;
    portENTER_CRITICAL(&tickMux);
    cur_FL = ticksFL;
    cur_FR = ticksFR;
    cur_RL = ticksRL;
    cur_RR = ticksRR;
    portEXIT_CRITICAL(&tickMux);

    long d_FL = cur_FL - prev_FL;
    long d_FR = cur_FR - prev_FR;
    long d_RL = cur_RL - prev_RL;
    long d_RR = cur_RR - prev_RR;
    prev_FL = cur_FL;
    prev_FR = cur_FR;
    prev_RL = cur_RL;
    prev_RR = cur_RR;

    // ── Step 3: Ticks → wheel arc angles (radians) ───────────
    // ISRs count A+B both on RISING = 2x mode → 2 × 374 = 748 ticks/rev
    const float k = (2.0f * float(M_PI)) / float(ODO_TICKS_PER_REV);
    float p_FL = float(d_FL) * k;
    float p_FR = float(d_FR) * k;
    float p_RL = float(d_RL) * k;
    float p_RR = float(d_RR) * k;

    // ── Step 4: Mecanum forward kinematics ───────────────────
    // Robot-frame displacement this tick interval:
    //   dVx = lateral  (+ = right)
    //   dVy = forward  (+ = forward)
    //   dW  = heading change (+ = CCW)
    const float R = ODO_WHEEL_RADIUS;
    const float L = ODO_LX + ODO_LY;

    float dVx = (R / 4.0f) * (-p_FL + p_FR + p_RL - p_RR);
    float dVy = (R / 4.0f) * ( p_FL + p_FR + p_RL + p_RR);
    float dW  = (R / (4.0f * L)) * (-p_FL + p_FR - p_RL + p_RR);

    // ── Step 5: Heading ───────────────────────────────────────
    float new_theta;

    if (ODO_USE_IMU_HEADING) {
        // Convert IMU yaw (degrees, ±180) to radians matching odometry frame.
        new_theta = (float)imu_yaw_deg * (float(M_PI) / 180.0f);
        while (new_theta >  float(M_PI)) new_theta -= 2.0f * float(M_PI);
        while (new_theta < -float(M_PI)) new_theta += 2.0f * float(M_PI);
    } else {
        // Encoder-only heading integration
        new_theta = g_pose.theta + dW;
        while (new_theta >  float(M_PI)) new_theta -= 2.0f * float(M_PI);
        while (new_theta < -float(M_PI)) new_theta += 2.0f * float(M_PI);
    }

    // ── Step 6: Update Global Position (X, Y) ────────────────
    float mid_theta = (g_pose.theta + new_theta) * 0.5f;
    float cos_th = cosf(mid_theta);
    float sin_th = sinf(mid_theta);

    if (ODO_USE_IMU_POSITION) {
        // Double integrate IMU acceleration to get world dx/dy
        odo_imu_vel_x += imu_ax_mps2 * dt;
        odo_imu_vel_y += imu_ay_mps2 * dt;
        
        // Simple friction decay to limit runaway drift when stationary
        odo_imu_vel_x *= 0.99f;
        odo_imu_vel_y *= 0.99f;

        g_pose.x += odo_imu_vel_x * dt;
        g_pose.y += odo_imu_vel_y * dt;
    } else {
        // Encoder-based forward kinematics
        g_pose.x += (dVx * cos_th - dVy * sin_th);
        g_pose.y += (dVx * sin_th + dVy * cos_th);
    }

    // ── Step 7: Accumulate pose (mutex-protected write) ───────
    portENTER_CRITICAL(&poseMux);
    g_pose.theta  = new_theta;
    portEXIT_CRITICAL(&poseMux);
}

void Odometry_ResetPose() {
    // Re-snapshot ticks so next delta starts from zero, not accumulated total
    portENTER_CRITICAL(&tickMux);
    prev_FL = ticksFL;
    prev_FR = ticksFR;
    prev_RL = ticksRL;
    prev_RR = ticksRR;
    portEXIT_CRITICAL(&tickMux);

    portENTER_CRITICAL(&poseMux);
    g_pose = {0.0f, 0.0f, 0.0f};
    portEXIT_CRITICAL(&poseMux);

    Serial.println("[ODO] Pose reset to (0, 0, 0°).");
}

Pose Odometry_GetPose() {
    Pose snapshot;
    portENTER_CRITICAL(&poseMux);
    snapshot = g_pose;
    portEXIT_CRITICAL(&poseMux);
    return snapshot;
}
