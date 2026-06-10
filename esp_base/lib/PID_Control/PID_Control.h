#pragma once

// ============================================================
//  PID_CONTROL.H  — v3  (velocity-only)
//
//  Architecture:
//    • One velocity PID loop per wheel (50 ms sample period).
//    • Feed-forward (FF) provides base PWM proportional to setpoint.
//    • PID corrects the error on top of FF.
//    • Position PID removed — not needed for this robot.
//
//  Speed command:
//    PID_SetDriveSpeed(float m_s)  sets global drive speed for manual cmds.
//    Default 0.5 m/s, range 0.10 – 1.50 m/s.
// ============================================================

struct WheelGains {
  float Kp, Ki, Kd;
  WheelGains() : Kp(0.f), Ki(0.f), Kd(0.f) {}
  WheelGains(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
};

struct PIDTelemetry {
  // Velocity set-points and actual values (ticks/sample, signed)
  float velSetFL, velSetFR, velSetRL, velSetRR;
  float velActFL, velActFR, velActRL, velActRR;
  // Gains — stored as arrays, no pointer arithmetic
  WheelGains velGains[4];
  // Legacy named refs kept so StateMachine JSON builder still compiles
  WheelGains &velGainFL = velGains[0];
  WheelGains &velGainFR = velGains[1];
  WheelGains &velGainRL = velGains[2];
  WheelGains &velGainRR = velGains[3];
};

// Initialise GPIO, PWM channels, encoder ISRs, PID instances.
void PID_Init();

// Start the FreeRTOS PID task (call after queues are created).
void PID_StartTask();

// Drive all four wheels; Vx/Vy/Wz are in ticks/sample (signed).
// Called internally by PID_TaskCode.
void PID_Compute(float Vx, float Vy, float Wz);

// Read low-pass filtered actual velocities (ticks/sample, signed).
void PID_GetActualSpeeds(double *fl, double *fr, double *rl, double *rr);

// Set PID gains at runtime.
// mode 0 = velocity; wheel 0-3 = FL FR RL RR.
void PID_SetGains(int mode, int wheel, float Kp, float Ki, float Kd);

// Fill a PIDTelemetry struct with the latest values.
void PID_GetTelemetry(PIDTelemetry &t);

// Global drive speed (m/s) set by GUI SPEED: command.
void PID_SetDriveSpeed(float mps);
float PID_GetDriveSpeed();

// Convert m/s to encoder ticks per 50 ms sample (public so StateMachine can use
// it).
float speedToTicks(float speed_ms);
