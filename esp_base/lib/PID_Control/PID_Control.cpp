// ============================================================
//  PID_CONTROL.CPP  — v5 (PID_v1 Integrated Velocity Control)
// ============================================================

#include "PID_Control.h"
#include "UART_Master.h"
#include "WorldState.h"
#include <PID_v1.h>
#include "Odometry.h" // Needed to fall back to encoder yaw if PID_USE_IMU_YAW=false

// ── PID IMU Flags ──────────────────────────────────────────────
// PID_USE_IMU_VELOCITY : Blends IMU accel integration with encoder delta ticks.
// PID_USE_IMU_YAW      : Uses IMU yaw to hold heading direction.
//   When true, the PID corrects speed magnitude from unsigned encoder ticks
//   (pure wheel speed, slip-immune direction) scaled by SPEED_HEADING_GAIN.
//   Direction is fully determined by the IMU-corrected mecanum IK outputs.
static constexpr bool PID_USE_IMU_VELOCITY = false;
static constexpr bool PID_USE_IMU_YAW      = true;

// State variables for IMU velocity integration
static float g_imuVelX = 0.0f;
static float g_imuVelY = 0.0f;
static unsigned long g_imuVelLastMs = 0;

extern QueueHandle_t pidMailbox;

// ==========================================
// 1. HARDWARE PINS  (DO NOT CHANGE)
// ==========================================
static const int PWM_FL = 12, DIR_A_FL = 32, DIR_B_FL = 27;
static const int ENC_A_FL = 35, ENC_B_FL = 36;

static const int PWM_FR = 26, DIR_A_FR = 14, DIR_B_FR = 25;
static const int ENC_A_FR = 34, ENC_B_FR = 39;

static const int PWM_RL = 33, DIR_A_RL = 13, DIR_B_RL = 15;
static const int ENC_A_RL = 18, ENC_B_RL = 19;

static const int PWM_RR = 17, DIR_A_RR = 16, DIR_B_RR = 4;
static const int ENC_A_RR = 21, ENC_B_RR = 22;

static const int PWM_FREQ = 25000;
static const int PWM_RES = 8; // 0–255

// ==========================================
// 2. PHYSICAL CONSTANTS
// ==========================================
static const float WHEEL_DIAMETER_M = 0.097f;
static const float WHEEL_CIRCUM_M = 3.14159265f * WHEEL_DIAMETER_M;
static const int ENCODER_PPR = 748;
static const int PID_SAMPLE_MS = 50;

float speedToTicks(float speed_ms) {
  return (speed_ms / WHEEL_CIRCUM_M) * ENCODER_PPR * (PID_SAMPLE_MS / 1000.0f);
}

// ==========================================
// 3. ENCODER STATE
// ==========================================
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;
portMUX_TYPE tickMux = portMUX_INITIALIZER_UNLOCKED;

// Original variables preserved for system integration
static float g_velActFL = 0, g_velActFR = 0, g_velActRL = 0, g_velActRR = 0;
static float g_velSetFL = 0, g_velSetFR = 0, g_velSetRL = 0, g_velSetRR = 0;
static long g_prevFL = 0, g_prevFR = 0, g_prevRL = 0, g_prevRR = 0;

volatile int global_dirRL = 1;

void IRAM_ATTR isrFL_A() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksFL += (digitalRead(ENC_B_FL) == LOW) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrFL_B() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksFL += (digitalRead(ENC_A_FL) == HIGH) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrFR_A() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksFR += (digitalRead(ENC_B_FR) == LOW) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrFR_B() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksFR += (digitalRead(ENC_A_FR) == HIGH) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrRL_A() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksRL += 2 * global_dirRL;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrRR_A() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksRR += (digitalRead(ENC_B_RR) == LOW) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}
void IRAM_ATTR isrRR_B() {
  portENTER_CRITICAL_ISR(&tickMux);
  ticksRR += (digitalRead(ENC_A_RR) == HIGH) ? +1 : -1;
  portEXIT_CRITICAL_ISR(&tickMux);
}

// ==========================================
// 4. PID_v1 INSTANCES & GAINS
// ==========================================
static double velSet[4] = {0, 0, 0, 0};
static double velAct[4] = {0, 0, 0, 0};
static double velOut[4] = {0, 0, 0, 0};

static WheelGains velGains[4] = {{2.5f, 2.0f, 0.0f},
                                 {2.5f, 2.0f, 0.0f},
                                 {2.5f, 2.0f, 0.0f},
                                 {2.5f, 2.0f, 0.0f}};

static PID pidVel[4] = {PID(&velAct[0], &velOut[0], &velSet[0], velGains[0].Kp,
                            velGains[0].Ki, velGains[0].Kd, DIRECT),
                        PID(&velAct[1], &velOut[1], &velSet[1], velGains[1].Kp,
                            velGains[1].Ki, velGains[1].Kd, DIRECT),
                        PID(&velAct[2], &velOut[2], &velSet[2], velGains[2].Kp,
                            velGains[2].Ki, velGains[2].Kd, DIRECT),
                        PID(&velAct[3], &velOut[3], &velSet[3], velGains[3].Kp,
                            velGains[3].Ki, velGains[3].Kd, DIRECT)};

static const float FF_GAIN = 1.5f;
// Speed gain applied to unsigned encoder magnitude when PID_USE_IMU_YAW is active.
// Runtime-adjustable via PID_SetSpeedGain() so it can be tuned from the GUI.
static float g_speedHeadingGain = 1.0f;
static unsigned long lastGateTime = 0;

// ==========================================
// 4b. HEADING INNER-LOOP PID
// ==========================================
// This is the INNER loop of the cascade:
//   input  = heading error (degrees, wrapped ±180) — stored in hdgAct
//   setpoint = 0 (drive error to zero)
//   output = Wz correction (ticks/sample) — added to commanded Wz
//
// DIRECT mode: imu_yaw_deg increases CCW (MPU6050 right-hand Z-up convention).
//   heading_err = target_yaw - current_yaw.
//   Robot drifts CW → current_yaw falls → heading_err > 0 → need Wz > 0 (CCW).
//   DIRECT: positive error → positive output → Wz correction positive ✓
//
// Output limits ±3.0 ticks/sample keeps the heading correction
// gentle (adjust via PID_TUNE:hdg:0:Kp:Ki:Kd from GUI).
static double    hdgAct   = 0.0;   // current heading error (deg, fed as input)
static double    hdgOut_d = 0.0;   // Wz correction output (ticks/sample)
static double    hdgSet   = 0.0;   // always 0 — drive error to zero
static WheelGains hdgGains = {0.05f, 0.001f, 0.005f};
static PID hdgPID(&hdgAct, &hdgOut_d, &hdgSet,
                  hdgGains.Kp, hdgGains.Ki, hdgGains.Kd, DIRECT);

// ==========================================
// 5. DRIVE SPEED
// ==========================================
static float g_driveSpeed = 0.50f;

void PID_SetDriveSpeed(float mps) {
  if (mps < 0.10f)
    mps = 0.10f;
  if (mps > 1.50f)
    mps = 1.50f;
  g_driveSpeed = mps;
  Serial.printf("[DRV] Drive speed set to %.2f m/s\n", g_driveSpeed);
}
float PID_GetDriveSpeed() { return g_driveSpeed; }

// PID_SetGains — routes tuning to the correct PID instance.
//   mode 0 = outer velocity loop  (wheel 0–3 = FL/FR/RL/RR)
//   mode 1 = inner heading loop   (wheel param ignored)
void PID_SetGains(int mode, int wheel, float Kp, float Ki, float Kd) {
  if (mode == 1) {
    // Inner heading PID
    hdgGains = {Kp, Ki, Kd};
    hdgPID.SetTunings(Kp, Ki, Kd);
    Serial.printf("[PID] HDG: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    return;
  }
  // mode 0 = outer velocity PID per wheel
  if (wheel < 0 || wheel > 3) return;
  velGains[wheel] = {Kp, Ki, Kd};
  pidVel[wheel].SetTunings(Kp, Ki, Kd);
  Serial.printf("[PID] VEL W%d: Kp=%.3f Ki=%.3f Kd=%.3f\n", wheel, Kp, Ki, Kd);
}

// Tune the speed magnitude gain used in IMU-heading mode (replaces the constant).
void PID_SetSpeedGain(float gain) {
  if (gain < 0.1f) gain = 0.1f;
  if (gain > 5.0f) gain = 5.0f;
  g_speedHeadingGain = gain;
  Serial.printf("[PID] SpeedGain set to %.3f\n", g_speedHeadingGain);
}
float PID_GetSpeedGain() { return g_speedHeadingGain; }

// Returns average actual wheel speed in m/s (unsigned, for dashboard display).
float PID_GetAvgSpeedMps() {
  // velAct is in ticks/sample (50 ms). Convert back to m/s.
  // WHEEL_CIRCUM_M / ENCODER_PPR * (1000 / PID_SAMPLE_MS) = m per tick * samples/sec
  const float TICKS_TO_MPS = WHEEL_CIRCUM_M / (float)ENCODER_PPR * (1000.0f / (float)PID_SAMPLE_MS);
  float avg = (fabs(velAct[0]) + fabs(velAct[1]) + fabs(velAct[2]) + fabs(velAct[3])) * 0.25f;
  return avg * TICKS_TO_MPS;
}

void PID_GetActualSpeeds(double *fl, double *fr, double *rl, double *rr) {
  if (fl)
    *fl = velAct[0];
  if (fr)
    *fr = velAct[1];
  if (rl)
    *rl = velAct[2];
  if (rr)
    *rr = velAct[3];
}

void PID_GetTelemetry(PIDTelemetry &t) {
  // Outer velocity loop data
  t.velSetFL = (float)velSet[0];
  t.velSetFR = (float)velSet[1];
  t.velSetRL = (float)velSet[2];
  t.velSetRR = (float)velSet[3];

  t.velActFL = (float)velAct[0];
  t.velActFR = (float)velAct[1];
  t.velActRL = (float)velAct[2];
  t.velActRR = (float)velAct[3];

  for (int i = 0; i < 4; i++) {
    t.velGains[i] = velGains[i];
  }

  // Inner heading loop data (for real-time dashboard tuning graphs)
  t.hdgErr   = (float)hdgAct;    // heading error in degrees
  t.hdgOut   = (float)hdgOut_d;  // Wz correction output
  t.hdgGains = hdgGains;
}

// ==========================================
// 6. MOTOR DRIVER
// ==========================================
static void driveMotor(int channel, int in1, int in2, double pwm) {
  bool fwd = (pwm >= 0.0);
  double mag = fabs(pwm);
  if (mag < 8.0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(channel, 0);
    return;
  }
  if (mag > 255.0)
    mag = 255.0;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  ledcWrite(channel, (uint32_t)mag);
}

// ==========================================
// 7. INIT
// ==========================================
void PID_Init() {
  for (auto pin : {DIR_A_FL, DIR_B_FL, DIR_A_FR, DIR_B_FR, DIR_A_RL, DIR_B_RL,
                   DIR_A_RR, DIR_B_RR})
    pinMode(pin, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_FL, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_FR, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_RL, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_RR, 3);

  pinMode(ENC_A_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_FL), isrFL_A, RISING);
  pinMode(ENC_B_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FL), isrFL_B, RISING);
  pinMode(ENC_A_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_FR), isrFR_A, RISING);
  pinMode(ENC_B_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FR), isrFR_B, RISING);
  pinMode(ENC_A_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RL), isrRL_A, RISING);
  pinMode(ENC_B_RL, INPUT_PULLUP);
  pinMode(ENC_A_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isrRR_A, RISING);
  pinMode(ENC_B_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_B_RR), isrRR_B, RISING);

  // Initialize outer velocity PID instances
  for (int i = 0; i < 4; i++) {
    pidVel[i].SetOutputLimits(-100.0, 100.0); // PID correction trim ±100
    pidVel[i].SetSampleTime(PID_SAMPLE_MS);
    pidVel[i].SetMode(AUTOMATIC);
  }

  // Initialize inner heading PID
  // Output = Wz correction in ticks/sample. ±3.0 is gentle; tune Kp from GUI.
  // Sample time = 10 ms (matches PID_Compute call rate) so the inner loop
  // runs 5× faster than the 50 ms outer velocity gate — true cascade benefit.
  hdgPID.SetOutputLimits(-3.0, 3.0);
  hdgPID.SetSampleTime(10); // 10 ms — fast inner loop (PID_Compute is called every 10 ms)
  hdgPID.SetMode(AUTOMATIC);

  lastGateTime = millis();
}

// ==========================================
// 8. COMPUTE — PID Controlled Velocity
// ==========================================
static double finalPWM[4] = {0, 0, 0, 0};

void PID_Compute(float Vx, float Vy, float Wz) {
  // Save original commanded velocities BEFORE heading PID injects its Wz.
  // Used below for 'stopped' detection so the velocity PID integrators reset
  // correctly when the chassis is commanded to stop, regardless of any
  // residual heading correction that might be non-zero.
  const float Vx_cmd = Vx;
  const float Vy_cmd = Vy;
  const float Wz_cmd = Wz;

  // ── 1. INNER LOOP: Heading PID (IMU yaw) ───────────────
  // Runs every PID_Compute call (10 ms) — true fast inner loop.
  // hdgPID sample time = 10 ms; outer velocity gate = 50 ms.
  // While turning (Wz commanded): track heading, reset integrator.
  // While translating (Vx or Vy ≠ 0): run heading PID → Wz correction.
  // While stopped: freeze output (no integrator windup).
  if (PID_USE_IMU_YAW) {
      static float target_yaw = 0.0f;
      static bool  is_turning  = false;
      float current_yaw = imu_yaw_deg;
      bool  translating = (fabs(Vx_cmd) > 0.01f || fabs(Vy_cmd) > 0.01f);

      if (fabs(Wz_cmd) > 0.01f) {
          // Intentional rotation: track heading, disable integrator to avoid windup
          is_turning = true;
          target_yaw = current_yaw;
          hdgPID.SetMode(MANUAL);
          hdgOut_d = 0.0;
          hdgPID.SetMode(AUTOMATIC);
      } else {
          if (is_turning) {
              // Just finished a turn — lock reached heading as new target
              target_yaw = current_yaw;
              is_turning = false;
          }
          if (translating) {
              // Compute heading error (wrapped ±180°) and feed to inner PID
              float heading_err = target_yaw - current_yaw;
              while (heading_err >  180.0f) heading_err -= 360.0f;
              while (heading_err < -180.0f) heading_err += 360.0f;
              hdgAct = (double)heading_err; // input to heading PID
              hdgPID.Compute();             // output -> hdgOut_d (Wz correction)
              Wz += (float)hdgOut_d;        // inject into commanded rotation
          } else {
              // Stopped: freeze heading PID output, no integrator windup
              hdgPID.SetMode(MANUAL);
              hdgOut_d = 0.0;
              hdgPID.SetMode(AUTOMATIC);
          }
      }
  }

  // ── 2. Mecanum inverse kinematics ──────────────────────────
  double tFL = (double)(Vy + Vx - Wz);
  double tFR = (double)(Vy - Vx + Wz);
  double tRL = (double)(Vy - Vx - Wz);
  double tRR = (double)(Vy + Vx + Wz);

  // Update RL direction under the same mutex the ISR reads it in,
  // preventing a stale read mid-write (even though int write is atomic on LX6).
  {
    int new_dirRL = (tRL > 0.1) ? 1 : (tRL < -0.1) ? -1 : global_dirRL;
    portENTER_CRITICAL(&tickMux);
    global_dirRL = new_dirRL;
    portEXIT_CRITICAL(&tickMux);
  }

  unsigned long now = millis();
  // Execute velocity measurement and PID correction every PID_SAMPLE_MS (50ms)
  if (now - lastGateTime >= (unsigned long)PID_SAMPLE_MS) {
    lastGateTime = now;

    long curFL, curFR, curRL, curRR;
    // Keep the critical section short for safety
    portENTER_CRITICAL(&tickMux);
    curFL = ticksFL;
    curFR = ticksFR;
    curRL = ticksRL;
    curRR = ticksRR;
    portEXIT_CRITICAL(&tickMux);

    long dFL = curFL - g_prevFL;
    long dFR = curFR - g_prevFR;
    long dRL = curRL - g_prevRL;
    long dRR = curRR - g_prevRR;

    // Spike rejection
    if (abs(dFL) > 150)
      dFL = (long)velAct[0];
    if (abs(dFR) > 150)
      dFR = (long)velAct[1];
    if (abs(dRL) > 300)
      dRL = (long)velAct[2];
    if (abs(dRR) > 150)
      dRR = (long)velAct[3];

    g_prevFL = curFL;
    g_prevFR = curFR;
    g_prevRL = curRL;
    g_prevRR = curRR;

    // ── 3. Actual velocity measurement ─────────────────────
    if (PID_USE_IMU_VELOCITY) {
        // Optional: double-integrate IMU acceleration for velAct
        unsigned long nowMs = millis();
        float dtImu = (nowMs - g_imuVelLastMs) / 1000.0f;
        g_imuVelLastMs = nowMs;
        if (dtImu > 0.0f && dtImu < 0.5f) {
            g_imuVelX += imu_ax_mps2 * dtImu;
            g_imuVelY += imu_ay_mps2 * dtImu;
            g_imuVelX *= 0.95f;
            g_imuVelY *= 0.95f;
            if (Vx == 0 && Vy == 0 && Wz == 0) {
                g_imuVelX = 0.0f;
                g_imuVelY = 0.0f;
            }
            velAct[0] = speedToTicks(g_imuVelY + g_imuVelX);
            velAct[1] = speedToTicks(g_imuVelY - g_imuVelX);
            velAct[2] = speedToTicks(g_imuVelY - g_imuVelX);
            velAct[3] = speedToTicks(g_imuVelY + g_imuVelX);
        }
    } else if (PID_USE_IMU_YAW) {
        // IMU-heading mode: encoders give pure speed magnitude (direction = IMU).
        // velAct = |encoder ticks| × g_speedHeadingGain (unsigned, slip-immune direction).
        // velSet will also be unsigned so PID corrects speed only; the
        // mecanum IK target signs (tFL/tFR/tRL/tRR) carry direction to driveMotor.
        velAct[0] = 0.5 * velAct[0] + 0.5 * (g_speedHeadingGain * fabs((double)dFL));
        velAct[1] = 0.5 * velAct[1] + 0.5 * (g_speedHeadingGain * fabs((double)dFR));
        velAct[2] = 0.5 * velAct[2] + 0.5 * (g_speedHeadingGain * fabs((double)dRL));
        velAct[3] = 0.5 * velAct[3] + 0.5 * (g_speedHeadingGain * fabs((double)dRR));
    } else {
        // Default encoder-only: signed IIR low-pass (direction from encoder sign)
        velAct[0] = 0.5 * velAct[0] + 0.5 * (double)dFL;
        velAct[1] = 0.5 * velAct[1] + 0.5 * (double)dFR;
        velAct[2] = 0.5 * velAct[2] + 0.5 * (double)dRL;
        velAct[3] = 0.5 * velAct[3] + 0.5 * (double)dRR;
    }

    // Assign Setpoints
    // When using IMU heading, setpoints are unsigned magnitude (speed only)
    // so the PID error = |target speed| - |actual speed|. Direction is
    // handled entirely by the mecanum IK sign applied in driveMotor().
    if (PID_USE_IMU_YAW) {
        velSet[0] = fabs(tFL);
        velSet[1] = fabs(tFR);
        velSet[2] = fabs(tRL);
        velSet[3] = fabs(tRR);
    } else {
        velSet[0] = tFL;
        velSet[1] = tFR;
        velSet[2] = tRL;
        velSet[3] = tRR;
    }

    // Maintain original variables requested
    g_velActFL = (float)velAct[0];
    g_velActFR = (float)velAct[1];
    g_velActRL = (float)velAct[2];
    g_velActRR = (float)velAct[3];
    g_velSetFL = (float)tFL;
    g_velSetFR = (float)tFR;
    g_velSetRL = (float)tRL;
    g_velSetRR = (float)tRR;

    // Use original commanded velocities (before heading PID modified Wz) so
    // the velocity PID integrators always reset when the chassis is commanded
    // to stop, even if a residual heading correction Wz is non-zero.
    bool stopped = (fabs(Vx_cmd) < 0.01f && fabs(Vy_cmd) < 0.01f && fabs(Wz_cmd) < 0.01f);

    for (int i = 0; i < 4; i++) {
      if (stopped) {
        // Reset integrator windup natively when stopped
        pidVel[i].SetMode(MANUAL);
        velOut[i] = 0.0;
        pidVel[i].SetMode(AUTOMATIC);
      } else {
        pidVel[i].Compute();
      }
    }

    double targets[4] = {tFL, tFR, tRL, tRR};
    for (int i = 0; i < 4; i++) {
      if (fabs(targets[i]) < 0.1) {
        finalPWM[i] = 0.0;
      } else {
        // Feed-Forward proportional base + PID trim.
        // velOut[i] is signed: positive = need more speed, negative = overspeed.
        // We clamp (ff + velOut) to ≥ 0 before applying the direction sign so
        // a large braking correction can never flip the motor to the wrong
        // direction — it can only reduce PWM toward zero, not reverse it.
        double ff   = fabs(targets[i]) * FF_GAIN;
        double sign = (targets[i] >= 0.0) ? 1.0 : -1.0;

        double mag = ff + velOut[i];      // add signed PID trim to FF magnitude
        if (mag < 0.0) mag = 0.0;        // clamp: never flip direction via PID
        if (mag > 255.0) mag = 255.0;    // clamp upper limit

        finalPWM[i] = sign * mag;
      }
    }
  }

  driveMotor(0, DIR_A_FL, DIR_B_FL, finalPWM[0]);
  driveMotor(1, DIR_A_FR, DIR_B_FR, finalPWM[1]);
  driveMotor(2, DIR_A_RL, DIR_B_RL, finalPWM[2]);
  driveMotor(3, DIR_A_RR, DIR_B_RR, finalPWM[3]);
}

// ==========================================
// 9. TASK
// ==========================================
void PID_TaskCode(void *pvParameters) {
  ChassisMotion motion;
  float Vx = 0, Vy = 0, Wz = 0;

  bool distActive = false;
  long distTargetTicks = 0;
  long distStartFL = 0;
  float distVx = 0, distVy = 0;

  while (true) {
    if (xQueueReceive(pidMailbox, &motion, 0) == pdTRUE) {
      distActive = false;
      Vx = Vy = Wz = 0;

      float spd = speedToTicks(motion.speed);
      float diagSpd = spd * 0.7071f;
      float omgTicks = speedToTicks(motion.omega);

      switch (motion.move_type) {
      case DriveCommand::CMD_FWD:
        Vy = spd;
        break;
      case DriveCommand::CMD_BWD:
        Vy = -spd;
        break;
      case DriveCommand::CMD_LEFT:
        Vx = -spd;
        break;
      case DriveCommand::CMD_RIGHT:
        Vx = spd;
        break;
      case DriveCommand::CMD_FWD_L:
        Vy = diagSpd;
        Vx = -diagSpd;
        break;
      case DriveCommand::CMD_FWD_R:
        Vy = diagSpd;
        Vx = diagSpd;
        break;
      case DriveCommand::CMD_BWD_L:
        Vy = -diagSpd;
        Vx = -diagSpd;
        break;
      case DriveCommand::CMD_BWD_R:
        Vy = -diagSpd;
        Vx = diagSpd;
        break;
      case DriveCommand::CMD_ROT_L:
        Wz = -omgTicks;
        break;
      case DriveCommand::CMD_ROT_R:
        Wz = omgTicks;
        break;
      case DriveCommand::CMD_DIRECT:
        Vx = speedToTicks(motion.Vx * 0.5f);
        Vy = speedToTicks(motion.Vy * 0.5f);
        Wz = speedToTicks(motion.Wz * 0.5f);
        break;
      case DriveCommand::CMD_DIST:
        distVx = speedToTicks(motion.Vx * motion.speed / 0.5f);
        distVy = speedToTicks(motion.Vy * motion.speed / 0.5f);
        distTargetTicks = motion.tickTarget;
        portENTER_CRITICAL(&tickMux);
        distStartFL = ticksFL;
        portEXIT_CRITICAL(&tickMux);
        distActive = true;
        Vx = distVx;
        Vy = distVy;
        break;
      case DriveCommand::CMD_STOP:
      default:
        break;
      }
    }

    if (distActive) {
      long curFL;
      portENTER_CRITICAL(&tickMux);
      curFL = ticksFL;
      portEXIT_CRITICAL(&tickMux);
      if (abs(curFL - distStartFL) >= distTargetTicks) {
        distActive = false;
        Vx = Vy = Wz = 0;
      } else {
        Vx = distVx;
        Vy = distVy;
        Wz = 0;
      }
    }

    PID_Compute(Vx, Vy, Wz);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void PID_StartTask() {
  xTaskCreatePinnedToCore(PID_TaskCode, "PIDTask", 4096, NULL, 3, NULL, 1);
}