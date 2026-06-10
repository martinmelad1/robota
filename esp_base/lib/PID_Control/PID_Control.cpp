// ============================================================
//  PID_CONTROL.CPP  — v5 (PID_v1 Integrated Velocity Control)
// ============================================================

#include "PID_Control.h"
#include "WorldState.h"
#include <PID_v1.h>

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
static unsigned long lastGateTime = 0;

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

// Link GUI PID Tuning to PID_v1 instances
void PID_SetGains(int mode, int wheel, float Kp, float Ki, float Kd) {
  if (wheel < 0 || wheel > 3)
    return;
  // mode 0 handles velocity since position loop was deleted
  velGains[wheel] = {Kp, Ki, Kd};
  pidVel[wheel].SetTunings(Kp, Ki, Kd);
  Serial.printf("[PID] VEL W%d: Kp=%.3f Ki=%.3f Kd=%.3f\n", wheel, Kp, Ki, Kd);
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
  // Pass latest loop data directly into telemetry
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

  pinMode(ENC_A_FL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_FL), isrFL_A, RISING);
  pinMode(ENC_B_FL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FL), isrFL_B, RISING);
  pinMode(ENC_A_FR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_FR), isrFR_A, RISING);
  pinMode(ENC_B_FR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FR), isrFR_B, RISING);
  pinMode(ENC_A_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RL), isrRL_A, RISING);
  pinMode(ENC_B_RL, INPUT_PULLUP);
  pinMode(ENC_A_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isrRR_A, RISING);
  pinMode(ENC_B_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_B_RR), isrRR_B, RISING);

  // Initialize PID instances
  for (int i = 0; i < 4; i++) {
    pidVel[i].SetOutputLimits(-100.0, 100.0); // Limits PID correction trim
    pidVel[i].SetSampleTime(PID_SAMPLE_MS);
    pidVel[i].SetMode(AUTOMATIC);
  }
  lastGateTime = millis();
}

// ==========================================
// 8. COMPUTE — PID Controlled Velocity
// ==========================================
static double finalPWM[4] = {0, 0, 0, 0};

void PID_Compute(float Vx, float Vy, float Wz) {
  // Mecanum inverse kinematics
  double tFL = (double)(Vy + Vx - Wz);
  double tFR = (double)(Vy - Vx + Wz);
  double tRL = (double)(Vy - Vx - Wz);
  double tRR = (double)(Vy + Vx + Wz);

  if (tRL > 0.1)
    global_dirRL = 1;
  else if (tRL < -0.1)
    global_dirRL = -1;

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

    // Apply low-pass filter natively keeping signed values
    velAct[0] = 0.5 * velAct[0] + 0.5 * (double)dFL;
    velAct[1] = 0.5 * velAct[1] + 0.5 * (double)dFR;
    velAct[2] = 0.5 * velAct[2] + 0.5 * (double)dRL;
    velAct[3] = 0.5 * velAct[3] + 0.5 * (double)dRR;

    // Assign Setpoints
    velSet[0] = tFL;
    velSet[1] = tFR;
    velSet[2] = tRL;
    velSet[3] = tRR;

    // Maintain original variables requested
    g_velActFL = (float)velAct[0];
    g_velActFR = (float)velAct[1];
    g_velActRL = (float)velAct[2];
    g_velActRR = (float)velAct[3];
    g_velSetFL = (float)tFL;
    g_velSetFR = (float)tFR;
    g_velSetRL = (float)tRL;
    g_velSetRR = (float)tRR;

    bool stopped = (fabs(Vx) < 0.01f && fabs(Vy) < 0.01f && fabs(Wz) < 0.01f);

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
        // Combine Feed-Forward proportional base + Signed PID trimming output
        double ff = fabs(targets[i]) * FF_GAIN;
        double sign = (targets[i] >= 0.0) ? 1.0 : -1.0;

        double pwm = sign * ff + velOut[i];
        if (pwm > 255.0)
          pwm = 255.0;
        if (pwm < -255.0)
          pwm = -255.0;

        finalPWM[i] = pwm;
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