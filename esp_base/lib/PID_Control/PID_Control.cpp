#include "PID_Control.h"
#include <PID_v1.h>
#include "WorldState.h"
#include "Odometry.h"   // for heading-hold feedback

extern QueueHandle_t pidMailbox;

// ==========================================
// 1. HARDWARE PIN DEFINITIONS
// ==========================================

// Front Left (FL) — L298N Channel A
const int PWM_FL   = 12;  // ENA
const int DIR_A_FL = 14;  // IN1
const int DIR_B_FL = 25;  // IN2
const int ENC_A_FL = 36;  // Encoder A (input-only GPIO — no internal pull resistor)
const int ENC_B_FL = 35;  // Encoder B (input-only GPIO — no internal pull resistor)

// Front Right (FR) — L298N Channel B
const int PWM_FR   = 26;  // ENB
const int DIR_A_FR = 32;  // IN3
const int DIR_B_FR = 27;  // IN4
const int ENC_A_FR = 34;  // Encoder A (input-only GPIO — no internal pull resistor)
const int ENC_B_FR = 39;  // Encoder B (input-only GPIO — no internal pull resistor)

// Rear Left (RL) — L298N Channel A
const int PWM_RL   = 33;  // ENA
const int DIR_A_RL = 13;  // IN1
const int DIR_B_RL = 15;  // IN2
const int ENC_A_RL = 18;  // Encoder A
const int ENC_B_RL = 19;  // Encoder B

// Rear Right (RR) — L298N Channel B
const int PWM_RR   = 17;  // ENB
const int DIR_A_RR = 16;  // IN3
const int DIR_B_RR = 4;   // IN4
const int ENC_A_RR = 21;  // Encoder A
const int ENC_B_RR = 22;  // Encoder B

// PWM Channels (ESP32 Specific)
const int freq = 25000; // 25kHz — above human hearing range, eliminates motor coil whine
const int res = 8; // 0-255

// Wheel & Encoder Constants
const float WHEEL_DIAMETER_M = 0.097;  // 97 mm
const float WHEEL_CIRCUM_M   = 3.14159265 * WHEEL_DIAMETER_M; // ~0.3047 m
const int   ENCODER_PPR      = 374;    // Ticks per output-shaft revolution (from datasheet)
const int   PID_SAMPLE_MS    = 50;

// Convert m/s to encoder ticks per sample period
float speedToTicks(float speed_ms) {
  // revolutions per second = speed / circumference
  // ticks per second = rev/s * PPR
  // ticks per sample = ticks/s * (sample_ms / 1000)
  return (speed_ms / WHEEL_CIRCUM_M) * ENCODER_PPR * (PID_SAMPLE_MS / 1000.0f);
}

// ==========================================
// 2. ENCODER VARIABLES & INTERRUPTS
// ==========================================
// Tick counters — both encoder channels (A+B) increment for 2x resolution
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;

// Critical section mutex for atomic tick reads (dual-core safe)
portMUX_TYPE tickMux = portMUX_INITIALIZER_UNLOCKED;

// BUG1 FIX: Quadrature direction decoding.
// A-rising ISR: if B is LOW → forward (+1), if B is HIGH → backward (−1)
// B-rising ISR: if A is HIGH → forward (+1), if A is LOW → backward (−1)
// NOTE: If the robot moves opposite to expected, swap the HIGH/LOW logic per wheel.
void IRAM_ATTR isrFL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFL += (digitalRead(ENC_B_FL) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFL_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFL += (digitalRead(ENC_A_FL) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFR += (digitalRead(ENC_B_FR) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFR += (digitalRead(ENC_A_FR) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }

// ── RL Burnt Encoder Workaround ──
// One channel of RL is burnt, so we can't use quadrature logic.
// We deduce the direction from the command, and add 2 ticks per edge to compensate.
volatile int global_dirRL = 1;
void IRAM_ATTR isrRL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRL += 2 * global_dirRL; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRL_B() { portENTER_CRITICAL_ISR(&tickMux); ticksRL += 2 * global_dirRL; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRR += (digitalRead(ENC_B_RR) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksRR += (digitalRead(ENC_A_RR) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }

// ==========================================
// 3. PID VARIABLES & SETUP
// ==========================================
// ==========================================
// 3. PID VARIABLES & SETUP (COMMENTED OUT FOR PURE OPEN-LOOP)
// ==========================================
// The target speeds (Setpoints)
// double setFL = 0, setFR = 0, setRL = 0, setRR = 0;
// The actual speeds (Inputs)
double inFL = 0, inFR = 0, inRL = 0, inRR = 0; // Keeping these for GUI feedback
// The PWM outputs calculated by PID
// double outFL = 0, outFR = 0, outRL = 0, outRR = 0;

// TUNING VALUES: Change these during your tuning phase
// double Kp = 4.5;
// double Ki = 0.4;
// double Kd = 0.39;

// PID pidFL(&inFL, &outFL, &setFL, Kp, Ki, Kd, DIRECT);
// PID pidFR(&inFR, &outFR, &setFR, Kp, Ki, Kd, DIRECT);
// PID pidRL(&inRL, &outRL, &setRL, Kp, Ki, Kd, DIRECT);
// PID pidRR(&inRR, &outRR, &setRR, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;
// BUG2 FIX: Private snapshots for speed measurement — shared counters are NEVER zeroed.
long lastSnapFL = 0, lastSnapFR = 0, lastSnapRL = 0, lastSnapRR = 0;

// ==========================================
// 4. MOTOR DRIVER FUNCTION
// ==========================================
void driveMotor(int pwmChannel, int in1, int in2, double output, bool forward)
{
  // Deadband: below 8/255 PWM the motor won't move — just brake to avoid stutter
  if (output < 8.0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
    return;
  }
  if (forward)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(pwmChannel, (uint32_t)output);
}

void PID_Init()
{
  // BUG4 FIX: Serial.begin() removed — already called in main setup().

  // Setup Motor Direction Pins
  pinMode(DIR_A_FR, OUTPUT); pinMode(DIR_B_FR, OUTPUT);
  pinMode(DIR_A_FL, OUTPUT); pinMode(DIR_B_FL, OUTPUT);
  pinMode(DIR_A_RR, OUTPUT); pinMode(DIR_B_RR, OUTPUT);
  pinMode(DIR_A_RL, OUTPUT); pinMode(DIR_B_RL, OUTPUT);

  // Setup ESP32 PWM Channels
  ledcSetup(0, freq, res); ledcAttachPin(PWM_FL, 0); // FL: Channel 0
  ledcSetup(1, freq, res); ledcAttachPin(PWM_FR, 1); // FR: Channel 1
  ledcSetup(2, freq, res); ledcAttachPin(PWM_RL, 2); // RL: Channel 2
  ledcSetup(3, freq, res); ledcAttachPin(PWM_RR, 3); // RR: Channel 3

  // Setup Encoders — A+B channels on RISING for 2x resolution
  // FL/FR use input-only GPIOs (34,35,36,39) — no internal pull resistor, use INPUT
  pinMode(ENC_A_FL, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_A_FL), isrFL_A, RISING);
  pinMode(ENC_B_FL, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_B_FL), isrFL_B, RISING);
  pinMode(ENC_A_FR, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_A_FR), isrFR_A, RISING);
  pinMode(ENC_B_FR, INPUT); attachInterrupt(digitalPinToInterrupt(ENC_B_FR), isrFR_B, RISING);
  // RL/RR use regular GPIOs — INPUT_PULLUP
  pinMode(ENC_A_RL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_A_RL), isrRL_A, RISING);
  // BURNT ENCODER WORKAROUND: Do NOT attach channel B to avoid floating pin noise interrupts
  pinMode(ENC_B_RL, INPUT_PULLUP); // attachInterrupt(digitalPinToInterrupt(ENC_B_RL), isrRL_B, RISING);
  pinMode(ENC_A_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isrRR_A, RISING);
  pinMode(ENC_B_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_B_RR), isrRR_B, RISING);

  // Setup PID (COMMENTED OUT)
  // pidFL.SetMode(AUTOMATIC); pidFL.SetOutputLimits(0, 255); pidFL.SetSampleTime(50);
  // pidFR.SetMode(AUTOMATIC); pidFR.SetOutputLimits(0, 255); pidFR.SetSampleTime(50);
  // pidRL.SetMode(AUTOMATIC); pidRL.SetOutputLimits(0, 255); pidRL.SetSampleTime(50);
  // pidRR.SetMode(AUTOMATIC); pidRR.SetOutputLimits(0, 255); pidRR.SetSampleTime(50);
}

void PID_Compute(float Vx, float Vy, float Wz)
{
  // ==========================================
  // STEP 1: SET KINEMATICS (Direction Control)
  // ==========================================
  // ── Mechanical Weight Imbalance Compensation ──
  // The robot rests slightly heavier on the FR and RL diagonal (like a wobbly table).
  // Because they carry more weight, FR and RL generate STRONGER lateral forces than FL and RR.
  // We compensate by artificially reducing the lateral command to FR/RL and boosting FL/RR.
  // This balances the sideways force vectors without affecting pure Forward/Backward!
  float lat_grip_comp = 0.80f; // Reduce the grippy wheels by 20%
  float lat_slip_comp = 1.20f; // Boost the slippy wheels by 20%

  float Vx_FL_RR = Vx * lat_slip_comp;
  float Vx_FR_RL = Vx * lat_grip_comp;

  double targetFL = Vy + Vx_FL_RR + Wz;
  double targetFR = Vy - Vx_FR_RL - Wz;
  double targetRL = Vy - Vx_FR_RL + Wz;
  double targetRR = Vy + Vx_FL_RR - Wz;

  // ── RL Burnt Encoder Workaround ──
  // Give the ISRs the intended direction of the RL wheel since they can't sense it.
  // Avoid flipping direction if target is absolute 0 to prevent jitter.
  if (targetRL > 0.01) global_dirRL = 1;
  else if (targetRL < -0.01) global_dirRL = -1;

  // ==========================================
  // STEP 2: MEASURE ACTUAL SPEED & CLOSED-LOOP PI CONTROL
  // BUG2 FIX: Read-delta approach. Shared counters are NEVER zeroed.
  // ==========================================
  static double outFL = 0, outFR = 0, outRL = 0, outRR = 0;

  if (millis() - lastTime >= 50)
  {
    long curFL, curFR, curRL, curRR;
    portENTER_CRITICAL(&tickMux);
    curFL = ticksFL; curFR = ticksFR; curRL = ticksRL; curRR = ticksRR;
    portEXIT_CRITICAL(&tickMux);

    // ── Professional Spike-Rejection & Low-Pass Filter ──
    // Prevents electrical noise from tricking the PI controller into dropping power.
    long dFL = abs(curFL - lastSnapFL);
    long dFR = abs(curFR - lastSnapFR);
    long dRL = abs(curRL - lastSnapRL);
    long dRR = abs(curRR - lastSnapRR);

    // If reading is physically impossible (>80 ticks/50ms is >1.3m/s), ignore the spike
    if (dFL > 80) dFL = inFL;
    if (dFR > 80) dFR = inFR;
    if (dRL > 80) dRL = inRL;
    if (dRR > 80) dRR = inRR;

    inFL = (0.5 * inFL) + (0.5 * dFL);  
    inFR = (0.5 * inFR) + (0.5 * dFR);
    inRL = (0.5 * inRL) + (0.5 * dRL);  
    inRR = (0.5 * inRR) + (0.5 * dRR);
    
    lastSnapFL = curFL; lastSnapFR = curFR;
    lastSnapRL = curRL; lastSnapRR = curRR;
    lastTime = millis();

    // ── Professional PI + Feed-Forward Velocity Controller ──
    static const float Kp_spd = 2.5f;   // Proportional
    static const float Ki_spd = 2.0f;   // Very strong integral for smooth torque ramping
    static const float FF_gain = 3.5f;  // Base feed-forward
    
    static float errSumFL = 0, errSumFR = 0, errSumRL = 0, errSumRR = 0;

    auto applyPI = [](double target, double actual, float& errSum) -> double {
        double absTarget = abs(target);
        if (absTarget < 0.1) {
            errSum = 0; // Reset integral when stopped to prevent windup
            return 0.0;
        }
        double error = absTarget - actual;
        errSum += error;
        
        // Allow the integral term to build smoothly all the way to max power (255)
        if (errSum >  150.0f) errSum =  150.0f;
        if (errSum < -150.0f) errSum = -150.0f;
        
        double pwm = (absTarget * FF_gain) + (Kp_spd * error) + (Ki_spd * errSum);
        if (pwm > 255.0) pwm = 255.0;
        if (pwm < 30.0)  pwm = 30.0;
        
        return pwm;
    };

    outFL = applyPI(targetFL, inFL, errSumFL);
    outFR = applyPI(targetFR, inFR, errSumFR);
    outRL = applyPI(targetRL, inRL, errSumRL);
    outRR = applyPI(targetRR, inRR, errSumRR);
  }

  // ==========================================
  // STEP 3: DRIVE MOTORS — runs EVERY call (100 Hz, 10 ms latency)
  // LATENCY1 FIX: Motor update decoupled from 50 ms speed-measurement gate.
  // Direction is strictly determined by the intended kinematic target.
  // ==========================================
  driveMotor(0, DIR_A_FL, DIR_B_FL, outFL, targetFL >= 0);
  driveMotor(1, DIR_A_FR, DIR_B_FR, outFR, targetFR >= 0);
  driveMotor(2, DIR_A_RL, DIR_B_RL, outRL, targetRL >= 0);
  driveMotor(3, DIR_A_RR, DIR_B_RR, outRR, targetRR >= 0);
}

void PID_GetActualSpeeds(double* fl, double* fr, double* rl, double* rr) {
  if (fl) *fl = inFL;
  if (fr) *fr = inFR;
  if (rl) *rl = inRL;
  if (rr) *rr = inRR;
}

void PID_TaskCode(void *pvParameters) {
  ChassisMotion motion;
  float Vx = 0;
  float Vy = 0;
  float Wz = 0;

  while (true) {
    if (xQueueReceive(pidMailbox, &motion, 0) == pdTRUE) {
      Vx = 0; Vy = 0; Wz = 0;

      // Convert m/s → ticks/sample so PID gets proper setpoints
      float speed      = speedToTicks(motion.speed);
      float diag_speed = speed * 0.7071f;
      float omega_ticks = speedToTicks(motion.omega);

      switch (motion.move_type) {
        case DriveCommand::CMD_FWD:     Vy =  speed; break;
        case DriveCommand::CMD_BWD:     Vy = -speed; break;
        case DriveCommand::CMD_LEFT:    Vx = -speed; break;
        case DriveCommand::CMD_RIGHT:   Vx =  speed; break;
        case DriveCommand::CMD_FWD_L:   Vy =  diag_speed; Vx = -diag_speed; break;
        case DriveCommand::CMD_FWD_R:   Vy =  diag_speed; Vx =  diag_speed; break;
        case DriveCommand::CMD_BWD_L:   Vy = -diag_speed; Vx = -diag_speed; break;
        case DriveCommand::CMD_BWD_R:   Vy = -diag_speed; Vx =  diag_speed; break;
        case DriveCommand::CMD_ROT_L:   Wz =  omega_ticks; break;
        case DriveCommand::CMD_ROT_R:   Wz = -omega_ticks; break;
        case DriveCommand::CMD_DIRECT:
          Vx = speedToTicks(motion.Vx * 0.5f);
          Vy = speedToTicks(motion.Vy * 0.5f);
          Wz = speedToTicks(motion.Wz * 0.5f);
          break;
        case DriveCommand::CMD_STOP:
        default:
          break;
      }
    }

    // PI loops and kinematics are handled inside PID_Compute.
    PID_Compute(Vx, Vy, Wz);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void PID_StartTask() {
  xTaskCreatePinnedToCore(PID_TaskCode, "PIDTask", 4096, NULL, 3, NULL, 1);
}