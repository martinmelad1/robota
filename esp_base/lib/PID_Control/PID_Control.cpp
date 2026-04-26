#include "PID_Control.h"
#include <PID_v1.h>
#include "WorldState.h"

extern QueueHandle_t pidMailbox;

// ==========================================
// 1. HARDWARE PIN DEFINITIONS
// ==========================================

// Front Left (FL) — L298N Channel A
const int PWM_FL   = 32;  // ENA
const int DIR_A_FL = 25;  // IN1
const int DIR_B_FL = 26;  // IN2
const int ENC_A_FL = 34;  // Encoder A (input-only GPIO — no internal pull resistor)
const int ENC_B_FL = 35;  // Encoder B (input-only GPIO — no internal pull resistor)

// Front Right (FR) — L298N Channel B
const int PWM_FR   = 33;  // ENB
const int DIR_A_FR = 14;  // IN3
const int DIR_B_FR = 27;  // IN4
const int ENC_A_FR = 36;  // Encoder A (input-only GPIO — no internal pull resistor)
const int ENC_B_FR = 39;  // Encoder B (input-only GPIO — no internal pull resistor)

// Rear Left (RL) — L298N Channel A
const int PWM_RL   = 12;  // ENA
const int DIR_A_RL = 15;  // IN1
const int DIR_B_RL = 13;  // IN2
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

void IRAM_ATTR isrFL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFL++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFL_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFL++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFR++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFR++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRL++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRL_B() { portENTER_CRITICAL_ISR(&tickMux); ticksRL++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRR++; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksRR++; portEXIT_CRITICAL_ISR(&tickMux); }

// ==========================================
// 3. PID VARIABLES & SETUP
// ==========================================
// The target speeds (Setpoints)
double setFL = 0, setFR = 0, setRL = 0, setRR = 0;
// The actual speeds (Inputs)
double inFL = 0, inFR = 0, inRL = 0, inRR = 0;
// The PWM outputs calculated by PID
double outFL = 0, outFR = 0, outRL = 0, outRR = 0;

// TUNING VALUES: Change these during your tuning phase
double Kp = 4.5;
double Ki = 0.4;
double Kd = 0.39;

PID pidFL(&inFL, &outFL, &setFL, Kp, Ki, Kd, DIRECT);
PID pidFR(&inFR, &outFR, &setFR, Kp, Ki, Kd, DIRECT);
PID pidRL(&inRL, &outRL, &setRL, Kp, Ki, Kd, DIRECT);
PID pidRR(&inRR, &outRR, &setRR, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;

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
  Serial.begin(115200);

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
  pinMode(ENC_B_RL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_B_RL), isrRL_B, RISING);
  pinMode(ENC_A_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isrRR_A, RISING);
  pinMode(ENC_B_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_B_RR), isrRR_B, RISING);

  // Setup PID
  pidFL.SetMode(AUTOMATIC); pidFL.SetOutputLimits(0, 255); pidFL.SetSampleTime(50);
  pidFR.SetMode(AUTOMATIC); pidFR.SetOutputLimits(0, 255); pidFR.SetSampleTime(50);
  pidRL.SetMode(AUTOMATIC); pidRL.SetOutputLimits(0, 255); pidRL.SetSampleTime(50);
  pidRR.SetMode(AUTOMATIC); pidRR.SetOutputLimits(0, 255); pidRR.SetSampleTime(50);
}

void PID_Compute(float Vx, float Vy, float Wz)
{
  // ==========================================
  // STEP 1: SET KINEMATICS (Direction Control)
  // ==========================================
  // Mecanum Equations
  double targetFL = Vy + Vx + Wz;
  double targetFR = Vy - Vx - Wz;
  double targetRL = Vy - Vx + Wz;
  double targetRR = Vy + Vx - Wz;

  // ==========================================
  // STEP 2: MEASURE ACTUAL SPEED & COMPUTE PID
  // ==========================================
  if (millis() - lastTime >= 50)
  {
    // Atomic tick snapshot — prevents ISR race condition on dual-core ESP32
    long snapFL, snapFR, snapRL, snapRR;
    portENTER_CRITICAL(&tickMux);
    snapFL = ticksFL; ticksFL = 0;
    snapFR = ticksFR; ticksFR = 0;
    snapRL = ticksRL; ticksRL = 0;
    snapRR = ticksRR; ticksRR = 0;
    portEXIT_CRITICAL(&tickMux);
    inFL = snapFL; inFR = snapFR; inRL = snapRL; inRR = snapRR;
    lastTime = millis();

    // Pass absolute target to PID
    setFL = abs(targetFL);
    setFR = abs(targetFR);
    setRL = abs(targetRL);
    setRR = abs(targetRR);

    // Reset integral when motor is commanded to stop (prevents windup jerk on restart)
    auto resetPID = [](PID& pid, double& out, double set) {
      if (set == 0.0) { pid.SetMode(MANUAL); out = 0; pid.SetMode(AUTOMATIC); }
    };
    resetPID(pidFL, outFL, setFL);
    resetPID(pidFR, outFR, setFR);
    resetPID(pidRL, outRL, setRL);
    resetPID(pidRR, outRR, setRR);

    // Compute PWM Output
    pidFL.Compute();
    pidFR.Compute();
    pidRL.Compute();
    pidRR.Compute();

    // ==========================================
    // STEP 3: DRIVE MOTORS
    // ==========================================
    driveMotor(0, DIR_A_FL, DIR_B_FL, outFL, targetFL >= 0);
    driveMotor(1, DIR_A_FR, DIR_B_FR, outFR, targetFR >= 0);
    driveMotor(2, DIR_A_RL, DIR_B_RL, outRL, targetRL >= 0);
    driveMotor(3, DIR_A_RR, DIR_B_RR, outRR, targetRR >= 0);

    // ==========================================
    // STEP 4: TELEPLOT OUTPUT (Tuning Mode)
    // ==========================================
    // Plotting Front-Left motor for tuning
    // Serial.print(">Target:");
    // Serial.println(setFL);
    // Serial.print(">Actual:");
    // Serial.println(inFL);
    // Serial.print(">PWM:"); Serial.println(outFL);
  }
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
      float speed = speedToTicks(motion.speed);
      float diag_speed = speed * 0.7071f; // maintain constant speed magnitude
      float omega_ticks = speedToTicks(motion.omega); // same conversion for rotation

      switch (motion.move_type) {
        case DriveCommand::CMD_FWD:     Vy = speed; break;
        case DriveCommand::CMD_BWD:     Vy = -speed; break;
        case DriveCommand::CMD_LEFT:    Vx = -speed; break;
        case DriveCommand::CMD_RIGHT:   Vx = speed; break;
        case DriveCommand::CMD_FWD_L:   Vy = diag_speed; Vx = -diag_speed; break;
        case DriveCommand::CMD_FWD_R:   Vy = diag_speed; Vx = diag_speed; break;
        case DriveCommand::CMD_BWD_L:   Vy = -diag_speed; Vx = -diag_speed; break;
        case DriveCommand::CMD_BWD_R:   Vy = -diag_speed; Vx = diag_speed; break;
        case DriveCommand::CMD_ROT_L:   Wz = omega_ticks; break;
        case DriveCommand::CMD_ROT_R:   Wz = -omega_ticks; break;
        case DriveCommand::CMD_STOP:
        default:
          break;
      }
    }
    PID_Compute(Vx, Vy, Wz);
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void PID_StartTask() {
  xTaskCreatePinnedToCore(PID_TaskCode, "PIDTask", 4096, NULL, 3, NULL, 1);
}