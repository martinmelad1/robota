#include "PID_Control.h"
#include <PID_v1.h>
#include "WorldState.h"

extern QueueHandle_t pidMailbox;

// ==========================================
// 1. HARDWARE PIN DEFINITIONS (CORRECTED)
// ==========================================
// Front Left (FL)
const int ENA_FL = 32, IN1_FL = 25, IN2_FL = 26, ENC_FL = 18;

// Front Right (FR) 
const int ENB_FR = 33, IN3_FR = 27, IN4_FR = 14, ENC_FR = 19;

// Rear Left (RL)
const int ENA_RL = 12, IN1_RL = 13, IN2_RL = 15, ENC_RL = 21;

// Rear Right (RR)
const int ENB_RR = 2, IN3_RR = 4, IN4_RR = 16, ENC_RR = 0;

// PWM Channels (ESP32 Specific)
const int freq = 5000;
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
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;

void IRAM_ATTR isrFL() { ticksFL++; }
void IRAM_ATTR isrFR() { ticksFR++; }
void IRAM_ATTR isrRL() { ticksRL++; }
void IRAM_ATTR isrRR() { ticksRR++; }

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
  if (output == 0)
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
  ledcWrite(pwmChannel, abs(output));
}

void PID_Init()
{
  Serial.begin(115200);

  // Setup Motor Pins
  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN3_FR, OUTPUT);
  pinMode(IN4_FR, OUTPUT);
  pinMode(IN1_RL, OUTPUT);
  pinMode(IN2_RL, OUTPUT);
  pinMode(IN3_RR, OUTPUT);
  pinMode(IN4_RR, OUTPUT);

  // Setup ESP32 PWM
  ledcSetup(0, freq, res);
  ledcAttachPin(ENA_FL, 0); // FL: Channel 0
  ledcSetup(1, freq, res);
  ledcAttachPin(ENB_FR, 1); // FR: Channel 1
  ledcSetup(2, freq, res);
  ledcAttachPin(ENA_RL, 2); // RL: Channel 2
  ledcSetup(3, freq, res);
  ledcAttachPin(ENB_RR, 3); // RR: Channel 3

  // Setup Encoders
  pinMode(ENC_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FL), isrFL, RISING);
  pinMode(ENC_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FR), isrFR, RISING);
  pinMode(ENC_RL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_RL), isrRL, RISING);
  pinMode(ENC_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_RR), isrRR, RISING);

  // Setup PID
  pidFL.SetMode(AUTOMATIC);
  pidFR.SetMode(AUTOMATIC);
  pidRL.SetMode(AUTOMATIC);
  pidRR.SetMode(AUTOMATIC);

  // Sample time 50ms for smooth RPM calculation
  pidFL.SetSampleTime(50);
  pidFR.SetSampleTime(50);
  pidRL.SetSampleTime(50);
  pidRR.SetSampleTime(50);
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
    // Read ticks
    inFL = ticksFL;
    ticksFL = 0;
    inFR = ticksFR;
    ticksFR = 0;
    inRL = ticksRL;
    ticksRL = 0;
    inRR = ticksRR;
    ticksRR = 0;
    lastTime = millis();

    // Pass absolute target to PID
    setFL = abs(targetFL);
    setFR = abs(targetFR);
    setRL = abs(targetRL);
    setRR = abs(targetRR);

    // Compute PWM Output
    pidFL.Compute();
    pidFR.Compute();
    pidRL.Compute();
    pidRR.Compute();

    // ==========================================
    // STEP 3: DRIVE MOTORS
    // ==========================================
    driveMotor(0, IN1_FL, IN2_FL, outFL, targetFL >= 0);
    driveMotor(1, IN3_FR, IN4_FR, outFR, targetFR >= 0);
    driveMotor(2, IN1_RL, IN2_RL, outRL, targetRL >= 0);
    driveMotor(3, IN3_RR, IN4_RR, outRR, targetRR >= 0);

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