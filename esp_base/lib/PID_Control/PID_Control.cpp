// ============================================================
//  PID_CONTROL.CPP  — v4  (open-loop 255 PWM, no PID)
// ============================================================

#include "PID_Control.h"
#include "WorldState.h"

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
static const float WHEEL_CIRCUM_M   = 3.14159265f * WHEEL_DIAMETER_M;
static const int   ENCODER_PPR      = 748;
static const int   PID_SAMPLE_MS    = 50;

float speedToTicks(float speed_ms)
{
    return (speed_ms / WHEEL_CIRCUM_M) * ENCODER_PPR * (PID_SAMPLE_MS / 1000.0f);
}

// ==========================================
// 3. ENCODER STATE  (kept for CMD_DIST odometry)
// ==========================================
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;
portMUX_TYPE tickMux = portMUX_INITIALIZER_UNLOCKED;

// ── Velocity tracking (snapshotted under tickMux each PID call) ──
// Written only inside portENTER_CRITICAL(&tickMux) in PID_Compute(),
// read under the same lock in PID_GetTelemetry / PID_GetActualSpeeds.
// NOT volatile — these are task-to-task, not ISR-shared.
static float g_velActFL = 0, g_velActFR = 0, g_velActRL = 0, g_velActRR = 0;
static float g_velSetFL = 0, g_velSetFR = 0, g_velSetRL = 0, g_velSetRR = 0;
static long  g_prevFL = 0,   g_prevFR = 0,   g_prevRL = 0,   g_prevRR = 0;

volatile int global_dirRL = 1;

void IRAM_ATTR isrFL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFL += (digitalRead(ENC_B_FL) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFL_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFL += (digitalRead(ENC_A_FL) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksFR += (digitalRead(ENC_B_FR) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrFR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksFR += (digitalRead(ENC_A_FR) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRL_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRL += 2 * global_dirRL; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_A() { portENTER_CRITICAL_ISR(&tickMux); ticksRR += (digitalRead(ENC_B_RR) == LOW) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }
void IRAM_ATTR isrRR_B() { portENTER_CRITICAL_ISR(&tickMux); ticksRR += (digitalRead(ENC_A_RR) == HIGH) ? +1 : -1; portEXIT_CRITICAL_ISR(&tickMux); }

// ==========================================
// 4. DRIVE SPEED
// ==========================================
static float g_driveSpeed = 0.50f;

void PID_SetDriveSpeed(float mps)
{
    if (mps < 0.10f) mps = 0.10f;
    if (mps > 1.50f) mps = 1.50f;
    g_driveSpeed = mps;
    Serial.printf("[DRV] Drive speed set to %.2f m/s\n", g_driveSpeed);
}
float PID_GetDriveSpeed() { return g_driveSpeed; }

// Stubs — no gains to set, kept so callers still compile
void PID_SetGains(int /*mode*/, int /*wheel*/, float /*Kp*/, float /*Ki*/, float /*Kd*/) {}

void PID_GetActualSpeeds(double *fl, double *fr, double *rl, double *rr)
{
    portENTER_CRITICAL(&tickMux);
    float aFL = g_velActFL, aFR = g_velActFR;
    float aRL = g_velActRL, aRR = g_velActRR;
    portEXIT_CRITICAL(&tickMux);
    if (fl) *fl = (double)aFL;
    if (fr) *fr = (double)aFR;
    if (rl) *rl = (double)aRL;
    if (rr) *rr = (double)aRR;
}

void PID_GetTelemetry(PIDTelemetry &t)
{
    portENTER_CRITICAL(&tickMux);
    t.velSetFL = g_velSetFL; t.velSetFR = g_velSetFR;
    t.velSetRL = g_velSetRL; t.velSetRR = g_velSetRR;
    t.velActFL = g_velActFL; t.velActFR = g_velActFR;
    t.velActRL = g_velActRL; t.velActRR = g_velActRR;
    portEXIT_CRITICAL(&tickMux);
    for (int i = 0; i < 4; i++) t.velGains[i] = {0, 0, 0};
}

// ==========================================
// 5. MOTOR DRIVER
// ==========================================
static void driveMotor(int channel, int in1, int in2, double pwm)
{
    bool fwd = (pwm >= 0.0);
    double mag = fabs(pwm);
    if (mag < 8.0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(channel, 0);
        return;
    }
    if (mag > 255.0) mag = 255.0;
    digitalWrite(in1, fwd ? HIGH : LOW);
    digitalWrite(in2, fwd ? LOW  : HIGH);
    ledcWrite(channel, (uint32_t)mag);
}

// ==========================================
// 6. INIT
// ==========================================
void PID_Init()
{
    for (auto pin : {DIR_A_FL, DIR_B_FL, DIR_A_FR, DIR_B_FR,
                     DIR_A_RL, DIR_B_RL, DIR_A_RR, DIR_B_RR})
        pinMode(pin, OUTPUT);

    ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_FL, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_FR, 1);
    ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_RL, 2);
    ledcSetup(3, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_RR, 3);

    pinMode(ENC_A_FL, INPUT);        attachInterrupt(digitalPinToInterrupt(ENC_A_FL), isrFL_A, RISING);
    pinMode(ENC_B_FL, INPUT);        attachInterrupt(digitalPinToInterrupt(ENC_B_FL), isrFL_B, RISING);
    pinMode(ENC_A_FR, INPUT);        attachInterrupt(digitalPinToInterrupt(ENC_A_FR), isrFR_A, RISING);
    pinMode(ENC_B_FR, INPUT);        attachInterrupt(digitalPinToInterrupt(ENC_B_FR), isrFR_B, RISING);
    pinMode(ENC_A_RL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_A_RL), isrRL_A, RISING);
    pinMode(ENC_B_RL, INPUT_PULLUP);
    pinMode(ENC_A_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_A_RR), isrRR_A, RISING);
    pinMode(ENC_B_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENC_B_RR), isrRR_B, RISING);
}

// ==========================================
// 7. COMPUTE  — open-loop, fixed 255 PWM
// ==========================================
static double finalPWM[4] = {0, 0, 0, 0};

void PID_Compute(float Vx, float Vy, float Wz)
{
    // Mecanum inverse kinematics — same signs as before
    double tFL = (double)(Vy + Vx - Wz);
    double tFR = (double)(Vy - Vx + Wz);
    double tRL = (double)(Vy - Vx - Wz);
    double tRR = (double)(Vy + Vx + Wz);

    if (tRL > 0.1)       global_dirRL =  1;
    else if (tRL < -0.1) global_dirRL = -1;

    // ── Snapshot tick counts + compute actuals + store setpoints atomically ──
    // All done under tickMux so Core-0 readers (telemetry) see a consistent
    // snapshot and can never read a half-written float.
    long curFL, curFR, curRL, curRR;
    portENTER_CRITICAL(&tickMux);
    curFL = ticksFL; curFR = ticksFR;
    curRL = ticksRL; curRR = ticksRR;

    g_velActFL = (float)(curFL - g_prevFL);
    g_velActFR = (float)(curFR - g_prevFR);
    g_velActRL = (float)(curRL - g_prevRL);
    g_velActRR = (float)(curRR - g_prevRR);

    g_prevFL = curFL; g_prevFR = curFR;
    g_prevRL = curRL; g_prevRR = curRR;

    // Record setpoints inside same lock so reads are always paired
    g_velSetFL = (float)tFL;
    g_velSetFR = (float)tFR;
    g_velSetRL = (float)tRL;
    g_velSetRR = (float)tRR;
    portEXIT_CRITICAL(&tickMux);

    double targets[4] = {tFL, tFR, tRL, tRR};
    for (int i = 0; i < 4; i++)
    {
        if (fabs(targets[i]) < 0.1)
            finalPWM[i] = 0.0;
        else
            finalPWM[i] = (targets[i] >= 0.0) ? 255.0 : -255.0;
    }

    driveMotor(0, DIR_A_FL, DIR_B_FL, finalPWM[0]);
    driveMotor(1, DIR_A_FR, DIR_B_FR, finalPWM[1]);
    driveMotor(2, DIR_A_RL, DIR_B_RL, finalPWM[2]);
    driveMotor(3, DIR_A_RR, DIR_B_RR, finalPWM[3]);
}

// ==========================================
// 8. TASK
// ==========================================
void PID_TaskCode(void *pvParameters)
{
    ChassisMotion motion;
    float Vx = 0, Vy = 0, Wz = 0;

    bool distActive      = false;
    long distTargetTicks = 0;
    long distStartFL     = 0;
    float distVx = 0, distVy = 0;

    while (true)
    {
        if (xQueueReceive(pidMailbox, &motion, 0) == pdTRUE)
        {
            distActive = false;
            Vx = Vy = Wz = 0;

            float spd     = speedToTicks(motion.speed);
            float diagSpd = spd * 0.7071f;
            float omgTicks = speedToTicks(motion.omega);

            switch (motion.move_type)
            {
            case DriveCommand::CMD_FWD:   Vy =  spd;                      break;
            case DriveCommand::CMD_BWD:   Vy = -spd;                      break;
            case DriveCommand::CMD_LEFT:  Vx = -spd;                      break;
            case DriveCommand::CMD_RIGHT: Vx =  spd;                      break;
            case DriveCommand::CMD_FWD_L: Vy =  diagSpd; Vx = -diagSpd;  break;
            case DriveCommand::CMD_FWD_R: Vy =  diagSpd; Vx =  diagSpd;  break;
            case DriveCommand::CMD_BWD_L: Vy = -diagSpd; Vx = -diagSpd;  break;
            case DriveCommand::CMD_BWD_R: Vy = -diagSpd; Vx =  diagSpd;  break;
            case DriveCommand::CMD_ROT_L: Wz =  omgTicks;                 break;
            case DriveCommand::CMD_ROT_R: Wz = -omgTicks;                 break;
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

        if (distActive)
        {
            long curFL;
            portENTER_CRITICAL(&tickMux);
            curFL = ticksFL;
            portEXIT_CRITICAL(&tickMux);
            if (abs(curFL - distStartFL) >= distTargetTicks)
            {
                distActive = false;
                Vx = Vy = Wz = 0;
            }
            else
            {
                Vx = distVx;
                Vy = distVy;
                Wz = 0;
            }
        }

        PID_Compute(Vx, Vy, Wz);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void PID_StartTask()
{
    xTaskCreatePinnedToCore(PID_TaskCode, "PIDTask", 4096, NULL, 3, NULL, 1);
}