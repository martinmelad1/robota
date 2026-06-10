#include "Motion.h"

// ====== TIMING CALIBRATION ======
// You MUST tune these experimentally
float speed_m_per_sec = 0.5;     // robot speed
float deg_per_sec     = 90.0;    // rotation speed

// ================= MOTOR CONTROL =================

// FL, FR, RL, RR
void setMotor(int in1, int in2, int speed, bool forward)
{
    if (forward) {
        analogWrite(in1, speed);
        analogWrite(in2, 0);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, speed);
    }
}

void stopMotors()
{
    analogWrite(FL_IN1, 0); analogWrite(FL_IN2, 0);
    analogWrite(FR_IN1, 0); analogWrite(FR_IN2, 0);
    analogWrite(RL_IN1, 0); analogWrite(RL_IN2, 0);
    analogWrite(RR_IN1, 0); analogWrite(RR_IN2, 0);
}

// ================= BASIC MOTIONS =================

// Forward (+X)
void moveForward(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, true);
    setMotor(FR_IN1, FR_IN2, speed, true);
    setMotor(RL_IN1, RL_IN2, speed, true);
    setMotor(RR_IN1, RR_IN2, speed, true);
}

// Backward (-X)
void moveBackward(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, false);
    setMotor(FR_IN1, FR_IN2, speed, false);
    setMotor(RL_IN1, RL_IN2, speed, false);
    setMotor(RR_IN1, RR_IN2, speed, false);
}

// Strafe Right (+Y)
void strafeRight(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, true);
    setMotor(FR_IN1, FR_IN2, speed, false);
    setMotor(RL_IN1, RL_IN2, speed, false);
    setMotor(RR_IN1, RR_IN2, speed, true);
}

// Strafe Left (-Y)
void strafeLeft(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, false);
    setMotor(FR_IN1, FR_IN2, speed, true);
    setMotor(RL_IN1, RL_IN2, speed, true);
    setMotor(RR_IN1, RR_IN2, speed, false);
}

// Rotate CW
void rotateCW(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, true);
    setMotor(FR_IN1, FR_IN2, speed, false);
    setMotor(RL_IN1, RL_IN2, speed, true);
    setMotor(RR_IN1, RR_IN2, speed, false);
}

// Rotate CCW
void rotateCCW(int speed)
{
    setMotor(FL_IN1, FL_IN2, speed, false);
    setMotor(FR_IN1, FR_IN2, speed, true);
    setMotor(RL_IN1, RL_IN2, speed, false);
    setMotor(RR_IN1, RR_IN2, speed, true);
}

// ✅ 45° diagonal (NO ROTATION, mecanum correct)
void moveDiagonal45(int speed)
{
    // Only two wheels active
    setMotor(FL_IN1, FL_IN2, speed, true);
    setMotor(FR_IN1, FR_IN2, 0, true);
    setMotor(RL_IN1, RL_IN2, 0, true);
    setMotor(RR_IN1, RR_IN2, speed, true);
}

// ================= DISTANCE CONTROL =================

void moveDistance(void (*moveFunc)(int), float distance_m, int speed)
{
    float time_s = distance_m / speed_m_per_sec;
    unsigned long duration = time_s * 1000;

    moveFunc(speed);
    delay(duration);
    stopMotors();
    delay(300);
}

void rotateAngle(void (*rotFunc)(int), float angle_deg, int speed)
{
    float time_s = angle_deg / deg_per_sec;
    unsigned long duration = time_s * 1000;

    rotFunc(speed);
    delay(duration);
    stopMotors();
    delay(300);
}

//MOTION.CPP