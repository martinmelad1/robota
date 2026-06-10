#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// Motor pins (EDIT if needed)
#define FL_IN1 32
#define FL_IN2 33

#define FR_IN1 25
#define FR_IN2 26

#define RL_IN1 27
#define RL_IN2 14

#define RR_IN1 12
#define RR_IN2 13

// Function declarations
void stopMotors();

void moveForward(int speed);
void moveBackward(int speed);

void strafeRight(int speed);
void strafeLeft(int speed);

void rotateCW(int speed);
void rotateCCW(int speed);

void moveDiagonal45(int speed); // 45° strafe

void moveDistance(void (*moveFunc)(int), float distance_m, int speed);
void rotateAngle(void (*rotFunc)(int), float angle_deg, int speed);

#endif

//MOTION.H