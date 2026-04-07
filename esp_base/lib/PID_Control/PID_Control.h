#pragma once
#include <Arduino.h>

void PID_Init();
void PID_StartTask();
void PID_Compute(float Vx, float Vy, float Wz);
void PID_GetActualSpeeds(double* fl, double* fr, double* rl, double* rr);
