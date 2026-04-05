#ifndef UART_MASTER_H
#define UART_MASTER_H

void UART_Master_Init();

// CAMERA
void CAM_RequestQR();

// ARM
void ARM_MoveXYZ(float x, float y, float z);
void ARM_Grip(int close);

// Tasks
void UART_Cam_Task(void *arg);
void UART_Arm_Task(void *arg);

#endif