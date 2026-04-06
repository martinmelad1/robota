#ifndef UART_MASTER_H
#define UART_MASTER_H

void UART_Master_Init();
void UART_Master_ProcessSerial();

// CAMERA
void CAM_RequestQR();

// ARM
void ARM_MoveXYZ(float x, float y, float z);
void ARM_Grip(int grip_cmd);
void ARM_MoveJoint(int joint_id, int dir);

// Tasks
void UART_Cam_Task(void *arg);
void UART_Arm_Task(void *arg);

#endif