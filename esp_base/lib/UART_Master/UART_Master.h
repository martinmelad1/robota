#ifndef UART_MASTER_H
#define UART_MASTER_H

void UART_Master_Init();
void UART_Master_ProcessSerial();

// CAMERA
void CAM_RequestQR();
extern volatile float ultrasonic_distance_cm;

// ARM — commands OUT
void ARM_MoveXYZ(float x, float y, float z);
void ARM_Grip(int grip_cmd);
void ARM_MoveJoint(int joint_id, int dir);
void ARM_SendColor(const char* color);     // e.g. "red" — sent when autonomous starts
void ARM_SendReached(const char* color);   // e.g. "red" — sent when robot arrives at station

// ARM — telemetry IN (updated by UART_Arm_Task from "JOINT_FB:j1,j2,j3" messages)
extern volatile float arm_j1_deg;
extern volatile float arm_j2_deg;
extern volatile float arm_j3_deg;

// IMU data from arm (yaw in deg, accel in m/s²)
// Updated from "IMU_FB:yaw,ax,ay" messages
extern volatile float imu_yaw_deg;
extern volatile float imu_ax_mps2;
extern volatile float imu_ay_mps2;

// Tasks
void UART_Cam_Task(void *arg);
void UART_Arm_Task(void *arg);

#endif