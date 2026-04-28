#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Define servo pins (change these to match actual wiring. Avoid pins 12,13,14,15 as they often conflict with JTAG)
#define SERVO_1_PIN 33
#define SERVO_2_PIN 25
#define SERVO_3_PIN 26

#define SERVO_GRIP_PIN 27 // 5th servo for the gripper

// Physical limits of the Servos (Adjust these if the servo hits a hard stop and jitters!)
#define SERVO_MIN_ANGLE_J1 0
#define SERVO_MAX_ANGLE_J1 180
#define SERVO_MIN_ANGLE_J2 0
#define SERVO_MAX_ANGLE_J2 50
#define SERVO_MIN_ANGLE_J3 0
#define SERVO_MAX_ANGLE_J3 130
#define SERVO_MIN_ANGLE_GRIPPER 0
#define SERVO_MAX_ANGLE_GRIPPER 150
// Gripper Fixed Angles
#define GRIP_ANGLE_OPEN  60
#define GRIP_ANGLE_CLOSE 120
#define GRIP_ANGLE_PICK  90

// Structure to hold servo commands
struct ServoCommand {
    int joint_id;  // 1 to 3. 0 implies no specific joint
    int direction; // 1 = Increase angle, -1 = Decrease angle, 0 = STOP
};

// Mailbox queue handle
extern QueueHandle_t servoMailbox;

// Initialization function
void Servo_Control_Init();

// FreeRTOS task handling servo continuous movement logic
void Servo_Control_Task(void *arg);

#endif // SERVO_CONTROL_H
