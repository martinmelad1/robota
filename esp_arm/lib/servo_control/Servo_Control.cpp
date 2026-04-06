#include "Servo_Control.h"

// Define the Queue mailbox
QueueHandle_t servoMailbox;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servoGrip; // 5th servo for the gripper

// Initial angles (start at 90 degrees)
int angle1 = 90;
int angle2 = 90;
int angle3 = 90;
int angle4 = 90;

// Variables to keep track of current continuous movement
int current_active_joint = 0;
int current_direction = 0;

void Servo_Control_Init() {
    // Allocate all timers for ESP32Servo to ensure PWM can be generated
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // 50 Hz PWM period for typical RC servos
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo3.setPeriodHertz(50);
    servo4.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);

    // Attach pins (min/max pulse width in microseconds)
    servo1.attach(SERVO_1_PIN, 500, 2400);
    servo2.attach(SERVO_2_PIN, 500, 2400);
    servo3.attach(SERVO_3_PIN, 500, 2400);
    servo4.attach(SERVO_4_PIN, 500, 2400);
    servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);

    // Go to initial position
    servo1.write(angle1);
    servo2.write(angle2);
    servo3.write(angle3);
    servo4.write(angle4);
    servoGrip.write(GRIP_ANGLE_OPEN); // Default gripper open


    // Create mailbox (queue) with size 10 to receive commands
    servoMailbox = xQueueCreate(10, sizeof(ServoCommand));
    if (servoMailbox == NULL) {
        Serial.println("Error creating the servo mailbox");
    }
}

// Helper to update state and send to physical servo with limits
void update_angle(int joint, int dir) {
    if (dir == 0) return;

    if (joint == 1) {
        int old_angle = angle1;
        angle1 += dir;
        if (angle1 > SERVO_MAX_ANGLE) angle1 = SERVO_MAX_ANGLE;
        if (angle1 < SERVO_MIN_ANGLE) angle1 = SERVO_MIN_ANGLE;
        if (angle1 != old_angle) {
            servo1.write(angle1);
            if (angle1 % 5 == 0) Serial.printf("Actuating Servo 1 to %d deg\n", angle1);
        }
    } else if (joint == 2) {
        int old_angle = angle2;
        angle2 += dir;
        if (angle2 > SERVO_MAX_ANGLE) angle2 = SERVO_MAX_ANGLE;
        if (angle2 < SERVO_MIN_ANGLE) angle2 = SERVO_MIN_ANGLE;
        if (angle2 != old_angle) {
            servo2.write(angle2);
            if (angle2 % 5 == 0) Serial.printf("Actuating Servo 2 to %d deg\n", angle2);
        }
    } else if (joint == 3) {
        int old_angle = angle3;
        angle3 += dir;
        if (angle3 > SERVO_MAX_ANGLE) angle3 = SERVO_MAX_ANGLE;
        if (angle3 < SERVO_MIN_ANGLE) angle3 = SERVO_MIN_ANGLE;
        if (angle3 != old_angle) {
            servo3.write(angle3);
            if (angle3 % 5 == 0) Serial.printf("Actuating Servo 3 to %d deg\n", angle3);
        }
    } else if (joint == 4) {
        int old_angle = angle4;
        angle4 += dir;
        if (angle4 > SERVO_MAX_ANGLE) angle4 = SERVO_MAX_ANGLE;
        if (angle4 < SERVO_MIN_ANGLE) angle4 = SERVO_MIN_ANGLE;
        if (angle4 != old_angle) {
            servo4.write(angle4);
            if (angle4 % 5 == 0) Serial.printf("Actuating Servo 4 to %d deg\n", angle4);
        }
    }
}

// Task executed continuously every 20ms (Matches 50Hz PWM perfectly)
void Servo_Control_Task(void *arg) {
    ServoCommand cmd;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // exactly 0.02 seconds

    while (1) {
        // Poll queue, don't wait/block. Read latest command if any.
        if (xQueueReceive(servoMailbox, &cmd, 0) == pdPASS) {
            
            if (cmd.joint_id == 5) {
                // Gripper is instantaneous (not continuous sweep)
                if (cmd.direction == 1) {
                    servoGrip.write(GRIP_ANGLE_OPEN);
                    Serial.println("Gripper Action -> OPEN");
                } else if (cmd.direction == -1) {
                    servoGrip.write(GRIP_ANGLE_CLOSE);
                    Serial.println("Gripper Action -> CLOSE");
                } else if (cmd.direction == 2) {
                    servoGrip.write(GRIP_ANGLE_PICK);
                    Serial.println("Gripper Action -> PICK");
                }
            } else {
                // Continuous Sweep Arms (1-4)
                Serial.print("Servo_Control_Task Received Command -> Joint: ");
                Serial.print(cmd.joint_id);
                Serial.print(", Direction: ");
                Serial.println(cmd.direction);

                if (cmd.joint_id == 0 && cmd.direction == 0) {
                    // Stop command received (Release of button)
                    current_active_joint = 0;
                    current_direction = 0;
                } else {
                    // Start tracking long press of a button
                    current_active_joint = cmd.joint_id;
                    current_direction = cmd.direction;
                }
            }
        }

        // Apply movement iteratively every loop tick if a joint is active
        if (current_active_joint != 0 && current_direction != 0) {
            update_angle(current_active_joint, current_direction);
        }

        // Sleep to enforce exact 50ms time window cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
