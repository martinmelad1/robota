#include "UART_Master.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "WorldState.h"

extern QueueHandle_t armMailbox;

// ========================
// UART CONFIG
// ========================
#define UART_CAM UART_NUM_1
#define UART_ARM UART_NUM_2

#define BUF_SIZE 1024

// Pins — MUST NOT conflict with PID motor pins (25,26,14,15,etc.)
#define CAM_TX 17
#define CAM_RX 5

#define ARM_TX 23
#define ARM_RX 35   // GPIO 35 is input-only, perfect for RX

// ========================
// INIT
// ========================
void UART_Master_Init(void)
{
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Camera UART
    uart_param_config(UART_CAM, &config);
    uart_set_pin(UART_CAM, CAM_TX, CAM_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_CAM, BUF_SIZE, 0, 0, NULL, 0);

    // Arm UART
    uart_param_config(UART_ARM, &config);
    uart_set_pin(UART_ARM, ARM_TX, ARM_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_ARM, BUF_SIZE, 0, 0, NULL, 0);
}

// ========================
// SERIAL TESTING
// ========================
void UART_Master_ProcessSerial(void)
{
    // Simple serial testing interface for hardware testing
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "test_move") {
            ARM_MoveXYZ(10.5, 20.0, 30.5);
            Serial.println("Sent test_move to ARM");
        } else if (cmd == "test_grip_close") {
            ARM_Grip(1);
            Serial.println("Sent test_grip_close to ARM");
        } else if (cmd == "test_grip_open") {
            ARM_Grip(0);
            Serial.println("Sent test_grip_open to ARM");
        } else if (cmd == "test_qr") {
            CAM_RequestQR();
            Serial.println("Sent test_qr to CAM");
        }
    }
}

// ========================
// CAMERA FUNCTIONS
// ========================
void CAM_RequestQR(void)
{
    const char *cmd = "SCAN_QR\n";
    uart_write_bytes(UART_CAM, cmd, strlen(cmd));
    printf("Sent to CAM: SCAN_QR\n");
}

// ========================
// ARM FUNCTIONS
// ========================
void ARM_MoveXYZ(float x, float y, float z)
{
    char cmd[64];
    sprintf(cmd, "MOVE:%.2f,%.2f,%.2f\n", x, y, z);

    uart_write_bytes(UART_ARM, cmd, strlen(cmd));
    printf("Sent to ARM: %s", cmd);
}

void ARM_Grip(int grip_cmd)
{
    if (grip_cmd == 1)
    {
        uart_write_bytes(UART_ARM, "GRIP:CLOSE\n", 11);
        printf("Sent to ARM: GRIP:CLOSE\n");
    }
    else if (grip_cmd == 2)
    {
        uart_write_bytes(UART_ARM, "GRIP:PICK\n", 10);
        printf("Sent to ARM: GRIP:PICK\n");
    }
    else
    {
        uart_write_bytes(UART_ARM, "GRIP:OPEN\n", 10);
        printf("Sent to ARM: GRIP:OPEN\n");
    }
}

void ARM_MoveJoint(int joint_id, int dir)
{
    char cmd[64];
    sprintf(cmd, "JOINT:%d,%d\n", joint_id, dir);

    uart_write_bytes(UART_ARM, cmd, strlen(cmd));
    printf("Sent to ARM: %s", cmd);
}

// ========================
// CAMERA TASK
// ========================
void UART_Cam_Task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1)
    {
        int len = uart_read_bytes(UART_CAM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            data[len] = '\0';
            printf("CAM -> BASE: %s\n", data);

            // 🔥 TODO: Parse QR result here
            // Example:
            // if (strstr((char*)data, "QR_OK")) { ... }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ========================
// ARM TASK
// ========================
void UART_Arm_Task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1)
    {
        // 1. Process Mailbox from State Machine
        ArmMotion motion_cmd;
        if (xQueueReceive(armMailbox, &motion_cmd, 0) == pdTRUE) {
            // Forward it to ARM via UART
            if (motion_cmd.joint_id == 5) {
                if (motion_cmd.direction == ArmDir::UP) {
                    ARM_Grip(0); // Open
                } else if (motion_cmd.direction == ArmDir::DOWN) {
                    ARM_Grip(1); // Close
                } else if (motion_cmd.direction == ArmDir::STOP) {
                    ARM_Grip(2); // Pick
                }
            } else {
                ARM_MoveJoint(motion_cmd.joint_id, (int)motion_cmd.direction);
            }
        }

        // 2. Read incoming data from ARM
        int len = uart_read_bytes(UART_ARM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            data[len] = '\0';
            printf("ARM -> BASE: %s\n", data);

            // 🔥 TODO: Parse ARM feedback
            // Example:
            // if (strstr((char*)data, "BOX_PICKED")) { ... }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}