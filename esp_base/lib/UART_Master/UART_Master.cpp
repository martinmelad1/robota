#include "UART_Master.h"

#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ========================
// UART CONFIG
// ========================
#define UART_CAM UART_NUM_1
#define UART_ARM UART_NUM_2

#define BUF_SIZE 1024

// Pins
#define CAM_TX 17
#define CAM_RX 16

#define ARM_TX 25
#define ARM_RX 26

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
    sprintf(cmd, "MOVE %.2f %.2f %.2f\n", x, y, z);

    uart_write_bytes(UART_ARM, cmd, strlen(cmd));
    printf("Sent to ARM: %s", cmd);
}

void ARM_Grip(int close)
{
    if (close)
    {
        uart_write_bytes(UART_ARM, "GRIP_CLOSE\n", 11);
        printf("Sent to ARM: GRIP_CLOSE\n");
    }
    else
    {
        uart_write_bytes(UART_ARM, "GRIP_OPEN\n", 10);
        printf("Sent to ARM: GRIP_OPEN\n");
    }
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