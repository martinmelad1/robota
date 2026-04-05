#include "UART_Slave_Arm.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_1
#define TXD 17
#define RXD 16
#define BUF_SIZE 1024

void UART_Arm_Init() {
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &config);
    uart_set_pin(UART_PORT, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void sendResponse(const char *msg) {
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    uart_write_bytes(UART_PORT, "\n", 1);
}

void processArmCommand(char *cmd) {

    if (strncmp(cmd, "MOVE:", 5) == 0) {
        float x, y, z;
        sscanf(cmd + 5, "%f,%f,%f", &x, &y, &z);

        printf("Move to %.2f %.2f %.2f\n", x, y, z);
        sendResponse("OK");
    }

    else if (strncmp(cmd, "GRIP:", 5) == 0) {
        printf("Grip command\n");
        sendResponse("OK");
    }

    else {
        sendResponse("ERROR");
    }
}

void UART_Arm_Task(void *arg) {
    uint8_t data[BUF_SIZE];
    char line[128];
    int idx = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

        for (int i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\n') {
                line[idx] = '\0';
                processArmCommand(line);
                idx = 0;
            } else {
                if (idx < sizeof(line) - 1)
                    line[idx++] = c;
            }
        }
    }
}