#include "UART_slave_cam.h"

void UART_Slave_Init() {
    // Assuming we use standard Serial (UART0) to communicate with base.
    // Ensure baud rate matches ESP_BASE UART_CAM (115200).
    Serial.begin(115200);   
}

bool UART_CheckForCommand(String expectedCmd) {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == expectedCmd) {
            return true;
        }
    }
    return false;
}

void UART_SendResult(String result) {
    // Append newline so the base can process it easily.
    Serial.println(result);
}
