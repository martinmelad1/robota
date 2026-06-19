#ifndef UART_SLAVE_CAM_H
#define UART_SLAVE_CAM_H

#include <Arduino.h>

void UART_Slave_Init();
bool UART_CheckForCommand(String expectedCmd);
void UART_SendResult(String result);

#endif
