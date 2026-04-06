#ifndef UART_SLAVE_ARM_H
#define UART_SLAVE_ARM_H

#include <Arduino.h>

extern String latest_arm_cmd;
extern volatile bool new_arm_cmd;

void UART_Arm_Init();
void UART_Arm_Task(void *arg);

#endif