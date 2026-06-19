#ifndef UART_SLAVE_ARM_H
#define UART_SLAVE_ARM_H

#include <Arduino.h>

extern String latest_arm_cmd;
extern volatile bool new_arm_cmd;

extern volatile bool drop_sequence_finished;
extern char active_drop_color[16];

void UART_Arm_Init();
void UART_Arm_Task(void *arg);
void sendResponse(const char *msg);

#endif