#include "UART_Slave_Arm.h"
#include "Servo_Control.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_PORT UART_NUM_1
#define TXD 17
#define RXD 16
#define BUF_SIZE 1024

String latest_arm_cmd = "";
volatile bool new_arm_cmd = false;

void UART_Arm_Init() {
  uart_config_t config = {.baud_rate = 115200,
                          .data_bits = UART_DATA_8_BITS,
                          .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1,
                          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
  uart_param_config(UART_PORT, &config);
  uart_set_pin(UART_PORT, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void sendResponse(const char *msg) {
  uart_write_bytes(UART_PORT, msg, strlen(msg));
  uart_write_bytes(UART_PORT, "\n", 1);
}

void processArmCommand(char *cmd) {

  printf("\n>>> BASE ESP -> ARM ESP RECEIVED: '%s' <<<\n", cmd);

  // Pass it to the main loop
  latest_arm_cmd = String(cmd);
  new_arm_cmd = true;

  if (strncmp(cmd, "MOVE:", 5) == 0) {
    char *ptr = cmd + 5;
    float x = strtof(ptr, &ptr);
    if (*ptr == ',')
      ptr++;
    float y = strtof(ptr, &ptr);
    if (*ptr == ',')
      ptr++;
    float z = strtof(ptr, NULL);

    printf("Move to %.2f %.2f %.2f\n", x, y, z);
    sendResponse("OK");
  }

  else if (strncmp(cmd, "GRIP:", 5) == 0) {
    ServoCommand sc;
    sc.joint_id = 6;

    if (strncmp(cmd + 5, "OPEN", 4) == 0) {
      sc.direction = 1;
    } else if (strncmp(cmd + 5, "CLOSE", 5) == 0) {
      sc.direction = -1;
    } else if (strncmp(cmd + 5, "PICK", 4) == 0) {
      sc.direction = 2;
    } else {
      sendResponse("ERROR");
      return;
    }

    if (servoMailbox != NULL) {
      xQueueSend(servoMailbox, &sc, 0);
    }
    printf("Grip command: %s\n", cmd + 5);
    sendResponse("OK");
  }

  else if (strncmp(cmd, "JOINT:", 6) == 0) {
    char *ptr = cmd + 6;
    int joint_id = strtol(ptr, &ptr, 10);
    int direction = 0;
    bool valid = false;

    if (*ptr == ',') {
      ptr++;
      direction = strtol(ptr, NULL, 10);
      valid = true;
    }

    if (valid) {
      ServoCommand sc;
      sc.joint_id = joint_id;
      sc.direction = direction;

      printf("Joint command - ID: %d, Dir: %d\n", joint_id, direction);

      if (servoMailbox != NULL) {
        xQueueSend(servoMailbox, &sc, 0);
      }
      sendResponse("OK");
    } else {
      sendResponse("ERROR");
    }
  }

  // ── COLOR pre-announcement ─────────────────────────────────
  // BASE sends this before navigation begins so the ARM can prepare.
  // Nothing to do yet — colour is acted on when REACHED arrives.
  else if (strncmp(cmd, "COLOR:", 6) == 0) {
    Serial.printf("[ARM] Incoming colour pre-announced: %s\n", cmd + 6);
    sendResponse("OK");
  }

  // ── REACHED: execute pick-from-slot + drop sequence ───────
  // BASE sends this when the robot arrives at the drop zone.
  // Triggers the full autonomous drop sequence on the ARM side.
  else if (strncmp(cmd, "REACHED:", 8) == 0) {
    const char *color = cmd + 8;
    Serial.printf("[ARM] REACHED received for colour: %s\n", color);
    Servo_QueueDropSequence(color);
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
  int fb_counter = 0;

  while (1) {
    int len =
        uart_read_bytes(UART_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

    if (len > 0) {
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

    // Send joint feedback every ~100ms (5 ticks of 20ms)
    if (++fb_counter >= 5) {
      fb_counter = 0;
      char fb_msg[64];
      // Format must match what esp_base expects: JOINT_FB:j1,j2,j3
      snprintf(fb_msg, sizeof(fb_msg), "JOINT_FB:%d.0,%d.0,%d.0\n", angle1,
               angle2, angle3);
      uart_write_bytes(UART_PORT, fb_msg, strlen(fb_msg));
    }
  }
}