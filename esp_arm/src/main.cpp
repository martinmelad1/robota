#include <Arduino.h>
#include "UART_Slave_Arm.h"
#include "Servo_Control.h"
#include "Standalone_Server.h"
#include "IMU.h"

void setup() {
  Serial.begin(115200);

  // IMU: init + calibrate (robot must be still, blocks ~1 s)
  // Must run before tasks so Wire is initialised on the main core.
  IMU_Init();

  // Initialize Servo logic and Mailbox
  Servo_Control_Init();

  // Initialize UART port and task
  UART_Arm_Init();

  xTaskCreatePinnedToCore(UART_Arm_Task,       "UART_Arm_Task",      4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Servo_Control_Task,  "Servo_Control_Task", 4096, NULL, 1, NULL, 1);

  // IMU update task runs on Core 0 (separate from servo Core 1) at priority 2
  IMU_StartTask();

  // Fallback WiFi & WebServer
  Standalone_Server_Init();

  Serial.println("ARM Subsystem Ready. Waiting for commands...");
}

void loop() {
  Standalone_Server_Update();

  if (new_arm_cmd) {
    Serial.println("MAIN LOOP SAW THE COMMAND! It is: " + latest_arm_cmd);
    new_arm_cmd = false;
  }

  delay(20);
}