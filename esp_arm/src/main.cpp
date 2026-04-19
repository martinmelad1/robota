#include <Arduino.h>
#include "UART_Slave_Arm.h"
#include "Servo_Control.h"
#include "Standalone_Server.h"

void setup() {
  Serial.begin(115200);
  
  // Initialize Servo logic and Mailbox
  Servo_Control_Init();

  // Initialize UART port and task
  UART_Arm_Init();

  xTaskCreatePinnedToCore(UART_Arm_Task, "UART_Arm_Task", 4096, NULL, 1, NULL, 1);
  
  // Start Servo task on Core 1
  xTaskCreatePinnedToCore(Servo_Control_Task, "Servo_Control_Task", 4096, NULL, 1, NULL, 1);

  // Initialize modular Fallback WiFi & WebServer
  Standalone_Server_Init();

  Serial.println("ARM Subsystem Ready. Waiting for commands...");
}

void loop() {
  // Update Standalone Server state (tracks UART connection and cleans up WebSockets)
  Standalone_Server_Update();

  // Read string forwarded from the UART Slave background task
  if (new_arm_cmd) {
    Serial.println("MAIN LOOP SAW THE COMMAND! It is: " + latest_arm_cmd);
    new_arm_cmd = false; // Reset the flag
  }

  delay(20);
}