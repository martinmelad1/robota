#include <Arduino.h>
#include "UART_Slave_Arm.h"
#include "Servo_Control.h"

void setup() {
  Serial.begin(115200);
  
  // Initialize Servo logic and Mailbox
  Servo_Control_Init();

  // Initialize UART port and task
  UART_Arm_Init();
  xTaskCreatePinnedToCore(UART_Arm_Task, "UART_Arm_Task", 4096, NULL, 1, NULL, 1);
  
  // Start Servo task on Core 1
  xTaskCreatePinnedToCore(Servo_Control_Task, "Servo_Control_Task", 4096, NULL, 1, NULL, 1);

  Serial.println("ARM Subsystem Ready. Waiting for commands...");
}

void loop() {
  // Read string forwarded from the UART Slave background task
  if (new_arm_cmd) {
    Serial.println("MAIN LOOP SAW THE COMMAND! It is: " + latest_arm_cmd);
    
    // Process what you need here...

    new_arm_cmd = false; // Reset the flag
  }

  delay(20);
}