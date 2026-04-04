#include "StateMachine.h"

extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;
extern QueueHandle_t armMailbox;
extern HardwareSerial CAM_UART;
extern HardwareSerial ARM_UART;

// (Memory Arrays initialized here - skipped for brevity)

MasterStateMachine::MasterStateMachine() {
    currentState = RobotState::MANUAL_MODE;
}

void MasterStateMachine::init() {
    Serial.println("SYSTEM BOOT: Master Logic Online.");
}

void MasterStateMachine::update() {
    GUIPacket local_gui;
    
    // 1. READ FROM GUI: Peek at the GUI Mailbox. 
    // The '0' means if the box is empty, don't wait, just skip to the else block.
    if (xQueuePeek(guiMailbox, &local_gui, 0) != pdTRUE) {
        // If we haven't received a single Wi-Fi packet yet, just return and do nothing.
        return; 
    }

    // 2. EMERGENCY MODE OVERRIDES
    if (local_gui.mode_trigger == GUITrigger::TRIGGER_MANUAL && currentState != RobotState::MANUAL_MODE) {
        Serial.println("OVERRIDE: Entering Manual Mode.");
        currentState = RobotState::MANUAL_MODE;
    } 
    else if (local_gui.mode_trigger == GUITrigger::TRIGGER_PICK && currentState == RobotState::MANUAL_MODE) {
        currentState = RobotState::START_PICK_SEQUENCE;
    }
    else if (local_gui.mode_trigger == GUITrigger::TRIGGER_AUTO && currentState == RobotState::MANUAL_MODE) {
        currentState = RobotState::START_AUTO_DROP_SEQUENCE;
    }

    // 3. THE AUTONOMOUS ROUTER
    switch (currentState) {
        case RobotState::MANUAL_MODE: {
            xQueueOverwrite(pidMailbox, &local_gui.base_motion);
            xQueueOverwrite(armMailbox, &local_gui.arm_motion);
            break;
        }


        case RobotState::START_PICK_SEQUENCE:
            Serial.println("SKELETON: Triggering QR Scan...");
            // CAM_UART.println("SCAN_QR");
            stateTimer = millis();
            currentState = RobotState::WAIT_FOR_VISION_QR;
            break;

        case RobotState::WAIT_FOR_VISION_QR:

            break;

        case RobotState::WAIT_FOR_ARM_PICK:

            break;

        case RobotState::START_AUTO_DROP_SEQUENCE:
            Serial.println("SKELETON: Navigating to Drop Zone...");
            // Path Planner Integration Goes Here
            currentState = RobotState::NAVIGATING_TO_DROP;
            break;
            
        case RobotState::NAVIGATING_TO_DROP:
        case RobotState::WAIT_FOR_VISION_COLOR:
        case RobotState::WAIT_FOR_ARM_DROP:
            // SKELETON: Implement Look-Move-Look and Drop logic later
            break;
    }
}