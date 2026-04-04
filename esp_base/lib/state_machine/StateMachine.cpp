#include "StateMachine.h"

// --- Global Mailboxes ---
// These must be created in your main.cpp setup()!
QueueHandle_t guiMailbox;
QueueHandle_t pidMailbox;

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
        
        // ==========================================
        // MANUAL CONTROL (GUI to PID Routing)
        // ==========================================
        case RobotState::MANUAL_MODE: {
            // Package the GUI joystick commands into a PID target struct
            VelocityTarget new_target;
            new_target.v_x = local_gui.cmd_payload.drive_vx;
            new_target.v_y = local_gui.cmd_payload.drive_vy;
            new_target.omega = local_gui.cmd_payload.drive_omega;
            
            // WRITE TO PID: Shove the target into the PID mailbox.
            // Overwrite ensures the PID always has the absolute newest instruction.
            xQueueOverwrite(pidMailbox, &new_target);
            
            // Send Arm Angles over UART (Skeleton placeholder)
            // ARM_UART.printf("..."); 
            break;
        }

        // ==========================================
        // HYBRID PICK SEQUENCE (Skeletons)
        // ==========================================
        case RobotState::START_PICK_SEQUENCE:
            Serial.println("SKELETON: Triggering QR Scan...");
            // CAM_UART.println("SCAN_QR");
            stateTimer = millis();
            currentState = RobotState::WAIT_FOR_VISION_QR;
            break;

        case RobotState::WAIT_FOR_VISION_QR:
            // SKELETON: Wait for Eyad's camera payload over UART
            // If Match -> ARM_UART.println("PICK ...") -> Goto WAIT_FOR_ARM_PICK
            break;

        case RobotState::WAIT_FOR_ARM_PICK:
            // SKELETON: Wait for "BOX_PICKED" over UART from Joe
            // If Success -> Log to storage -> Goto START_PICK_SEQUENCE
            break;

        // ==========================================
        // DROP SEQUENCE (Skeletons)
        // ==========================================
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