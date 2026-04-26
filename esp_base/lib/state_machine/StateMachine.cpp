#include "StateMachine.h"
#include "PID_Control.h"
#include "UART_Master.h"

extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;
extern QueueHandle_t armMailbox;
extern HardwareSerial CAM_UART;
extern HardwareSerial ARM_UART;
extern char cam_ip_address[20];
extern volatile float ultrasonic_distance_cm;

// Memory Arrays
FieldBox field_boxes[TOTAL_TARGETS] = {{"QR_1", "red", 0.0, 0.0, false},
                                       {"QR_2", "green", 0.0, 0.0, false},
                                       {"QR_3", "blue", 0.0, 0.0, false}};

StorageSlot storage_unit[TOTAL_TARGETS] = {{"red", false, 10.0, 0.0, 15.0},
                                           {"green", false, 20.0, 0.0, 15.0},
                                           {"blue", false, 30.0, 0.0, 15.0}};

DropoffZone dropoff_zones[TOTAL_TARGETS] = {{"red", 0.0, 0.0, false},
                                            {"green", 0.0, 0.0, false},
                                            {"blue", 0.0, 0.0, false}};
MasterStateMachine::MasterStateMachine() {
  currentState = RobotState::MANUAL_MODE;
}

void MasterStateMachine::init() {
  Serial.println("SYSTEM BOOT: Master Logic Online.");
}

String MasterStateMachine::getTelemetryJSON() {
  double fl = 0, fr = 0, rl = 0, rr = 0;
  PID_GetActualSpeeds(&fl, &fr, &rl, &rr);

  // Determine pick status for dashboard camera feed
  String pickStatus = "idle";
  if (currentState == RobotState::START_PICK_SEQUENCE || currentState == RobotState::WAIT_FOR_VISION_QR) {
    pickStatus = "scanning";
  } else if (currentState == RobotState::WAIT_FOR_ARM_PICK || currentState == RobotState::WAIT_FOR_ARM_PICK_FINISH) {
    pickStatus = "found";
  }

  String json = "{";
  json += "\"cmd\":\"" + lastCommand + "\",";
  json += "\"mode\":\"" + currentModeStr + "\",";
  json += "\"pick_status\":\"" + pickStatus + "\",";
  json += "\"pick_color\":\"" + foundColor + "\",";
  json += "\"cam_ip\":\"" + String(cam_ip_address) + "\",";
  json += "\"px\":0.0, \"py\":0.0, \"hdg\":0.0,";
  json += "\"fl\":" + String(fl) + ", \"fr\":" + String(fr) +
          ", \"rl\":" + String(rl) + ", \"rr\":" + String(rr) + ",";
  json += "\"j1\":0.0, \"j2\":0.0, \"j3\":0.0,";
  json += "\"grip\":1,";
  json += "\"dist\":" + String(ultrasonic_distance_cm, 2);
  json += "}";
  return json;
}

void MasterStateMachine::update() {
  StringMessage msg;

  // 1. READ FROM GUI
  while (xQueueReceive(guiMailbox, &msg, 0) == pdTRUE) {
    String cmdStr = String(msg.data);
    lastCommand = cmdStr;
    Serial.println("SM Received CMD: " + cmdStr);

    GUITrigger mode_trigger = GUITrigger::NONE;
    ChassisMotion base_motion;
    base_motion.move_type = DriveCommand::CMD_STOP;
    base_motion.speed = 0.5;
    base_motion.omega = 1.5708;

    ArmMotion arm_motion;
    arm_motion.joint_id = 0;
    arm_motion.direction = ArmDir::STOP;

    bool isChassisCmd = false;
    bool isArmCmd = false;

    if (cmdStr == "ESTOP") {
      base_motion.move_type = DriveCommand::CMD_STOP;
      arm_motion.joint_id = 0;
      arm_motion.direction = ArmDir::STOP;
      lastCommand = "EMERGENCY STOP";
      isChassisCmd = true;
      isArmCmd = true;

      // ESTOP also acts as the Manual button
      mode_trigger = GUITrigger::TRIGGER_MANUAL;
      currentModeStr = "MANUAL";
    } else if (cmdStr == "MODE_MANUAL") {
      mode_trigger = GUITrigger::TRIGGER_MANUAL;
      currentModeStr = "MANUAL";
    } else if (cmdStr == "MODE_AUTO") {
      mode_trigger = GUITrigger::TRIGGER_AUTO;
      currentModeStr = "AUTONOMOUS";
    } else if (cmdStr == "MODE_PICK") {
      mode_trigger = GUITrigger::TRIGGER_PICK;
      currentModeStr = "PICK";
    } else if (cmdStr == "FWD") {
      base_motion.move_type = DriveCommand::CMD_FWD;
      isChassisCmd = true;
    } else if (cmdStr == "BWD") {
      base_motion.move_type = DriveCommand::CMD_BWD;
      isChassisCmd = true;
    } else if (cmdStr == "LEFT") {
      base_motion.move_type = DriveCommand::CMD_LEFT;
      isChassisCmd = true;
    } else if (cmdStr == "RIGHT") {
      base_motion.move_type = DriveCommand::CMD_RIGHT;
      isChassisCmd = true;
    } else if (cmdStr == "FWD_LEFT") {
      base_motion.move_type = DriveCommand::CMD_FWD_L;
      isChassisCmd = true;
    } else if (cmdStr == "FWD_RIGHT") {
      base_motion.move_type = DriveCommand::CMD_FWD_R;
      isChassisCmd = true;
    } else if (cmdStr == "BWD_LEFT") {
      base_motion.move_type = DriveCommand::CMD_BWD_L;
      isChassisCmd = true;
    } else if (cmdStr == "BWD_RIGHT") {
      base_motion.move_type = DriveCommand::CMD_BWD_R;
      isChassisCmd = true;
    } else if (cmdStr == "ROT_L") {
      base_motion.move_type = DriveCommand::CMD_ROT_L;
      isChassisCmd = true;
    } else if (cmdStr == "ROT_R") {
      base_motion.move_type = DriveCommand::CMD_ROT_R;
      isChassisCmd = true;
    } else if (cmdStr == "STOP") {
      base_motion.move_type = DriveCommand::CMD_STOP;
      isChassisCmd = true;
    } else if (cmdStr == "J1_UP") {
      arm_motion.joint_id = 1;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J1_DOWN") {
      arm_motion.joint_id = 1;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "J2_UP") {
      arm_motion.joint_id = 2;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "J2_DOWN") {
      arm_motion.joint_id = 2;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J3_UP") {
      arm_motion.joint_id = 3;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J3_DOWN") {
      arm_motion.joint_id = 3;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_OPEN") {
      arm_motion.joint_id = 5;
      arm_motion.direction = ArmDir::UP; // Mapped to 0 in UART_Master
      isArmCmd = true;
    } else if (cmdStr == "GRIP_CLOSE") {
      arm_motion.joint_id = 5;
      arm_motion.direction = ArmDir::DOWN; // Mapped to 1 in UART_Master
      isArmCmd = true;
    } else if (cmdStr == "GRIP_PICK") {
      arm_motion.joint_id = 5;
      arm_motion.direction =
          ArmDir::STOP; // I mapped the STOP enum purely as a placeholder
                        // integer to trigger PICK!
      isArmCmd = true;
    } else if (cmdStr == "ARM_STOP") {
      arm_motion.joint_id = 0;
      arm_motion.direction = ArmDir::STOP;
      isArmCmd = true;
    } else if (cmdStr.startsWith("QR_OK:")) {
      String color = cmdStr.substring(6);
      if (currentState == RobotState::WAIT_FOR_VISION_QR) {
        if (color == "red" || color == "green" || color == "blue") {
          // Check if this color is already picked/full
          bool alreadyPicked = false;
          for (int i = 0; i < TOTAL_TARGETS; i++) {
              if (storage_unit[i].assignedColor == color && storage_unit[i].isFull) {
                  alreadyPicked = true;
                  break;
              }
          }

          if (alreadyPicked) {
            Serial.println("Vision confirmed color: " + color +
                           ", but its storage slot is ALREADY FULL! Aborting.");
            currentState = RobotState::MANUAL_MODE;
          } else {
            foundColor = color;
            Serial.println("Vision confirmed color: " + color +
                           ". Proceeding to PICK.");
            currentState = RobotState::WAIT_FOR_ARM_PICK;
            stateTimer = millis();
          }
        } else {
          Serial.println("Vision found no valid color. Aborting auto-pick.");
          currentState = RobotState::MANUAL_MODE;
        }
      }
    }

    // 2. EMERGENCY MODE OVERRIDES
    if (mode_trigger == GUITrigger::TRIGGER_MANUAL &&
        currentState != RobotState::MANUAL_MODE) {
      Serial.println("OVERRIDE: Entering Manual Mode.");
      currentState = RobotState::MANUAL_MODE;
    } else if (mode_trigger == GUITrigger::TRIGGER_PICK &&
               currentState == RobotState::MANUAL_MODE) {
      currentState = RobotState::START_PICK_SEQUENCE;
    } else if (mode_trigger == GUITrigger::TRIGGER_AUTO &&
               currentState == RobotState::MANUAL_MODE) {
      currentState = RobotState::START_AUTO_DROP_SEQUENCE;
    }

    if (currentState == RobotState::MANUAL_MODE) {
      if (isChassisCmd) {
        xQueueOverwrite(pidMailbox, &base_motion);
      }
      if (isArmCmd) {
        xQueueOverwrite(armMailbox, &arm_motion);
      }
    }
  }

  // 3. THE AUTONOMOUS ROUTER
  switch (currentState) {
  case RobotState::MANUAL_MODE: {
    break;
  }

  case RobotState::START_PICK_SEQUENCE: {
    Serial.println("Auto Pick: Stopping Chassis and Requesting QR Scan...");

    // Stop the chassis
    ChassisMotion stop_motion;
    stop_motion.move_type = DriveCommand::CMD_STOP;
    stop_motion.speed = 0;
    stop_motion.omega = 0;
    xQueueOverwrite(pidMailbox, &stop_motion);

    // Send instruction to CAM
    CAM_RequestQR();

    stateTimer = millis();
    currentState = RobotState::WAIT_FOR_VISION_QR;
    break;
  }

  case RobotState::WAIT_FOR_VISION_QR:
    // Waiting for QR_OK:xxx from UART_Cam_Task via guiMailbox
    if (millis() - stateTimer > 6000) {
      Serial.println("QR Scan timed out! Returning to Manual.");
      currentState = RobotState::MANUAL_MODE;
    }
    break;

  case RobotState::WAIT_FOR_ARM_PICK: {
    // We execute the ARM PICK motion immediately and transition to a wait state
    Serial.println("Executing Arm PICK Action...");
    ArmMotion pick_motion;
    pick_motion.joint_id = 5;             // Gripper
    pick_motion.direction = ArmDir::STOP; // Pick command mapping
    xQueueOverwrite(armMailbox, &pick_motion);
    
    // Transition to the actual wait state and reset our timer
    currentState = RobotState::WAIT_FOR_ARM_PICK_FINISH;
    stateTimer = millis();
    break;
  }

  case RobotState::WAIT_FOR_ARM_PICK_FINISH: {
    // Wait for 6 seconds for the pick action to complete
    if (millis() - stateTimer > 6000) {
      // It's been 6 seconds since pick, now move to the placement box based on colour
      float targetX = 0, targetY = 0, targetZ = 0;
      for (int i = 0; i < TOTAL_TARGETS; i++) {
        if (storage_unit[i].assignedColor == foundColor) {
          targetX = storage_unit[i].slotX;
          targetY = storage_unit[i].slotY;
          targetZ = storage_unit[i].slotZ;
          storage_unit[i].isFull = true;
          break;
        }
      }

      Serial.print("Moving Arm to placement box position based on color ");
      Serial.print(foundColor);
      Serial.printf(": X=%.2f Y=%.2f Z=%.2f\n", targetX, targetY, targetZ);

      ARM_MoveXYZ(targetX, targetY, targetZ);

      // Go back to manual after commanding pick & place sequence
      currentState = RobotState::MANUAL_MODE;
    }
    break;
  }

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