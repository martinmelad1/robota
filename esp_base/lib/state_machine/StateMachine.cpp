#include "StateMachine.h"
#include "PID_Control.h"

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

String MasterStateMachine::getTelemetryJSON() {
  double fl = 0, fr = 0, rl = 0, rr = 0;
  PID_GetActualSpeeds(&fl, &fr, &rl, &rr);

  String json = "{";
  json += "\"cmd\":\"" + lastCommand + "\",";
  json += "\"mode\":\"" + currentModeStr + "\",";
  json += "\"px\":0.0, \"py\":0.0, \"hdg\":0.0,";
  json += "\"fl\":" + String(fl) + ", \"fr\":" + String(fr) + ", \"rl\":" + String(rl) + ", \"rr\":" + String(rr) + ",";
  json += "\"j1\":0.0, \"j2\":0.0, \"j3\":0.0, \"j4\":0.0,";
  json += "\"grip\":1";
  json += "}";
  return json;
}

void MasterStateMachine::update() {
  StringMessage msg;

  // 1. READ FROM GUI
  if (xQueueReceive(guiMailbox, &msg, 0) == pdTRUE) {
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
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J2_DOWN") {
      arm_motion.joint_id = 2;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "J3_UP") {
      arm_motion.joint_id = 3;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J3_DOWN") {
      arm_motion.joint_id = 3;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "J4_UP") {
      arm_motion.joint_id = 4;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "J4_DOWN") {
      arm_motion.joint_id = 4;
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
      arm_motion.direction = ArmDir::STOP; // I mapped the STOP enum purely as a placeholder integer to trigger PICK!
      isArmCmd = true;
    } else if (cmdStr == "ARM_STOP") {
    arm_motion.joint_id = 0;
    arm_motion.direction = ArmDir::STOP;
    isArmCmd = true;
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