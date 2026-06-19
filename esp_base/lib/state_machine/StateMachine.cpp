#include "StateMachine.h"
#include "Odometry.h"
#include "PID_Control.h"
#include "PathPlanner.h"
#include "UART_Master.h"

extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;
extern QueueHandle_t armMailbox;
extern HardwareSerial CAM_UART;
extern HardwareSerial ARM_UART;
extern char cam_ip_address[20];
extern volatile float ultrasonic_distance_cm;
extern volatile float arm_j1_deg;
extern volatile float arm_j2_deg;
extern volatile float arm_j3_deg;
extern volatile float imu_yaw_deg;
extern volatile float imu_ax_mps2;
extern volatile float imu_ay_mps2;

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

static String gainsJSON(const WheelGains &g) {
  return "{\"Kp\":" + String(g.Kp, 3) + ",\"Ki\":" + String(g.Ki, 3) +
         ",\"Kd\":" + String(g.Kd, 3) + "}";
}

String MasterStateMachine::getTelemetryJSON() {
  double fl = 0, fr = 0, rl = 0, rr = 0;
  PID_GetActualSpeeds(&fl, &fr, &rl, &rr);

  Pose p = Odometry_GetPose();

  PIDTelemetry pid;
  PID_GetTelemetry(pid);

  // Convert ticks/sample → RPM
  // PID_Compute sample time is PID_SAMPLE_MS (50 ms); encoder PPR = 748
  const float TICKS_TO_RPM = 60000.0f / (748.0f * 50.0f);
  float rpmSetFL = pid.velSetFL * TICKS_TO_RPM;
  float rpmSetFR = pid.velSetFR * TICKS_TO_RPM;
  float rpmSetRL = pid.velSetRL * TICKS_TO_RPM;
  float rpmSetRR = pid.velSetRR * TICKS_TO_RPM;
  float rpmActFL = pid.velActFL * TICKS_TO_RPM;
  float rpmActFR = pid.velActFR * TICKS_TO_RPM;
  float rpmActRL = pid.velActRL * TICKS_TO_RPM;
  float rpmActRR = pid.velActRR * TICKS_TO_RPM;

  String pickStatus = "idle";
  if (currentState == RobotState::CAMERA_PREVIEW) {
    pickStatus = "preview";
  } else if (currentState == RobotState::START_PICK_SEQUENCE ||
      currentState == RobotState::WAIT_FOR_VISION_QR) {
    pickStatus = "scanning";
  } else if (currentState == RobotState::WAIT_FOR_ARM_PICK ||
             currentState == RobotState::WAIT_FOR_ARM_PICK_FINISH) {
    pickStatus = "found";
  }

  String json = "{";
  json += "\"cmd\":\"" + lastCommand + "\",";
  json += "\"mode\":\"" + currentModeStr + "\",";
  json += "\"pick_status\":\"" + pickStatus + "\",";
  json += "\"pick_color\":\"" + foundColor + "\",";
  json += "\"cam_ip\":\"" + String(cam_ip_address) + "\",";

  json += "\"driveSpeed\":" + String(PID_GetDriveSpeed(), 2) + ",";

  json += "\"px\":" + String(p.x, 3) + ",";
  json += "\"py\":" + String(p.y, 3) + ",";
  json += "\"hdg\":" + String(p.theta * 180.0f / M_PI, 1) + ",";

  json += "\"fl\":" + String(fl) + ",";
  json += "\"fr\":" + String(fr) + ",";
  json += "\"rl\":" + String(rl) + ",";
  json += "\"rr\":" + String(rr) + ",";

  // Raw ticks/sample setpoints & actuals
  json += "\"vSet\":[" + String(pid.velSetFL, 2) + "," +
          String(pid.velSetFR, 2) + "," + String(pid.velSetRL, 2) + "," +
          String(pid.velSetRR, 2) + "],";
  json += "\"vAct\":[" + String(pid.velActFL, 2) + "," +
          String(pid.velActFR, 2) + "," + String(pid.velActRL, 2) + "," +
          String(pid.velActRR, 2) + "],";

  // RPM setpoints (simulated / desired)
  json += "\"rpmSet\":[" + String(rpmSetFL, 1) + "," + String(rpmSetFR, 1) +
          "," + String(rpmSetRL, 1) + "," + String(rpmSetRR, 1) + "],";
  // RPM actuals (real / measured)
  json += "\"rpmAct\":[" + String(rpmActFL, 1) + "," + String(rpmActFR, 1) +
          "," + String(rpmActRL, 1) + "," + String(rpmActRR, 1) + "],";

  json += "\"velGains\":[";
  for (int i = 0; i < 4; i++) {
    json += gainsJSON(pid.velGains[i]);
    if (i < 3)
      json += ",";
  }
  json += "],";

  json += "\"j1\":" + String(arm_j1_deg, 1) + ",";
  json += "\"j2\":" + String(arm_j2_deg, 1) + ",";
  json += "\"j3\":" + String(arm_j3_deg, 1) + ",";
  json += "\"grip\":1,";
  json += "\"dist\":" + String(ultrasonic_distance_cm, 2) + ",";

  // ── Velocity & Direction (new) ───────────────────────────────
  json += "\"vel_mps\":" + String(PID_GetAvgSpeedMps(), 3) + ",";
  json += "\"imu_hdg\":" + String(imu_yaw_deg, 1) + ",";
  json += "\"speed_gain\":" + String(PID_GetSpeedGain(), 3) + ",";

  // ── Heading (inner) PID loop telemetry ─────────────────────────
  // hdg_err : current heading error (deg) — inner PID input
  // hdg_out : Wz correction output — inner PID output
  // hdgGains: current Kp/Ki/Kd of the heading PID
  json += "\"hdg_err\":" + String(pid.hdgErr, 2) + ",";
  json += "\"hdg_out\":" + String(pid.hdgOut, 3) + ",";
  json += "\"hdgGains\":" + gainsJSON(pid.hdgGains);
  json += "}";
  return json;
}

static void parsePIDTune(const String &cmd) {
  // Format: PID_TUNE:vel:FL:Kp:Ki:Kd  (outer velocity loop, per wheel)
  //         PID_TUNE:hdg:0:Kp:Ki:Kd   (inner heading loop, wheel ignored)
  String parts[6];
  int count = 0, start = 0;
  for (int i = 0; i <= (int)cmd.length() && count < 6; i++) {
    if (i == (int)cmd.length() || cmd[i] == ':') {
      parts[count++] = cmd.substring(start, i);
      start = i + 1;
    }
  }
  if (count < 6)
    return;

  // Determine mode from parts[1]
  int mode = 0;
  if (parts[1] == "hdg") mode = 1;

  int wheel = 0; // only used when mode==0
  if (mode == 0) {
    if      (parts[2] == "FL") wheel = 0;
    else if (parts[2] == "FR") wheel = 1;
    else if (parts[2] == "RL") wheel = 2;
    else if (parts[2] == "RR") wheel = 3;
    else return; // unknown wheel
  }

  PID_SetGains(mode, wheel, parts[3].toFloat(), parts[4].toFloat(),
               parts[5].toFloat());
}

void MasterStateMachine::update() {
  StringMessage msg;

  while (xQueueReceive(guiMailbox, &msg, 0) == pdTRUE) {
    String cmdStr = String(msg.data);
    lastCommand = cmdStr;
    Serial.println("SM Received CMD: " + cmdStr);

    if (cmdStr.startsWith("PID_TUNE:")) {
      parsePIDTune(cmdStr);
      continue;
    }

    if (cmdStr.startsWith("SPEED:")) {
      float spd = cmdStr.substring(6).toFloat();
      PID_SetDriveSpeed(spd);
      continue;
    }

    if (cmdStr.startsWith("SPEED_GAIN:")) {
      float gain = cmdStr.substring(11).toFloat();
      PID_SetSpeedGain(gain);
      continue;
    }

    // ── MOVE_DIST command parser ─────────────────────────────────
    if (cmdStr.startsWith("MOVE_DIST:")) {
      int split1 = cmdStr.indexOf(':');
      int split2 = cmdStr.indexOf(':', split1 + 1);
      if (split2 != -1) {
        String dir = cmdStr.substring(split1 + 1, split2);
        float dist_m = cmdStr.substring(split2 + 1).toFloat();

        ChassisMotion dist_motion;
        dist_motion.move_type = DriveCommand::CMD_DIST;
        dist_motion.speed = PID_GetDriveSpeed();
        dist_motion.Vx = 0.0f;
        dist_motion.Vy = 0.0f;
        dist_motion.Wz = 0.0f;

        if (dir == "FWD")
          dist_motion.Vy = 1.0f;
        else if (dir == "BWD")
          dist_motion.Vy = -1.0f;
        else if (dir == "LEFT")
          dist_motion.Vx = -1.0f;
        else if (dir == "RIGHT")
          dist_motion.Vx = 1.0f;
        else if (dir == "FWD_LEFT") {
          dist_motion.Vy = 0.707f;
          dist_motion.Vx = -0.707f;
        } else if (dir == "FWD_RIGHT") {
          dist_motion.Vy = 0.707f;
          dist_motion.Vx = 0.707f;
        } else if (dir == "BWD_LEFT") {
          dist_motion.Vy = -0.707f;
          dist_motion.Vx = -0.707f;
        } else if (dir == "BWD_RIGHT") {
          dist_motion.Vy = -0.707f;
          dist_motion.Vx = 0.707f;
        }

        // Convert meters to ticks (Circumference ~0.3047m, PPR = 748)
        dist_motion.tickTarget = (long)((dist_m / 0.304734f) * 748.0f);

        xQueueOverwrite(pidMailbox, &dist_motion);
      }
      continue;
    }

    ChassisMotion base_motion;
    base_motion.move_type = DriveCommand::CMD_STOP;
    base_motion.speed = PID_GetDriveSpeed();
    base_motion.omega = 1.5f;

    GUITrigger mode_trigger = GUITrigger::NONE;
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
      mode_trigger = GUITrigger::TRIGGER_MANUAL;
      currentModeStr = "MANUAL";
    } else if (cmdStr == "SHOW_CAM") {
      if (currentState == RobotState::MANUAL_MODE) {
        base_motion.move_type = DriveCommand::CMD_STOP;
        isChassisCmd = true;
        currentState = RobotState::CAMERA_PREVIEW;
        currentModeStr = "PICK";
      }
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
      base_motion.move_type = DriveCommand::CMD_ROT_R;
      isChassisCmd = true;
    } else if (cmdStr == "ROT_R") {
      base_motion.move_type = DriveCommand::CMD_ROT_L;
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
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "J3_DOWN") {
      arm_motion.joint_id = 3;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_TAP_OPEN") {
      arm_motion.joint_id = 6;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_TAP_CLOSE") {
      arm_motion.joint_id = 6;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_TAP_PICK" || cmdStr == "GRIP_PICK") {
      arm_motion.joint_id = 6;
      arm_motion.direction = ArmDir::STOP;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_OPEN") {
      arm_motion.joint_id = 5;
      arm_motion.direction = ArmDir::UP;
      isArmCmd = true;
    } else if (cmdStr == "GRIP_CLOSE") {
      arm_motion.joint_id = 5;
      arm_motion.direction = ArmDir::DOWN;
      isArmCmd = true;
    } else if (cmdStr == "ARM_STOP") {
      arm_motion.joint_id = 0;
      arm_motion.direction = ArmDir::STOP;
      isArmCmd = true;
    } else if (cmdStr.startsWith("DROP_DONE:")) {
      String color = cmdStr.substring(10);
      if (currentState == RobotState::WAIT_FOR_ARM_DROP) {
        // Drop confirmed by ARM. Update slot status and move to next point.
        for (int i = 0; i < TOTAL_TARGETS; i++) {
          const char *targetColor = (pendingDrop == DropAction::DROP_RED)  ? "red"
                                  : (pendingDrop == DropAction::DROP_BLUE) ? "blue"
                                                                           : "green";
          if (storage_unit[i].assignedColor == targetColor) {
            storage_unit[i].isFull = false;
            Serial.println(String("SM: Slot ") + targetColor + " marked EMPTY (Confirmed by ARM: " + color + ").");
            if (String(targetColor) != color) {
              Serial.println("WARNING: ARM reported different color drop than expected!");
            }
            break;
          }
        }
        pendingDrop = DropAction::NONE;
        PathPlanner_AcknowledgeAction();
        currentState = RobotState::NAVIGATING_TO_DROP;
      }
    } else if (cmdStr.startsWith("QR_OK:")) {
      String color = cmdStr.substring(6);
      if (currentState == RobotState::WAIT_FOR_VISION_QR) {
        if (color == "red" || color == "green" || color == "blue") {
          bool alreadyPicked = false;
          for (int i = 0; i < TOTAL_TARGETS; i++) {
            if (storage_unit[i].assignedColor == color &&
                storage_unit[i].isFull) {
              alreadyPicked = true;
              break;
            }
          }
          if (alreadyPicked) {
            Serial.println("Vision: color " + color + " slot FULL, aborting.");
            currentState = RobotState::MANUAL_MODE;
          } else {
            foundColor = color;
            currentState = RobotState::WAIT_FOR_ARM_PICK;
            stateTimer = millis();
          }
        } else {
          Serial.println("Vision: no valid color, aborting.");
          currentState = RobotState::MANUAL_MODE;
        }
      }
    }

    if (mode_trigger == GUITrigger::TRIGGER_MANUAL &&
        currentState != RobotState::MANUAL_MODE) {
      currentState = RobotState::MANUAL_MODE;
    } else if (mode_trigger == GUITrigger::TRIGGER_PICK &&
               (currentState == RobotState::MANUAL_MODE ||
                currentState == RobotState::CAMERA_PREVIEW)) {
      currentState = RobotState::START_PICK_SEQUENCE;
    } else if (mode_trigger == GUITrigger::TRIGGER_AUTO &&
               currentState == RobotState::MANUAL_MODE) {
      currentState = RobotState::START_AUTO_DROP_SEQUENCE;
    }

    if (currentState == RobotState::MANUAL_MODE) {
      if (isChassisCmd)
        xQueueOverwrite(pidMailbox, &base_motion);
    }

    if (currentState == RobotState::MANUAL_MODE ||
        currentState == RobotState::CAMERA_PREVIEW ||
        currentState == RobotState::START_PICK_SEQUENCE ||
        currentState == RobotState::WAIT_FOR_VISION_QR) {
      if (isArmCmd)
        xQueueOverwrite(armMailbox, &arm_motion);
    }
  }

  switch (currentState) {
  case RobotState::MANUAL_MODE:
    break;
  case RobotState::CAMERA_PREVIEW:
    break;
  case RobotState::START_PICK_SEQUENCE: {
    ChassisMotion stop;
    stop.move_type = DriveCommand::CMD_STOP;
    stop.speed = 0;
    stop.omega = 0;
    xQueueOverwrite(pidMailbox, &stop);
    CAM_RequestQR();
    stateTimer = millis();
    currentState = RobotState::WAIT_FOR_VISION_QR;
    break;
  }
  case RobotState::WAIT_FOR_VISION_QR:
    if (millis() - stateTimer > 6000)
      currentState = RobotState::MANUAL_MODE;
    break;
  case RobotState::WAIT_FOR_ARM_PICK: {
    // Box is already gripped — confirm CLOSE to prevent accidental release.
    // ArmDir::STOP (GRIP:PICK = 90°) was opening the gripper and dropping the
    // box.
    ArmMotion pick;
    pick.joint_id = 6;
    pick.direction = ArmDir::DOWN; // → GRIP:CLOSE (180°) — maintain secure grip
    xQueueOverwrite(armMailbox, &pick);
    currentState = RobotState::WAIT_FOR_ARM_PICK_FINISH;
    stateTimer = millis();
    break;
  }
  case RobotState::WAIT_FOR_ARM_PICK_FINISH:
    // Wait briefly for the gripper to fully close, then mark slot as loaded.
    if (millis() - stateTimer > 2000) {
      for (int i = 0; i < TOTAL_TARGETS; i++) {
        if (storage_unit[i].assignedColor == foundColor) {
          storage_unit[i].isFull = true;
          Serial.println("SM: Slot " + foundColor + " marked LOADED.");
          break;
        }
      }
      currentState = RobotState::MANUAL_MODE;
    }
    break;
  case RobotState::START_AUTO_DROP_SEQUENCE:
    // Begin the 12-step encoder-only position-controller path (RED → BLUE → GREEN)
    PathPlanner_Start();
    currentState = RobotState::NAVIGATING_TO_DROP;
    break;

  case RobotState::NAVIGATING_TO_DROP:
    if (PathPlanner_Update()) {
      // PathPlanner_Update() returns true every tick while waiting for ACK.
      // Only send the REACHED command ONCE on the first true return.
      // Use pendingDrop as the sent-guard (NONE = not yet sent).
      DropAction action = PathPlanner_GetPendingAction();
      if (pendingDrop == DropAction::NONE && action != DropAction::NONE) {
        pendingDrop = action;
        const char *dropColor = (pendingDrop == DropAction::DROP_RED)  ? "red"
                              : (pendingDrop == DropAction::DROP_BLUE) ? "blue"
                                                                       : "green";
        Serial.printf("SM: DROP reached — sending REACHED:%s to ARM via UART\n", dropColor);

        // Send REACHED:<color> over UART2 → arm executes full drop sequence autonomously.
        // ARM drop SM total ≈ 7.9 s; we wait 10 s for a comfortable safety margin.
        ARM_SendReached(dropColor);

        stateTimer = millis();
        currentState = RobotState::WAIT_FOR_ARM_DROP;
      }
    }
    break;

  case RobotState::WAIT_FOR_ARM_DROP:
    // Wait for the ARM to complete its drop sequence (≈ 7.9 s measured,
    // 10 s timeout gives a comfortable margin without stalling the path).
    if (millis() - stateTimer > 10000) {
      Serial.println("SM: Arm drop timeout — ACKing path planner.");
      pendingDrop = DropAction::NONE;   // clear sent-guard for next drop

      // Tell the path planner to advance past the PLACE step.
      PathPlanner_AcknowledgeAction();

      if (PathPlanner_IsComplete()) {
        Serial.println("SM: All drops complete — returning to MANUAL.");
        currentModeStr = "MANUAL";
        currentState   = RobotState::MANUAL_MODE;
      } else {
        currentState = RobotState::NAVIGATING_TO_DROP;
      }
    }
    break;
  }
}