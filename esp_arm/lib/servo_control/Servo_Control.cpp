#include "Servo_Control.h"

// ── Queues ────────────────────────────────────────────────────
QueueHandle_t servoMailbox   = NULL;
QueueHandle_t dropSeqMailbox = NULL;

// ── Servo objects ─────────────────────────────────────────────
Servo servo1;
Servo servo2;
Servo servo3;
Servo servoGrip;

// ── Current angles (updated as servos move) ───────────────────
int angle1    =  90;
int angle2    =  50;
int angle3    = 130;
int angleGrip = GRIP_ANGLE_OPEN;

// ── Manual continuous movement state ─────────────────────────
int current_active_joint = 0;
int current_direction    = 0;

// ── Drop sequence state machine ───────────────────────────────
enum class DropPhase {
    IDLE,
    GOTO_SLOT,    // move arm to storage slot, wait for arrival
    GRIP_CLOSE,   // close gripper on box
    GOTO_DROP,    // move arm to drop position, wait
    GRIP_OPEN,    // open gripper, release box
    RETURN_HOME   // move arm back to rest position
};

static DropPhase     dropPhase      = DropPhase::IDLE;
static unsigned long dropTimer      = 0;
static bool          dropFirstEntry = true;   // true when phase just started
static ArmPose       dropSlot       = {ARM_HOME_J1, ARM_HOME_J2, ARM_HOME_J3};
static ArmPose       dropTarget     = {ARM_HOME_J1, ARM_HOME_J2, ARM_HOME_J3};

// ── Helpers ───────────────────────────────────────────────────

// Write absolute angles to all three arm joints with clamping
static void writeArmPose(const ArmPose& pose) {
    int j1 = constrain(pose.j1, SERVO_MIN_ANGLE_J1, SERVO_MAX_ANGLE_J1);
    int j2 = constrain(pose.j2, SERVO_MIN_ANGLE_J2, SERVO_MAX_ANGLE_J2);
    int j3 = constrain(pose.j3, SERVO_MIN_ANGLE_J3, SERVO_MAX_ANGLE_J3);
    servo1.write(j1); angle1 = j1;
    servo2.write(j2); angle2 = j2;
    servo3.write(j3); angle3 = j3;
}

// Write home position
static void writeHome() {
    ArmPose home = {ARM_HOME_J1, ARM_HOME_J2, ARM_HOME_J3};
    writeArmPose(home);
    Serial.println("[SERVO] Returned to HOME position.");
}

// Continuous-sweep helper (manual control)
static void update_angle(int joint, int dir) {
    if (dir == 0) return;
    if (joint == 1) {
        angle1 = constrain(angle1 + dir, SERVO_MIN_ANGLE_J1, SERVO_MAX_ANGLE_J1);
        servo1.write(angle1);
        if (angle1 % 5 == 0) Serial.printf("J1 → %d°\n", angle1);
    } else if (joint == 2) {
        angle2 = constrain(angle2 + dir, SERVO_MIN_ANGLE_J2, SERVO_MAX_ANGLE_J2);
        servo2.write(angle2);
        if (angle2 % 5 == 0) Serial.printf("J2 → %d°\n", angle2);
    } else if (joint == 3) {
        angle3 = constrain(angle3 + dir, SERVO_MIN_ANGLE_J3, SERVO_MAX_ANGLE_J3);
        servo3.write(angle3);
        if (angle3 % 5 == 0) Serial.printf("J3 → %d°\n", angle3);
    } else if (joint == 5) {
        angleGrip = constrain(angleGrip + dir, SERVO_MIN_ANGLE_GRIPPER, SERVO_MAX_ANGLE_GRIPPER);
        servoGrip.write(angleGrip);
        if (angleGrip % 5 == 0) Serial.printf("GRIP → %d°\n", angleGrip);
    }
}

// ── Public: queue a drop sequence triggered by REACHED:<color> ─
void Servo_QueueDropSequence(const char* color) {
    DropSequenceCmd cmd;
    strncpy(cmd.color, color, sizeof(cmd.color) - 1);
    cmd.color[sizeof(cmd.color) - 1] = '\0';
    if (dropSeqMailbox != NULL) {
        if (xQueueSend(dropSeqMailbox, &cmd, 0) != pdPASS) {
            Serial.println("[SERVO] WARNING: dropSeqMailbox full, drop command dropped!");
        }
    }
}

// ── Init ──────────────────────────────────────────────────────
void Servo_Control_Init() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo3.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);

    servo1.attach(SERVO_1_PIN,    500, 2400);
    servo2.attach(SERVO_2_PIN,    500, 2400);
    servo3.attach(SERVO_3_PIN,    500, 2400);
    servoGrip.attach(SERVO_GRIP_PIN, 500, 2400);

    servo1.write(angle1);
    servo2.write(angle2);
    servo3.write(angle3);
    servoGrip.write(GRIP_ANGLE_OPEN);

    servoMailbox   = xQueueCreate(10, sizeof(ServoCommand));
    dropSeqMailbox = xQueueCreate(3,  sizeof(DropSequenceCmd));

    if (!servoMailbox || !dropSeqMailbox) {
        Serial.println("[SERVO] ERROR: Queue creation failed!");
    }
}

// ── FreeRTOS Task ─────────────────────────────────────────────
void Servo_Control_Task(void *arg) {
    ServoCommand      cmd;
    DropSequenceCmd   dsCmd;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t  xFreq  = pdMS_TO_TICKS(20); // 50 Hz

    while (1) {

        // ══════════════════════════════════════════════════════
        //  DROP SEQUENCE STATE MACHINE
        //  Priority: active drop sequence blocks manual control
        // ══════════════════════════════════════════════════════
        if (dropPhase != DropPhase::IDLE) {
            switch (dropPhase) {

            case DropPhase::GOTO_SLOT:
                if (dropFirstEntry) {
                    dropTimer      = millis();
                    dropFirstEntry = false;
                    writeArmPose(dropSlot);
                    Serial.printf("[DROP] → SLOT J1=%d J2=%d J3=%d\n",
                                  dropSlot.j1, dropSlot.j2, dropSlot.j3);
                }
                if (millis() - dropTimer >= DROP_TIME_GOTO_SLOT) {
                    dropPhase      = DropPhase::GRIP_CLOSE;
                    dropFirstEntry = true;
                }
                break;

            case DropPhase::GRIP_CLOSE:
                if (dropFirstEntry) {
                    dropTimer      = millis();
                    dropFirstEntry = false;
                    angleGrip      = GRIP_ANGLE_CLOSE;
                    servoGrip.write(angleGrip);
                    Serial.println("[DROP] → GRIP CLOSE");
                }
                if (millis() - dropTimer >= DROP_TIME_GRIP_CLOSE) {
                    dropPhase      = DropPhase::GOTO_DROP;
                    dropFirstEntry = true;
                }
                break;

            case DropPhase::GOTO_DROP:
                if (dropFirstEntry) {
                    dropTimer      = millis();
                    dropFirstEntry = false;
                    writeArmPose(dropTarget);
                    Serial.printf("[DROP] → DROP J1=%d J2=%d J3=%d\n",
                                  dropTarget.j1, dropTarget.j2, dropTarget.j3);
                }
                if (millis() - dropTimer >= DROP_TIME_GOTO_DROP) {
                    dropPhase      = DropPhase::GRIP_OPEN;
                    dropFirstEntry = true;
                }
                break;

            case DropPhase::GRIP_OPEN:
                if (dropFirstEntry) {
                    dropTimer      = millis();
                    dropFirstEntry = false;
                    angleGrip      = GRIP_ANGLE_OPEN;
                    servoGrip.write(angleGrip);
                    Serial.println("[DROP] → GRIP OPEN (box released)");
                }
                if (millis() - dropTimer >= DROP_TIME_GRIP_OPEN) {
                    dropPhase      = DropPhase::RETURN_HOME;
                    dropFirstEntry = true;
                }
                break;

            case DropPhase::RETURN_HOME:
                if (dropFirstEntry) {
                    dropTimer      = millis();
                    dropFirstEntry = false;
                    writeHome();
                }
                if (millis() - dropTimer >= DROP_TIME_RETURN_HOME) {
                    dropPhase      = DropPhase::IDLE;
                    dropFirstEntry = true;
                    Serial.println("[DROP] *** Sequence COMPLETE — IDLE ***");
                }
                break;

            default:
                break;
            }

            // While drop sequence is running, skip manual servo commands
            vTaskDelayUntil(&xLastWakeTime, xFreq);
            continue;
        }

        // ══════════════════════════════════════════════════════
        //  IDLE: check for new drop sequence trigger
        // ══════════════════════════════════════════════════════
        if (xQueueReceive(dropSeqMailbox, &dsCmd, 0) == pdPASS) {
            // Resolve colour → slot and drop angles
            if (strncmp(dsCmd.color, "red", 3) == 0) {
                dropSlot   = {SLOT_RED_J1,   SLOT_RED_J2,   SLOT_RED_J3};
                dropTarget = {DROP_RED_J1,   DROP_RED_J2,   DROP_RED_J3};
            } else if (strncmp(dsCmd.color, "blue", 4) == 0) {
                dropSlot   = {SLOT_BLUE_J1,  SLOT_BLUE_J2,  SLOT_BLUE_J3};
                dropTarget = {DROP_BLUE_J1,  DROP_BLUE_J2,  DROP_BLUE_J3};
            } else if (strncmp(dsCmd.color, "green", 5) == 0) {
                dropSlot   = {SLOT_GREEN_J1, SLOT_GREEN_J2, SLOT_GREEN_J3};
                dropTarget = {DROP_GREEN_J1, DROP_GREEN_J2, DROP_GREEN_J3};
            } else {
                Serial.printf("[DROP] Unknown colour '%s' — ignoring.\n", dsCmd.color);
                vTaskDelayUntil(&xLastWakeTime, xFreq);
                continue;
            }

            Serial.printf("[DROP] Starting sequence for colour: %s\n", dsCmd.color);
            dropPhase      = DropPhase::GOTO_SLOT;
            dropFirstEntry = true;

            vTaskDelayUntil(&xLastWakeTime, xFreq);
            continue;
        }

        // ══════════════════════════════════════════════════════
        //  IDLE: normal manual servo control from dashboard
        // ══════════════════════════════════════════════════════
        if (xQueueReceive(servoMailbox, &cmd, 0) == pdPASS) {

            if (cmd.joint_id == 6) {
                // Gripper instant position commands
                if (cmd.direction == 1) {
                    angleGrip = GRIP_ANGLE_OPEN;
                    servoGrip.write(angleGrip);
                    Serial.println("Gripper → OPEN");
                } else if (cmd.direction == -1) {
                    angleGrip = GRIP_ANGLE_CLOSE;
                    servoGrip.write(angleGrip);
                    Serial.println("Gripper → CLOSE");
                } else if (cmd.direction == 2) {
                    angleGrip = GRIP_ANGLE_PICK;
                    servoGrip.write(angleGrip);
                    Serial.println("Gripper → PICK");
                }
            } else {
                // Continuous sweep for joints 1–3 and 5
                if (cmd.joint_id == 0 && cmd.direction == 0) {
                    current_active_joint = 0;
                    current_direction    = 0;
                } else {
                    current_active_joint = cmd.joint_id;
                    current_direction    = cmd.direction;
                }
            }
        }

        // Apply continuous sweep movement this tick
        if (current_active_joint != 0 && current_direction != 0) {
            update_angle(current_active_joint, current_direction);
        }

        vTaskDelayUntil(&xLastWakeTime, xFreq);
    }
}
