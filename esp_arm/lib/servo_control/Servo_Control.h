#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H
#include <Arduino.h>
#include <ESP32Servo.h>

// ── Arm pose (absolute joint angles) ─────────────────────────
struct ArmPose
{
    int j1, j2, j3;
};

// ── IK pose cache ─────────────────────────────────────────────
// Populated by Arm_IK.cpp before a DropSequenceCmd is queued.
// The drop state machine reads this instead of hardcoded angles.
struct IKPoseCache
{
    ArmPose pick; // Gripper-close position (over storage slot)
    ArmPose drop; // Gripper-open  position (over drop zone)
    ArmPose fold; // Compact rest  position (after release)
    bool valid = false;
};
extern IKPoseCache gIKCache;
// ── Servo pin assignments ─────────────────────────────────────
// Avoid pins 12,13,14,15 (conflict with JTAG)
#define SERVO_1_PIN 33
#define SERVO_2_PIN 25
#define SERVO_3_PIN 26
#define SERVO_GRIP_PIN 27

// ── Physical joint limits (degrees) ──────────────────────────
// Adjust if servo hits a hard stop and jitters
#define SERVO_MIN_ANGLE_J1 0
#define SERVO_MAX_ANGLE_J1 180
#define SERVO_MIN_ANGLE_J2 0
#define SERVO_MAX_ANGLE_J2 50
#define SERVO_MIN_ANGLE_J3 0
#define SERVO_MAX_ANGLE_J3 130
#define SERVO_MIN_ANGLE_GRIPPER 0
#define SERVO_MAX_ANGLE_GRIPPER 150

// ── Gripper fixed positions (degrees) ────────────────────────
#define GRIP_ANGLE_OPEN 60
#define GRIP_ANGLE_CLOSE 150 // clamped to SERVO_MAX_ANGLE_GRIPPER
#define GRIP_ANGLE_PICK 90

// ── Home / rest position ──────────────────────────────────────
#define ARM_HOME_J1 90
#define ARM_HOME_J2 50
#define ARM_HOME_J3 130

// ═══════════════════════════════════════════════════════════════
//  HARDCODED ARM ANGLES  ← ADJUST THESE TO YOUR PHYSICAL ROBOT
//
//  SLOT angles: where each colour box is stored on the robot.
//    J1 = base rotation to face the slot
//    J2 = shoulder (how far down/in to reach)
//    J3 = elbow
//
//  DROP angles: arm position over the drop zone opening.
//    Typically J2/J3 extend the arm outward / lower than slot.
//
//  Adjust one joint at a time in manual mode using the dashboard,
//  note the angles printed in Serial Monitor, then paste them here.
// ═══════════════════════════════════════════════════════════════

// ── Drop sequence timing (milliseconds) ──────────────────────
// Increase if the arm doesn't reach position before the next step.
#define DROP_TIME_GOTO_SLOT 1800   // travel time to pick position
#define DROP_TIME_GRIP_CLOSE 700   // time to close gripper on box
#define DROP_TIME_GOTO_DROP 1800   // travel time to drop position
#define DROP_TIME_GRIP_OPEN 600    // time gripper is open before folding
#define DROP_TIME_GOTO_FOLD 1200   // travel time to folded rest pose
#define DROP_TIME_RETURN_HOME 1800 // travel time back to home pose
// Total ≈ 7900 ms

// ── Drop sequence trigger (UART slave → Servo task) ──────────
struct DropSequenceCmd
{
    char color[8]; // "red", "blue", or "green"
};
extern QueueHandle_t dropSeqMailbox;

// Queue a full pick-from-slot + drop + return-home sequence.
// Called by UART_Slave_Arm when BASE sends REACHED:<color>.
void Servo_QueueDropSequence(const char *color);

// ── Standard servo command (dashboard / manual control) ──────
struct ServoCommand
{
    int joint_id;  // 1–3 for arm joints, 5 for gripper sweep, 6 for gripper instant
    int direction; // 1 = increase angle, -1 = decrease, 0 = stop, 2 = PICK preset
};

// ── Queue handles ─────────────────────────────────────────────
extern QueueHandle_t servoMailbox;

// ── Current joint angles (updated by servo task) ─────────────
extern int angle1;
extern int angle2;
extern int angle3;

// ── Public API ────────────────────────────────────────────────
void Servo_Control_Init();
void Servo_Control_Task(void *arg);

#endif // SERVO_CONTROL_H
