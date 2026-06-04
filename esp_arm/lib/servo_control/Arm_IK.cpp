#include "Arm_IK.h"
#include <math.h>
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
//  IK_Solve  — 3-DOF geometric inverse kinematics
//
//  COORDINATE CONVENTION (match your robot):
//    J1 (base yaw)   — rotates around vertical Z axis
//    J2 (shoulder)   — rotates in the vertical plane
//    J3 (elbow)      — rotates in the vertical plane
//
//  ALL ANGLE PLACEHOLDERS ARE MARKED ← IK EQUATION
//  Replace each placeholder with your derived formula.
//  Link lengths ARM_L1 / ARM_L2 / ARM_L3 are defined in Arm_IK.h.
// ═══════════════════════════════════════════════════════════════

IKResult IK_Solve(float x, float y, float z)
{
    IKResult result;
    result.status = IKStatus::OK;

    // ── Pre-computed intermediates ────────────────────────────
    // Horizontal distance from base axis to target (XY plane)
    float r = sqrtf(x * x + y * y); // horizontal reach

    // Vertical distance from shoulder pivot to target
    float dz = z - ARM_L1; // height above shoulder

    // 2D reach from shoulder to wrist (planar distance)
    float D = sqrtf(r * r + dz * dz); // planar reach

    // ── Reachability check ────────────────────────────────────
    float maxReach = ARM_L2 + ARM_L3;
    float minReach = fabsf(ARM_L2 - ARM_L3);
    if (D > maxReach || D < minReach)
    {
        Serial.printf("[IK] UNREACHABLE  r=%.1f dz=%.1f D=%.1f (max=%.1f)\n",
                      r, dz, D, maxReach);
        result.status = IKStatus::UNREACHABLE;
        // Return home pose so the arm does not move to garbage angles
        result.pose = {ARM_HOME_J1, ARM_HOME_J2, ARM_HOME_J3};
        return result;
    }

    // ─────────────────────────────────────────────────────────
    //  ↓↓  PASTE YOUR DERIVED IK EQUATIONS BELOW  ↓↓
    //
    //  Available variables:
    //    x, y, z      — target position (mm)
    //    r            — sqrt(x²+y²)
    //    dz           — z − ARM_L1
    //    D            — sqrt(r²+dz²)
    //    ARM_L1/L2/L3 — link lengths (mm)
    //
    //  Output must be in DEGREES and assigned to theta1/2/3.
    // ─────────────────────────────────────────────────────────

    // Joint 1 — base yaw (rotation around Z)
    float theta1_deg = 0.0f; // ← IK EQUATION  e.g. atan2f(y, x) * RAD_TO_DEG

    // Joint 2 — shoulder pitch
    float theta2_deg = 0.0f; // ← IK EQUATION

    // Joint 3 — elbow pitch
    float theta3_deg = 0.0f; // ← IK EQUATION

    // ─────────────────────────────────────────────────────────

    // ── Clamp to physical joint limits ───────────────────────
    int j1 = (int)roundf(theta1_deg);
    int j2 = (int)roundf(theta2_deg);
    int j3 = (int)roundf(theta3_deg);

    bool clamped = false;
    if (j1 < SERVO_MIN_ANGLE_J1 || j1 > SERVO_MAX_ANGLE_J1)
    {
        clamped = true;
    }
    if (j2 < SERVO_MIN_ANGLE_J2 || j2 > SERVO_MAX_ANGLE_J2)
    {
        clamped = true;
    }
    if (j3 < SERVO_MIN_ANGLE_J3 || j3 > SERVO_MAX_ANGLE_J3)
    {
        clamped = true;
    }

    j1 = constrain(j1, SERVO_MIN_ANGLE_J1, SERVO_MAX_ANGLE_J1);
    j2 = constrain(j2, SERVO_MIN_ANGLE_J2, SERVO_MAX_ANGLE_J2);
    j3 = constrain(j3, SERVO_MIN_ANGLE_J3, SERVO_MAX_ANGLE_J3);

    if (clamped)
    {
        result.status = IKStatus::LIMIT_CLAMP;
        Serial.printf("[IK] CLAMPED  J1=%d J2=%d J3=%d\n", j1, j2, j3);
    }

    result.pose = {j1, j2, j3};

    Serial.printf("[IK] Solved  x=%.1f y=%.1f z=%.1f  →  J1=%d J2=%d J3=%d\n",
                  x, y, z, j1, j2, j3);
    return result;
}

// ─────────────────────────────────────────────────────────────
//  Internal helper: build a full DropSequenceCmd from IK results
//  and push it onto the dropSeqMailbox.
//
//  pickPose  — arm pose at the storage slot (gripper must close here)
//  dropPose  — arm pose over the drop zone  (gripper opens here)
//  foldPose  — compact rest pose after release
// ─────────────────────────────────────────────────────────────
static void queueIKSequence(const char *color,
                            const ArmPose &pickPose,
                            const ArmPose &dropPose,
                            const ArmPose &foldPose)
{
    // Servo_Control.cpp reads color only; the resolved poses are
    // stored in the IKPoseCache below so the state machine can pick them up.
    // (See IKPoseCache and the integration note in Servo_Control.cpp)

    extern IKPoseCache gIKCache; // defined in Servo_Control.cpp
    gIKCache.pick = pickPose;
    gIKCache.drop = dropPose;
    gIKCache.fold = foldPose;
    gIKCache.valid = true;

    DropSequenceCmd cmd;
    strncpy(cmd.color, color, sizeof(cmd.color) - 1);
    cmd.color[sizeof(cmd.color) - 1] = '\0';

    if (dropSeqMailbox != NULL)
    {
        if (xQueueSend(dropSeqMailbox, &cmd, 0) != pdPASS)
        {
            Serial.println("[IK] WARNING: dropSeqMailbox full — sequence dropped!");
            gIKCache.valid = false;
        }
    }
}

// ── Place functions ───────────────────────────────────────────

void Place_Red_Box()
{
    Serial.println("[IK] Resolving RED sequence via IK...");

    IKResult pick = IK_Solve(PICK_RED_X, PICK_RED_Y, PICK_RED_Z);
    IKResult drop = IK_Solve(DROP_RED_X, DROP_RED_Y, DROP_RED_Z);
    IKResult fold = IK_Solve(FOLD_X, FOLD_Y, FOLD_Z);

    if (pick.status == IKStatus::UNREACHABLE ||
        drop.status == IKStatus::UNREACHABLE ||
        fold.status == IKStatus::UNREACHABLE)
    {
        Serial.println("[IK] RED sequence aborted — unreachable target.");
        return;
    }

    queueIKSequence("red", pick.pose, drop.pose, fold.pose);
}

void Place_Blue_Box()
{
    Serial.println("[IK] Resolving BLUE sequence via IK...");

    IKResult pick = IK_Solve(PICK_BLUE_X, PICK_BLUE_Y, PICK_BLUE_Z);
    IKResult drop = IK_Solve(DROP_BLUE_X, DROP_BLUE_Y, DROP_BLUE_Z);
    IKResult fold = IK_Solve(FOLD_X, FOLD_Y, FOLD_Z);

    if (pick.status == IKStatus::UNREACHABLE ||
        drop.status == IKStatus::UNREACHABLE ||
        fold.status == IKStatus::UNREACHABLE)
    {
        Serial.println("[IK] BLUE sequence aborted — unreachable target.");
        return;
    }

    queueIKSequence("blue", pick.pose, drop.pose, fold.pose);
}

void Place_Green_Box()
{
    Serial.println("[IK] Resolving GREEN sequence via IK...");

    IKResult pick = IK_Solve(PICK_GREEN_X, PICK_GREEN_Y, PICK_GREEN_Z);
    IKResult drop = IK_Solve(DROP_GREEN_X, DROP_GREEN_Y, DROP_GREEN_Z);
    IKResult fold = IK_Solve(FOLD_X, FOLD_Y, FOLD_Z);

    if (pick.status == IKStatus::UNREACHABLE ||
        drop.status == IKStatus::UNREACHABLE ||
        fold.status == IKStatus::UNREACHABLE)
    {
        Serial.println("[IK] GREEN sequence aborted — unreachable target.");
        return;
    }

    queueIKSequence("green", pick.pose, drop.pose, fold.pose);
}