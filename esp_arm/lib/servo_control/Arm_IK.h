#ifndef ARM_IK_H
#define ARM_IK_H

#include "Servo_Control.h"

// ═══════════════════════════════════════════════════════════════
//  ARM DIMENSIONS  ← FILL IN YOUR PHYSICAL MEASUREMENTS (mm)
// ═══════════════════════════════════════════════════════════════
#define ARM_L1 0.0f // Base height      (joint 1 to joint 2, mm) ← ADJUST
#define ARM_L2 0.0f // Upper arm length (joint 2 to joint 3, mm) ← ADJUST
#define ARM_L3 0.0f // Forearm length   (joint 3 to gripper tip, mm) ← ADJUST

// ═══════════════════════════════════════════════════════════════
//  PICK POSITIONS  — XYZ (mm) where gripper tip must reach
//  to pick up each coloured box from its storage slot.
//  X = forward/back from base centre
//  Y = left (+) / right (−) from base centre
//  Z = height from ground
//  ← ADJUST ALL VALUES TO MATCH YOUR PHYSICAL SETUP
// ═══════════════════════════════════════════════════════════════
#define PICK_RED_X 0.0f // ← ADJUST
#define PICK_RED_Y 0.0f // ← ADJUST
#define PICK_RED_Z 0.0f // ← ADJUST

#define PICK_BLUE_X 0.0f // ← ADJUST
#define PICK_BLUE_Y 0.0f // ← ADJUST
#define PICK_BLUE_Z 0.0f // ← ADJUST

#define PICK_GREEN_X 0.0f // ← ADJUST
#define PICK_GREEN_Y 0.0f // ← ADJUST
#define PICK_GREEN_Z 0.0f // ← ADJUST

// ═══════════════════════════════════════════════════════════════
//  DROP POSITIONS  — XYZ (mm) of each colour's drop zone opening
// ═══════════════════════════════════════════════════════════════
#define DROP_RED_X 0.0f // ← ADJUST
#define DROP_RED_Y 0.0f // ← ADJUST
#define DROP_RED_Z 0.0f // ← ADJUST

#define DROP_BLUE_X 0.0f // ← ADJUST
#define DROP_BLUE_Y 0.0f // ← ADJUST
#define DROP_BLUE_Z 0.0f // ← ADJUST

#define DROP_GREEN_X 0.0f // ← ADJUST
#define DROP_GREEN_Y 0.0f // ← ADJUST
#define DROP_GREEN_Z 0.0f // ← ADJUST

// ═══════════════════════════════════════════════════════════════
//  FOLDED / REST POSITION  — XYZ (mm)
//  Where the arm moves after releasing the box (compact pose).
// ═══════════════════════════════════════════════════════════════
#define FOLD_X 0.0f // ← ADJUST
#define FOLD_Y 0.0f // ← ADJUST
#define FOLD_Z 0.0f // ← ADJUST

// ── IK result status ─────────────────────────────────────────
enum class IKStatus
{
    OK,          // Solution found, angles within joint limits
    UNREACHABLE, // Target outside workspace
    LIMIT_CLAMP  // Solution found but one or more angles were clamped
};

// ── IK result bundle ─────────────────────────────────────────
struct IKResult
{
    ArmPose pose; // Solved joint angles (degrees), clamped to limits
    IKStatus status;
};

// ── Public API ────────────────────────────────────────────────

/**
 * Solve inverse kinematics for the 3-DOF arm.
 *
 * @param x  Target X (mm) — forward/back from base
 * @param y  Target Y (mm) — lateral from base
 * @param z  Target Z (mm) — height from ground
 * @returns  IKResult with joint angles and status flag
 *
 * Equations placeholders are in Arm_IK.cpp — replace with your
 * derived formulas before running on hardware.
 */
IKResult IK_Solve(float x, float y, float z);

/**
 * High-level colour placement functions.
 * Called by Servo_QueueDropSequence() when UART delivers REACHED:<color>.
 * Each function resolves pick/drop/fold XYZ → IK angles, then
 * pushes a DropSequenceCmd onto dropSeqMailbox.
 */
void Place_Red_Box();
void Place_Blue_Box();
void Place_Green_Box();

#endif // ARM_IK_H