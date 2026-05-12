#pragma once
#include <Arduino.h>
#include "WorldState.h"

// ============================================================
//  PATH PLANNER — Position-based autonomous navigation
//
//  Competition map coordinates (world-frame, relative to (0,0) start):
//    RED   station: (-1.0,  0.0)  — pure X strafe left
//    GREEN station: (-1.0, -1.0)  — X + Y diagonal
//    BLUE  station: (-0.7, -0.7)  — diagonal
//
//  Typical call sequence per autonomous run:
//    PathPlanner_SetGoalByColor("red")   ← sets single goal from cube color
//    PathPlanner_CaptureStartPose()      ← locks pose at moment of MODE_AUTO
//    PathPlanner_Reset()                 ← resets goal index, stops chassis
//    then in NAVIGATING_TO_DROP loop:
//      PathPlanner_Update() == true  →  arrived
//        ARM_SendReached(color)
//        WAIT_FOR_ARM_DROP (5 s)
//          PathPlanner_AdvanceGoal()
//          PathPlanner_IsComplete() → MANUAL_MODE
// ============================================================

// Set a single goal from the cube color. Call BEFORE CaptureStartPose + Reset.
// Accepts "red", "green", "blue" (case-insensitive).
void PathPlanner_SetGoalByColor(const String& color);

// Capture current odometry pose as autonomous start reference.
void PathPlanner_CaptureStartPose();

// Reset goal index to 0 and stop chassis.
void PathPlanner_Reset();

// Drive toward current goal. Returns TRUE when within POSITION_TOLERANCE.
bool PathPlanner_Update();

// Advance to next goal (called from WAIT_FOR_ARM_DROP after arm timer).
void PathPlanner_AdvanceGoal();

// Returns true when all goals visited.
bool PathPlanner_IsComplete();
