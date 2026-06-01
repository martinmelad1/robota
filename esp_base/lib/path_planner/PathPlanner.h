#pragma once
#include <Arduino.h>

// ============================================================
//  PATH_PLANNER.H — Segment-based autonomous path
//
//  Fixed delivery sequence: RED → BLUE → GREEN
//
//  Integration with StateMachine:
//    1. Call PathPlanner_Start() when MODE_AUTO is triggered.
//    2. Call PathPlanner_Update() every Brain tick while NAVIGATING.
//       - Returns false → still moving, keep calling.
//       - Returns true  → a DROP segment has been reached.
//    3. Read PathPlanner_GetPendingAction() to know which colour.
//    4. Execute the drop (send REACHED to ARM, wait for completion).
//    5. Call PathPlanner_AcknowledgeAction() to advance to next segment.
//    6. Repeat until PathPlanner_IsComplete() returns true.
// ============================================================

// Action to execute when a navigation segment completes
enum class DropAction { NONE, DROP_RED, DROP_BLUE, DROP_GREEN };

// Start (or restart) from segment 0
void PathPlanner_Start();

// Call every Brain tick while in NAVIGATING state.
// Returns true only when a segment with a DROP action has been reached.
// Non-drop segments auto-advance internally without returning true.
bool PathPlanner_Update();

// Which drop to perform after Update() returns true
DropAction PathPlanner_GetPendingAction();

// Tell the planner the drop is finished; advances to the next segment.
void PathPlanner_AcknowledgeAction();

// True when all segments have been completed
bool PathPlanner_IsComplete();
