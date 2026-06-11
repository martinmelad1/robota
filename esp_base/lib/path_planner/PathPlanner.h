
#pragma once
#include <Arduino.h>

// ============================================================
//  PATH_PLANNER.H  — Odometry-based autonomous path (v2)
//
//  Uses ENCODER-ONLY odometry (no IMU) from Odometry.cpp.
//
//  Field path — 12-step state machine:
//    STEP_1         : Drive forward to first waypoint
//    STEP_2_ROTATE  : Rotate 90° CCW / left
//    STEP_3         : Drive to RED drop position        → DROP_RED
//    STEP_4_PLACE1  : Signal state machine; wait for ACK
//    STEP_5         : Back up slightly
//    STEP_6         : Strafe right to BLUE drop position → DROP_BLUE
//    STEP_7_PLACE2  : Signal state machine; wait for ACK
//    STEP_8         : Back up slightly (left strafe)
//    STEP_9_ROTATE  : Rotate 180°
//    STEP_10        : Drive forward
//    STEP_11_DIAGONAL: Drive diagonally to GREEN station → DROP_GREEN
//    STEP_12_PLACE3 : Signal state machine; wait for ACK
//    PATH_DONE      : Stop
//
//  Position control uses a PD controller on world-frame
//  odometry error (X, Y).  Heading is kept near 0 using
//  a P controller on pose_theta (encoder-derived).
//
//  Integration with StateMachine:
//    1. Call PathPlanner_Start()  when MODE_AUTO triggered.
//    2. Call PathPlanner_Update() every Brain tick (50 Hz).
//         Returns false → still moving, keep calling.
//         Returns true  → a DROP point was reached.
//    3. Read PathPlanner_GetPendingAction() for the colour.
//    4. Execute drop (StateMachine sends REACHED:<colour>).
//    5. Call PathPlanner_AcknowledgeAction() when arm done.
//    6. Repeat until PathPlanner_IsComplete() returns true.
// ============================================================

// Which drop to perform when Update() returns true
enum class DropAction { NONE, DROP_RED, DROP_BLUE, DROP_GREEN };

// ── Public API ────────────────────────────────────────────────

// Start (or restart) the path from the beginning.
// Resets odometry pose to (0,0,0) at the current position.
void PathPlanner_Start();

// Call every Brain tick while in NAVIGATING state.
// Returns true only when a DROP waypoint has been reached.
// Non-drop waypoints auto-advance without returning true.
bool PathPlanner_Update();

// Which drop action is pending (valid only after Update() == true)
DropAction PathPlanner_GetPendingAction();

// Tell the planner the arm has finished; advance to next step.
void PathPlanner_AcknowledgeAction();

// Returns true when every step has been completed.
bool PathPlanner_IsComplete();
