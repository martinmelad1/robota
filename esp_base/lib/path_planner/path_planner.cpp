// ============================================================
//  PATH_PLANNER.CPP — Position-based autonomous navigation
//
//  Uses Odometry_GetPose() to read real (x, y, θ) and drives
//  the chassis with a proportional position controller until
//  the robot is within POSITION_TOLERANCE of each goal.
//
//  Coordinate frame (matches odometry):
//    +X = right,  +Y = forward,  0° = heading at reset
//
//  All goal coordinates are expressed in world-frame metres,
//  RELATIVE TO THE POSE AT THE MOMENT MODE_AUTO WAS PRESSED
//  (captured by PathPlanner_CaptureStartPose).  This means:
//    • If robot is already at the origin → targets match exactly.
//    • If robot drove somewhere first → targets offset correctly.
// ============================================================

#include "PathPlanner.h"
#include "Odometry.h"
#include <Arduino.h>
#include <math.h>

extern QueueHandle_t pidMailbox;

// ── Tuning constants ─────────────────────────────────────────
// Kp_pos   : proportional gain — increase if robot is sluggish, decrease if it oscillates
// MAX_SPEED : maximum normalised motor speed [0.0, 1.0] sent to PID_Compute
// MIN_SPEED : minimum speed to overcome friction (deadband)
// TOLERANCE : arrival radius in metres
static constexpr float Kp_pos            = 1.2f;
static constexpr float MAX_SPEED         = 0.55f;  // 55 % of full PWM — transit speed
static constexpr float MIN_SPEED         = 0.20f;  // kept for reference; no longer snapped near target
static constexpr float POSITION_TOLERANCE = 0.09f; // 9 cm — large enough to absorb full-speed glide
static constexpr float Kp_theta          = 1.5f;   // heading correction gain

// ── Goal table ───────────────────────────────────────────────
// Coordinates are RELATIVE to the autonomous start snapshot.
// X positive = right, X negative = left
// Y positive = forward, Y negative = backward
//
// Adjust to match your competition field layout.
struct Goal {
    float       x;      // target X offset from autonomous-start (metres)
    float       y;      // target Y offset from autonomous-start (metres)
    const char* label;  // debug name
};

// Competition map (all relative to autonomous start = (0,0)):
//   RED   (-1.0,  0.0) — pure X strafe left
//   GREEN (-1.0, -1.0) — diagonal X+Y to green station
//   BLUE  (-0.7, -0.7) — diagonal to blue station
static const Goal goals[] = {
    { -1.0f,  0.0f, "RED"   },
    { -1.0f, -1.0f, "GREEN" },
    { -0.7f, -0.7f, "BLUE"  },
};
static const int NUM_GOALS = sizeof(goals) / sizeof(goals[0]);

// Active goal set — normally all 3, but SetGoalByColor sets just 1.
static Goal  activeGoals[3];
static int   activeNumGoals = NUM_GOALS;

// ── Module state ─────────────────────────────────────────────
static int   currentGoal   = 0;
static bool  goalAnnounced = false;  // for one-shot serial print
static int   dbg_ctr       = 0;      // BUG5 FIX: module-scope so Reset() can clear it

// Pose snapshot taken when MODE_AUTO is pressed
static Pose  startPose = {0.0f, 0.0f, 0.0f};

// ── Helpers ──────────────────────────────────────────────────

// Apply deadband + max-clamp to a normalised speed value.
// LATENCY3 FIX: MIN_SPEED snap removed — it forced full blast even on close approach,
// causing overshoot oscillation. The motor deadband (8/255 in driveMotor) handles
// signals too small for the motor to act on.
static float _clampSpeed(float v) {
    if (fabsf(v) < 0.01f) return 0.0f;      // pure zero deadband
    if (fabsf(v) > MAX_SPEED)                // cap at maximum
        v = (v > 0.0f) ? MAX_SPEED : -MAX_SPEED;
    return v;
}

// Push a ChassisMotion to the PID mailbox
static void _sendMotion(float Vx, float Vy, float Wz) {
    // PID_Compute expects Vx/Vy/Wz in units of speedToTicks(m/s).
    // Here we pass normalised [-1, 1] fractions that PID_TaskCode
    // converts via speedToTicks(motion.speed).  Instead we bypass
    // the speed field and directly pass Vx/Vy as the "speed" channels
    // by using CMD_STOP with custom fields — actually the cleanest path
    // is to extend ChassisMotion with direct Vx/Vy/Wz.
    //
    // Since PID_Control is open-loop and scales by 255, we can pass
    // our desired [-1,1] values directly as Vx/Vy/Wz and they become
    // the mecanum input.  We do this by setting move_type = CMD_STOP
    // (so the switch in PIDTask falls through to defaults) and setting
    // custom fields — BUT the PIDTask switch resets Vx=Vy=Wz=0 on STOP.
    //
    // SOLUTION: Add CMD_DIRECT to WorldState.h and handle it in PID_TaskCode.
    // For now we use the closest approximation: if we need simultaneous
    // X+Y motion (diagonal) we pick CMD_FWD_L/R etc. at the right speed.
    // Better: use raw Vx/Vy/Wz path described in PID_Control.
    //
    // We expose this cleanly via a new DriveCommand::CMD_DIRECT that
    // carries Vx/Vy/Wz in the speed/omega fields extended below.
    // See WorldState.h additions.

    ChassisMotion cmd;
    cmd.move_type = DriveCommand::CMD_DIRECT;
    cmd.Vx        = Vx;
    cmd.Vy        = Vy;
    cmd.Wz        = Wz;
    xQueueOverwrite(pidMailbox, &cmd);
}

// ── Public API ───────────────────────────────────────────────

void PathPlanner_SetGoalByColor(const String& color) {
    // Find matching goal from master table and configure a single-goal run.
    // Called once before CaptureStartPose + Reset.
    String c = color;
    c.toLowerCase();
    activeNumGoals = 0;

    for (int i = 0; i < NUM_GOALS; i++) {
        String label = String(goals[i].label);
        label.toLowerCase();
        if (label.startsWith(c)) {
            activeGoals[0] = goals[i];
            activeNumGoals = 1;
            Serial.printf("[PathPlanner] Single-goal mode: %s  (%.2f, %.2f)\n",
                          goals[i].label, goals[i].x, goals[i].y);
            return;
        }
    }
    // Unknown color — fall back to all goals
    for (int i = 0; i < NUM_GOALS; i++) activeGoals[i] = goals[i];
    activeNumGoals = NUM_GOALS;
    Serial.printf("[PathPlanner] Unknown color '%s', using full goal table.\n", color.c_str());
}

void PathPlanner_CaptureStartPose() {
    startPose = Odometry_GetPose();
    Serial.printf("[PathPlanner] Autonomous start pose captured: X=%.3f Y=%.3f TH=%.1f°\n",
                  startPose.x, startPose.y, startPose.theta * 180.0f / M_PI);
}

void PathPlanner_Reset() {
    currentGoal   = 0;
    goalAnnounced = false;
    dbg_ctr       = 0;   // BUG5 FIX: reset debug counter for each new run

    // Initialise activeGoals with full table if SetGoalByColor was not called
    if (activeNumGoals == 0) {
        for (int i = 0; i < NUM_GOALS; i++) activeGoals[i] = goals[i];
        activeNumGoals = NUM_GOALS;
    }

    // Stop chassis before starting
    ChassisMotion stop;
    stop.move_type = DriveCommand::CMD_STOP;
    stop.Vx = stop.Vy = stop.Wz = 0.0f;
    xQueueOverwrite(pidMailbox, &stop);

    Serial.println("[PathPlanner] Reset. Position-based navigation starting.");
}

bool PathPlanner_Update() {
    if (PathPlanner_IsComplete()) return false;

    const Goal& g = activeGoals[currentGoal];

    // World-frame absolute target = start snapshot + relative offset
    float target_x = startPose.x + g.x;
    float target_y = startPose.y + g.y;

    // Current pose
    Pose p = Odometry_GetPose();

    // Position error in world frame
    float err_x = target_x - p.x;
    float err_y = target_y - p.y;
    float dist  = sqrtf(err_x * err_x + err_y * err_y);

    // One-shot announcement
    if (!goalAnnounced) {
        goalAnnounced = true;
        Serial.printf("[PathPlanner] Goal %d/%d '%s'  target=(%.3f, %.3f)\n",
                      currentGoal + 1, activeNumGoals, g.label, target_x, target_y);
    }

    // ── Arrival check ────────────────────────────────────────
    if (dist < POSITION_TOLERANCE) {
        // Stop chassis
        ChassisMotion stop;
        stop.move_type = DriveCommand::CMD_STOP;
        stop.Vx = stop.Vy = stop.Wz = 0.0f;
        xQueueOverwrite(pidMailbox, &stop);

        Serial.printf("[PathPlanner] Goal %d ARRIVED (dist=%.3f m)\n",
                      currentGoal + 1, dist);
        return true;
    }

    // ── Proportional position controller ─────────────────────
    // Commands are in world frame — the mecanum drive is holonomic so
    // we can directly command world-frame Vx/Vy without rotating to
    // robot frame (the robot's heading correction handles rotation).
    float cmd_x = _clampSpeed(Kp_pos * err_x);
    float cmd_y = _clampSpeed(Kp_pos * err_y);

    // Heading correction — keep θ = start heading (robot shouldn't rotate)
    float theta_err = startPose.theta - p.theta;
    // Wrap to (-π, π)
    while (theta_err >  float(M_PI)) theta_err -= 2.0f * float(M_PI);
    while (theta_err < -float(M_PI)) theta_err += 2.0f * float(M_PI);
    float cmd_w = _clampSpeed(Kp_theta * theta_err);
    // Limit rotation correction to ±25 % to avoid fighting translation
    if (cmd_w >  0.25f) cmd_w =  0.25f;
    if (cmd_w < -0.25f) cmd_w = -0.25f;

    _sendMotion(cmd_x, cmd_y, cmd_w);

    // Periodic progress print every ~1 s (50 brain ticks × 20 ms)
    if (++dbg_ctr >= 50) {
        dbg_ctr = 0;
        Serial.printf("[PathPlanner] → pos=(%.3f,%.3f) err=(%.3f,%.3f) dist=%.3f\n",
                      p.x, p.y, err_x, err_y, dist);
    }

    return false;
}

void PathPlanner_AdvanceGoal() {
    currentGoal++;
    goalAnnounced = false;

    if (PathPlanner_IsComplete()) {
        Serial.println("[PathPlanner] All goals complete.");
    } else {
        Serial.printf("[PathPlanner] Advancing to goal %d/%d.\n",
                      currentGoal + 1, NUM_GOALS);
    }
}

bool PathPlanner_IsComplete() {
    return currentGoal >= activeNumGoals;
}