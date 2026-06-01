// ============================================================
//  PATH_PLANNER.CPP — Segment-based autonomous path
//
//  Field path (RED → BLUE → GREEN):
//
//    Seg  Type      Direction       Dist    Action
//    ─────────────────────────────────────────────────────
//    0    MOVE      Forward         0.60 m  —
//    1    ROTATE    CCW / Left      90°     —
//    2    MOVE      Forward         1.00 m  DROP RED
//    3    MOVE      Right (strafe)  1.40 m  DROP BLUE
//    4    ROTATE    CW  / Right     180°    —
//    5    MOVE      Forward         1.30 m  —
//    6    MOVE      Fwd+Right 45°   1.10 m  DROP GREEN
//    7    MOVE      Forward         0.01 m  — (stop marker)
//
//  Robot-local frame:
//    Vy = +1 → forward   Vx = +1 → strafe right   Wz = +1 → CCW/left
//
//  Distance tracking uses world-frame odometry displacement from the
//  start of each segment, so it is heading-independent.
//  Rotation tracking uses |Δθ| from segment start, wrapping-safe.
//
//  ⚠ Tuning:
//    Adjust POS_TOL upward if the robot coasts past target.
//    Adjust ROT_TOL upward if the robot keeps rotating past target angle.
//    MOVE_SPEED / ROT_SPEED only affect the sign (open-loop = full PWM).
// ============================================================

#include "PathPlanner.h"
#include "Odometry.h"
#include "WorldState.h"
#include <Arduino.h>
#include <math.h>

extern QueueHandle_t pidMailbox;

// ── Tuning constants ─────────────────────────────────────────
static constexpr float MOVE_SPEED = 0.50f;   // passed to CMD_DIRECT (open-loop → direction only)
static constexpr float ROT_SPEED  = 0.35f;   // rotation speed (open-loop)
static constexpr float POS_TOL    = 0.06f;   // arrival radius (m) — increase if robot overshoots
static constexpr float ROT_TOL    = 0.09f;   // heading tolerance (rad) ≈ 5° — increase if overshoots

// ── Segment type ─────────────────────────────────────────────
enum class SegType { MOVE, ROTATE };

struct PathSeg {
    SegType     type;
    float       Vx, Vy;    // MOVE: robot-frame unit direction (one of them is 0 or 0.707)
    float       Wz;        // ROTATE: +1 = CCW (left), -1 = CW (right)
    float       dist;      // MOVE: metres | ROTATE: radians (positive)
    DropAction  action;    // action to signal when this segment completes
    const char* label;     // debug name
};

static const float RAD90  = float(M_PI) / 2.0f;
static const float RAD180 = float(M_PI);

// ── HARDCODED FIELD PATH ──────────────────────────────────────
// All distances in metres. Adjust to match actual field measurements.
// Diagonal direction: Vx=Vy=0.707 gives true 45° FWD+RIGHT.
// ROT_CW_180 direction: -1 = clockwise. Flip to +1 if robot turns wrong way.
static const PathSeg PATH[] = {
    //  type               Vx      Vy       Wz    dist    action               label
    { SegType::MOVE,   0.0f,  1.0f,   0.0f, 0.60f, DropAction::NONE,      "INIT_FWD"   },
    { SegType::ROTATE, 0.0f,  0.0f,  +1.0f, RAD90, DropAction::NONE,      "ROT_L_90"   },
    { SegType::MOVE,   0.0f,  1.0f,   0.0f, 1.00f, DropAction::DROP_RED,  "TO_RED"     },
    { SegType::MOVE,  +1.0f,  0.0f,   0.0f, 1.40f, DropAction::DROP_BLUE, "TO_BLUE"    },
    { SegType::ROTATE, 0.0f,  0.0f,  -1.0f, RAD180,DropAction::NONE,      "ROT_CW_180" },
    { SegType::MOVE,   0.0f,  1.0f,   0.0f, 1.30f, DropAction::NONE,      "TO_GREEN_A" },
    { SegType::MOVE,  +0.707f,0.707f, 0.0f, 1.10f, DropAction::DROP_GREEN,"TO_GREEN_B" },
    { SegType::MOVE,   0.0f,  1.0f,   0.0f, 0.01f, DropAction::NONE,      "STOP_PT"    },
};
static const int PATH_LEN = (int)(sizeof(PATH) / sizeof(PATH[0]));

// ── Module state ──────────────────────────────────────────────
static int        g_seg           = 0;
static bool       g_segStarted    = false;
static float      g_startX        = 0.0f;
static float      g_startY        = 0.0f;
static float      g_startTheta    = 0.0f;
static DropAction g_pendingAction = DropAction::NONE;
static bool       g_waitingAck    = false;
static int        g_dbgCtr        = 0;

// ── Private helpers ───────────────────────────────────────────
static void _stop() {
    ChassisMotion s;
    s.move_type = DriveCommand::CMD_STOP;
    s.Vx = s.Vy = s.Wz = 0.0f;
    xQueueOverwrite(pidMailbox, &s);
}

static void _drive(float Vx, float Vy, float Wz) {
    ChassisMotion c;
    c.move_type = DriveCommand::CMD_DIRECT;
    c.Vx = Vx; c.Vy = Vy; c.Wz = Wz;
    xQueueOverwrite(pidMailbox, &c);
}

// Wrap angle to (-π, +π]
static float _wrap(float a) {
    while (a >  float(M_PI)) a -= 2.0f * float(M_PI);
    while (a < -float(M_PI)) a += 2.0f * float(M_PI);
    return a;
}

// ── Public API ────────────────────────────────────────────────

void PathPlanner_Start() {
    g_seg           = 0;
    g_segStarted    = false;
    g_pendingAction = DropAction::NONE;
    g_waitingAck    = false;
    g_dbgCtr        = 0;
    _stop();
    Serial.printf("[PATH] *** Autonomous path started — %d segments ***\n", PATH_LEN);
}

DropAction PathPlanner_GetPendingAction() {
    return g_pendingAction;
}

void PathPlanner_AcknowledgeAction() {
    g_pendingAction = DropAction::NONE;
    g_waitingAck    = false;
    g_seg++;
    g_segStarted    = false;
    g_dbgCtr        = 0;
    if (PathPlanner_IsComplete()) {
        Serial.println("[PATH] All segments complete. Path finished.");
    } else {
        Serial.printf("[PATH] ACK received — advancing to seg %d/%d\n",
                      g_seg + 1, PATH_LEN);
    }
}

bool PathPlanner_IsComplete() {
    return g_seg >= PATH_LEN;
}

bool PathPlanner_Update() {
    // Hold position and return true until the state machine ACKs the drop
    if (g_waitingAck) return true;

    if (PathPlanner_IsComplete()) return false;

    const PathSeg& seg = PATH[g_seg];

    // Snapshot the start pose once when a new segment begins
    if (!g_segStarted) {
        Pose p       = Odometry_GetPose();
        g_startX     = p.x;
        g_startY     = p.y;
        g_startTheta = p.theta;
        g_segStarted = true;
        Serial.printf("[PATH] Seg %d/%d '%s' — start=(%.3f, %.3f, %.1f°)\n",
                      g_seg + 1, PATH_LEN, seg.label,
                      p.x, p.y, p.theta * 180.0f / float(M_PI));
    }

    Pose p = Odometry_GetPose();

    // ── MOVE segment ─────────────────────────────────────────
    if (seg.type == SegType::MOVE) {
        float dx   = p.x - g_startX;
        float dy   = p.y - g_startY;
        float dist = sqrtf(dx * dx + dy * dy);

        // Periodic progress log (~every 1 s at 50 Hz brain tick)
        if (++g_dbgCtr >= 50) {
            g_dbgCtr = 0;
            Serial.printf("[PATH]   dist=%.3f/%.3f m\n", dist, seg.dist);
        }

        if (dist >= seg.dist - POS_TOL) {
            _stop();
            Serial.printf("[PATH] Seg %d '%s' ARRIVED (dist=%.3f m)\n",
                          g_seg + 1, seg.label, dist);

            if (seg.action != DropAction::NONE) {
                // Signal state machine to execute the drop, then wait for ACK
                g_pendingAction = seg.action;
                g_waitingAck    = true;
                return true;
            }
            // No drop action — auto-advance to the next segment
            g_seg++;
            g_segStarted = false;
            g_dbgCtr     = 0;
            return false;
        }

        // Keep driving (open-loop: no heading correction — Wz=0)
        _drive(seg.Vx * MOVE_SPEED, seg.Vy * MOVE_SPEED, 0.0f);
        return false;
    }

    // ── ROTATE segment ───────────────────────────────────────
    if (seg.type == SegType::ROTATE) {
        // Use |Δθ| so the check works correctly at 90° AND 180° without
        // wrapping ambiguity. The commanded Wz sign sets direction.
        float delta = fabsf(_wrap(p.theta - g_startTheta));

        if (++g_dbgCtr >= 50) {
            g_dbgCtr = 0;
            Serial.printf("[PATH]   rot=%.1f°/%.1f°\n",
                          delta * 180.0f / float(M_PI),
                          seg.dist * 180.0f / float(M_PI));
        }

        if (delta >= seg.dist - ROT_TOL) {
            _stop();
            Serial.printf("[PATH] Seg %d '%s' ROTATE DONE (Δθ=%.1f°)\n",
                          g_seg + 1, seg.label, delta * 180.0f / float(M_PI));
            g_seg++;
            g_segStarted = false;
            g_dbgCtr     = 0;
            return false;
        }

        _drive(0.0f, 0.0f, seg.Wz * ROT_SPEED);
        return false;
    }

    return false;
}