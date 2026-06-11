// PATH_PLANNER.CPP — Segment-based path, odometry stop condition
// See path_planner_notes.md for full path table and tuning guide.

#include "PathPlanner.h"
#include "Odometry.h"
#include "WorldState.h"
#include <Arduino.h>
#include <math.h>

extern QueueHandle_t pidMailbox;

// Tuning
static constexpr float MOVE_SPEED = 0.50f;  // speed fraction sent to PID
static constexpr float ROT_SPEED  = 0.35f;  // rotation fraction sent to PID
static constexpr float POS_TOL    = 0.03f;  // arrival buffer (metres)
static constexpr float ROT_TOL    = 0.06f;  // arrival buffer (radians)

enum class SegType { MOVE, ROTATE };

struct Seg {
    SegType    type;
    float      Vx, Vy;     // unit direction (MOVE)
    float      Wz;         // +1=CCW, -1=CW (ROTATE)
    float      dist;       // metres (MOVE) or radians (ROTATE)
    DropAction action;
    const char* label;
};

static const float R90  = float(M_PI) / 2.0f;
static const float R180 = float(M_PI);
static const float D    = 0.7071f;  // 45° diagonal unit component

static const Seg PATH[] = {
//  type              Vx     Vy      Wz    dist    action               label
  { SegType::MOVE,   0.0f,  1.0f,  0.0f, 0.60f, DropAction::NONE,      "FWD_0.6"    },
  { SegType::ROTATE, 0.0f,  0.0f, +1.0f, R90,   DropAction::NONE,      "ROT_CCW_90" },
  { SegType::MOVE,   0.0f,  1.0f,  0.0f, 1.35f, DropAction::DROP_RED,  "TO_RED"     },
  { SegType::MOVE,   0.0f, -1.0f,  0.0f, 0.25f, DropAction::NONE,      "BACK_0.25"  },
  { SegType::MOVE,  +1.0f,  0.0f,  0.0f, 1.70f, DropAction::DROP_BLUE, "TO_BLUE"    },
  { SegType::MOVE,  -1.0f,  0.0f,  0.0f, 0.20f, DropAction::NONE,      "LEFT_0.20"  },
  { SegType::ROTATE, 0.0f,  0.0f, -1.0f, R180,  DropAction::NONE,      "ROT_CW_180" },
  { SegType::MOVE,   0.0f,  1.0f,  0.0f, 0.90f, DropAction::NONE,      "FWD_0.9"    },
  { SegType::MOVE,  -D,     D,     0.0f, 1.42f, DropAction::DROP_GREEN, "TO_GREEN"   },
};
static const int PATH_LEN = (int)(sizeof(PATH) / sizeof(PATH[0]));

// Module state
static int        g_seg        = 0;
static bool       g_segStarted = false;
static float      g_startX     = 0.0f;
static float      g_startY     = 0.0f;
static float      g_startTheta = 0.0f;
static DropAction g_pending    = DropAction::NONE;
static bool       g_waitingAck = false;

// ── Helpers ───────────────────────────────────────────────────

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
    Odometry_ResetPose();
    g_seg = 0; g_segStarted = false;
    g_pending = DropAction::NONE; g_waitingAck = false;
    _stop();
    Serial.printf("[PATH] Started — %d segments. Pose zeroed.\n", PATH_LEN);
}

DropAction PathPlanner_GetPendingAction() { return g_pending; }

bool PathPlanner_IsComplete() { return g_seg >= PATH_LEN; }

void PathPlanner_AcknowledgeAction() {
    g_pending = DropAction::NONE;
    g_waitingAck = false;
    g_seg++;
    g_segStarted = false;
    if (PathPlanner_IsComplete())
        Serial.println("[PATH] All segments done.");
    else
        Serial.printf("[PATH] ACK — next seg %d/%d\n", g_seg + 1, PATH_LEN);
}

bool PathPlanner_Update() {
    if (g_waitingAck) return true;
    if (PathPlanner_IsComplete()) { _stop(); return false; }

    const Seg& seg = PATH[g_seg];

    // Snapshot pose at segment start
    if (!g_segStarted) {
        Pose p       = Odometry_GetPose();
        g_startX     = p.x;
        g_startY     = p.y;
        g_startTheta = p.theta;
        g_segStarted = true;
        Serial.printf("[PATH] Seg %d/%d '%s' start=(%.3f, %.3f, %.1f°)\n",
            g_seg + 1, PATH_LEN, seg.label,
            p.x, p.y, p.theta * 180.0f / float(M_PI));
    }

    Pose p = Odometry_GetPose();

    if (seg.type == SegType::MOVE) {
        float dx   = p.x - g_startX;
        float dy   = p.y - g_startY;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist >= seg.dist - POS_TOL) {
            _stop();
            Serial.printf("[PATH] Seg %d '%s' done (%.3f m)\n", g_seg + 1, seg.label, dist);
            if (seg.action != DropAction::NONE) {
                g_pending    = seg.action;
                g_waitingAck = true;
                return true;
            }
            g_seg++; g_segStarted = false;
            return false;
        }
        _drive(seg.Vx * MOVE_SPEED, seg.Vy * MOVE_SPEED, 0.0f);
        return false;
    }

    if (seg.type == SegType::ROTATE) {
        float delta = fabsf(_wrap(p.theta - g_startTheta));
        if (delta >= seg.dist - ROT_TOL) {
            _stop();
            Serial.printf("[PATH] Seg %d '%s' done (%.1f°)\n",
                g_seg + 1, seg.label, delta * 180.0f / float(M_PI));
            g_seg++; g_segStarted = false;
            return false;
        }
        _drive(0.0f, 0.0f, seg.Wz * ROT_SPEED);
        return false;
    }

    return false;
}