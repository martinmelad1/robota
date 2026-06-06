#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;
extern QueueHandle_t armMailbox;

// Physical Arena Tracking
struct FieldBox
{
    String expectedQR;
    String targetColor;
    float expectedX;
    float expectedY;
    bool isPickedUp;
};
struct StorageSlot
{
    String assignedColor;
    bool isFull;
    float slotX;
    float slotY;
    float slotZ;
};
struct DropoffZone
{
    String zoneColor;
    float dropX;
    float dropY;
    bool isFinished;
};

const int TOTAL_TARGETS = 3;
extern FieldBox field_boxes[TOTAL_TARGETS];
extern StorageSlot storage_unit[TOTAL_TARGETS];
extern DropoffZone dropoff_zones[TOTAL_TARGETS];

enum class GUITrigger
{
    NONE,
    TRIGGER_MANUAL,
    TRIGGER_PICK,
    TRIGGER_AUTO
};

enum class DriveCommand
{
    CMD_STOP,
    CMD_FWD,
    CMD_BWD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_FWD_L,
    CMD_FWD_R,
    CMD_BWD_L,
    CMD_BWD_R,
    CMD_ROT_L,
    CMD_ROT_R,
    CMD_DIRECT, // Raw Vx/Vy/Wz from PathPlanner — bypasses direction switch
    CMD_DIST    // Distance-based move: drive Vx/Vy/Wz until tickTarget reached
};

struct ChassisMotion
{
    DriveCommand move_type;
    float speed = 0.5f; // m/s — used by direction commands
    float omega = 1.5f; // rad/s — used by rotation commands
    // Direct holonomic / PathPlanner
    float Vx = 0.0f;
    float Vy = 0.0f;
    float Wz = 0.0f;
    // Distance-based move (CMD_DIST)
    long tickTarget = 0; // encoder ticks to travel (positive only)
};

enum class GripperStatus
{
    CLOSED,
    OPEN,
    PICK
};

enum class ArmDir : int8_t
{
    STOP = 0,
    UP = 1,
    DOWN = -1
};

struct ArmMotion
{
    int joint_id;
    ArmDir direction;
};

// 64 bytes fits PID_TUNE:vel:FL:2.500:2.000:0.000 + MOVE_DIST commands
struct StringMessage
{
    char data[64];
};