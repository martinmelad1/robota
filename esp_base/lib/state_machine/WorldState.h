#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> // The Queue library

extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;

// Physical Arena Tracking  
struct FieldBox { String expectedQR; String targetColor; float expectedX; float expectedY; bool isPickedUp; };
struct StorageSlot { String assignedColor; bool isFull; float slotX; float slotY; float slotZ; };
struct DropoffZone { String zoneColor; float dropX; float dropY; bool isFinished; };

const int TOTAL_TARGETS = 3;
extern FieldBox field_boxes[TOTAL_TARGETS];
extern StorageSlot storage_unit[TOTAL_TARGETS];
extern DropoffZone dropoff_zones[TOTAL_TARGETS];

enum class GUITrigger {
    NONE,
    TRIGGER_MANUAL,
    TRIGGER_PICK,
    TRIGGER_AUTO
};

enum class DriveCommand {
    CMD_STOP,
    CMD_FWD, CMD_BWD, CMD_LEFT, CMD_RIGHT,
    CMD_FWD_L, CMD_FWD_R, CMD_BWD_L, CMD_BWD_R,
    CMD_ROT_L, CMD_ROT_R
};

struct ChassisMotion {
    DriveCommand move_type; 
    float speed;            
    float omega;     
};

enum class GripperStatus {
CLOSED,
OPEN,
PICK
};
enum class ArmDir : int8_t {
    STOP = 0,
    UP = 1,
    DOWN = -1
};

struct ArmMotion {
    int joint_id;
    ArmDir direction; // No more magic characters!
};

struct StringMessage {
    char data[32];
};