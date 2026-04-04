#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> // The Queue library

// --- GUI Input Data ---
enum class GUITrigger {
    NONE,
    TRIGGER_MANUAL,
    TRIGGER_PICK,
    TRIGGER_AUTO
};

struct WirelessCmd {
    float drive_vx;
    float drive_vy;
    float drive_omega;
    float arm_angles[5]; 
};

struct GUIPacket {
    GUITrigger mode_trigger;
    WirelessCmd cmd_payload;
};

// --- State Machine Output Data ---
struct VelocityTarget {
    float v_x;
    float v_y;
    float omega;
};

// ==========================================
// THE MAILBOXES (Shared Memory Pointers)
// ==========================================
extern QueueHandle_t guiMailbox;
extern QueueHandle_t pidMailbox;

// --- Physical Arena Tracking (Skeletons for later) ---
struct FieldBox { String expectedQR; String targetColor; float expectedX; float expectedY; bool isPickedUp; };
struct StorageSlot { String assignedColor; bool isFull; float slotX; float slotY; float slotZ; };
struct DropoffZone { String zoneColor; float dropX; float dropY; bool isFinished; };

const int TOTAL_TARGETS = 3;
extern FieldBox field_boxes[TOTAL_TARGETS];
extern StorageSlot storage_unit[TOTAL_TARGETS];
extern DropoffZone dropoff_zones[TOTAL_TARGETS];