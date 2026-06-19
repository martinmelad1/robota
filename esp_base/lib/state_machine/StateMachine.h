#pragma once
#include <Arduino.h>
#include "WorldState.h"
#include "PathPlanner.h"

enum class RobotState {
    MANUAL_MODE,
    CAMERA_PREVIEW,
    START_PICK_SEQUENCE,
    WAIT_FOR_VISION_QR,
    WAIT_FOR_ARM_PICK,
    WAIT_FOR_ARM_PICK_FINISH,
    START_AUTO_DROP_SEQUENCE,
    NAVIGATING_TO_DROP,
    WAIT_FOR_ARM_DROP
};

class MasterStateMachine {
private:
    RobotState currentState;
    unsigned long stateTimer;
    
    int currentBoxTarget;
    int currentZoneTarget;

    String lastCommand      = "IDLE";
    String currentModeStr   = "MANUAL";
    String foundColor        = "";
    DropAction pendingDrop   = DropAction::NONE; // set when PathPlanner signals a drop

public:
    MasterStateMachine();
    void init();
    void update();
    String getTelemetryJSON();
};