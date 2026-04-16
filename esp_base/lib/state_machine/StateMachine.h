#pragma once
#include <Arduino.h>
#include "WorldState.h"

enum class RobotState {
    MANUAL_MODE,
    START_PICK_SEQUENCE,
    WAIT_FOR_VISION_QR,
    WAIT_FOR_ARM_PICK,
    START_AUTO_DROP_SEQUENCE,
    NAVIGATING_TO_DROP,
    WAIT_FOR_VISION_COLOR,
    WAIT_FOR_ARM_DROP,
    WAIT_FOR_ARM_PICK_FINISH
};

class MasterStateMachine {
private:
    RobotState currentState;
    unsigned long stateTimer;
    
    int currentBoxTarget;
    int currentZoneTarget;

    String lastCommand = "IDLE";
    String currentModeStr = "MANUAL";
    String foundColor = "";

public:
    MasterStateMachine();
    void init();
    void update();
    String getTelemetryJSON();
};