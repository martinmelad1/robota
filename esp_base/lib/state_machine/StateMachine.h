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
    WAIT_FOR_ARM_DROP
};

class MasterStateMachine {
private:
    RobotState currentState;
    unsigned long stateTimer;
    
    int currentBoxTarget;
    int currentZoneTarget;

public:
    MasterStateMachine();
    void init();
    void update();
};