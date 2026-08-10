#ifndef STATE_FUNCTIONS_H
#define STATE_FUNCTIONS_H

#include <Arduino.h>
#include "Config.h"

enum SystemMode {
    MODE_IDLE,
    MODE_MANUAL,
    MODE_SMART,
    MODE_BURST,
    MODE_CALIBRATION,
    MODE_ERROR
};

// Global State Variables
extern SystemMode currentMode;

void initStates();
void updateStateMachine();
void setYieldRatio(float ratio);
void setManualTarget(float pressure);
void setManualMode(bool targetMode);
void setPID(float kp, float ki, float kd);

// Specific implementation
void updateIdle();
void updateManual();
void updateSmart();
void updateBurst();
void updateCalibration();
void updateError();

// Helpers
void triggerError(String e1, String e2);

#endif // STATE_FUNCTIONS_H
