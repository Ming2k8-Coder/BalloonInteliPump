#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Motor.h"

// Unified Configuration Structure
struct SystemConfig {
    uint32_t version;
    SensorCalibration pressureCal;
    SensorCalibration voltageCal;
    SensorCalibration currentCal;
    SafetySettings motorSafety;
};

void initStorage();
void saveSystemConfig();
void loadSystemConfig();
void factoryReset();

#endif // STORAGE_H
