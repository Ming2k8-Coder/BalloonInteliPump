#ifndef BIP_STORAGE_H
#define BIP_STORAGE_H

#include "bip_config.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "esp_err.h"

typedef struct {
    uint32_t version;
    SensorCalibration pressureCal;
    SensorCalibration voltageCal;
    SensorCalibration currentCal;
    SafetySettings motorSafety;
} SystemConfig;

esp_err_t init_storage();
esp_err_t save_system_config();
esp_err_t load_system_config();

#endif // BIP_STORAGE_H
