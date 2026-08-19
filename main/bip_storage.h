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
    float learnedBurstThreshold;
    uint32_t writeCount;
    uint32_t crc32;
} SystemConfig;

esp_err_t init_storage();
esp_err_t save_system_config();
esp_err_t load_system_config();
uint32_t get_nvs_write_count();

#endif // BIP_STORAGE_H


