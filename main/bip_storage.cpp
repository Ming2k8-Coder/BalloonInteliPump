#include "bip_storage.h"
#include "bip_balloon_physics.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstddef>

static const char *TAG = "BIP_STORAGE";
static uint32_t persistent_write_count = 0;


static uint32_t compute_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

uint32_t get_nvs_write_count() {
    return persistent_write_count;
}

esp_err_t init_storage() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret == ESP_OK) {
        load_system_config();
    }
    return ret;
}

esp_err_t save_system_config() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    persistent_write_count++;

    SystemConfig cfg = {};
    cfg.version = 3;
    cfg.pressureCal = pressureSensor.getCalibration();
    cfg.voltageCal = voltageSensor.getCalibration();
    cfg.currentCal = currentSensor.getCalibration();
    cfg.motorSafety = pumpMotor.getSafetySettings();
    cfg.learnedBurstThreshold = get_learned_burst_threshold();
    cfg.writeCount = persistent_write_count;
    cfg.crc32 = compute_crc32((const uint8_t *)&cfg, offsetof(SystemConfig, crc32));

    err = nvs_set_blob(my_handle, NVS_CFG_KEY, &cfg, sizeof(SystemConfig));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        ESP_LOGI(TAG, "Config saved (Writes: %lu, CRC32: 0x%08LX, Burst Thresh: %.2f)",
                 (unsigned long)cfg.writeCount, (unsigned long)cfg.crc32, cfg.learnedBurstThreshold);
    }
    nvs_close(my_handle);
    return err;
}

esp_err_t load_system_config() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;

    SystemConfig cfg = {};
    size_t required_size = sizeof(SystemConfig);
    err = nvs_get_blob(my_handle, NVS_CFG_KEY, &cfg, &required_size);
    if (err == ESP_OK) {
        uint32_t expected_crc = compute_crc32((const uint8_t *)&cfg, offsetof(SystemConfig, crc32));
        if (cfg.version == 3 && cfg.crc32 != expected_crc) {
            ESP_LOGE(TAG, "CONFIG CRC32 MISMATCH! Expected 0x%08LX, got 0x%08LX. Using defaults.",
                     (unsigned long)expected_crc, (unsigned long)cfg.crc32);
            nvs_close(my_handle);
            return ESP_ERR_INVALID_CRC;
        }

        pressureSensor.setCalibration(cfg.pressureCal);
        voltageSensor.setCalibration(cfg.voltageCal);
        currentSensor.setCalibration(cfg.currentCal);
        pumpMotor.setSafetySettings(cfg.motorSafety);
        if (cfg.learnedBurstThreshold < -10.0f) {
            set_learned_burst_threshold(cfg.learnedBurstThreshold);
        }
        persistent_write_count = cfg.writeCount;
        ESP_LOGI(TAG, "System config restored (Version: %lu, Writes: %lu, CRC: 0x%08LX)",
                 (unsigned long)cfg.version, (unsigned long)persistent_write_count, (unsigned long)cfg.crc32);
    }
    nvs_close(my_handle);
    return err;
}


