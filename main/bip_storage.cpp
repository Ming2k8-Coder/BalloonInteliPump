#include "bip_storage.h"
#include "bip_balloon_physics.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "BIP_STORAGE";

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

    SystemConfig cfg = {};
    cfg.version = 2;
    cfg.pressureCal = pressureSensor.getCalibration();
    cfg.voltageCal = voltageSensor.getCalibration();
    cfg.currentCal = currentSensor.getCalibration();
    cfg.motorSafety = pumpMotor.getSafetySettings();
    cfg.learnedBurstThreshold = get_learned_burst_threshold();

    err = nvs_set_blob(my_handle, NVS_CFG_KEY, &cfg, sizeof(SystemConfig));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        ESP_LOGI(TAG, "System configuration saved to NVS storage (Learned Burst Thresh: %.2f)", cfg.learnedBurstThreshold);
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
        pressureSensor.setCalibration(cfg.pressureCal);
        voltageSensor.setCalibration(cfg.voltageCal);
        currentSensor.setCalibration(cfg.currentCal);
        pumpMotor.setSafetySettings(cfg.motorSafety);
        if (cfg.learnedBurstThreshold < -10.0f) {
            set_learned_burst_threshold(cfg.learnedBurstThreshold);
        }
        ESP_LOGI(TAG, "System configuration restored from NVS storage");
    }
    nvs_close(my_handle);
    return err;
}

