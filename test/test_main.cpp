#include "unity.h"
#include "esp_log.h"

extern void run_sensor_tests(void);
extern void run_motor_tests(void);
extern void run_state_tests(void);
extern void run_storage_tests(void);

static const char *TAG = "BIP_UNITY_RUNNER";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== Starting BalloonInteliPump Unity Test Suite ===");
    
    UNITY_BEGIN();
    
    run_sensor_tests();
    run_motor_tests();
    run_state_tests();
    run_storage_tests();
    
    UNITY_END();
}
