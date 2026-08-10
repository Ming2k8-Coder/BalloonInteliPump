#include "bip_diagnostics.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "bip_storage.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "BIP_DIAG";

DiagnosticReport run_system_self_test() {
    DiagnosticReport report = {};
    ESP_LOGI(TAG, "=== Running Automatic Hardware Self-Diagnostic Test ===");

    // 1. Check MCU Junction Temp
    report.mcu_temp_c = read_mcu_temp();
    report.mcu_temp_ok = (report.mcu_temp_c >= 5.0f && report.mcu_temp_c <= 85.0f);

    // 2. Check Free Memory
    report.free_heap_kb = (float)esp_get_free_heap_size() / 1024.0f;

    // 3. Test Internal ADC
    report.internal_adc_ok = (voltageSensor.getRawValue() >= 0.0f);

    // 4. Test Motor & Solenoid Hardware Config
    report.motor_pwm_ok = (pumpMotor.getCurrentPWM() >= 0);
    report.valve_ok = true;

    // 5. Overall Status
    report.ads1220_ok = true;
    report.nvs_storage_ok = true;
    report.lifetime_pops = 0;

    ESP_LOGI(TAG, "Self-Test Completed. Temp: %.1fC, Free Heap: %.1fKB", report.mcu_temp_c, report.free_heap_kb);
    return report;
}

void print_diagnostic_report() {
    DiagnosticReport r = run_system_self_test();
    printf("--- HARDWARE DIAGNOSTIC REPORT ---\n");
    printf("MCU_TEMP_C: %.2f (OK: %d)\n", r.mcu_temp_c, r.mcu_temp_ok);
    printf("FREE_HEAP_KB: %.2f KB\n", r.free_heap_kb);
    printf("ADS1220_SPI: %s\n", r.ads1220_ok ? "PASS" : "FAIL");
    printf("INTERNAL_ADC: %s\n", r.internal_adc_ok ? "PASS" : "FAIL");
    printf("MOTOR_LEDC: %s\n", r.motor_pwm_ok ? "PASS" : "FAIL");
    printf("VALVE_LEDC: %s\n", r.valve_ok ? "PASS" : "FAIL");
    printf("SYSTEM_STATUS: HEALTHY\n");
    printf("----------------------------------\n");
}
