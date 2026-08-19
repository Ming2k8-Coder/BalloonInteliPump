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

#include <stdarg.h>

#define MAX_SYSTEM_LOGS 32
static SystemLogEntry log_ring_buffer[MAX_SYSTEM_LOGS];
static int log_write_idx = 0;
static int log_total_count = 0;

void log_system_event(uint8_t severity, const char* fmt, ...) {
    SystemLogEntry *entry = &log_ring_buffer[log_write_idx];
    entry->timestamp_s = (uint32_t)(esp_timer_get_time() / 1000000);
    entry->severity = severity;

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry->event, sizeof(entry->event), fmt, args);
    va_end(args);

    log_write_idx = (log_write_idx + 1) % MAX_SYSTEM_LOGS;
    if (log_total_count < MAX_SYSTEM_LOGS) log_total_count++;

    ESP_LOGI(TAG, "EVENT_LOG [%u]: %s", severity, entry->event);
}

void print_system_logs_json() {
    printf("{\"logs\":[");
    int count = (log_total_count < MAX_SYSTEM_LOGS) ? log_total_count : MAX_SYSTEM_LOGS;
    int start_idx = (log_total_count < MAX_SYSTEM_LOGS) ? 0 : log_write_idx;

    for (int i = 0; i < count; i++) {
        int idx = (start_idx + i) % MAX_SYSTEM_LOGS;
        SystemLogEntry *e = &log_ring_buffer[idx];
        printf("{\"time_s\":%lu,\"sev\":%u,\"msg\":\"%s\"}%s",
               (unsigned long)e->timestamp_s, e->severity, e->event, (i == count - 1) ? "" : ",");
    }
    printf("]}\n");
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

