#ifndef BIP_DIAGNOSTICS_H
#define BIP_DIAGNOSTICS_H

#include "bip_config.h"
#include "esp_err.h"

typedef struct {
    bool ads1220_ok;
    bool internal_adc_ok;
    bool mcu_temp_ok;
    bool nvs_storage_ok;
    bool motor_pwm_ok;
    bool valve_ok;
    float mcu_temp_c;
    float free_heap_kb;
    uint32_t lifetime_pops;
} DiagnosticReport;

typedef struct {
    uint32_t timestamp_s;
    char event[64];
    uint8_t severity; // 0=INFO, 1=WARN, 2=ERROR
} SystemLogEntry;


DiagnosticReport run_system_self_test();
void print_diagnostic_report();

void log_system_event(uint8_t severity, const char* fmt, ...);
void print_system_logs_json();

#endif // BIP_DIAGNOSTICS_H

