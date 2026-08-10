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

DiagnosticReport run_system_self_test();
void print_diagnostic_report();

#endif // BIP_DIAGNOSTICS_H
