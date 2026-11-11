#ifndef ADC_ONESHOT_H
#define ADC_ONESHOT_H

#include "esp_err.h"

typedef void* adc_oneshot_unit_handle_t;
typedef int adc_channel_t;

typedef struct {
    int unit_id;
} adc_oneshot_unit_init_cfg_t;

typedef struct {
    int atten;
    int bitwidth;
} adc_oneshot_chan_cfg_t;

inline esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *init_config, adc_oneshot_unit_handle_t *ret_unit) {
    (void)init_config; (void)ret_unit; return ESP_OK;
}
inline esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel, const adc_oneshot_chan_cfg_t *config) {
    (void)handle; (void)channel; (void)config; return ESP_OK;
}
inline esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t chan, int *out_raw) {
    (void)handle; (void)chan; if (out_raw) *out_raw = 0; return ESP_OK;
}

#endif // ADC_ONESHOT_H
