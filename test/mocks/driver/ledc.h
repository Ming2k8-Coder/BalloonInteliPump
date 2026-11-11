#ifndef DRIVER_LEDC_H
#define DRIVER_LEDC_H

#include "esp_err.h"

typedef enum {
    LEDC_LOW_SPEED_MODE = 0,
    LEDC_SPEED_MODE_MAX
} ledc_mode_t;

typedef enum {
    LEDC_TIMER_0 = 0,
    LEDC_TIMER_MAX
} ledc_timer_t;

typedef enum {
    LEDC_CHANNEL_0 = 0,
    LEDC_CHANNEL_1 = 1,
    LEDC_CHANNEL_MAX
} ledc_channel_t;

typedef enum {
    LEDC_TIMER_8_BIT = 8,
    LEDC_TIMER_10_BIT = 10,
    LEDC_TIMER_12_BIT = 12
} ledc_timer_bit_t;

typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
    int clk_cfg;
} ledc_timer_config_t;

typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    int intr_type;
    ledc_timer_t timer_num;
    uint32_t duty;
    int hpoint;
} ledc_channel_config_t;

inline esp_err_t ledc_timer_config(const ledc_timer_config_t* timer_conf) { (void)timer_conf; return ESP_OK; }
inline esp_err_t ledc_channel_config(const ledc_channel_config_t* ledc_conf) { (void)ledc_conf; return ESP_OK; }
inline esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty) { (void)speed_mode; (void)channel; (void)duty; return ESP_OK; }
inline esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel) { (void)speed_mode; (void)channel; return ESP_OK; }

#endif // DRIVER_LEDC_H
