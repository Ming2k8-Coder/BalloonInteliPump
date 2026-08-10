#ifndef BIP_STATE_H
#define BIP_STATE_H

#include "bip_config.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

enum SystemMode {
    MODE_IDLE = 0,
    MODE_MANUAL = 1,
    MODE_SMART = 2,
    MODE_BURST = 3,
    MODE_PULSE = 4,      // Dynamic Breathing / Heartbeat Mode
    MODE_PATTERN = 5,    // Rhythmic Waveform Player
    MODE_CALIBRATION = 6,
    MODE_ERROR = 7
};

enum WaveformPattern {
    PATTERN_SINE = 0,
    PATTERN_STAIRS = 1,
    PATTERN_TRIANGLE = 2,
    PATTERN_CRESCENDO = 3
};

// FreeRTOS Event Bits for Cross-Core Synchronization
#define BIP_EVENT_MODE_IDLE       (1 << 0)
#define BIP_EVENT_MODE_MANUAL     (1 << 1)
#define BIP_EVENT_MODE_SMART      (1 << 2)
#define BIP_EVENT_MODE_BURST      (1 << 3)
#define BIP_EVENT_MODE_PULSE      (1 << 4)
#define BIP_EVENT_MODE_PATTERN    (1 << 5)
#define BIP_EVENT_MODE_ERROR      (1 << 6)
#define BIP_EVENT_POP_TRIGGERED   (1 << 7)

extern EventGroupHandle_t sysEventGroup;
extern SystemMode currentMode;

void init_states();
void update_state_machine();
void post_mode_change(SystemMode newMode);

void set_yield_ratio(float ratio);
void set_manual_target(float pressure);
void set_manual_mode(bool targetMode);
void set_pid(float kp, float ki, float kd);
void set_pid_feedforward(float kff);
void trigger_error(const char* e1, const char* e2);

// Looner & Balloon Specific Play Modes
void set_pulse_params(float base_pressure, float amplitude, float frequency_hz);
void set_pattern(WaveformPattern pattern, float min_p, float max_p, float period_sec);

// Advanced Local Algorithms
float get_local_dp_dt();
float get_local_d2p_dt2();
float predict_local_pressure_forecast(int steps_ahead);

// High-Precision Pop Black Box RAM Recording
void record_pop_sample(float p, float rawV, uint8_t pwm, float currentA);
bool is_pop_recorded();
int get_pop_sample_count();
PopBlackBoxSample get_pop_sample(int index);
float get_last_pop_peak();
void dump_pop_recording();

#endif // BIP_STATE_H
