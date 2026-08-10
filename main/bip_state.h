#ifndef BIP_STATE_H
#define BIP_STATE_H

#include "bip_config.h"
#include "bip_sensors.h"
#include "bip_motor.h"

enum SystemMode {
    MODE_IDLE = 0,
    MODE_MANUAL = 1,
    MODE_SMART = 2,
    MODE_BURST = 3,
    MODE_CALIBRATION = 4,
    MODE_ERROR = 5
};

extern SystemMode currentMode;

void init_states();
void update_state_machine();
void set_yield_ratio(float ratio);
void set_manual_target(float pressure);
void set_manual_mode(bool targetMode);
void set_pid(float kp, float ki, float kd);
void set_pid_feedforward(float kff);
void trigger_error(const char* e1, const char* e2);

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
