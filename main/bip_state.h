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

#endif // BIP_STATE_H
