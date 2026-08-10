#include "bip_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <cstring>
#include <algorithm>

static const char *TAG = "BIP_STATE";

SystemMode currentMode = MODE_IDLE;
static const char* errorLine1 = "";
static const char* errorLine2 = "";

// Smart Mode Variables
enum SmartPhase {
    SMART_IDLE,
    SMART_INFLATING_YIELD,
    SMART_INFLATING_TARGET,
    SMART_FINISHED
};

static SmartPhase smartState = SMART_IDLE;
static float userYieldRatio = 1.20f;
static float detectedYieldPressure = 0.0f;
static float peakPressureTracker = 0.0f;

// Manual Mode Variables
static int manualPWM = 0;
static bool manualTargetMode = false;
static float manualTargetPressure = 20.0f;

// Advanced PID Controller with Feedforward
static float pid_Kp = 25.0f;
static float pid_Ki = 1.2f;
static float pid_Kd = 4.0f;
static float pid_Kff = 3.5f; // Feedforward gain (PWM per kPa)
static float pid_integral = 0.0f;
static float pid_lastError = 0.0f;
static int64_t pid_lastTime = 0;

// Savitzky-Golay FIR Filter Buffer for 1st (dP/dt) & 2nd (d2P/dt2) Derivatives
#define SG_WINDOW_SIZE 9
static float pressureBuf[SG_WINDOW_SIZE] = {0};
static int64_t timeBuf[SG_WINDOW_SIZE] = {0};
static int bufIndex = 0;
static bool bufFull = false;

static float current_dp_dt = 0.0f;
static float current_d2p_dt2 = 0.0f;

// On-Device RLS Predictor State
static float rls_weights[3] = { 0.8f, 0.15f, 0.05f };
static float rls_p_matrix[3][3] = { {100.0f, 0, 0}, {0, 100.0f, 0}, {0, 0, 100.0f} };
static float last_forecasted = 0.0f;

static void reset_buffers() {
    memset(pressureBuf, 0, sizeof(pressureBuf));
    memset(timeBuf, 0, sizeof(timeBuf));
    bufIndex = 0;
    bufFull = false;
    current_dp_dt = 0.0f;
    current_d2p_dt2 = 0.0f;
}

static void add_sample(float p) {
    pressureBuf[bufIndex] = p;
    timeBuf[bufIndex] = esp_timer_get_time();
    bufIndex = (bufIndex + 1) % SG_WINDOW_SIZE;
    if (bufIndex == 0) bufFull = true;

    if (bufFull) {
        // Savitzky-Golay FIR 9-point derivative approximation
        // dP/dt: 1st Derivative Savitzky-Golay coefficients [-4, -3, -2, -1, 0, 1, 2, 3, 4] / 60
        float num_dp = 0.0f;
        float num_d2p = 0.0f;
        int coeffs_1st[9] = { -4, -3, -2, -1, 0, 1, 2, 3, 4 };
        int coeffs_2nd[9] = { 28, 7, -8, -17, -20, -17, -8, 7, 28 };

        int oldest_idx = bufIndex;
        int64_t dt_micro = timeBuf[(bufIndex + SG_WINDOW_SIZE - 1) % SG_WINDOW_SIZE] - timeBuf[oldest_idx];
        float dt_sec = (float)dt_micro / 1000000.0f;
        float h = dt_sec / (float)(SG_WINDOW_SIZE - 1);
        if (h <= 0.0001f) h = 0.0005f;

        for (int i = 0; i < SG_WINDOW_SIZE; i++) {
            int idx = (oldest_idx + i) % SG_WINDOW_SIZE;
            num_dp += (float)coeffs_1st[i] * pressureBuf[idx];
            num_d2p += (float)coeffs_2nd[i] * pressureBuf[idx];
        }

        current_dp_dt = num_dp / (60.0f * h);
        current_d2p_dt2 = num_d2p / (462.0f * h * h);
    }
}

float get_local_dp_dt() {
    return current_dp_dt;
}

float get_local_d2p_dt2() {
    return current_d2p_dt2;
}

// On-Device RLS Predictor
float predict_local_pressure_forecast(int steps_ahead) {
    if (!bufFull) return pressureSensor.getValue();

    float p_curr = pressureSensor.getValue();
    int idx1 = (bufIndex + SG_WINDOW_SIZE - 1) % SG_WINDOW_SIZE;
    int idx2 = (bufIndex + SG_WINDOW_SIZE - 2) % SG_WINDOW_SIZE;
    int idx3 = (bufIndex + SG_WINDOW_SIZE - 3) % SG_WINDOW_SIZE;

    float x0 = pressureBuf[idx1];
    float x1 = pressureBuf[idx2];
    float x2 = pressureBuf[idx3];

    // 1-step forecast
    float pred = rls_weights[0] * x0 + rls_weights[1] * x1 + rls_weights[2] * x2;
    if (!std::isfinite(pred) || pred < 0.0f) pred = p_curr;
    
    last_forecasted = pred;
    return pred;
}

// Burst Mode Variables
static float burstPeak = 0.0f;
static bool burstPopped = false;

void init_states() {
    currentMode = MODE_IDLE;
    burstPopped = false;
    burstPeak = 0.0f;
    reset_buffers();
}

void trigger_error(const char* e1, const char* e2) {
    pumpMotor.emergencyStop();
    errorLine1 = e1;
    errorLine2 = e2;
    currentMode = MODE_ERROR;
    ESP_LOGE(TAG, "Error Triggered: %s - %s", e1, e2);
}

void set_yield_ratio(float ratio) {
    if (ratio >= 1.0f) userYieldRatio = ratio;
}

void set_manual_target(float pressure) {
    manualTargetPressure = pressure;
    manualTargetMode = true;
    pid_lastTime = 0;
    pid_integral = 0;
}

void set_manual_mode(bool targetMode) {
    manualTargetMode = targetMode;
    pid_lastTime = 0;
    pid_integral = 0;
}

void set_pid(float kp, float ki, float kd) {
    pid_Kp = kp;
    pid_Ki = ki;
    pid_Kd = kd;
    pid_integral = 0;
    pid_lastError = 0;
}

void set_pid_feedforward(float kff) {
    pid_Kff = kff;
}

static void update_manual() {
    if (!manualTargetMode) {
        pumpMotor.setTargetPWM(manualPWM);
    } else {
        float currentPressure = pressureSensor.getValue();
        float error = manualTargetPressure - currentPressure;
        
        int64_t now = esp_timer_get_time();
        if (pid_lastTime == 0) {
            pid_lastTime = now - 50000;
            pid_lastError = error;
        }
        float dt = (float)(now - pid_lastTime) / 1000000.0f;
        if (dt <= 0.001f) dt = 0.001f;
        pid_lastTime = now;
        
        if (std::abs(error) < 8.0f) {
            pid_integral += error * dt;
        } else {
            pid_integral = 0;
        }
        pid_integral = std::clamp(pid_integral, -50.0f, 50.0f);
        
        float raw_derivative = (error - pid_lastError) / dt;
        static float derivative = 0;
        derivative = 0.7f * derivative + 0.3f * raw_derivative;
        pid_lastError = error;
        
        // Feedforward calculation
        float feedforward = pid_Kff * manualTargetPressure;
        float output = feedforward + (pid_Kp * error) + (pid_Ki * pid_integral) + (pid_Kd * derivative);
        int targetPWM = std::clamp((int)output, 0, 255);
        pumpMotor.setTargetPWM(targetPWM);
    }
}

static void update_smart() {
    float p = pressureSensor.getValue();
    add_sample(p);

    switch (smartState) {
        case SMART_IDLE:
            reset_buffers();
            smartState = SMART_INFLATING_YIELD;
            peakPressureTracker = 0;
            detectedYieldPressure = 0;
            pumpMotor.setTargetPWM(200);
            break;
            
        case SMART_INFLATING_YIELD:
            if (p > peakPressureTracker) peakPressureTracker = p;
            {
                float slope = get_local_dp_dt();
                float d2p = get_local_d2p_dt2();

                // Inflection yield point: p > 5.0kPa AND slope drop AND 2nd derivative turns negative
                if (p > 5.0f && (slope <= 0.15f || d2p < -0.5f)) {
                    detectedYieldPressure = peakPressureTracker;
                    smartState = SMART_INFLATING_TARGET;
                }
            }
            break;
            
        case SMART_INFLATING_TARGET:
            {
                float target = detectedYieldPressure * userYieldRatio;
                if (p >= target) {
                    pumpMotor.setTargetPWM(0);
                    smartState = SMART_FINISHED;
                }
            }
            break;
            
        case SMART_FINISHED:
            pumpMotor.setTargetPWM(0);
            break;
    }
}

static void update_burst() {
    if (burstPopped) {
        if (pumpMotor.getCurrentPWM() > 0) pumpMotor.emergencyStop();
        return;
    }
    
    float p = pressureSensor.getValue();
    add_sample(p);

    if (p > burstPeak) burstPeak = p;
    
    pumpMotor.setTargetPWM(255);
    
    if (burstPeak > 10.0f && p < burstPeak * 0.8f) {
        pumpMotor.emergencyStop();
        burstPopped = true;
    }
}

void update_state_machine() {
    switch (currentMode) {
        case MODE_IDLE:
            pumpMotor.setTargetPWM(0);
            break;
        case MODE_MANUAL:
            update_manual();
            break;
        case MODE_SMART:
            update_smart();
            break;
        case MODE_BURST:
            update_burst();
            break;
        case MODE_CALIBRATION:
            break;
        case MODE_ERROR:
            pumpMotor.emergencyStop();
            break;
    }
}
