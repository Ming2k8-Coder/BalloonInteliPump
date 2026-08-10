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

// PID Controller
static float pid_Kp = 25.0f;
static float pid_Ki = 1.2f;
static float pid_Kd = 4.0f;
static float pid_integral = 0.0f;
static float pid_lastError = 0.0f;
static int64_t pid_lastTime = 0;

// Slope History
#define SLOPE_WINDOW_SIZE 10
static float pressureHistory[SLOPE_WINDOW_SIZE] = {0};
static int64_t timeHistory[SLOPE_WINDOW_SIZE] = {0};
static int historyIndex = 0;
static bool historyFull = false;
static int flatCount = 0;

static void reset_history() {
    memset(pressureHistory, 0, sizeof(pressureHistory));
    memset(timeHistory, 0, sizeof(timeHistory));
    historyIndex = 0;
    historyFull = false;
    flatCount = 0;
}

static void add_history_point(float p) {
    pressureHistory[historyIndex] = p;
    timeHistory[historyIndex] = esp_timer_get_time();
    historyIndex = (historyIndex + 1) % SLOPE_WINDOW_SIZE;
    if (historyIndex == 0) historyFull = true;
}

static float calculate_slope() {
    if (!historyFull) return 1.0f;
    
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    int n = SLOPE_WINDOW_SIZE;
    int64_t baseTime = timeHistory[historyIndex];
    
    for (int i = 0; i < n; i++) {
        int idx = (historyIndex + i) % n;
        float x = (float)(timeHistory[idx] - baseTime) / 1000000.0f;
        float y = pressureHistory[idx];
        
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumXX += x * x;
    }
    
    float denominator = (n * sumXX - sumX * sumX);
    if (std::abs(denominator) < 0.0001f) return 0.0f;
    return (n * sumXY - sumX * sumY) / denominator;
}

// Burst Mode
static float burstPeak = 0.0f;
static bool burstPopped = false;

void init_states() {
    currentMode = MODE_IDLE;
    burstPopped = false;
    burstPeak = 0.0f;
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
        
        if (std::abs(error) < 10.0f) {
            pid_integral += error * dt;
        } else {
            pid_integral = 0;
        }
        pid_integral = std::clamp(pid_integral, -100.0f, 100.0f);
        
        float raw_derivative = (error - pid_lastError) / dt;
        static float derivative = 0;
        derivative = 0.7f * derivative + 0.3f * raw_derivative;
        pid_lastError = error;
        
        float output = (pid_Kp * error) + (pid_Ki * pid_integral) + (pid_Kd * derivative);
        int targetPWM = std::clamp((int)output, 0, 255);
        pumpMotor.setTargetPWM(targetPWM);
    }
}

static void update_smart() {
    float p = pressureSensor.getValue();
    if (smartState == SMART_INFLATING_YIELD) {
        add_history_point(p);
    }
    
    switch (smartState) {
        case SMART_IDLE:
            reset_history();
            smartState = SMART_INFLATING_YIELD;
            peakPressureTracker = 0;
            detectedYieldPressure = 0;
            pumpMotor.setTargetPWM(200);
            break;
            
        case SMART_INFLATING_YIELD:
            if (p > peakPressureTracker) peakPressureTracker = p;
            {
                float slope = calculate_slope();
                if (p > 5.0f && slope <= 0.15f) {
                    flatCount++;
                    if (flatCount >= 10) {
                        detectedYieldPressure = peakPressureTracker;
                        smartState = SMART_INFLATING_TARGET;
                        flatCount = 0;
                    }
                } else {
                    flatCount = 0;
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
