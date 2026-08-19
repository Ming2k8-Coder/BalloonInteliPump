#include "bip_state.h"
#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <cstring>
#include <algorithm>

static const char *TAG = "BIP_STATE";

EventGroupHandle_t sysEventGroup = NULL;
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

// Pulse / Breathing Mode Variables
static float pulseBasePressure = 15.0f;
static float pulseAmplitude = 3.0f;
static float pulseFreqHz = 0.5f;

// Pattern Mode Variables
static WaveformPattern currentPattern = PATTERN_SINE;
static float patternMinP = 10.0f;
static float patternMaxP = 25.0f;
static float patternPeriodSec = 10.0f;

// Idea #6: Diameter Mode Variables
static float targetDiameterCm = 25.0f;

// Idea #11: Ride Mode Variables
static float riderWeightKg = 70.0f;
static float targetSinkCm = 8.0f;
static float rideBaselinePressure = 0.0f;
static int64_t rideStartTimeUs = 0;

// Idea #12: Bounce Rhythm Detector State
static BounceRhythmState bounceRhythm = {};
static int64_t lastBouncePeakTimeUs = 0;
static float bouncePeriodEmaMs = 0.0f;
static int consecutiveBouncePeriods = 0;
static float prevDpDtForPeak = 0.0f;

// Idea #14: Conditioning Mode Sub-State Machine
typedef enum {
    COND_IDLE = 0,
    COND_RAMP_INFLATE,
    COND_RELAX_HOLD,
    COND_CONTROLLED_DEFLATE,
    COND_ANALYZE_LOOP,
    COND_FINISHED
} ConditioningSubPhase;

static ConditioningSubPhase condPhase = COND_IDLE;
static int condCurrentCycle = 0;
static int condMaxCycles = 4;
static int64_t condPhaseStartUs = 0;
static float condYieldPressures[6] = {0};
static float condPeakPressures[6] = {0};
static float condSofteningPct = 0.0f;
static float condConvergencePct = 0.0f;
static bool condComplete = false;

// Idea #4: Hysteresis Band Settings
static float pid_hysteresis_band = 0.5f; // ±0.5 kPa deadband

// Advanced PID Controller with Feedforward
static float pid_Kp = 25.0f;
static float pid_Ki = 1.2f;
static float pid_Kd = 4.0f;
static float pid_Kff = 3.5f;
static float pid_integral = 0.0f;
static float pid_lastError = 0.0f;
static int64_t pid_lastTime = 0;

// Savitzky-Golay FIR Filter Buffer
#define SG_WINDOW_SIZE 9
static float pressureBuf[SG_WINDOW_SIZE] = {0};
static int64_t timeBuf[SG_WINDOW_SIZE] = {0};
static int bufIndex = 0;
static bool bufFull = false;

static float current_dp_dt = 0.0f;
static float current_d2p_dt2 = 0.0f;

// RLS Predictor State
static float rls_weights[3] = { 0.8f, 0.15f, 0.05f };
static float last_forecasted = 0.0f;

// DUAL-RAM SYSTEM: LIVE CONTINUOUS RING BUFFER + FAST MEMCPY SAFE POP RAM
static PopBlackBoxSample liveRingBuffer[POP_BUF_SIZE];
static PopBlackBoxSample safePopRAMBuffer[POP_BUF_SIZE];

static int liveWriteIndex = 0;
static int safePopHeadIndex = 0;
static bool safePopDataReady = false;
static int postPopHoldCountdown = 0;
static float burstPeak = 0.0f;
static bool burstPopped = false;

static void reset_buffers() {
    memset(pressureBuf, 0, sizeof(pressureBuf));
    memset(timeBuf, 0, sizeof(timeBuf));
    bufIndex = 0;
    bufFull = false;
    current_dp_dt = 0.0f;
    current_d2p_dt2 = 0.0f;
    postPopHoldCountdown = 0;
    pid_integral = 0;
    pid_lastError = 0;
    pid_lastTime = 0;
    rideStartTimeUs = 0;
    rideBaselinePressure = 0.0f;
}

void set_pulse_params(float base_pressure, float amplitude, float frequency_hz) {
    pulseBasePressure = std::clamp(base_pressure, 1.0f, 60.0f);
    pulseAmplitude = std::clamp(amplitude, 0.1f, 15.0f);
    pulseFreqHz = std::clamp(frequency_hz, 0.05f, 5.0f);
}

void set_pattern(WaveformPattern pattern, float min_p, float max_p, float period_sec) {
    currentPattern = pattern;
    patternMinP = std::clamp(min_p, 1.0f, 60.0f);
    patternMaxP = std::clamp(max_p, min_p + 1.0f, 70.0f);
    patternPeriodSec = std::clamp(period_sec, 1.0f, 60.0f);
}

void set_target_diameter(float diameter_cm) {
    targetDiameterCm = std::clamp(diameter_cm, 5.0f, 120.0f);
    pid_lastTime = 0;
    pid_integral = 0;
    post_mode_change(MODE_DIAMETER);
    ESP_LOGI(TAG, "Target Diameter set to %.1f cm", targetDiameterCm);
}

void set_ride_params(float rider_weight_kg, float sink_depth_cm) {
    riderWeightKg = std::clamp(rider_weight_kg, 20.0f, 200.0f);
    targetSinkCm = std::clamp(sink_depth_cm, 2.0f, 25.0f);
    rideStartTimeUs = 0;
    post_mode_change(MODE_RIDE);
    ESP_LOGI(TAG, "Ride Params updated: Rider %.1f kg, Sink %.1f cm", riderWeightKg, targetSinkCm);
}

BounceRhythmState get_bounce_rhythm() {
    return bounceRhythm;
}

void start_balloon_conditioning(int cycles) {
    condMaxCycles = std::clamp(cycles, 2, 6);
    condCurrentCycle = 1;
    condComplete = false;
    condPhase = COND_RAMP_INFLATE;
    condPhaseStartUs = esp_timer_get_time();
    memset(condYieldPressures, 0, sizeof(condYieldPressures));
    memset(condPeakPressures, 0, sizeof(condPeakPressures));
    post_mode_change(MODE_CONDITION);
    ESP_LOGI(TAG, "Automated Balloon Conditioning Started (%d Cycles Target)", condMaxCycles);
}

bool is_conditioning_complete() {
    return condComplete;
}

void set_hysteresis_band(float deadband_kpa) {
    pid_hysteresis_band = std::clamp(deadband_kpa, 0.0f, 3.0f);
}

void post_mode_change(SystemMode newMode) {
    if (currentMode == newMode) return;

    ESP_LOGI(TAG, "Mode Transition: %d -> %d", (int)currentMode, (int)newMode);
    currentMode = newMode;

    if (sysEventGroup != NULL) {
        xEventGroupClearBits(sysEventGroup,
                             BIP_EVENT_MODE_IDLE | BIP_EVENT_MODE_MANUAL |
                             BIP_EVENT_MODE_SMART | BIP_EVENT_MODE_BURST |
                             BIP_EVENT_MODE_PULSE | BIP_EVENT_MODE_PATTERN |
                             BIP_EVENT_MODE_ERROR | BIP_EVENT_MODE_RIDE |
                             BIP_EVENT_MODE_DIAMETER);
    }

    reset_buffers();

    switch (newMode) {
        case MODE_IDLE:
            pumpMotor.setTargetPWM(0);
            write_solenoid_pwm(0);
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_IDLE);
            break;
        case MODE_MANUAL:
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_MANUAL);
            break;
        case MODE_SMART:
            smartState = SMART_IDLE;
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_SMART);
            break;
        case MODE_BURST:
            burstPopped = false;
            burstPeak = 0.0f;
            safePopDataReady = false;
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_BURST);
            break;
        case MODE_PULSE:
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_PULSE);
            break;
        case MODE_PATTERN:
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_PATTERN);
            break;
        case MODE_DIAMETER:
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_DIAMETER);
            break;
        case MODE_RIDE:
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_RIDE);
            break;
        case MODE_CONDITION:
            break;
        case MODE_CALIBRATION:
            pumpMotor.setTargetPWM(0);
            break;
        case MODE_ERROR:
            pumpMotor.emergencyStop();
            write_solenoid_pwm(255);
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_ERROR);
            break;
    }
}

void record_pop_sample(float p, float rawV, uint8_t pwm, float currentA) {
    liveRingBuffer[liveWriteIndex].timestamp_us = (uint32_t)esp_timer_get_time();
    liveRingBuffer[liveWriteIndex].pressure = p;
    liveRingBuffer[liveWriteIndex].rawVolts = rawV;
    liveRingBuffer[liveWriteIndex].pwm = (uint16_t)pwm;
    liveRingBuffer[liveWriteIndex].current_ma = (uint16_t)(currentA * 1000.0f);

    liveWriteIndex = (liveWriteIndex + 1) % POP_BUF_SIZE;

    if (postPopHoldCountdown > 0) {
        postPopHoldCountdown--;
        if (postPopHoldCountdown == 0) {
            memcpy(safePopRAMBuffer, liveRingBuffer, sizeof(liveRingBuffer));
            safePopHeadIndex = liveWriteIndex;
            safePopDataReady = true;
            if (sysEventGroup != NULL) xEventGroupSetBits(sysEventGroup, BIP_EVENT_POP_TRIGGERED);
            ESP_LOGI(TAG, "POP CAPTURED! Executed fast memcpy into Safe RAM.");
            update_burst_threshold_from_pop();
        }
    }
}

bool is_pop_recorded() { return safePopDataReady; }
int get_pop_sample_count() { return POP_BUF_SIZE; }

PopBlackBoxSample get_pop_sample(int index) {
    if (index < 0 || index >= POP_BUF_SIZE || !safePopDataReady) {
        PopBlackBoxSample empty = {};
        return empty;
    }
    int realIdx = (safePopHeadIndex + index) % POP_BUF_SIZE;
    return safePopRAMBuffer[realIdx];
}

float get_last_pop_peak() { return burstPeak; }

void dump_pop_recording() {
    if (!safePopDataReady) {
        printf("POP_DUMP_ERR,No_Pop_Captured\n");
        return;
    }
    printf("--- POP BLACK BOX SAFE RAM DUMP START (2000 SPS) ---\n");
    printf("Sample,Time_us,Pressure_kPa,Raw_Volts,PWM,Current_mA\n");
    for (int i = 0; i < POP_BUF_SIZE; i++) {
        PopBlackBoxSample s = get_pop_sample(i);
        printf("%d,%lu,%.4f,%.6f,%u,%u\n",
               i, (unsigned long)s.timestamp_us, s.pressure, s.rawVolts, s.pwm, s.current_ma);
    }
    printf("--- POP BLACK BOX DUMP END ---\n");
}

static void add_sample(float p) {
    pressureBuf[bufIndex] = p;
    timeBuf[bufIndex] = esp_timer_get_time();
    bufIndex = (bufIndex + 1) % SG_WINDOW_SIZE;
    if (bufIndex == 0) bufFull = true;

    if (bufFull) {
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

float get_local_dp_dt() { return current_dp_dt; }
float get_local_d2p_dt2() { return current_d2p_dt2; }

float predict_local_pressure_forecast(int steps_ahead) {
    if (!bufFull) return pressureSensor.getValue();
    float p_curr = pressureSensor.getValue();
    int idx1 = (bufIndex + SG_WINDOW_SIZE - 1) % SG_WINDOW_SIZE;
    int idx2 = (bufIndex + SG_WINDOW_SIZE - 2) % SG_WINDOW_SIZE;
    int idx3 = (bufIndex + SG_WINDOW_SIZE - 3) % SG_WINDOW_SIZE;
    float x0 = pressureBuf[idx1];
    float x1 = pressureBuf[idx2];
    float x2 = pressureBuf[idx3];
    float pred = rls_weights[0] * x0 + rls_weights[1] * x1 + rls_weights[2] * x2;
    if (!std::isfinite(pred) || pred < 0.0f) pred = p_curr;
    last_forecasted = pred;
    return pred;
}

void init_states() {
    if (sysEventGroup == NULL) {
        sysEventGroup = xEventGroupCreate();
    }
    post_mode_change(MODE_IDLE);
}

void trigger_error(const char* e1, const char* e2) {
    errorLine1 = e1;
    errorLine2 = e2;
    post_mode_change(MODE_ERROR);
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
        
        float feedforward = pid_Kff * manualTargetPressure;
        float output = feedforward + (pid_Kp * error) + (pid_Ki * pid_integral) + (pid_Kd * derivative);
        int targetPWM = std::clamp((int)output, 0, 255);
        pumpMotor.setTargetPWM(targetPWM);
    }
}

// Idea #6: Closed-Loop Balloon Diameter Controller
static void update_diameter() {
    BalloonPhysicsEstimate est = get_balloon_physics_estimate();
    BalloonMaterialPhysics phys = get_balloon_physics_state();

    if (phys.yield_probability > 0.88f) {
        pumpMotor.emergencyStop();
        trigger_error("DIAMETER", "Rupture risk > 88%");
        return;
    }

    float currentDiameter = est.diameter_cm;
    float error = targetDiameterCm - currentDiameter;

    int64_t now = esp_timer_get_time();
    if (pid_lastTime == 0) pid_lastTime = now - 50000;
    float dt = (float)(now - pid_lastTime) / 1000000.0f;
    if (dt <= 0.001f) dt = 0.001f;
    pid_lastTime = now;

    if (std::abs(error) < 5.0f) pid_integral += error * dt;
    else pid_integral = 0.0f;
    pid_integral = std::clamp(pid_integral, -20.0f, 20.0f);

    float output = (pid_Kp * 1.5f * error) + (pid_Ki * pid_integral) + (pid_Kff * 1.2f * targetDiameterCm);
    pumpMotor.setTargetPWM(std::clamp((int)output, 0, 255));

    if (error < -1.0f) write_solenoid_pwm(140);
    else write_solenoid_pwm(0);
}

// Idea #12: Bounce Rhythm Detector Algorithm
static void update_bounce_rhythm_detector(float dp_dt, float pressure_kpa) {
    int64_t now = esp_timer_get_time();

    if (prevDpDtForPeak > 12.0f && dp_dt < 0.0f && pressure_kpa > 4.0f) {
        if (lastBouncePeakTimeUs > 0) {
            float measured_period_ms = (float)(now - lastBouncePeakTimeUs) / 1000.0f;
            if (measured_period_ms >= 250.0f && measured_period_ms <= 2000.0f) {
                if (bouncePeriodEmaMs <= 0.0f) bouncePeriodEmaMs = measured_period_ms;
                else bouncePeriodEmaMs = 0.70f * bouncePeriodEmaMs + 0.30f * measured_period_ms;

                consecutiveBouncePeriods++;
                bounceRhythm.bounce_count++;
                record_bounce_cycle(get_balloon_physics_state().hyperelastic_stress_kpa);
            }
        }
        lastBouncePeakTimeUs = now;
    }
    prevDpDtForPeak = dp_dt;

    bounceRhythm.rhythm_locked = (consecutiveBouncePeriods >= 4 && bouncePeriodEmaMs > 200.0f);
    if (bounceRhythm.rhythm_locked) {
        bounceRhythm.period_ms = bouncePeriodEmaMs;
        bounceRhythm.frequency_hz = 1000.0f / bouncePeriodEmaMs;
        float elapsed_in_cycle = (float)(now - lastBouncePeakTimeUs) / 1000.0f;
        bounceRhythm.phase_angle = fmodf(elapsed_in_cycle / bouncePeriodEmaMs, 1.0f);
    }
}

// Idea #11: Ride Mode Active Weight-Bearing Pressure Controller
static void update_ride() {
    float p = pressureSensor.getValue();
    float dp_dt = get_local_dp_dt();
    int64_t now = esp_timer_get_time();

    if (rideStartTimeUs == 0) {
        rideStartTimeUs = now;
        rideBaselinePressure = p;
        return;
    }

    update_bounce_rhythm_detector(dp_dt, p);

    RideInflationAdvice advice = compute_ride_inflation(riderWeightKg, BALLOON_36INCH, 0.5f);
    float p_target_ride = advice.recommended_pressure_kpa;

    static float slow_leak_integral = 0.0f;
    float slow_error = p_target_ride - rideBaselinePressure;
    slow_leak_integral += slow_error * 0.0005f;
    slow_leak_integral = std::clamp(slow_leak_integral, 0.0f, 180.0f);

    bool airborne_phase = (p < rideBaselinePressure + 2.5f);
    if (bounceRhythm.rhythm_locked) {
        airborne_phase = (bounceRhythm.phase_angle > 0.55f && bounceRhythm.phase_angle < 0.95f);
    }

    if (airborne_phase && slow_leak_integral > 10.0f) {
        pumpMotor.setTargetPWM(std::clamp((int)slow_leak_integral, 50, 200));
    } else {
        pumpMotor.setTargetPWM(0);
    }

    if (p > advice.max_safe_pressure_kpa) {
        write_solenoid_pwm(180);
    } else {
        write_solenoid_pwm(0);
    }
}

// Idea #14: Automated Latex Pre-Conditioning Protocol State Engine
static void update_conditioning() {
    float p = pressureSensor.getValue();
    float dp_dt = get_local_dp_dt();
    float d2p = get_local_d2p_dt2();
    int64_t now = esp_timer_get_time();
    float elapsed_sec = (float)(now - condPhaseStartUs) / 1000000.0f;

    switch (condPhase) {
        case COND_IDLE:
            pumpMotor.setTargetPWM(0);
            write_solenoid_pwm(0);
            break;

        case COND_RAMP_INFLATE: {
            float cycle_scale = 0.60f + (0.08f * (condCurrentCycle - 1));
            float target_p_limit = 28.0f * cycle_scale;

            pumpMotor.setTargetPWM(160);
            write_solenoid_pwm(0);

            bool yield_detected = (p > 4.0f && dp_dt < 0.12f && d2p < -0.3f);
            bool limit_reached = (p >= target_p_limit);

            if (yield_detected || limit_reached) {
                condPeakPressures[condCurrentCycle - 1] = p;
                condYieldPressures[condCurrentCycle - 1] = yield_detected ? p : target_p_limit;
                pumpMotor.setTargetPWM(0);
                condPhase = COND_RELAX_HOLD;
                condPhaseStartUs = now;
                ESP_LOGI(TAG, "Conditioning Cycle %d Peak: %.2f kPa. Relaxing...", condCurrentCycle, p);
            }
            break;
        }

        case COND_RELAX_HOLD: {
            pumpMotor.setTargetPWM(0);
            write_solenoid_pwm(0);

            if (elapsed_sec >= 10.0f || (elapsed_sec >= 4.0f && std::fabs(dp_dt) < 0.02f)) {
                condPhase = COND_CONTROLLED_DEFLATE;
                condPhaseStartUs = now;
            }
            break;
        }

        case COND_CONTROLLED_DEFLATE: {
            pumpMotor.setTargetPWM(0);
            write_solenoid_pwm(140);

            if (p <= 2.0f || elapsed_sec >= 15.0f) {
                write_solenoid_pwm(0);
                condPhase = COND_ANALYZE_LOOP;
                condPhaseStartUs = now;
            }
            break;
        }

        case COND_ANALYZE_LOOP: {
            write_solenoid_pwm(0);
            float cycle1_p = condYieldPressures[0];
            float curr_p = condYieldPressures[condCurrentCycle - 1];
            if (cycle1_p > 0.01f) condSofteningPct = (1.0f - (curr_p / cycle1_p)) * 100.0f;

            if (condCurrentCycle >= 2) {
                float prev_p = condYieldPressures[condCurrentCycle - 2];
                if (prev_p > 0.01f) condConvergencePct = (std::fabs(curr_p - prev_p) / prev_p) * 100.0f;
            } else {
                condConvergencePct = 99.0f;
            }

            ESP_LOGI(TAG, "Conditioning Cycle %d Analyzed -> Softening: %.1f%%, Loop Delta: %.2f%%",
                     condCurrentCycle, condSofteningPct, condConvergencePct);

            if ((condCurrentCycle >= 3 && condConvergencePct < 2.5f) || condCurrentCycle >= condMaxCycles) {
                condComplete = true;
                condPhase = COND_FINISHED;
                ESP_LOGI(TAG, "CONDITIONING COMPLETE! Softening: %.1f%%", condSofteningPct);
                post_mode_change(MODE_IDLE);
            } else {
                condCurrentCycle++;
                condPhase = COND_RAMP_INFLATE;
                condPhaseStartUs = now;
            }
            break;
        }

        case COND_FINISHED:
            pumpMotor.setTargetPWM(0);
            write_solenoid_pwm(0);
            break;
    }
}

// Idea #4: Pulse Breathing Mode with Hysteresis Banding
static void update_pulse() {
    double t_sec = (double)esp_timer_get_time() / 1000000.0;
    float sine_val = sinf(2.0f * M_PI * pulseFreqHz * (float)t_sec);
    float dynamic_target = pulseBasePressure + (pulseAmplitude * sine_val);

    float currentPressure = pressureSensor.getValue();
    float error = dynamic_target - currentPressure;

    int64_t now = esp_timer_get_time();
    if (pid_lastTime == 0) pid_lastTime = now - 50000;
    float dt = (float)(now - pid_lastTime) / 1000000.0f;
    if (dt <= 0.001f) dt = 0.001f;
    pid_lastTime = now;

    pid_integral += error * dt;
    pid_integral = std::clamp(pid_integral, -30.0f, 30.0f);

    float output = (pid_Kff * dynamic_target) + (pid_Kp * error) + (pid_Ki * pid_integral);
    int targetPWM = std::clamp((int)output, 0, 255);

    // Idea #4: Hysteresis deadband valve control to stop chattering
    if (currentPressure > dynamic_target + pid_hysteresis_band + 1.5f) {
        write_solenoid_pwm(150);
    } else if (currentPressure < dynamic_target + pid_hysteresis_band) {
        write_solenoid_pwm(0);
    }

    pumpMotor.setTargetPWM(targetPWM);
}

static void update_pattern() {
    double t_sec = (double)esp_timer_get_time() / 1000000.0;
    float phase = fmod(t_sec, (double)patternPeriodSec) / patternPeriodSec;
    float target_p = patternMinP;

    switch (currentPattern) {
        case PATTERN_SINE:
            target_p = patternMinP + ((patternMaxP - patternMinP) * 0.5f * (1.0f + sinf(2.0f * M_PI * phase)));
            break;
        case PATTERN_STAIRS:
            {
                int steps = 4;
                int current_step = (int)(phase * steps);
                target_p = patternMinP + ((patternMaxP - patternMinP) * ((float)current_step / (float)(steps - 1)));
            }
            break;
        case PATTERN_TRIANGLE:
            if (phase < 0.5f) {
                target_p = patternMinP + ((patternMaxP - patternMinP) * (phase * 2.0f));
            } else {
                target_p = patternMaxP - ((patternMaxP - patternMinP) * ((phase - 0.5f) * 2.0f));
            }
            break;
        case PATTERN_CRESCENDO:
            {
                float pulse = sinf(2.0f * M_PI * phase * 5.0f);
                target_p = patternMinP + ((patternMaxP - patternMinP) * phase) + (2.0f * pulse);
            }
            break;
    }

    float currentPressure = pressureSensor.getValue();
    float error = target_p - currentPressure;
    float output = (pid_Kff * target_p) + (pid_Kp * error);
    pumpMotor.setTargetPWM(std::clamp((int)output, 0, 255));
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
    float p = pressureSensor.getValue();
    add_sample(p);

    if (burstPopped) {
        if (pumpMotor.getCurrentPWM() > 0) pumpMotor.emergencyStop();
        return;
    }
    
    if (p > burstPeak) burstPeak = p;
    
    pumpMotor.setTargetPWM(255);
    
    if (burstPeak > 10.0f && p < burstPeak * 0.8f && postPopHoldCountdown == 0 && !safePopDataReady) {
        pumpMotor.emergencyStop();
        burstPopped = true;
        postPopHoldCountdown = 1000;
        ESP_LOGI(TAG, "POP DETECTED! Peak: %.2f kPa. Recording 1000 post-pop samples before safe memcpy...", burstPeak);
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
        case MODE_PULSE:
            update_pulse();
            break;
        case MODE_PATTERN:
            update_pattern();
            break;
        case MODE_DIAMETER:
            update_diameter();
            break;
        case MODE_RIDE:
            update_ride();
            break;
        case MODE_CONDITION:
            update_conditioning();
            break;
        case MODE_CALIBRATION:
            break;
        case MODE_ERROR:
            pumpMotor.emergencyStop();
            break;
    }
}

float update_scurve_profile(float current_val, float target_val, float max_vel, float dt_sec) {
    if (dt_sec <= 0.0f) return target_val;
    float error = target_val - current_val;
    float step = max_vel * dt_sec;
    if (std::abs(error) <= step) return target_val;
    return current_val + ((error > 0.0f) ? step : -step);
}


