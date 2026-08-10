#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include "bip_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "BIP_BALLOON_PHYS";

static BalloonType currentPreset = BALLOON_11INCH;
static float initialDiameterCm = 7.0f; // Default 11" uninflated neck diameter
static float maxSafeStretchLambda = 5.5f;

// Mooney-Rivlin Hyperelastic Base Coefficients for Latex Rubber (at 25°C)
static const float BASE_C10 = 180.0f; // kPa
static const float BASE_C01 = 20.0f;  // kPa

// Idea #1: Standard Linear Solid (SLS) Viscoelastic Model
static const float E_r = 80.0f;     // Viscous modulus (kPa)
static const float TAU_R = 1.2f;    // Relaxation time constant (seconds)
static float viscous_stress_kpa = 0.0f;

// Idea #7: Adaptive Burst Threshold Learning
static float learned_burst_threshold = -40.0f; // Initial threshold (kPa/s)

// Idea #9: Bounce Classifier State
static ImpactClassification current_impact = IMPACT_NONE;
static float positive_spike_peak = 0.0f;
static int64_t positive_spike_time = 0;

// Idea #10: Miner's Rule Cyclic Fatigue State
static FatigueState fatigue = {};
static const float FATIGUE_K = 1.0e8f;
static const float FATIGUE_M = 3.5f;

static BalloonMaterialPhysics currentPhysics = {};

void init_balloon_physics(BalloonType type) {
    set_balloon_preset(type);
    currentPhysics = {};
    viscous_stress_kpa = 0.0f;
    current_impact = IMPACT_NONE;
    positive_spike_peak = 0.0f;
    positive_spike_time = 0;
}

void reset_fatigue_tracker() {
    fatigue = {};
    fatigue.safe_pressure_remaining_pct = 100.0f;
    ESP_LOGI(TAG, "Cyclic Fatigue Tracker reset.");
}

FatigueState get_fatigue_state() {
    return fatigue;
}

void record_bounce_cycle(float peak_stress_kpa) {
    if (peak_stress_kpa <= 1.0f) return;
    fatigue.total_bounce_count++;
    if (peak_stress_kpa > fatigue.max_bounce_stress_kpa) {
        fatigue.max_bounce_stress_kpa = peak_stress_kpa;
    }

    // Miner's Rule: N_f = K / σ^m
    float Nf = FATIGUE_K / std::pow(peak_stress_kpa, FATIGUE_M);
    if (Nf < 1.0f) Nf = 1.0f;
    fatigue.accumulated_damage += (1.0f / Nf);
    fatigue.accumulated_damage = std::clamp(fatigue.accumulated_damage, 0.0f, 1.0f);
    fatigue.safe_pressure_remaining_pct = (1.0f - fatigue.accumulated_damage) * 100.0f;

    if (fatigue.accumulated_damage > 0.70f) {
        ESP_LOGW(TAG, "FATIGUE WARNING: D=%.3f (%lu bounces). Capacity down to %.1f%%",
                 fatigue.accumulated_damage, (unsigned long)fatigue.total_bounce_count,
                 fatigue.safe_pressure_remaining_pct);
    }
}

void set_balloon_preset(BalloonType type) {
    currentPreset = type;
    switch (type) {
        case BALLOON_5INCH:
            initialDiameterCm = 3.5f;
            maxSafeStretchLambda = 4.2f;
            break;
        case BALLOON_11INCH:
            initialDiameterCm = 7.0f;
            maxSafeStretchLambda = 5.5f;
            break;
        case BALLOON_16INCH:
            initialDiameterCm = 10.0f;
            maxSafeStretchLambda = 6.0f;
            break;
        case BALLOON_36INCH:
            initialDiameterCm = 20.0f;
            maxSafeStretchLambda = 7.5f;
            break;
    }
    init_volume_estimator(initialDiameterCm);
    ESP_LOGI(TAG, "Balloon Preset updated: Type %d (D0: %.1f cm, Max Lambda: %.1f)",
             (int)type, initialDiameterCm, maxSafeStretchLambda);
}

// Idea #9: Impact Event Classifier (Distinguishes Bounces from Pop Bursts)
ImpactClassification classify_pressure_event(float dp_dt, float d2p_dt2, float pressure_kpa) {
    int64_t now = esp_timer_get_time();

    // 1. Detect positive compression spike (body landing on balloon)
    if (dp_dt > 50.0f) {
        positive_spike_peak = dp_dt;
        positive_spike_time = now;
        return IMPACT_BOUNCE;
    }

    // 2. If negative rebound occurs within 350ms of positive spike -> Confirm BOUNCE
    if (dp_dt < -25.0f && positive_spike_peak > 40.0f) {
        int64_t elapsed_us = now - positive_spike_time;
        if (elapsed_us > 10000 && elapsed_us < 350000) {
            positive_spike_peak = 0.0f;
            return IMPACT_BOUNCE;
        }
    }

    // 3. True BURST: Negative shock without recent positive bounce spike, pressure collapsing < 3.0 kPa
    if (dp_dt < learned_burst_threshold && pressure_kpa < 4.0f && positive_spike_peak < 30.0f) {
        return IMPACT_BURST;
    }

    // 4. Slow Squeeze (leaning/kneading)
    if (dp_dt > 5.0f && dp_dt <= 50.0f && d2p_dt2 < 1.0f) {
        return IMPACT_SQUEEZE;
    }

    return IMPACT_NONE;
}

ImpactClassification get_last_impact() {
    return current_impact;
}

void update_balloon_physics(float pressure_kpa, float dp_dt, float dt_sec) {
    update_balloon_physics_ext(pressure_kpa, dp_dt, 0.0f, 30.0f, dt_sec);
}

void update_balloon_physics_ext(float pressure_kpa, float dp_dt, float d2p_dt2, float mcu_temp_c, float dt_sec) {
    if (dt_sec <= 0.0f) dt_sec = 0.0005f;

    BalloonPhysicsEstimate volEst = get_balloon_physics_estimate();

    // 1. Calculate Stretch Ratio Lambda = D / D0
    float lambda = volEst.diameter_cm / initialDiameterCm;
    if (lambda < 1.0f) lambda = 1.0f;
    currentPhysics.stretch_ratio = lambda;

    // Idea #8: Thermal Softening Adjustment for Latex
    float ambient_est_c = std::clamp(mcu_temp_c - 15.0f, 15.0f, 55.0f);
    float delta_T = ambient_est_c - 25.0f;
    float thermal_softening = std::clamp(1.0f - 0.005f * delta_T, 0.65f, 1.05f);

    float effective_C10 = BASE_C10 * thermal_softening;
    float effective_C01 = BASE_C01 * thermal_softening;

    // 2. Mooney-Rivlin 2-Parameter Hyperelastic Stress Model
    float lambda_sq = lambda * lambda;
    float inv_lambda = 1.0f / lambda;
    float hyper_stress = 2.0f * (lambda_sq - inv_lambda) * (effective_C10 + effective_C01 * inv_lambda);

    // Idea #1: SLS Viscoelastic Stress Relaxation ODE Integration
    static float prev_lambda = 1.0f;
    float d_lambda_dt = (lambda - prev_lambda) / dt_sec;
    prev_lambda = lambda;

    float d_viscous = (E_r * d_lambda_dt - (viscous_stress_kpa / TAU_R)) * dt_sec;
    viscous_stress_kpa += d_viscous;
    if (viscous_stress_kpa < 0.0f) viscous_stress_kpa = 0.0f;

    currentPhysics.viscous_stress_kpa = viscous_stress_kpa;
    currentPhysics.hyperelastic_stress_kpa = hyper_stress + viscous_stress_kpa;

    // 3. Strain Energy Density W (Joules)
    float I1 = lambda_sq + 2.0f * inv_lambda;
    float I2 = 2.0f * lambda + (1.0f / lambda_sq);
    float W_density = effective_C10 * (I1 - 3.0f) + effective_C01 * (I2 - 3.0f);
    currentPhysics.strain_energy_j = W_density * (volEst.volume_liters * 0.001f);

    // Idea #10: Fatigue Derating on Max Safe Stretch
    float fatigue_derating = 1.0f - (fatigue.accumulated_damage * 0.30f);
    float effective_max_lambda = maxSafeStretchLambda * fatigue_derating;
    currentPhysics.yield_probability = std::clamp(lambda / effective_max_lambda, 0.0f, 1.0f);

    // Idea #9: Impact Event Classification & Shock Filter
    current_impact = classify_pressure_event(dp_dt, d2p_dt2, pressure_kpa);
    currentPhysics.impact_type = current_impact;

    if (current_impact == IMPACT_BURST) {
        currentPhysics.burst_shock_detected = true;
        ESP_LOGW(TAG, "BURST SHOCK CONFIRMED! dP/dt = %.2f kPa/s at P = %.2f kPa", dp_dt, pressure_kpa);
    } else {
        currentPhysics.burst_shock_detected = false;
    }
}

BalloonMaterialPhysics get_balloon_physics_state() {
    return currentPhysics;
}

// Idea #13: Pre-Ride Weight-to-Pressure Optimal Inflation Calculator
RideInflationAdvice compute_ride_inflation(float rider_kg, BalloonType type, float firmness_0to1) {
    RideInflationAdvice advice = {};

    float D0_cm = 7.0f;
    float maxLambda = 5.5f;
    switch (type) {
        case BALLOON_5INCH:  D0_cm = 3.5f; maxLambda = 4.2f; break;
        case BALLOON_11INCH: D0_cm = 7.0f; maxLambda = 5.5f; break;
        case BALLOON_16INCH: D0_cm = 10.0f; maxLambda = 6.0f; break;
        case BALLOON_36INCH: D0_cm = 20.0f; maxLambda = 7.5f; break;
    }

    float working_lambda = 1.0f + (maxLambda - 1.0f) * 0.70f;
    float D_inflated_cm = D0_cm * working_lambda;
    float D_m = D_inflated_cm / 100.0f;

    float F_N = rider_kg * 9.81f;
    float target_sink_m = 0.04f + 0.10f * (1.0f - std::clamp(firmness_0to1, 0.0f, 1.0f));
    float r_contact_m = std::sqrt(target_sink_m * D_m);
    if (r_contact_m < 0.02f) r_contact_m = 0.02f;

    float delta_P_body_kpa = (F_N / (M_PI * r_contact_m * r_contact_m)) / 1000.0f;

    float burst_stress = 2.0f * (maxLambda * maxLambda - 1.0f / maxLambda) * (BASE_C10 + BASE_C01 / maxLambda);
    float t_wall_mm = 0.35f / working_lambda;
    float p_burst_kpa = 2.0f * (t_wall_mm / 1000.0f) * burst_stress / (D_m / 2.0f);

    float safety_factor = 0.65f;
    advice.max_safe_pressure_kpa = p_burst_kpa * safety_factor;
    advice.recommended_pressure_kpa = std::max(3.0f, advice.max_safe_pressure_kpa - delta_P_body_kpa);
    advice.predicted_sink_depth_cm = target_sink_m * 100.0f;
    advice.burst_safety_margin_pct = ((p_burst_kpa - advice.recommended_pressure_kpa - delta_P_body_kpa) / p_burst_kpa) * 100.0f;

    return advice;
}

// Idea #7: Adaptive Burst Threshold Learning
void set_learned_burst_threshold(float threshold) {
    if (threshold < -10.0f) learned_burst_threshold = threshold;
}

float get_learned_burst_threshold() {
    return learned_burst_threshold;
}

void update_burst_threshold_from_pop() {
    if (!is_pop_recorded()) return;

    int total_samples = get_pop_sample_count();
    float min_dpdt = 0.0f;

    for (int i = 1; i < total_samples; i++) {
        PopBlackBoxSample s1 = get_pop_sample(i - 1);
        PopBlackBoxSample s2 = get_pop_sample(i);
        uint32_t dt_us = s2.timestamp_us - s1.timestamp_us;
        if (dt_us > 0) {
            float dt_sec = (float)dt_us / 1000000.0f;
            float dpdt = (s2.pressure - s1.pressure) / dt_sec;
            if (dpdt < min_dpdt) min_dpdt = dpdt;
        }
    }

    if (min_dpdt < -20.0f) {
        float new_threshold = min_dpdt * 0.80f; // 20% safety margin
        learned_burst_threshold = 0.70f * learned_burst_threshold + 0.30f * new_threshold;
        ESP_LOGI(TAG, "Adaptive Burst Threshold Updated: %.2f kPa/s (Pop Min dP/dt: %.2f)",
                 learned_burst_threshold, min_dpdt);
    }
}

