#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include "bip_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "BIP_BALLOON_PHYS";

static BalloonType currentPreset = BALLOON_11INCH;
static float initialDiameterCm = 7.0f; // Default 11" uninflated neck diameter
static float maxSafeStretchLambda = 5.5f;

// Mooney-Rivlin Hyperelastic Base Coefficients for Latex Rubber (at 25°C)
static const float BASE_C10 = 180.0f; // kPa
static const float BASE_C01 = 20.0f;  // kPa

// Material geometry constants
static const float INITIAL_WALL_THICKNESS_M = 0.00035f; // 0.35 mm uninflated thickness

// Idea #1: Standard Linear Solid (SLS) Viscoelastic Model
static const float E_r = 80.0f;     // Viscous modulus (kPa)
static const float TAU_R = 1.2f;    // Relaxation time constant (seconds)
static float viscous_stress_kpa = 0.0f;
static float s_prev_lambda = 1.0f;
static bool s_first_physics_step = true;

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

// Forward declaration of RLS reset variables
static float rls_sum_w = 0.0f;
static float rls_sum_x = 0.0f;
static float rls_sum_y = 0.0f;
static float rls_sum_xy = 0.0f;
static float rls_sum_xx = 0.0f;
static float rls_sum_yy = 0.0f;
static int rls_sample_count = 0;

static BalloonMaterialPhysics currentPhysics = {};
static portMUX_TYPE s_physics_mux = portMUX_INITIALIZER_UNLOCKED;

void init_balloon_physics(BalloonType type) {
    set_balloon_preset(type);
    portENTER_CRITICAL(&s_physics_mux);
    currentPhysics = {};
    viscous_stress_kpa = 0.0f;
    s_prev_lambda = 1.0f;
    s_first_physics_step = true;
    rls_sum_w = 0.0f;
    rls_sum_x = 0.0f;
    rls_sum_y = 0.0f;
    rls_sum_xy = 0.0f;
    rls_sum_xx = 0.0f;
    rls_sum_yy = 0.0f;
    rls_sample_count = 0;
    portEXIT_CRITICAL(&s_physics_mux);
    current_impact = IMPACT_NONE;
    positive_spike_peak = 0.0f;
    positive_spike_time = 0;
}

void reset_fatigue_tracker() {
    portENTER_CRITICAL(&s_physics_mux);
    fatigue = {};
    fatigue.safe_pressure_remaining_pct = 100.0f;
    portEXIT_CRITICAL(&s_physics_mux);
    ESP_LOGI(TAG, "Cyclic Fatigue Tracker reset.");
}

FatigueState get_fatigue_state() {
    FatigueState out;
    portENTER_CRITICAL(&s_physics_mux);
    out = fatigue;
    portEXIT_CRITICAL(&s_physics_mux);
    return out;
}

void record_bounce_cycle(float peak_stress_kpa) {
    if (peak_stress_kpa <= 1.0f) return;

    // ALG-MR-03: Convert stress from kPa to MPa (Basquin curve calibrated in MPa)
    float peak_stress_mpa = peak_stress_kpa * 0.001f;
    if (peak_stress_mpa < 0.01f) peak_stress_mpa = 0.01f;

    portENTER_CRITICAL(&s_physics_mux);
    fatigue.total_bounce_count++;
    if (peak_stress_kpa > fatigue.max_bounce_stress_kpa) {
        fatigue.max_bounce_stress_kpa = peak_stress_kpa;
    }

    // Miner's Rule: N_f = K / σ_MPa^m
    float Nf = FATIGUE_K / std::pow(peak_stress_mpa, FATIGUE_M);
    if (Nf < 1.0f) Nf = 1.0f;
    fatigue.accumulated_damage += (1.0f / Nf);
    fatigue.accumulated_damage = std::clamp(fatigue.accumulated_damage, 0.0f, 1.0f);
    fatigue.safe_pressure_remaining_pct = (1.0f - fatigue.accumulated_damage) * 100.0f;
    portEXIT_CRITICAL(&s_physics_mux);

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

// Online Mooney-Rivlin Parameter Identification RLS State
static OnlineMaterialParams onlineParams = { .estimated_C10 = 180.0f, .estimated_C01 = 20.0f, .r_squared = 0.0f, .converged = false };

// Micro-Flutter Tear Precursor Detector State
static float flutter_window[16] = {0};
static int flutter_idx = 0;
static float current_flutter_variance = 0.0f;
static bool tear_precursor_detected = false;

void update_online_material_identification(float pressure_kpa, float stretch_ratio) {
    if (!std::isfinite(stretch_ratio) || !std::isfinite(pressure_kpa)) return;
    if (stretch_ratio <= 1.05f || pressure_kpa <= 2.0f) return;

    float lambda = std::clamp(stretch_ratio, 1.0f, 10.0f);
    float lambda_sq = lambda * lambda;
    float inv_lambda4 = 1.0f / (lambda_sq * lambda_sq);
    float denom = 2.0f * (lambda_sq - inv_lambda4);
    if (denom <= 0.001f) return;

    // Equibiaxial Cauchy stress from pressure: sigma = P * (D0 / 4*t0) * lambda^3
    float t0_cm = INITIAL_WALL_THICKNESS_M * 100.0f; // 0.035 cm
    float geom_ratio = initialDiameterCm / (4.0f * t0_cm);
    float sigma_meas = pressure_kpa * geom_ratio * (lambda_sq * lambda);

    // Linear regression y = C10 + C01 * x, where y = sigma_meas / denom, x = lambda^2
    float y = sigma_meas / denom;
    float x = lambda_sq;

    // Online Exponential Forgetting Factor (gamma = 0.995)
    const float gamma = 0.995f;
    rls_sum_w  = gamma * rls_sum_w  + 1.0f;
    rls_sum_x  = gamma * rls_sum_x  + x;
    rls_sum_y  = gamma * rls_sum_y  + y;
    rls_sum_xy = gamma * rls_sum_xy + (x * y);
    rls_sum_xx = gamma * rls_sum_xx + (x * x);
    rls_sum_yy = gamma * rls_sum_yy + (y * y);
    rls_sample_count++;

    // Guard: Require sufficient history and persistent excitation in stretch ratio
    if (rls_sample_count >= 50 && rls_sum_w >= 30.0f) {
        float w = rls_sum_w;
        float denominator = (w * rls_sum_xx - rls_sum_x * rls_sum_x);
        float var_x = denominator / (w * w);

        // Persistent excitation guard: stretch variance must be non-zero
        if (var_x > 0.005f) {
            float num_xy = (w * rls_sum_xy - rls_sum_x * rls_sum_y);
            float slope_C01 = num_xy / denominator;
            float intercept_C10 = (rls_sum_y - slope_C01 * rls_sum_x) / w;

            if (intercept_C10 > 50.0f && intercept_C10 < 400.0f && slope_C01 >= 0.0f && slope_C01 < 100.0f) {
                float denom_y = (w * rls_sum_yy - rls_sum_y * rls_sum_y);
                float r2 = 0.0f;
                if (denom_y > 1e-4f) {
                    r2 = (num_xy * num_xy) / (denominator * denom_y);
                }

                portENTER_CRITICAL(&s_physics_mux);
                onlineParams.estimated_C10 = 0.95f * onlineParams.estimated_C10 + 0.05f * intercept_C10;
                onlineParams.estimated_C01 = 0.95f * onlineParams.estimated_C01 + 0.05f * slope_C01;
                onlineParams.converged = true;
                onlineParams.r_squared = std::clamp(r2, 0.0f, 1.0f);
                portEXIT_CRITICAL(&s_physics_mux);
            }
        }
    }
}

OnlineMaterialParams get_online_material_params() {
    OnlineMaterialParams out;
    portENTER_CRITICAL(&s_physics_mux);
    out = onlineParams;
    portEXIT_CRITICAL(&s_physics_mux);
    return out;
}

bool is_tear_precursor_flutter_detected() {
    return tear_precursor_detected;
}

float get_flutter_variance() {
    return current_flutter_variance;
}

void update_balloon_physics_ext(float pressure_kpa, float dp_dt, float d2p_dt2, float mcu_temp_c, float dt_sec) {
    if (dt_sec <= 0.0f) dt_sec = 0.0005f;

    BalloonMaterialPhysics local_phys = {};
    BalloonPhysicsEstimate volEst = get_balloon_physics_estimate();

    // 1. Calculate Stretch Ratio Lambda = D / D0 with strict bounds
    float safe_d0 = (initialDiameterCm >= 1.0f) ? initialDiameterCm : 7.0f;
    float lambda = volEst.diameter_cm / safe_d0;
    
    // Numerical Guard: NaN check and physical stretch boundaries [1.0, 10.0]
    if (!std::isfinite(lambda) || lambda < 1.0f) {
        lambda = 1.0f;
    } else if (lambda > 10.0f) {
        lambda = 10.0f;
    }
    local_phys.stretch_ratio = lambda;

    // Idea #8: Thermal Softening Adjustment for Latex
    float ambient_est_c = std::clamp(mcu_temp_c - 15.0f, 15.0f, 55.0f);
    float delta_T = ambient_est_c - 25.0f;
    float thermal_softening = std::clamp(1.0f - 0.005f * delta_T, 0.65f, 1.05f);

    // Online Material Estimation Update & Mooney-Rivlin Wall Stress
    update_online_material_identification(pressure_kpa, lambda);
    float active_C10 = onlineParams.converged ? onlineParams.estimated_C10 : BASE_C10;
    float active_C01 = onlineParams.converged ? onlineParams.estimated_C01 : BASE_C01;

    float effective_C10 = active_C10 * thermal_softening;
    float effective_C01 = active_C01 * thermal_softening;

    // High-Frequency Micro-Flutter Tear Precursor Detection
    flutter_window[flutter_idx] = dp_dt;
    flutter_idx = (flutter_idx + 1) % 16;
    float mean_flutter = 0.0f;
    for (int i = 0; i < 16; i++) mean_flutter += flutter_window[i];
    mean_flutter /= 16.0f;

    float var_sum = 0.0f;
    for (int i = 0; i < 16; i++) {
        float diff = flutter_window[i] - mean_flutter;
        var_sum += diff * diff;
    }
    current_flutter_variance = var_sum / 16.0f;
    tear_precursor_detected = (pressure_kpa > 15.0f && current_flutter_variance > 120.0f);

    // 2. Mooney-Rivlin Equibiaxial Spherical Membrane Model (ALG-MR-01)
    float lambda_sq = lambda * lambda;
    float inv_lambda_sq = 1.0f / lambda_sq;
    float inv_lambda_4 = inv_lambda_sq * inv_lambda_sq;
    float hyper_stress = 2.0f * (lambda_sq - inv_lambda_4) * (effective_C10 + effective_C01 * lambda_sq);
    if (hyper_stress < 0.0f) hyper_stress = 0.0f;

    // Idea #1: Unconditionally Stable SLS Viscoelastic Exponential Integrator
    float d_lambda = 0.0f;
    if (s_first_physics_step) {
        s_prev_lambda = lambda;
        s_first_physics_step = false;
    } else {
        d_lambda = lambda - s_prev_lambda;
        s_prev_lambda = lambda;
    }

    float safe_dt = std::clamp(dt_sec, 0.0001f, 1.0f);
    float alpha = std::exp(-safe_dt / TAU_R);
    viscous_stress_kpa = (viscous_stress_kpa * alpha) + (E_r * d_lambda);
    if (viscous_stress_kpa < 0.0f) viscous_stress_kpa = 0.0f;

    local_phys.viscous_stress_kpa = viscous_stress_kpa;
    local_phys.hyperelastic_stress_kpa = hyper_stress + viscous_stress_kpa;

    // 3. Strain Energy Density W (Joules) & Constant Rubber Volume Integration (ALG-MR-02)
    float I1 = 2.0f * lambda_sq + inv_lambda_4;
    float I2 = (lambda_sq * lambda_sq) + 2.0f * inv_lambda_sq;
    float W_density = effective_C10 * (I1 - 3.0f) + effective_C01 * (I2 - 3.0f);
    if (W_density < 0.0f) W_density = 0.0f;

    float r0_m = (initialDiameterCm * 0.01f) * 0.5f;
    float v_rubber_m3 = 4.0f * (float)M_PI * r0_m * r0_m * INITIAL_WALL_THICKNESS_M;
    local_phys.strain_energy_j = (W_density * 1000.0f) * v_rubber_m3;

    // Idea #10: Fatigue Derating on Max Safe Stretch
    float fatigue_derating = 1.0f - (fatigue.accumulated_damage * 0.30f);
    float effective_max_lambda = maxSafeStretchLambda * fatigue_derating;
    local_phys.yield_probability = std::clamp(lambda / effective_max_lambda, 0.0f, 1.0f);

    // Idea #9: Impact Event Classification & Shock Filter
    current_impact = classify_pressure_event(dp_dt, d2p_dt2, pressure_kpa);
    local_phys.impact_type = current_impact;

    if (current_impact == IMPACT_BURST || tear_precursor_detected) {
        local_phys.burst_shock_detected = true;
        if (tear_precursor_detected) {
            ESP_LOGW(TAG, "MICRO-FLUTTER TEAR PRECURSOR DETECTED! Var: %.1f", current_flutter_variance);
        } else {
            ESP_LOGW(TAG, "BURST SHOCK CONFIRMED! dP/dt = %.2f kPa/s at P = %.2f kPa", dp_dt, pressure_kpa);
        }
    } else {
        local_phys.burst_shock_detected = false;
    }

    portENTER_CRITICAL(&s_physics_mux);
    currentPhysics = local_phys;
    portEXIT_CRITICAL(&s_physics_mux);
}


BalloonMaterialPhysics get_balloon_physics_state() {
    BalloonMaterialPhysics out;
    portENTER_CRITICAL(&s_physics_mux);
    out = currentPhysics;
    portEXIT_CRITICAL(&s_physics_mux);
    return out;
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

    float maxLambda_sq = maxLambda * maxLambda;
    float inv_maxLambda4 = 1.0f / (maxLambda_sq * maxLambda_sq);
    float burst_stress = 2.0f * (maxLambda_sq - inv_maxLambda4) * (BASE_C10 + BASE_C01 * maxLambda_sq);
    float working_lambda_sq = working_lambda * working_lambda;
    float t_wall_mm = 0.35f / working_lambda_sq;
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

    PopBlackBoxSample prev = get_pop_sample(0);

    for (int i = 1; i < total_samples; i++) {
        PopBlackBoxSample curr = get_pop_sample(i);
        // FW-PERF-01: Only compute derivative for negative pressure drops
        if (curr.pressure < prev.pressure) {
            uint32_t dt_us = curr.timestamp_us - prev.timestamp_us;
            if (dt_us > 0) {
                float dpdt = (curr.pressure - prev.pressure) * 1000000.0f / (float)dt_us;
                if (dpdt < min_dpdt) min_dpdt = dpdt;
            }
        }
        prev = curr;
    }

    if (min_dpdt < -20.0f) {
        float new_threshold = min_dpdt * 0.80f; // 20% safety margin
        learned_burst_threshold = 0.70f * learned_burst_threshold + 0.30f * new_threshold;
        ESP_LOGI(TAG, "Adaptive Burst Threshold Updated: %.2f kPa/s (Pop Min dP/dt: %.2f)",
                 learned_burst_threshold, min_dpdt);
    }
}

