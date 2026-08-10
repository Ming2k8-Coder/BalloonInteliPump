#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "BIP_BALLOON_PHYS";

static BalloonType currentPreset = BALLOON_11INCH;
static float initialDiameterCm = 7.0f; // Default 11" uninflated neck diameter
static float maxSafeStretchLambda = 5.5f;

// Mooney-Rivlin Hyperelastic Coefficients for Latex Rubber
static const float C10 = 180.0f; // kPa
static const float C01 = 20.0f;  // kPa

static BalloonMaterialPhysics currentPhysics = {};

void init_balloon_physics(BalloonType type) {
    set_balloon_preset(type);
    currentPhysics = {};
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

void update_balloon_physics(float pressure_kpa, float dp_dt, float dt_sec) {
    BalloonPhysicsEstimate volEst = get_balloon_physics_estimate();

    // 1. Calculate Stretch Ratio Lambda = D / D0
    float lambda = volEst.diameter_cm / initialDiameterCm;
    if (lambda < 1.0f) lambda = 1.0f;
    currentPhysics.stretch_ratio = lambda;

    // 2. Mooney-Rivlin 2-Parameter Hyperelastic Stress Model
    // Sigma = 2 * (lambda^2 - 1/lambda) * (C10 + C01/lambda)
    float lambda_sq = lambda * lambda;
    float inv_lambda = 1.0f / lambda;
    float hyper_stress = 2.0f * (lambda_sq - inv_lambda) * (C10 + C01 * inv_lambda);
    currentPhysics.hyperelastic_stress_kpa = hyper_stress;

    // 3. Strain Energy Density W = C10*(I1-3) + C01*(I2-3)
    float I1 = lambda_sq + 2.0f * inv_lambda;
    float I2 = 2.0f * lambda + (1.0f / lambda_sq);
    float W_density = C10 * (I1 - 3.0f) + C01 * (I2 - 3.0f);
    currentPhysics.strain_energy_j = W_density * (volEst.volume_liters * 0.001f); // Joules

    // 4. Material Yield Risk Factor (0.0 to 1.0)
    currentPhysics.yield_probability = std::clamp(lambda / maxSafeStretchLambda, 0.0f, 1.0f);

    // 5. Burst Shock Detection (Rapid pressure collapse: dP/dt < -40.0 kPa/s while pressure > 5kPa)
    if (pressure_kpa > 5.0f && dp_dt < -40.0f) {
        currentPhysics.burst_shock_detected = true;
        ESP_LOGW(TAG, "BURST SHOCK DETECTED! dP/dt = %.2f kPa/s at P = %.2f kPa", dp_dt, pressure_kpa);
    } else {
        currentPhysics.burst_shock_detected = false;
    }
}

BalloonMaterialPhysics get_balloon_physics_state() {
    return currentPhysics;
}
