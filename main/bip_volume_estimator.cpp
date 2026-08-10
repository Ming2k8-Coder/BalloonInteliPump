#include "bip_volume_estimator.h"
#include <cmath>
#include <algorithm>

static float initial_diameter_cm = 5.0f; // Default uninflated balloon diameter
static float cumulative_volume_liters = 0.0f;

// Pump flow rate model: 100% PWM = 15 Liters/min (0.25 L/sec) at zero backpressure
static const float MAX_PUMP_FLOW_LPS = 0.25f;

void init_volume_estimator(float uninflated_diameter_cm) {
    initial_diameter_cm = std::max(1.0f, uninflated_diameter_cm);
    cumulative_volume_liters = 0.0f;
}

void update_volume_estimator(float current_pressure_kpa, float motor_pwm, float dt_sec) {
    if (dt_sec <= 0.0f) return;

    // Air flow rate decreases linearly with backpressure
    float duty_ratio = motor_pwm / 255.0f;
    float backpressure_factor = std::clamp(1.0f - (current_pressure_kpa / 70.0f), 0.05f, 1.0f);
    float current_flow_lps = MAX_PUMP_FLOW_LPS * duty_ratio * backpressure_factor;

    cumulative_volume_liters += current_flow_lps * dt_sec;
}

BalloonPhysicsEstimate get_balloon_physics_estimate() {
    BalloonPhysicsEstimate est = {};
    est.volume_liters = cumulative_volume_liters;

    // Convert Volume (L) -> Volume (cm3): 1 L = 1000 cm3
    float vol_cm3 = cumulative_volume_liters * 1000.0f;
    
    // Spherical volume: V = (4/3) * pi * r^3 -> r = (3V / (4*pi))^(1/3)
    float radius_cm = std::cbrtf((3.0f * vol_cm3) / (4.0f * M_PI));
    est.diameter_cm = radius_cm * 2.0f;
    if (est.diameter_cm < initial_diameter_cm) est.diameter_cm = initial_diameter_cm;

    // Stretch Strain (%): ((D - D0) / D0) * 100
    est.stretch_strain_pct = ((est.diameter_cm - initial_diameter_cm) / initial_diameter_cm) * 100.0f;

    // Hoop stress estimate: sigma = (P * r) / (2 * t)
    est.wall_stress_kpa = est.diameter_cm * 2.5f; // Simplified proportional stress estimate

    return est;
}
