#include "bip_volume_estimator.h"
#include <cmath>
#include <algorithm>

static float initial_diameter_cm = 5.0f; // Default uninflated balloon diameter
static float cumulative_volume_liters = 0.0f;

// Pump flow rate model: 100% PWM = 15 Liters/min (0.25 L/sec) at zero backpressure
static const float MAX_PUMP_FLOW_LPS = 0.25f;

// Idea #5: Calibrated Pump Deadzone & Non-linear Stall Backpressure Model
static float pwm_deadzone = 35.0f;       // Motor friction deadzone PWM
static float stall_pressure_kpa = 72.0f;  // Pump maximum stall pressure (kPa)

// Idea #2: Non-Spherical Shape Correction & Neck Offset
static float neck_volume_liters = 0.05f;  // Initial 50 mL neck extension volume
static float oblate_shape_factor = 0.85f; // Ratio polar/equatorial radius (0.85 for standard balloon)

void init_volume_estimator(float uninflated_diameter_cm) {
    initial_diameter_cm = std::max(1.0f, uninflated_diameter_cm);
    cumulative_volume_liters = 0.0f;
}

void set_pump_model_params(float deadzone_pwm, float stall_p_kpa) {
    pwm_deadzone = std::clamp(deadzone_pwm, 0.0f, 150.0f);
    stall_pressure_kpa = std::max(10.0f, stall_p_kpa);
}

void set_balloon_shape_params(float neck_vol_l, float shape_factor) {
    neck_volume_liters = std::max(0.0f, neck_vol_l);
    oblate_shape_factor = std::clamp(shape_factor, 0.5f, 1.2f);
}

void update_volume_estimator(float current_pressure_kpa, float motor_pwm, float dt_sec) {
    update_volume_estimator_ext(current_pressure_kpa, motor_pwm, 25.0f, dt_sec);
}

void update_volume_estimator_ext(float current_pressure_kpa, float motor_pwm, float mcu_temp_c, float dt_sec) {
    if (dt_sec <= 0.0f) return;

    // Idea #5: Hammerstein Non-linear Motor & Stall Model
    float duty_effective = 0.0f;
    if (motor_pwm > pwm_deadzone) {
        duty_effective = (motor_pwm - pwm_deadzone) / (255.0f - pwm_deadzone);
    }
    float flow_duty_nl = std::pow(std::clamp(duty_effective, 0.0f, 1.0f), 1.25f);
    
    // Backpressure non-linear attenuation curve
    float p_ratio = std::clamp(current_pressure_kpa / stall_pressure_kpa, 0.0f, 1.0f);
    float backpressure_factor = std::max(0.05f, 1.0f - std::pow(p_ratio, 0.75f));

    float raw_flow_lps = MAX_PUMP_FLOW_LPS * flow_duty_nl * backpressure_factor;

    // Idea #3: Thermodynamic Gas Law PV=nRT Air Mass Correction
    // Temperature: (T_ref_K / T_amb_K), Pressure: (P_ref / (P_ref + P_gauge))
    float ambient_temp_c = std::clamp(mcu_temp_c - 15.0f, 10.0f, 50.0f); // Ambient estimation
    float temp_correction = (25.0f + 273.15f) / (ambient_temp_c + 273.15f);
    float p_abs_kpa = 101.325f + std::max(0.0f, current_pressure_kpa);
    float pressure_correction = 101.325f / p_abs_kpa;

    float corrected_flow_lps = raw_flow_lps * temp_correction * pressure_correction;
    cumulative_volume_liters += corrected_flow_lps * dt_sec;
}

BalloonPhysicsEstimate get_balloon_physics_estimate() {
    BalloonPhysicsEstimate est = {};
    est.volume_liters = cumulative_volume_liters;

    // Idea #2: Neck Volume Subtraction & Oblate Spheroid Radius Calculation
    float body_volume_l = std::max(0.0f, cumulative_volume_liters - neck_volume_liters);
    float vol_cm3 = body_volume_l * 1000.0f;
    
    // Oblate Spheroid Volume: V = (4/3) * pi * a^2 * c where c = e_shape * a
    // V = (4/3) * pi * e_shape * a^3  => a = (3V / (4*pi*e_shape))^(1/3)
    float radius_equatorial_cm = std::cbrtf((3.0f * vol_cm3) / (4.0f * M_PI * oblate_shape_factor));
    est.diameter_cm = radius_equatorial_cm * 2.0f;
    if (est.diameter_cm < initial_diameter_cm) est.diameter_cm = initial_diameter_cm;

    // Stretch Strain (%): ((D - D0) / D0) * 100
    est.stretch_strain_pct = ((est.diameter_cm - initial_diameter_cm) / initial_diameter_cm) * 100.0f;

    // Hoop stress estimate: sigma = (P * r) / (2 * t)
    est.wall_stress_kpa = est.diameter_cm * 2.5f;

    return est;
}

