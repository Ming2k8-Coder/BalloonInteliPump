#ifndef BIP_VOLUME_ESTIMATOR_H
#define BIP_VOLUME_ESTIMATOR_H

#include "bip_config.h"

typedef struct {
    float volume_liters;       // Estimated total volume delivered (L)
    float diameter_cm;         // Estimated balloon diameter (cm)
    float stretch_strain_pct;  // Material elongation strain (%)
    float wall_stress_kpa;     // Wall stress (kPa)
} BalloonPhysicsEstimate;

void init_volume_estimator(float uninflated_diameter_cm);
void update_volume_estimator(float current_pressure_kpa, float motor_pwm, float dt_sec);
BalloonPhysicsEstimate get_balloon_physics_estimate();

#endif // BIP_VOLUME_ESTIMATOR_H
