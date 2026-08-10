#ifndef BIP_BALLOON_PHYSICS_H
#define BIP_BALLOON_PHYSICS_H

#include "bip_config.h"

enum BalloonType {
    BALLOON_5INCH = 0,   // Standard 5" Latex Balloon
    BALLOON_11INCH = 1,  // Standard 11" Party Balloon
    BALLOON_16INCH = 2,  // Large 16" Balloon
    BALLOON_36INCH = 3   // Giant 3-foot Looner Balloon
};

typedef struct {
    float stretch_ratio;          // Stretch Lambda (D / D0)
    float hyperelastic_stress_kpa; // Mooney-Rivlin Wall Stress (kPa)
    float strain_energy_j;         // Hyperelastic Strain Energy (Joules)
    float yield_probability;       // Material rupture risk (0.0 to 1.0)
    bool  burst_shock_detected;   // Microsecond acoustic/pressure burst shock flag
} BalloonMaterialPhysics;

void init_balloon_physics(BalloonType type);
void update_balloon_physics(float pressure_kpa, float dp_dt, float dt_sec);
BalloonMaterialPhysics get_balloon_physics_state();
void set_balloon_preset(BalloonType type);

#endif // BIP_BALLOON_PHYSICS_H
