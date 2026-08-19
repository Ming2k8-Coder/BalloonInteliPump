#ifndef BIP_BALLOON_PHYSICS_H
#define BIP_BALLOON_PHYSICS_H

#include "bip_config.h"

enum BalloonType {
    BALLOON_5INCH = 0,   // Standard 5" Latex Balloon
    BALLOON_11INCH = 1,  // Standard 11" Party Balloon
    BALLOON_16INCH = 2,  // Large 16" Balloon
    BALLOON_36INCH = 3   // Giant 3-foot Looner Balloon
};

typedef enum {
    IMPACT_NONE = 0,
    IMPACT_BOUNCE = 1,   // Compression + elastic rebound detected
    IMPACT_BURST = 2,    // Irreversible pop collapse
    IMPACT_SQUEEZE = 3   // Slow sustained compression (hand squeeze)
} ImpactClassification;

typedef struct {
    float stretch_ratio;          // Stretch Lambda (D / D0)
    float hyperelastic_stress_kpa; // Mooney-Rivlin Wall Stress (kPa)
    float viscous_stress_kpa;      // SLS Viscoelastic Stress (kPa)
    float strain_energy_j;         // Strain Energy (Joules)
    float yield_probability;       // Material rupture risk (0.0 to 1.0)
    bool  burst_shock_detected;    // Microsecond acoustic/pressure burst shock flag
    ImpactClassification impact_type; // Current impact classification
} BalloonMaterialPhysics;

typedef struct {
    float accumulated_damage;          // Miner's D (0.0 -> 1.0 = failure)
    uint32_t total_bounce_count;       // Total detected bounces
    float max_bounce_stress_kpa;       // Peak stress seen during any bounce
    float safe_pressure_remaining_pct; // Safe operating pressure capacity (%)
} FatigueState;

typedef struct {
    float recommended_pressure_kpa;  // Target inflate-to pressure
    float max_safe_pressure_kpa;     // Absolute ceiling under body weight
    float predicted_sink_depth_cm;   // Expected deformation depth (cm)
    float burst_safety_margin_pct;   // Distance to burst limit (%)
} RideInflationAdvice;

typedef struct {
    float estimated_C10;           // Identified C10 parameter (kPa)
    float estimated_C01;           // Identified C01 parameter (kPa)
    float r_squared;               // Goodness of fit (0.0 to 1.0)
    bool  converged;               // Parameter identification convergence flag
} OnlineMaterialParams;

void init_balloon_physics(BalloonType type);
void update_balloon_physics(float pressure_kpa, float dp_dt, float dt_sec);
void update_balloon_physics_ext(float pressure_kpa, float dp_dt, float d2p_dt2, float mcu_temp_c, float dt_sec);

BalloonMaterialPhysics get_balloon_physics_state();
void set_balloon_preset(BalloonType type);

// Bouncing & Impact Classification
ImpactClassification classify_pressure_event(float dp_dt, float d2p_dt2, float pressure_kpa);
ImpactClassification get_last_impact();

// Cyclic Fatigue & Lifetime Management
void reset_fatigue_tracker();
void record_bounce_cycle(float peak_stress_kpa);
FatigueState get_fatigue_state();

// Pre-Ride Inflation Calculator
RideInflationAdvice compute_ride_inflation(float rider_kg, BalloonType type, float firmness_0to1);

// Adaptive Burst Threshold Learning
void set_learned_burst_threshold(float threshold);
float get_learned_burst_threshold();
void update_burst_threshold_from_pop();

// Online Mooney-Rivlin RLS Parameter Identification
void update_online_material_identification(float pressure_kpa, float stretch_ratio);
OnlineMaterialParams get_online_material_params();

// Acoustic Tear Precursor Micro-Flutter Detector
bool is_tear_precursor_flutter_detected();
float get_flutter_variance();

#endif // BIP_BALLOON_PHYSICS_H


