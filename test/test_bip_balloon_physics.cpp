#include "unity.h"
#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include <cmath>

void test_mooney_rivlin_hyperelastic_stress(void) {
    init_balloon_physics(BALLOON_11INCH);
    
    // Simulate stretch ratio lambda = 2.0
    // Expected Mooney-Rivlin wall stress at lambda=2.0 (C10=180, C01=20):
    // sigma = 2 * (4.0 - 0.5) * (180 + 20/2) = 2 * 3.5 * 190 = 1330 kPa
    update_balloon_physics_ext(15.0f, 0.0f, 0.0f, 25.0f, 0.001f);
    BalloonMaterialPhysics state = get_balloon_physics_state();
    
    TEST_ASSERT_TRUE(state.stretch_ratio >= 1.0f);
    TEST_ASSERT_TRUE(state.hyperelastic_stress_kpa > 0.0f);
    TEST_ASSERT_TRUE(state.yield_probability >= 0.0f && state.yield_probability <= 1.0f);
}

void test_sls_viscoelastic_stress_relaxation(void) {
    init_balloon_physics(BALLOON_36INCH);

    // Step 1: Rapid inflation (high d_lambda/dt) -> viscous stress accumulates
    for (int i = 0; i < 100; i++) {
        update_balloon_physics_ext(10.0f + (float)i * 0.1f, 5.0f, 0.0f, 25.0f, 0.001f);
    }
    BalloonMaterialPhysics state1 = get_balloon_physics_state();
    float viscous_peak = state1.viscous_stress_kpa;
    TEST_ASSERT_TRUE(viscous_peak > 0.0f);

    // Step 2: Hold constant strain (dp/dt = 0) -> viscous stress relaxes towards 0
    for (int i = 0; i < 500; i++) {
        update_balloon_physics_ext(20.0f, 0.0f, 0.0f, 25.0f, 0.005f);
    }
    BalloonMaterialPhysics state2 = get_balloon_physics_state();
    TEST_ASSERT_TRUE(state2.viscous_stress_kpa < viscous_peak);
}

void test_thermal_softening_guard(void) {
    init_balloon_physics(BALLOON_11INCH);

    // Standard room temperature 25°C
    update_balloon_physics_ext(15.0f, 0.0f, 0.0f, 25.0f, 0.001f);
    float stress_room = get_balloon_physics_state().hyperelastic_stress_kpa;

    // Elevated MCU/ambient temperature (45°C) -> rubber softens -> stress drops
    update_balloon_physics_ext(15.0f, 0.0f, 0.0f, 45.0f, 0.001f);
    float stress_hot = get_balloon_physics_state().hyperelastic_stress_kpa;

    TEST_ASSERT_TRUE(stress_hot < stress_room);
}

void test_impact_classifier_bounce_vs_burst(void) {
    init_balloon_physics(BALLOON_36INCH);

    // Test 1: Bouncing Event (+dP/dt compression followed by -dP/dt rebound)
    ImpactClassification imp1 = classify_pressure_event(70.0f, 5.0f, 18.0f);
    TEST_ASSERT_EQUAL_INT(IMPACT_BOUNCE, imp1);

    // Rebound phase within 100ms
    ImpactClassification imp2 = classify_pressure_event(-35.0f, -2.0f, 16.0f);
    TEST_ASSERT_EQUAL_INT(IMPACT_BOUNCE, imp2);

    // Test 2: True Pop Burst Event (-dP/dt collapse to < 4kPa without positive spike)
    ImpactClassification imp3 = classify_pressure_event(-55.0f, -10.0f, 2.0f);
    TEST_ASSERT_EQUAL_INT(IMPACT_BURST, imp3);

    // Test 3: Gentle Squeeze
    ImpactClassification imp4 = classify_pressure_event(15.0f, 0.5f, 12.0f);
    TEST_ASSERT_EQUAL_INT(IMPACT_SQUEEZE, imp4);
}

void test_fatigue_tracker_miners_rule(void) {
    reset_fatigue_tracker();
    FatigueState f_init = get_fatigue_state();
    TEST_ASSERT_EQUAL_UINT32(0, f_init.total_bounce_count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, f_init.safe_pressure_remaining_pct);

    // Simulate 50 high-stress bounce cycles
    for (int i = 0; i < 50; i++) {
        record_bounce_cycle(28.0f);
    }
    FatigueState f_after = get_fatigue_state();
    TEST_ASSERT_EQUAL_UINT32(50, f_after.total_bounce_count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 28.0f, f_after.max_bounce_stress_kpa);
    TEST_ASSERT_TRUE(f_after.accumulated_damage > 0.0f);
    TEST_ASSERT_TRUE(f_after.safe_pressure_remaining_pct < 100.0f);
}

void test_ride_inflation_calculator(void) {
    // Test for 70kg rider on 36" balloon
    RideInflationAdvice a_70kg = compute_ride_inflation(70.0f, BALLOON_36INCH, 0.5f);
    TEST_ASSERT_TRUE(a_70kg.recommended_pressure_kpa > 2.0f);
    TEST_ASSERT_TRUE(a_70kg.max_safe_pressure_kpa > a_70kg.recommended_pressure_kpa);
    TEST_ASSERT_TRUE(a_70kg.predicted_sink_depth_cm > 0.0f);

    // Test for 110kg rider -> recommended pressure should be lower due to larger weight pressure offset
    RideInflationAdvice a_110kg = compute_ride_inflation(110.0f, BALLOON_36INCH, 0.5f);
    TEST_ASSERT_TRUE(a_110kg.recommended_pressure_kpa <= a_70kg.recommended_pressure_kpa);
}

void test_adaptive_burst_threshold_learning(void) {
    set_learned_burst_threshold(-40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -40.0f, get_learned_burst_threshold());

    set_learned_burst_threshold(-48.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -48.5f, get_learned_burst_threshold());
}

void test_online_mooney_rivlin_identification(void) {
    init_balloon_physics(BALLOON_11INCH);

    // Feed 60 synthetic (pressure, stretch_ratio) samples to train online RLS
    for (int i = 0; i < 60; i++) {
        float lambda = 1.2f + (float)i * 0.02f;
        float p = 2.0f * (lambda * lambda - 1.0f / lambda) * (180.0f + 20.0f / lambda) * 0.01f;
        update_online_material_identification(p, lambda);
    }

    OnlineMaterialParams params = get_online_material_params();
    TEST_ASSERT_TRUE(params.converged);
    TEST_ASSERT_TRUE(params.estimated_C10 > 0.0f);
}

void test_tear_precursor_flutter_detector(void) {
    init_balloon_physics(BALLOON_11INCH);

    // Simulate high-frequency noise spikes in dP/dt
    for (int i = 0; i < 20; i++) {
        float spike = (i % 2 == 0) ? 250.0f : -250.0f;
        update_balloon_physics_ext(20.0f, spike, 0.0f, 25.0f, 0.001f);
    }

    TEST_ASSERT_TRUE(get_flutter_variance() > 50.0f);
}

void run_balloon_physics_tests(void) {
    RUN_TEST(test_mooney_rivlin_hyperelastic_stress);
    RUN_TEST(test_sls_viscoelastic_stress_relaxation);
    RUN_TEST(test_thermal_softening_guard);
    RUN_TEST(test_impact_classifier_bounce_vs_burst);
    RUN_TEST(test_fatigue_tracker_miners_rule);
    RUN_TEST(test_ride_inflation_calculator);
    RUN_TEST(test_adaptive_burst_threshold_learning);
    RUN_TEST(test_online_mooney_rivlin_identification);
    RUN_TEST(test_tear_precursor_flutter_detector);
}

