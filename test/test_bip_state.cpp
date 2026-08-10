#include "unity.h"
#include "bip_state.h"
#include "bip_balloon_physics.h"

void test_pid_parameter_tuning(void) {
    set_pid(30.0f, 1.5f, 5.0f);
    set_manual_target(25.0f);
    init_states();
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, currentMode);
}

void test_yield_ratio_constraints(void) {
    set_yield_ratio(1.35f);
    set_yield_ratio(0.5f);
    set_yield_ratio(1.25f);
}

void test_manual_mode_target_switch(void) {
    init_states();
    set_manual_target(15.0f);
    currentMode = MODE_MANUAL;
    TEST_ASSERT_EQUAL_INT(MODE_MANUAL, currentMode);
}

void test_new_modes_transitions(void) {
    init_states();
    set_target_diameter(30.0f);
    TEST_ASSERT_EQUAL_INT(MODE_DIAMETER, currentMode);

    set_ride_params(80.0f, 6.0f);
    TEST_ASSERT_EQUAL_INT(MODE_RIDE, currentMode);

    start_balloon_conditioning(3);
    TEST_ASSERT_EQUAL_INT(MODE_CONDITION, currentMode);

    post_mode_change(MODE_IDLE);
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, currentMode);
}

void test_physics_fatigue_and_impact_classification(void) {
    reset_fatigue_tracker();
    FatigueState f1 = get_fatigue_state();
    TEST_ASSERT_EQUAL_FLOAT(100.0f, f1.safe_pressure_remaining_pct);

    record_bounce_cycle(25.0f);
    FatigueState f2 = get_fatigue_state();
    TEST_ASSERT_EQUAL_UINT32(1, f2.total_bounce_count);
    TEST_ASSERT_TRUE(f2.accumulated_damage > 0.0f);

    ImpactClassification imp = classify_pressure_event(65.0f, 0.0f, 10.0f);
    TEST_ASSERT_EQUAL_INT(IMPACT_BOUNCE, imp);
}

void test_ride_inflation_calculator(void) {
    RideInflationAdvice advice = compute_ride_inflation(75.0f, BALLOON_36INCH, 0.5f);
    TEST_ASSERT_TRUE(advice.recommended_pressure_kpa > 0.0f);
    TEST_ASSERT_TRUE(advice.max_safe_pressure_kpa > advice.recommended_pressure_kpa);
    TEST_ASSERT_TRUE(advice.predicted_sink_depth_cm > 0.0f);
}

void run_state_tests(void) {
    RUN_TEST(test_pid_parameter_tuning);
    RUN_TEST(test_yield_ratio_constraints);
    RUN_TEST(test_manual_mode_target_switch);
    RUN_TEST(test_new_modes_transitions);
    RUN_TEST(test_physics_fatigue_and_impact_classification);
    RUN_TEST(test_ride_inflation_calculator);
}

