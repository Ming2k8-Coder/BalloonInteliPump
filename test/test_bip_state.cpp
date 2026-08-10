#include "unity.h"
#include "bip_state.h"

void test_pid_parameter_tuning(void) {
    set_pid(30.0f, 1.5f, 5.0f);
    set_manual_target(25.0f);
    // State machine initialization
    init_states();
    TEST_ASSERT_EQUAL_INT(MODE_IDLE, currentMode);
}

void test_yield_ratio_constraints(void) {
    set_yield_ratio(1.35f);
    // Invalid ratio (< 1.0) should be rejected by guard
    set_yield_ratio(0.5f);
    set_yield_ratio(1.25f);
}

void test_manual_mode_target_switch(void) {
    init_states();
    set_manual_target(15.0f);
    currentMode = MODE_MANUAL;
    TEST_ASSERT_EQUAL_INT(MODE_MANUAL, currentMode);
}

void run_state_tests(void) {
    RUN_TEST(test_pid_parameter_tuning);
    RUN_TEST(test_yield_ratio_constraints);
    RUN_TEST(test_manual_mode_target_switch);
}
