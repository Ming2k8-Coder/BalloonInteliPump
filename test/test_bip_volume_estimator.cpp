#include "unity.h"
#include "bip_volume_estimator.h"
#include <cmath>

void test_hammerstein_pump_model_deadzone(void) {
    init_volume_estimator(7.0f);
    set_pump_model_params(35.0f, 72.0f); // 35 PWM deadzone

    // Case 1: PWM below deadzone (30 PWM <= 35) -> 0 volume added
    update_volume_estimator(10.0f, 30.0f, 1.0f);
    BalloonPhysicsEstimate est1 = get_balloon_physics_estimate();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est1.volume_liters);

    // Case 2: PWM above deadzone (150 PWM > 35) -> volume accumulates
    update_volume_estimator(10.0f, 150.0f, 1.0f);
    BalloonPhysicsEstimate est2 = get_balloon_physics_estimate();
    TEST_ASSERT_TRUE(est2.volume_liters > 0.0f);
}

void test_ideal_gas_pv_nrt_scaling(void) {
    init_volume_estimator(7.0f);
    set_pump_model_params(0.0f, 72.0f);

    // Baseline integration at 25°C
    update_volume_estimator_ext(10.0f, 200.0f, 40.0f, 1.0f); // ~25°C ambient
    float vol_standard = get_balloon_physics_estimate().volume_liters;

    // Elevated temperature integration (60°C MCU die -> 45°C ambient)
    init_volume_estimator(7.0f);
    update_volume_estimator_ext(10.0f, 200.0f, 60.0f, 1.0f);
    float vol_hot = get_balloon_physics_estimate().volume_liters;

    // Hotter air -> less dense -> less mass/effective STP volume delivered
    TEST_ASSERT_TRUE(vol_hot < vol_standard);
}

void test_oblate_spheroid_diameter_calculation(void) {
    init_volume_estimator(7.0f);
    
    // Set standard party balloon oblate shape factor (0.85)
    set_balloon_shape_params(0.0f, 0.85f);
    update_volume_estimator(15.0f, 200.0f, 10.0f); // Accumulate volume
    BalloonPhysicsEstimate est_oblate = get_balloon_physics_estimate();

    // Set perfect spherical shape factor (1.0)
    init_volume_estimator(7.0f);
    set_balloon_shape_params(0.0f, 1.00f);
    update_volume_estimator(15.0f, 200.0f, 10.0f);
    BalloonPhysicsEstimate est_sphere = get_balloon_physics_estimate();

    // Oblate spheroid (compressed polar axis) has LARGER equatorial diameter for same volume
    TEST_ASSERT_TRUE(est_oblate.diameter_cm > est_sphere.diameter_cm);
}

void test_neck_volume_offset(void) {
    init_volume_estimator(7.0f);

    // Set 0.05 L neck extension offset
    set_balloon_shape_params(0.05f, 0.85f);
    
    // Small volume delivered (0.03 L <= 0.05 L neck) -> diameter remains initial neck diameter
    update_volume_estimator(5.0f, 100.0f, 0.2f);
    BalloonPhysicsEstimate est = get_balloon_physics_estimate();

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 7.0f, est.diameter_cm);
}

void run_volume_estimator_tests(void) {
    RUN_TEST(test_hammerstein_pump_model_deadzone);
    RUN_TEST(test_ideal_gas_pv_nrt_scaling);
    RUN_TEST(test_oblate_spheroid_diameter_calculation);
    RUN_TEST(test_neck_volume_offset);
}
