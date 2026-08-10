#include "unity.h"
#include "bip_sensors.h"
#include <cmath>

static BIP_Sensor testPressure("TestPressure");

void test_sensor_multipoint_interpolation(void) {
    testPressure.resetCalibration();
    // 0.5V -> 0.0 kPa, 4.5V -> 200.0 kPa
    testPressure.addCalibrationPoint(0.5f, 0.0f);
    testPressure.addCalibrationPoint(4.5f, 200.0f);

    testPressure.setRawValue(0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, testPressure.getValue());

    testPressure.setRawValue(2.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, testPressure.getValue());

    testPressure.setRawValue(4.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, testPressure.getValue());
}

void test_sensor_tare_zeroing(void) {
    testPressure.resetCalibration();
    testPressure.resetTare();
    testPressure.addCalibrationPoint(0.5f, 0.0f);
    testPressure.addCalibrationPoint(4.5f, 200.0f);

    // Apply baseline pressure of 10.0 kPa (0.7V)
    testPressure.setRawValue(0.7f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, testPressure.getValue());

    // Execute Tare zeroing
    testPressure.tare();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, testPressure.getValue());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, testPressure.getZeroOffset());

    // Clean up
    testPressure.resetTare();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, testPressure.getZeroOffset());
}

void test_sensor_duplicate_point_div_zero_guard(void) {
    testPressure.resetCalibration();
    // Add identical raw point twice
    testPressure.addCalibrationPoint(1.0f, 10.0f);
    testPressure.addCalibrationPoint(1.0f, 20.0f); // Should be rejected by guard

    TEST_ASSERT_EQUAL_INT(1, testPressure.getCalibration().numPoints);

    // Verify system does not crash or return NaN/INF
    testPressure.setRawValue(1.0f);
    TEST_ASSERT_FALSE(std::isnan(testPressure.getValue()));
    TEST_ASSERT_FALSE(std::isinf(testPressure.getValue()));
}

void run_sensor_tests(void) {
    RUN_TEST(test_sensor_multipoint_interpolation);
    RUN_TEST(test_sensor_tare_zeroing);
    RUN_TEST(test_sensor_duplicate_point_div_zero_guard);
}
