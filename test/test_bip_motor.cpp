#include "unity.h"
#include "bip_motor.h"

static BIP_Motor testMotor(PIN_MOTOR_PWM);

void test_motor_soft_start_ramping(void) {
    testMotor.clearSafetyLockout();
    testMotor.setRampRate(5.0f);
    testMotor.setTargetPWM(100);

    // Initial state: PWM = 0
    TEST_ASSERT_EQUAL_INT(0, testMotor.getCurrentPWM());

    // Cycle 1: PWM increases by 5.0 -> 5
    testMotor.update(0.5f, 12.0f, 12.0f);
    TEST_ASSERT_EQUAL_INT(5, testMotor.getCurrentPWM());

    // Cycle 2: PWM increases by 5.0 -> 10
    testMotor.update(0.5f, 12.0f, 12.0f);
    TEST_ASSERT_EQUAL_INT(10, testMotor.getCurrentPWM());

    // Stop motor cleanly
    testMotor.emergencyStop();
    TEST_ASSERT_EQUAL_INT(0, testMotor.getCurrentPWM());
}

void test_motor_overcurrent_inrush_debounce(void) {
    testMotor.clearSafetyLockout();
    SafetySettings s = { .maxCurrent = 4.0f, .minVoltage = 10.0f, .sagLimitPercent = 10 };
    testMotor.setSafetySettings(s);
    testMotor.setTargetPWM(150);

    // Transient inrush test: 20 samples of 4.5A (> 4.0A, 10ms @ 2000 SPS) -> Must NOT trip
    for (int i = 0; i < 20; i++) {
        testMotor.update(4.5f, 12.0f, 12.0f);
        TEST_ASSERT_FALSE(testMotor.isSafetyTriggered());
    }

    // Current recovers to normal (2.0A) for 20 samples -> Leaky bucket counter clears
    for (int i = 0; i < 20; i++) {
        testMotor.update(2.0f, 12.0f, 12.0f);
        TEST_ASSERT_FALSE(testMotor.isSafetyTriggered());
    }

    // Sustained overcurrent test: 49 samples of 4.5A -> Still NOT tripped (debounce limit = 50 samples / 25ms)
    for (int i = 0; i < 49; i++) {
        testMotor.update(4.5f, 12.0f, 12.0f);
        TEST_ASSERT_FALSE(testMotor.isSafetyTriggered());
    }

    // Sample 50: Sustained overcurrent reaches 50 samples (25ms @ 2000 SPS) -> TRIPPED!
    testMotor.update(4.5f, 12.0f, 12.0f);
    TEST_ASSERT_TRUE(testMotor.isSafetyTriggered());
    TEST_ASSERT_EQUAL_STRING("OC_LOCK", testMotor.getSafetyError());

    // Clear Lockout
    testMotor.clearSafetyLockout();
    TEST_ASSERT_FALSE(testMotor.isSafetyTriggered());
}

void test_motor_undervoltage_lockout(void) {
    testMotor.clearSafetyLockout();
    SafetySettings s = { .maxCurrent = 4.0f, .minVoltage = 10.5f, .sagLimitPercent = 10 };
    testMotor.setSafetySettings(s);
    testMotor.setTargetPWM(100);

    // Undervoltage input (9.5V < 10.5V min) -> Immediate trip
    testMotor.update(1.0f, 9.5f, 12.0f);
    TEST_ASSERT_TRUE(testMotor.isSafetyTriggered());
    TEST_ASSERT_EQUAL_STRING("UV_LOCK", testMotor.getSafetyError());

    // Clean up
    testMotor.clearSafetyLockout();
    testMotor.emergencyStop();
}

void run_motor_tests(void) {
    RUN_TEST(test_motor_soft_start_ramping);
    RUN_TEST(test_motor_overcurrent_inrush_debounce);
    RUN_TEST(test_motor_undervoltage_lockout);
}
