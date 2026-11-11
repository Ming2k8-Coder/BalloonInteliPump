#include "unity.h"
#include <stdio.h>

extern void run_sensor_tests(void);
extern void run_motor_tests(void);
extern void run_state_tests(void);
extern void run_storage_tests(void);
extern void run_balloon_physics_tests(void);
extern void run_volume_estimator_tests(void);

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    printf("=== Starting BalloonInteliPump Native Host Unity Test Suite ===\n");
    
    UNITY_BEGIN();
    
    run_sensor_tests();
    run_motor_tests();
    run_state_tests();
    run_storage_tests();
    run_balloon_physics_tests();
    run_volume_estimator_tests();
    
    return UNITY_END();
}
