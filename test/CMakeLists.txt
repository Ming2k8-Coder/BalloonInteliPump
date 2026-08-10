idf_component_register(
    SRCS "test_main.cpp"
         "test_bip_sensors.cpp"
         "test_bip_motor.cpp"
         "test_bip_state.cpp"
         "test_bip_storage.cpp"
         "test_bip_balloon_physics.cpp"
         "test_bip_volume_estimator.cpp"
    INCLUDE_DIRS "." "../main"
    REQUIRES unity main driver nvs_flash esp_timer esp_adc
)

