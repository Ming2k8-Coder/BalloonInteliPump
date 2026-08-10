idf_component_register(
    SRCS "test_main.cpp"
         "test_bip_sensors.cpp"
         "test_bip_motor.cpp"
         "test_bip_state.cpp"
         "test_bip_storage.cpp"
    INCLUDE_DIRS "." "../main"
    REQUIRES unity main driver nvs_flash esp_timer esp_adc
)
