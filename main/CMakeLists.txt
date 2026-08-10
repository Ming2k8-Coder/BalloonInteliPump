idf_component_register(
    SRCS "main.cpp"
         "bip_sensors.cpp"
         "bip_motor.cpp"
         "bip_storage.cpp"
         "bip_state.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi lwip esp_timer esp_adc log
)
