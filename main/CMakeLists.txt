idf_component_register(
    SRCS "main.cpp"
         "bip_sensors.cpp"
         "bip_motor.cpp"
         "bip_storage.cpp"
         "bip_state.cpp"
         "bip_diagnostics.cpp"
         "bip_volume_estimator.cpp"
         "bip_balloon_physics.cpp"
         "bip_webserver.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi lwip esp_timer esp_adc log esp_http_server
)

