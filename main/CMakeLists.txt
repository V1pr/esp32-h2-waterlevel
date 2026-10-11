idf_component_register(
    SRCS "main.c" "vl53l0x.c" "battery.c" "water_level.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_timer esp_adc driver esp-zigbee-lib
)
