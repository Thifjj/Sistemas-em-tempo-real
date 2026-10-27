idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_touch_sens esp_timer freertos esp_wifi esp_event esp_netif nvs_flash lwip
)
