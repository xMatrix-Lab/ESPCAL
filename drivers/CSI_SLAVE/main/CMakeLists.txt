idf_component_register(
    SRCS "my_spi.c" "my_wifi.c" "app_main.c"
    PRIV_REQUIRES spi_flash nvs_flash
    INCLUDE_DIRS ""
    REQUIRES esp_wifi esp_driver_spi esp_driver_gpio
)

