idf_component_register(
    SRCS "my_eth.c" "my_wifi.c" "my_spi.c" "hello_world_main.c"
    PRIV_REQUIRES spi_flash nvs_flash          
    INCLUDE_DIRS ""
    REQUIRES esp_wifi esp_driver_spi esp_driver_gpio esp_timer esp_netif esp_eth
)