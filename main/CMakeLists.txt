idf_component_register(SRCS "espnow_link.c" "main.c" "motion_level.c"
                       PRIV_REQUIRES esp_lcd
                                     esp_timer
                                     esp_driver_gpio
                                     esp_driver_spi
                                     esp_driver_i2c
                                     esp_driver_ledc
                                     esp_driver_sdspi
                                     esp_adc
                                     esp_hw_support
                                     esp_wifi
                                     esp_event
                                     esp_netif
                                     nvs_flash
                                     heap
                                     spi_flash
                                     fatfs
                                     sdmmc
                                     lvgl
                       INCLUDE_DIRS "")

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
