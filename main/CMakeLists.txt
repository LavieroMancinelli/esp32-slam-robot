idf_component_register( SRCS "main.c" "VL53L4CD_api.c" "platform.c" "wifi.c"
                        INCLUDE_DIRS "."
                        REQUIRES driver esp_driver_gpio esp_driver_ledc esp_driver_mcpwm esp_driver_gptimer nvs_flash esp_wifi esp_http_server)