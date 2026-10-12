idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_hw_support
                             esp_timer
                             esp_driver_gpio
                             esp_driver_uart
                             log
                             esp_system)
