idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lsm6dsox esp_driver_i2c esp_timer esp_driver_tsens)
