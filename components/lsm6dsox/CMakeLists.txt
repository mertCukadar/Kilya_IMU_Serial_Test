idf_component_register(SRCS "lsm6dsox.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c esp_driver_gpio)
