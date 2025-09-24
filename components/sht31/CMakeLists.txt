idf_component_register(SRCS "sht31.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common freertos esp_driver_i2c)