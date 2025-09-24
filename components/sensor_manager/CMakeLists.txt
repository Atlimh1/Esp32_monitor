idf_component_register(SRCS "sensor_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver sht31 bmp280
                    PRIV_REQUIRES esp_timer)