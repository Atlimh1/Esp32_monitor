idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES sensor_manager wifi_manager http_client
                    PRIV_REQUIRES esp_timer esp_event esp_wifi)