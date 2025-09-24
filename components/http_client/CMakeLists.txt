idf_component_register(SRCS "http_client.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_client sensor_manager json
                    PRIV_REQUIRES esp_timer esp_event)