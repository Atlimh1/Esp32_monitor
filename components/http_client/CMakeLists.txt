idf_component_register(SRCS "http_client.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_client sensor_manager json esp_common
    PRIV_REQUIRES esp_timer esp_event)