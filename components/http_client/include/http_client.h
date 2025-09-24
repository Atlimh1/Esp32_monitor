#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include "esp_err.h"
#include "sensor_manager.h"

// HTTP client configuration
typedef struct {
    char server_url[128];        // Your Raspberry Pi server URL
    char api_endpoint[64];       // API endpoint path
    uint16_t port;               // Server port (usually 80 or 3000, 8080, etc)
    uint32_t send_interval_ms;   // How often to send data
    bool auto_send_enabled;      // Auto-send enabled flag
} http_client_config_t;

// Function prototypes
esp_err_t http_client_init(const http_client_config_t *config);
esp_err_t http_client_deinit(void);
esp_err_t http_client_send_sensor_data(void);
esp_err_t http_client_start_auto_send(void);
esp_err_t http_client_stop_auto_send(void);
esp_err_t http_client_test_connection(void);

#endif // HTTP_CLIENT_H