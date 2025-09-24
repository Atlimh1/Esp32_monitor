#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sensor_manager.h"
#include "wifi_manager.h"
#include "http_client.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_wifi.h"

static const char *TAG = "MAIN";

// ============= CONFIGURATION =============
// Set to 0 for production, 1 for debug mode
#define DEBUG_MODE 1

// WiFi credentials
#define WIFI_SSID "Funhouse"
#define WIFI_PASSWORD "Elmadogg12"

// Server configuration
#define SERVER_IP "192.168.1.100"     // Your Raspberry Pi IP
#define SERVER_PORT 3000               // Your server port
#define SERVER_ENDPOINT "/api/sensors" // Your API endpoint

// I2C Configuration
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_PORT      0

// Timing configuration
#define SENSOR_READ_INTERVAL_MS  5000   // Read sensors every 5 seconds
#define HTTP_SEND_INTERVAL_MS    30000  // Send to server every 30 seconds
#define DISPLAY_INTERVAL_MS      10000  // Display status every 10 seconds

// =========================================

// Status monitoring structure
typedef struct {
    bool wifi_connected;
    bool http_last_success;
    uint32_t http_success_count;
    uint32_t http_fail_count;
    int64_t last_http_time;
    char last_error[64];
} system_status_t;

static system_status_t sys_status = {0};

#if DEBUG_MODE
// Debug display task
static void debug_display_task(void *pvParameters) {
    ESP_LOGI(TAG, "Debug display task started");
    
    while (1) {
        printf("\n========== SYSTEM STATUS ==========\n");
        printf("WiFi: %s\n", sys_status.wifi_connected ? "Connected" : "Disconnected");
        printf("HTTP Success/Fail: %lu/%lu\n", 
               sys_status.http_success_count, sys_status.http_fail_count);
        
        if (sys_status.last_http_time > 0) {
            int64_t ago = (esp_timer_get_time() - sys_status.last_http_time) / 1000000;
            printf("Last HTTP: %lld seconds ago (%s)\n", 
                   ago, sys_status.http_last_success ? "OK" : "FAILED");
        }
        
        if (strlen(sys_status.last_error) > 0) {
            printf("Last Error: %s\n", sys_status.last_error);
        }
        
        sensor_manager_print_status();
        sensor_manager_print_data();
        printf("===================================\n");
        
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}

// Interactive control task
static void debug_control_task(void *pvParameters) {
    char c;
    
    printf("\n=================================\n");
    printf("    DEBUG MODE ENABLED\n");
    printf("=================================\n");
    printf("Commands:\n");
    printf("  w - WiFi status\n");
    printf("  h - Test HTTP send\n");
    printf("  r - Read sensors now\n");
    printf("  s - Sensor status\n");
    printf("  d - Display all data\n");
    printf("  ? - Show this help\n");
    printf("=================================\n\n");
    
    while (1) {
        c = getchar();
        if (c != EOF) {
            switch (c) {
                case 'w':
                case 'W':
                    printf("WiFi: %s\n", sys_status.wifi_connected ? 
                           "Connected" : "Disconnected");
                    if (sys_status.wifi_connected) {
                        // Could add IP address display here
                    }
                    break;
                    
                case 'h':
                case 'H':
                    printf("Testing HTTP connection...\n");
                    esp_err_t ret = http_client_send_sensor_data();
                    if (ret == ESP_OK) {
                        printf("HTTP test successful!\n");
                        sys_status.http_success_count++;
                        sys_status.http_last_success = true;
                    } else {
                        printf("HTTP test failed: %s\n", esp_err_to_name(ret));
                        sys_status.http_fail_count++;
                        sys_status.http_last_success = false;
                    }
                    sys_status.last_http_time = esp_timer_get_time();
                    break;
                    
                case 'r':
                case 'R':
                    printf("Reading sensors...\n");
                    sensor_manager_read_all();
                    sensor_manager_print_data();
                    break;
                    
                case 's':
                case 'S':
                    sensor_manager_print_status();
                    break;
                    
                case 'd':
                case 'D':
                    sensor_manager_print_data();
                    printf("\nHTTP Stats: Success=%lu, Fail=%lu\n",
                           sys_status.http_success_count, 
                           sys_status.http_fail_count);
                    break;
                    
                case '?':
                    printf("\nCommands:\n");
                    printf("  w - WiFi status\n");
                    printf("  h - Test HTTP send\n");
                    printf("  r - Read sensors now\n");
                    printf("  s - Sensor status\n");
                    printf("  d - Display all data\n");
                    printf("  ? - Show this help\n\n");
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif // DEBUG_MODE

// HTTP send task with error tracking
static void http_send_task(void *pvParameters) {
    ESP_LOGI(TAG, "HTTP send task started");
    
    while (1) {
        if (sys_status.wifi_connected) {
            esp_err_t ret = http_client_send_sensor_data();
            
            if (ret == ESP_OK) {
                sys_status.http_success_count++;
                sys_status.http_last_success = true;
                sys_status.last_error[0] = '\0';
                ESP_LOGI(TAG, "Data sent successfully (%lu total)", 
                         sys_status.http_success_count);
            } else {
                sys_status.http_fail_count++;
                sys_status.http_last_success = false;
                snprintf(sys_status.last_error, sizeof(sys_status.last_error),
                         "HTTP failed: %s", esp_err_to_name(ret));
                ESP_LOGE(TAG, "%s", sys_status.last_error);
            }
            sys_status.last_http_time = esp_timer_get_time();
        } else {
            ESP_LOGW(TAG, "WiFi not connected, skipping HTTP send");
        }
        
        vTaskDelay(pdMS_TO_TICKS(HTTP_SEND_INTERVAL_MS));
    }
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        sys_status.wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        sys_status.wifi_connected = false;
        ESP_LOGW(TAG, "WiFi disconnected");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Environmental Monitor Starting...");
    
#if DEBUG_MODE
    ESP_LOGW(TAG, "DEBUG MODE IS ENABLED");
    // Set higher log level for debugging
    esp_log_level_set("*", ESP_LOG_INFO);
#else
    // Production mode - less logging
    esp_log_level_set("*", ESP_LOG_WARN);
#endif
    
    // Initialize WiFi with event handler
    ESP_LOGI(TAG, "Initializing WiFi...");
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for connection
    
    // Initialize sensor manager
    ESP_LOGI(TAG, "Initializing Sensor Manager...");
    sensor_manager_config_t sensor_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_pin = I2C_MASTER_SDA_IO,
        .scl_pin = I2C_MASTER_SCL_IO,
        .read_interval_ms = SENSOR_READ_INTERVAL_MS,
        .auto_read_enabled = false
    };
    
    esp_err_t ret = sensor_manager_init(&sensor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor manager");
        strcpy(sys_status.last_error, "Sensor init failed");
    }
    
    // Start sensor auto-read
    sensor_status_t sensor_status;
    sensor_manager_get_status(&sensor_status);
    if (sensor_status.total_sensors > 0) {
        sensor_manager_start_auto_read();
        ESP_LOGI(TAG, "Sensor auto-read started");
    } else {
        ESP_LOGW(TAG, "No sensors found!");
        strcpy(sys_status.last_error, "No sensors found");
    }
    
    // Initialize HTTP client
    ESP_LOGI(TAG, "Initializing HTTP Client...");
    http_client_config_t http_config = {
        .server_url = SERVER_IP,
        .api_endpoint = SERVER_ENDPOINT,
        .port = SERVER_PORT,
        .send_interval_ms = HTTP_SEND_INTERVAL_MS,
        .auto_send_enabled = false
    };
    
    ret = http_client_init(&http_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        strcpy(sys_status.last_error, "HTTP init failed");
    }
    
    // Create HTTP send task
    xTaskCreate(http_send_task, "http_send", 4096, NULL, 5, NULL);
    
#if DEBUG_MODE
    // Create debug tasks
    xTaskCreate(debug_display_task, "debug_display", 3072, NULL, 3, NULL);
    xTaskCreate(debug_control_task, "debug_control", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "Debug tasks created");
#endif
    
    ESP_LOGI(TAG, "System initialized. Mode: %s", 
             DEBUG_MODE ? "DEBUG" : "PRODUCTION");
}