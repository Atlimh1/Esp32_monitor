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
#include <string.h>
#include "driver/i2c.h"

static const char *TAG = "MAIN";

// ============= CONFIGURATION =============
// Set to 0 for production, 1 for debug mode
#define DEBUG_MODE 0

// WiFi credentials
#define WIFI_SSID "Funhouse"
#define WIFI_PASSWORD "Elmadogg12"

// Server configuration
#define SERVER_IP "192.168.8.138"     // Your Raspberry Pi IP
#define SERVER_PORT 5000               // Your server port
#define SERVER_ENDPOINT "/api/sensors" // Your API endpoint

// I2C Configuration
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_PORT      0

// Timing configuration
#define SENSOR_READ_INTERVAL_MS  5000   // Read sensors every 5 seconds
#define HTTP_SEND_INTERVAL_MS    30000  // Send to server every 30 seconds

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

// I2C Scanner function
static void i2c_scanner(void) {
    printf("\n=== I2C Bus Scanner ===\n");
    printf("Scanning I2C bus %d...\n", I2C_MASTER_PORT);
    
    int devices_found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("Device found at address 0x%02X", addr);
            switch(addr) {
                case 0x44: printf(" (SHT31 - ADDR pin LOW)"); break;
                case 0x45: printf(" (SHT31 - ADDR pin HIGH)"); break;
                case 0x76: printf(" (BMP280/BME280 - SDO to GND)"); break;
                case 0x77: printf(" (BMP280/BME280 - SDO to VDD)"); break;
                default: printf(" (Unknown device)"); break;
            }
            printf("\n");
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        printf("No I2C devices found!\n");
        printf("Check connections:\n");
        printf("  - SDA: GPIO%d\n", I2C_MASTER_SDA_IO);
        printf("  - SCL: GPIO%d\n", I2C_MASTER_SCL_IO);
        printf("  - 3.3V power and GND\n");
        printf("  - Pull-up resistors (4.7kΩ)\n");
    } else {
        printf("Total devices found: %d\n", devices_found);
    }
    printf("=====================\n\n");
}

// Print current sensor status (no refresh)
static void print_sensor_status(void) {
    printf("\n=== Current Sensor Status ===\n");
    sensor_status_t status;
    esp_err_t ret = sensor_manager_get_status(&status);
    
    if (ret == ESP_OK) {
        printf("Sensors found: %d\n", status.total_sensors);
        printf("SHT31:  %s", status.sht31_found ? "Found" : "Not found");
        if (status.sht31_found) printf(" (0x%02X)", status.sht31_address);
        printf("\n");
        
        printf("BMP280: %s", status.bmp280_found ? "Found" : "Not found");
        if (status.bmp280_found) printf(" (0x%02X)", status.bmp280_address);
        printf("\n");
        
        printf("Reads: %lu successful, %lu errors\n", status.read_count, status.error_count);
        
        if (status.last_read_time > 0) {
            uint32_t seconds_ago = (esp_timer_get_time() / 1000 - status.last_read_time) / 1000;
            printf("Last read: %lu seconds ago\n", seconds_ago);
        }
    } else {
        printf("Failed to get sensor status\n");
    }
    printf("============================\n\n");
}

// Print current sensor readings (no refresh) 
static void print_sensor_data(void) {
    printf("\n=== Latest Sensor Readings ===\n");
    
    sensor_data_t sht31_data, bmp280_data;
    esp_err_t ret = sensor_manager_get_latest_data(&sht31_data, &bmp280_data);
    
    if (ret == ESP_OK) {
        sensor_status_t status;
        sensor_manager_get_status(&status);
        
        if (status.sht31_found) {
            printf("SHT31:\n");
            printf("  Temperature: %.2f°C (%.1f°F)\n", 
                   sht31_data.temperature, 
                   sht31_data.temperature * 9.0/5.0 + 32.0);
            printf("  Humidity:    %.1f%%\n", sht31_data.humidity);
            printf("  Dew Point:   %.1f°C\n", sht31_data.dew_point);
        }
        
        if (status.bmp280_found) {
            printf("BMP280:\n");
            printf("  Temperature: %.2f°C (%.1f°F)\n", 
                   bmp280_data.temperature,
                   bmp280_data.temperature * 9.0/5.0 + 32.0);
            printf("  Pressure:    %.0f Pa (%.2f hPa)\n", 
                   bmp280_data.pressure, 
                   bmp280_data.pressure / 100.0);
            printf("  Altitude:    %.1f m (%.1f ft)\n", 
                   bmp280_data.altitude,
                   bmp280_data.altitude * 3.28084);
        }
    } else {
        printf("No sensor data available\n");
    }
    printf("=============================\n\n");
}

// Print system status (no refresh)
static void print_system_status(void) {
    printf("\n=== System Status ===\n");
    printf("WiFi: %s\n", sys_status.wifi_connected ? "Connected" : "Disconnected");
    printf("HTTP: %lu success, %lu failed\n", 
           sys_status.http_success_count, sys_status.http_fail_count);
    
    if (sys_status.last_http_time > 0) {
        int64_t seconds_ago = (esp_timer_get_time() - sys_status.last_http_time) / 1000000;
        printf("Last HTTP: %lld sec ago (%s)\n", 
               seconds_ago, 
               sys_status.http_last_success ? "Success" : "Failed");
    }
    
    if (strlen(sys_status.last_error) > 0) {
        printf("Last error: %s\n", sys_status.last_error);
    }
    
    printf("Uptime: %.1f minutes\n", esp_timer_get_time() / 60000000.0);
    printf("====================\n\n");
}

#if DEBUG_MODE
// Interactive control task - no automatic refresh
static void debug_control_task(void *pvParameters) {
    int c;
    
    printf("\n======================================\n");
    printf("    ESP32 Environmental Monitor\n");
    printf("        DEBUG MODE ENABLED\n");
    printf("======================================\n");
    printf("Commands:\n");
    printf("  r - Read sensors now\n");
    printf("  s - Show sensor status\n");
    printf("  d - Show sensor data\n");
    printf("  y - Show system status\n");
    printf("  w - Show WiFi status\n");
    printf("  h - Test HTTP connection\n");
    printf("  i - Scan I2C bus\n");
    printf("  ? - Show this help\n");
    printf("======================================\n\n");
    printf("Ready for commands (press key + enter):\n");
    
    while (1) {
        c = getchar();
        if (c != '\n' && c != '\r' && c >= 0) {
            switch (c) {
                case 'r':
                case 'R':
                    printf("Reading sensors...\n");
                    esp_err_t ret = sensor_manager_read_all();
                    if (ret == ESP_OK) {
                        printf("Sensor read completed successfully\n");
                        print_sensor_data();
                    } else {
                        printf("Sensor read failed: %s\n", esp_err_to_name(ret));
                    }
                    break;
                    
                case 's':
                case 'S':
                    print_sensor_status();
                    break;
                    
                case 'd':
                case 'D':
                    print_sensor_data();
                    break;
                    
                case 'y':
                case 'Y':
                    print_system_status();
                    break;
                    
                case 'w':
                case 'W':
                    printf("WiFi Status: %s\n", 
                           sys_status.wifi_connected ? "Connected" : "Disconnected");
                    break;
                    
                case 'h':
                case 'H':
                    printf("Testing HTTP connection...\n");
                    ret = http_client_send_sensor_data();
                    if (ret == ESP_OK) {
                        printf("HTTP test successful!\n");
                        sys_status.http_success_count++;
                        sys_status.http_last_success = true;
                        sys_status.last_error[0] = '\0';
                    } else {
                        printf("HTTP test failed: %s\n", esp_err_to_name(ret));
                        sys_status.http_fail_count++;
                        sys_status.http_last_success = false;
                        snprintf(sys_status.last_error, sizeof(sys_status.last_error),
                                "HTTP: %s", esp_err_to_name(ret));
                    }
                    sys_status.last_http_time = esp_timer_get_time();
                    break;
                    
                case 'i':
                case 'I':
                    i2c_scanner();
                    break;
                    
                case '?':
                    printf("Commands:\n");
                    printf("  r - Read sensors now\n");
                    printf("  s - Show sensor status\n"); 
                    printf("  d - Show sensor data\n");
                    printf("  y - Show system status\n");
                    printf("  w - Show WiFi status\n");
                    printf("  h - Test HTTP connection\n");
                    printf("  i - Scan I2C bus\n");
                    printf("  ? - Show this help\n\n");
                    break;
                    
                default:
                    printf("Unknown command '%c'. Press '?' for help.\n", c);
                    break;
            }
            printf("Ready for next command:\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif // DEBUG_MODE

// HTTP send task with error tracking (no console spam)
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
                         "HTTP: %s", esp_err_to_name(ret));
                ESP_LOGE(TAG, "%s", sys_status.last_error);
            }
            sys_status.last_http_time = esp_timer_get_time();
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
    // Set log level for debugging but prevent sensor spam
    esp_log_level_set("SENSOR_MGR", ESP_LOG_WARN);
    esp_log_level_set("SHT31", ESP_LOG_WARN);
    esp_log_level_set("BMP280", ESP_LOG_WARN);
    esp_log_level_set("HTTP_CLIENT", ESP_LOG_WARN);
#else
    // Production mode - minimal logging
    esp_log_level_set("*", ESP_LOG_WARN);
#endif
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    
    esp_err_t ret = wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);
    if (ret == ESP_OK) {
        sys_status.wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected successfully");
    } else {
        ESP_LOGW(TAG, "WiFi connection failed");
        strcpy(sys_status.last_error, "WiFi connection failed");
    }
    
    // Initialize sensor manager with corrected I2C speed
    ESP_LOGI(TAG, "Initializing Sensor Manager...");
    sensor_manager_config_t sensor_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_pin = I2C_MASTER_SDA_IO,
        .scl_pin = I2C_MASTER_SCL_IO,
        .read_interval_ms = SENSOR_READ_INTERVAL_MS,
        .auto_read_enabled = false
    };
    
    ret = sensor_manager_init(&sensor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor manager");
        strcpy(sys_status.last_error, "Sensor init failed");
    }
    
    // Start sensor auto-read
    sensor_status_t sensor_status;
    sensor_manager_get_status(&sensor_status);
    if (sensor_status.total_sensors > 0) {
        sensor_manager_start_auto_read();
        ESP_LOGI(TAG, "Found %d sensor(s), auto-read started", sensor_status.total_sensors);
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
    // Create debug control task (interactive, no refresh)
    xTaskCreate(debug_control_task, "debug_control", 4096, NULL, 4, NULL);
#endif
    
    ESP_LOGI(TAG, "System initialized. Mode: %s", 
             DEBUG_MODE ? "DEBUG" : "PRODUCTION");
    
    // Initial status display
    printf("\n=== ESP32 Environmental Monitor Ready ===\n");
    print_sensor_status();
    if (sensor_status.total_sensors > 0) {
        // Do an initial sensor read
        vTaskDelay(pdMS_TO_TICKS(2000)); // Let sensors settle
        sensor_manager_read_all();
        print_sensor_data();
    }
    print_system_status();
    
#if DEBUG_MODE
    printf("Use debug commands above to interact with the system.\n");
#endif
}