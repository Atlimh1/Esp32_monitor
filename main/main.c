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
#include "esp_heap_caps.h"
#include "soc/rtc.h"

static const char *TAG = "MAIN";

// ============= CONFIGURATION =============
// Set to 0 for production, 1 for debug mode
#define DEBUG_MODE 1

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

// Replace your debug_control_task function in main.c with this enhanced version:

#if DEBUG_MODE
// Enhanced testing interface with more useful features
static void debug_control_task(void *pvParameters) {
    int c;
    int continuous_read_count = 0;
    bool continuous_mode = false;
    
    printf("\n========================================\n");
    printf("    ESP32 Environmental Monitor\n");
    printf("        ENHANCED TESTING INTERFACE\n");
    printf("========================================\n");
    printf("SENSOR COMMANDS:\n");
    printf("  r - Read sensors once\n");
    printf("  c - Continuous reading (press any key to stop)\n");
    printf("  t - Sensor stress test (10 rapid reads)\n");
    printf("  s - Show sensor status\n");
    printf("  d - Show sensor data with timestamps\n");
    printf("  i - Scan I2C bus\n");
    printf("  v - Validate sensor readings (range check)\n");
    printf("\n");
    printf("NETWORK COMMANDS:\n");
    printf("  w - WiFi status & signal strength\n");
    printf("  h - Test HTTP connection once\n");
    printf("  H - HTTP stress test (5 requests)\n");
    printf("  n - Network diagnostics\n");
    printf("\n");
    printf("SYSTEM COMMANDS:\n");
    printf("  y - System status & health\n");
    printf("  m - Memory usage\n");
    printf("  u - Uptime & performance stats\n");
    printf("  R - Reset error counters\n");
    printf("  ? - Show this help\n");
    printf("========================================\n\n");
    printf("Ready for commands (press key + enter):\n");
    
    while (1) {
        c = getchar();
        if (c != '\n' && c != '\r' && c >= 0) {
            // Stop continuous mode if any key pressed
            if (continuous_mode) {
                continuous_mode = false;
                printf("Continuous mode stopped after %d reads.\n", continuous_read_count);
                printf("Ready for next command:\n");
                continue;
            }
            
            switch (c) {
                case 'r':
                case 'R':
                    printf("Reading sensors...\n");
                    uint32_t start_time = esp_timer_get_time() / 1000;
                    esp_err_t ret = sensor_manager_read_all();
                    uint32_t read_time = (esp_timer_get_time() / 1000) - start_time;
                    
                    if (ret == ESP_OK) {
                        printf("✓ Sensor read completed successfully (took %lu ms)\n", read_time);
                        print_sensor_data();
                    } else {
                        printf("✗ Sensor read failed: %s (took %lu ms)\n", 
                               esp_err_to_name(ret), read_time);
                    }
                    break;
                    
                case 'c':
                case 'C':
                    printf("Starting continuous sensor reading... (press any key to stop)\n");
                    continuous_mode = true;
                    continuous_read_count = 0;
                    while (continuous_mode) {
                        sensor_manager_read_all();
                        continuous_read_count++;
                        printf("Read #%d: ", continuous_read_count);
                        print_sensor_data_compact();
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        
                        // Check for input to stop
                        if (getchar() != EOF) {
                            continuous_mode = false;
                        }
                    }
                    break;
                    
                case 't':
                case 'T':
                    printf("Running sensor stress test (10 rapid reads)...\n");
                    int success_count = 0;
                    uint32_t total_time = esp_timer_get_time() / 1000;
                    
                    for (int i = 1; i <= 10; i++) {
                        uint32_t test_start = esp_timer_get_time() / 1000;
                        ret = sensor_manager_read_all();
                        uint32_t test_time = (esp_timer_get_time() / 1000) - test_start;
                        
                        if (ret == ESP_OK) {
                            success_count++;
                            printf("Test %d/10: ✓ (%lu ms)\n", i, test_time);
                        } else {
                            printf("Test %d/10: ✗ %s (%lu ms)\n", i, esp_err_to_name(ret), test_time);
                        }
                        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms between tests
                    }
                    total_time = (esp_timer_get_time() / 1000) - total_time;
                    printf("Stress test complete: %d/10 successful (total time: %lu ms)\n", 
                           success_count, total_time);
                    break;
                    
                case 's':
                case 'S':
                    print_sensor_status_enhanced();
                    break;
                    
                case 'd':
                case 'D':
                    print_sensor_data();
                    break;
                    
                case 'i':
                case 'I':
                    i2c_scanner();
                    break;
                    
                case 'v':
                case 'V':
                    validate_sensor_readings();
                    break;
                    
                case 'w':
                case 'W':
                    print_wifi_diagnostics();
                    break;
                    
                case 'h':
                    printf("Testing HTTP connection...\n");
                    start_time = esp_timer_get_time() / 1000;
                    ret = http_client_send_sensor_data();
                    uint32_t http_time = (esp_timer_get_time() / 1000) - start_time;
                    
                    if (ret == ESP_OK) {
                        printf("✓ HTTP test successful! (%lu ms)\n", http_time);
                        sys_status.http_success_count++;
                        sys_status.http_last_success = true;
                        sys_status.last_error[0] = '\0';
                    } else {
                        printf("✗ HTTP test failed: %s (%lu ms)\n", 
                               esp_err_to_name(ret), http_time);
                        sys_status.http_fail_count++;
                        sys_status.http_last_success = false;
                        snprintf(sys_status.last_error, sizeof(sys_status.last_error),
                                "HTTP: %s", esp_err_to_name(ret));
                    }
                    sys_status.last_http_time = esp_timer_get_time();
                    break;
                    
                case 'H':
                    printf("Running HTTP stress test (5 requests)...\n");
                    int http_success = 0;
                    for (int i = 1; i <= 5; i++) {
                        start_time = esp_timer_get_time() / 1000;
                        ret = http_client_send_sensor_data();
                        http_time = (esp_timer_get_time() / 1000) - start_time;
                        
                        if (ret == ESP_OK) {
                            http_success++;
                            printf("HTTP %d/5: ✓ (%lu ms)\n", i, http_time);
                        } else {
                            printf("HTTP %d/5: ✗ %s (%lu ms)\n", i, esp_err_to_name(ret), http_time);
                        }
                        vTaskDelay(pdMS_TO_TICKS(1000)); // 1s between requests
                    }
                    printf("HTTP stress test: %d/5 successful\n", http_success);
                    break;
                    
                case 'n':
                case 'N':
                    print_network_diagnostics();
                    break;
                    
                case 'y':
                case 'Y':
                    print_system_health();
                    break;
                    
                case 'm':
                case 'M':
                    print_memory_usage();
                    break;
                    
                case 'u':
                case 'U':
                    print_performance_stats();
                    break;
                    
                case 'R':
                    printf("Resetting all error counters...\n");
                    sys_status.http_success_count = 0;
                    sys_status.http_fail_count = 0;
                    sys_status.last_error[0] = '\0';
                    // Reset sensor error counts if possible
                    printf("✓ Error counters reset\n");
                    break;
                    
                case '?':
                    printf("\nCOMMANDS:\n");
                    printf("SENSORS: r=read, c=continuous, t=stress test, s=status, d=data, i=i2c scan, v=validate\n");
                    printf("NETWORK: w=wifi, h=http test, H=http stress, n=network diag\n");
                    printf("SYSTEM: y=health, m=memory, u=performance, R=reset counters\n");
                    printf("? = help\n\n");
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

// Helper function for compact sensor data display
static void print_sensor_data_compact(void) {
    sensor_data_t sht31_data, bmp280_data;
    esp_err_t ret = sensor_manager_get_latest_data(&sht31_data, &bmp280_data);
    
    if (ret == ESP_OK) {
        sensor_status_t status;
        sensor_manager_get_status(&status);
        
        if (status.sht31_found && status.bmp280_found) {
            printf("SHT31: %.1f°C %.1f%% | BMP280: %.1f°C %.0fPa\n", 
                   sht31_data.temperature, sht31_data.humidity,
                   bmp280_data.temperature, bmp280_data.pressure);
        } else if (status.sht31_found) {
            printf("SHT31: %.1f°C %.1f%%\n", sht31_data.temperature, sht31_data.humidity);
        } else if (status.bmp280_found) {
            printf("BMP280: %.1f°C %.0fPa\n", bmp280_data.temperature, bmp280_data.pressure);
        }
    }
}

// Enhanced sensor status with more details
static void print_sensor_status_enhanced(void) {
    printf("\n=== ENHANCED SENSOR STATUS ===\n");
    sensor_status_t status;
    esp_err_t ret = sensor_manager_get_status(&status);
    
    if (ret == ESP_OK) {
        printf("Total sensors: %d\n", status.total_sensors);
        printf("Success rate: %.1f%% (%lu successful, %lu errors)\n",
               status.read_count > 0 ? (float)(status.read_count - status.error_count) / status.read_count * 100 : 0,
               status.read_count - status.error_count, status.error_count);
        
        printf("\nSHT31 (Temperature/Humidity):\n");
        printf("  Status: %s", status.sht31_found ? "✓ FOUND" : "✗ NOT FOUND");
        if (status.sht31_found) printf(" at 0x%02X", status.sht31_address);
        printf("\n");
        
        printf("BMP280 (Pressure/Temperature):\n");
        printf("  Status: %s", status.bmp280_found ? "✓ FOUND" : "✗ NOT FOUND");
        if (status.bmp280_found) printf(" at 0x%02X", status.bmp280_address);
        printf("\n");
        
        if (status.last_read_time > 0) {
            uint32_t seconds_ago = (esp_timer_get_time() / 1000 - status.last_read_time) / 1000;
            printf("Last successful read: %lu seconds ago\n", seconds_ago);
        }
    } else {
        printf("Failed to get sensor status: %s\n", esp_err_to_name(ret));
    }
    printf("=============================\n\n");
}

// Validate sensor readings are in expected ranges
static void validate_sensor_readings(void) {
    printf("\n=== SENSOR VALIDATION ===\n");
    sensor_data_t sht31_data, bmp280_data;
    esp_err_t ret = sensor_manager_get_latest_data(&sht31_data, &bmp280_data);
    
    if (ret != ESP_OK) {
        printf("Cannot validate - no sensor data available\n");
        return;
    }
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    bool all_valid = true;
    
    if (status.sht31_found) {
        printf("SHT31 Validation:\n");
        // Temperature range: -40°C to +125°C
        if (sht31_data.temperature >= -40.0 && sht31_data.temperature <= 125.0) {
            printf("  Temperature: ✓ %.2f°C (valid range)\n", sht31_data.temperature);
        } else {
            printf("  Temperature: ✗ %.2f°C (OUT OF RANGE! Expected: -40 to 125°C)\n", sht31_data.temperature);
            all_valid = false;
        }
        
        // Humidity range: 0% to 100%
        if (sht31_data.humidity >= 0.0 && sht31_data.humidity <= 100.0) {
            printf("  Humidity: ✓ %.1f%% (valid range)\n", sht31_data.humidity);
        } else {
            printf("  Humidity: ✗ %.1f%% (OUT OF RANGE! Expected: 0-100%%)\n", sht31_data.humidity);
            all_valid = false;
        }
    }
    
    if (status.bmp280_found) {
        printf("BMP280 Validation:\n");
        // Temperature range: -40°C to +85°C
        if (bmp280_data.temperature >= -40.0 && bmp280_data.temperature <= 85.0) {
            printf("  Temperature: ✓ %.2f°C (valid range)\n", bmp280_data.temperature);
        } else {
            printf("  Temperature: ✗ %.2f°C (OUT OF RANGE! Expected: -40 to 85°C)\n", bmp280_data.temperature);
            all_valid = false;
        }
        
        // Pressure range: 30000 to 110000 Pa
        if (bmp280_data.pressure >= 30000.0 && bmp280_data.pressure <= 110000.0) {
            printf("  Pressure: ✓ %.0f Pa (valid range)\n", bmp280_data.pressure);
        } else {
            printf("  Pressure: ✗ %.0f Pa (OUT OF RANGE! Expected: 30000-110000 Pa)\n", bmp280_data.pressure);
            all_valid = false;
        }
    }
    
    printf("\nOverall validation: %s\n", all_valid ? "✓ ALL READINGS VALID" : "✗ SOME READINGS INVALID");
    printf("========================\n\n");
}

// WiFi diagnostics
static void print_wifi_diagnostics(void) {
    printf("\n=== WiFi DIAGNOSTICS ===\n");
    printf("Connection: %s\n", sys_status.wifi_connected ? "✓ CONNECTED" : "✗ DISCONNECTED");
    
    if (sys_status.wifi_connected) {
        // Try to get more WiFi info if available
        printf("Network: %s\n", WIFI_SSID);
        printf("Status: Ready for HTTP requests\n");
    }
    printf("=======================\n\n");
}

// Network diagnostics
static void print_network_diagnostics(void) {
    printf("\n=== NETWORK DIAGNOSTICS ===\n");
    printf("WiFi: %s\n", sys_status.wifi_connected ? "Connected" : "Disconnected");
    printf("Server: %s:%d\n", SERVER_IP, SERVER_PORT);
    printf("Endpoint: %s\n", SERVER_ENDPOINT);
    printf("HTTP Success: %lu\n", sys_status.http_success_count);
    printf("HTTP Failures: %lu\n", sys_status.http_fail_count);
    
    if (strlen(sys_status.last_error) > 0) {
        printf("Last Error: %s\n", sys_status.last_error);
    }
    printf("=========================\n\n");
}

// System health check
static void print_system_health(void) {
    printf("\n=== SYSTEM HEALTH ===\n");
    printf("Uptime: %.2f minutes\n", esp_timer_get_time() / 60000000.0);
    printf("WiFi: %s\n", sys_status.wifi_connected ? "✓ Connected" : "✗ Disconnected");
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    printf("Sensors: %d found\n", status.total_sensors);
    
    if (status.read_count > 0) {
        float success_rate = (float)(status.read_count - status.error_count) / status.read_count * 100;
        printf("Sensor Success Rate: %.1f%%\n", success_rate);
    }
    
    printf("Free Heap: %lu bytes\n", esp_get_free_heap_size());
    printf("Min Free Heap: %lu bytes\n", esp_get_minimum_free_heap_size());
    printf("=====================\n\n");
}

// Memory usage
static void print_memory_usage(void) {
    printf("\n=== MEMORY USAGE ===\n");
    printf("Free Heap: %lu bytes\n", esp_get_free_heap_size());
    printf("Minimum Free Heap: %lu bytes\n", esp_get_minimum_free_heap_size());
    printf("Largest Free Block: %lu bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    printf("===================\n\n");
}

// Performance statistics
static void print_performance_stats(void) {
    printf("\n=== PERFORMANCE STATS ===\n");
    printf("System Uptime: %.2f minutes\n", esp_timer_get_time() / 60000000.0);
    printf("CPU Frequency: %lu MHz\n", rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get()) / 1000000);
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    printf("Total Sensor Reads: %lu\n", status.read_count);
    printf("Sensor Errors: %lu\n", status.error_count);
    printf("HTTP Requests: %lu successful, %lu failed\n", 
           sys_status.http_success_count, sys_status.http_fail_count);
    printf("========================\n\n");
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
static void print_sensor_data_compact(void);
static void print_sensor_status_enhanced(void);
static void validate_sensor_readings(void);
static void print_wifi_diagnostics(void);
static void print_network_diagnostics(void);
static void print_system_health(void);
static void print_memory_usage(void);
static void print_performance_stats(void);
#endif

#if DEBUG_MODE
static void debug_control_task(void *pvParameters) {
    int c;
    int continuous_read_count = 0;
    bool continuous_mode = false;
    
    printf("\n========================================\n");
    printf("    ESP32 Environmental Monitor\n");
    printf("        ENHANCED TESTING INTERFACE\n");
    printf("========================================\n");
    printf("SENSOR COMMANDS:\n");
    printf("  r - Read sensors once\n");
    printf("  c - Continuous reading (press any key to stop)\n");
    printf("  t - Sensor stress test (10 rapid reads)\n");
    printf("  s - Show sensor status\n");
    printf("  d - Show sensor data with timestamps\n");
    printf("  i - Scan I2C bus\n");
    printf("  v - Validate sensor readings (range check)\n");
    printf("\n");
    printf("NETWORK COMMANDS:\n");
    printf("  w - WiFi status & signal strength\n");
    printf("  h - Test HTTP connection once\n");
    printf("  H - HTTP stress test (5 requests)\n");
    printf("  n - Network diagnostics\n");
    printf("\n");
    printf("SYSTEM COMMANDS:\n");
    printf("  y - System status & health\n");
    printf("  m - Memory usage\n");
    printf("  u - Uptime & performance stats\n");
    printf("  x - Reset error counters\n");  // Changed from 'R' to 'x'
    printf("  ? - Show this help\n");
    printf("========================================\n\n");
    printf("Ready for commands (press key + enter):\n");
    
    while (1) {
        c = getchar();
        if (c != '\n' && c != '\r' && c >= 0) {
            // Stop continuous mode if any key pressed
            if (continuous_mode) {
                continuous_mode = false;
                printf("Continuous mode stopped after %d reads.\n", continuous_read_count);
                printf("Ready for next command:\n");
                continue;
            }
            
            switch (c) {
                case 'r':
                    printf("Reading sensors...\n");
                    uint32_t start_time = esp_timer_get_time() / 1000;
                    esp_err_t ret = sensor_manager_read_all();
                    uint32_t read_time = (esp_timer_get_time() / 1000) - start_time;
                    
                    if (ret == ESP_OK) {
                        printf("✓ Sensor read completed successfully (took %lu ms)\n", read_time);
                        print_sensor_data();
                    } else {
                        printf("✗ Sensor read failed: %s (took %lu ms)\n", 
                               esp_err_to_name(ret), read_time);
                    }
                    break;
                    
                case 'c':
                    printf("Starting continuous sensor reading... (press any key to stop)\n");
                    continuous_mode = true;
                    continuous_read_count = 0;
                    while (continuous_mode) {
                        sensor_manager_read_all();
                        continuous_read_count++;
                        printf("Read #%d: ", continuous_read_count);
                        print_sensor_data_compact();
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        
                        // Check for input to stop
                        if (getchar() != EOF) {
                            continuous_mode = false;
                        }
                    }
                    break;
                    
                case 't':
                    printf("Running sensor stress test (10 rapid reads)...\n");
                    int success_count = 0;
                    uint32_t total_time = esp_timer_get_time() / 1000;
                    
                    for (int i = 1; i <= 10; i++) {
                        uint32_t test_start = esp_timer_get_time() / 1000;
                        ret = sensor_manager_read_all();
                        uint32_t test_time = (esp_timer_get_time() / 1000) - test_start;
                        
                        if (ret == ESP_OK) {
                            success_count++;
                            printf("Test %d/10: ✓ (%lu ms)\n", i, test_time);
                        } else {
                            printf("Test %d/10: ✗ %s (%lu ms)\n", i, esp_err_to_name(ret), test_time);
                        }
                        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms between tests
                    }
                    total_time = (esp_timer_get_time() / 1000) - total_time;
                    printf("Stress test complete: %d/10 successful (total time: %lu ms)\n", 
                           success_count, total_time);
                    break;
                    
                case 's':
                    print_sensor_status_enhanced();
                    break;
                    
                case 'd':
                    print_sensor_data();
                    break;
                    
                case 'i':
                    i2c_scanner();
                    break;
                    
                case 'v':
                    validate_sensor_readings();
                    break;
                    
                case 'w':
                    print_wifi_diagnostics();
                    break;
                    
                case 'h':
                    printf("Testing HTTP connection...\n");
                    start_time = esp_timer_get_time() / 1000;
                    ret = http_client_send_sensor_data();
                    uint32_t http_time = (esp_timer_get_time() / 1000) - start_time;
                    
                    if (ret == ESP_OK) {
                        printf("✓ HTTP test successful! (%lu ms)\n", http_time);
                        sys_status.http_success_count++;
                        sys_status.http_last_success = true;
                        sys_status.last_error[0] = '\0';
                    } else {
                        printf("✗ HTTP test failed: %s (%lu ms)\n", 
                               esp_err_to_name(ret), http_time);
                        sys_status.http_fail_count++;
                        sys_status.http_last_success = false;
                        snprintf(sys_status.last_error, sizeof(sys_status.last_error),
                                "HTTP: %s", esp_err_to_name(ret));
                    }
                    sys_status.last_http_time = esp_timer_get_time();
                    break;
                    
                case 'H':
                    printf("Running HTTP stress test (5 requests)...\n");
                    int http_success = 0;
                    for (int i = 1; i <= 5; i++) {
                        start_time = esp_timer_get_time() / 1000;
                        ret = http_client_send_sensor_data();
                        http_time = (esp_timer_get_time() / 1000) - start_time;
                        
                        if (ret == ESP_OK) {
                            http_success++;
                            printf("HTTP %d/5: ✓ (%lu ms)\n", i, http_time);
                        } else {
                            printf("HTTP %d/5: ✗ %s (%lu ms)\n", i, esp_err_to_name(ret), http_time);
                        }
                        vTaskDelay(pdMS_TO_TICKS(1000)); // 1s between requests
                    }
                    printf("HTTP stress test: %d/5 successful\n", http_success);
                    break;
                    
                case 'n':
                    print_network_diagnostics();
                    break;
                    
                case 'y':
                    print_system_health();
                    break;
                    
                case 'm':
                    print_memory_usage();
                    break;
                    
                case 'u':
                    print_performance_stats();
                    break;
                    
                case 'x':  // Changed from 'R' to avoid duplicate
                    printf("Resetting all error counters...\n");
                    sys_status.http_success_count = 0;
                    sys_status.http_fail_count = 0;
                    sys_status.last_error[0] = '\0';
                    printf("✓ Error counters reset\n");
                    break;
                    
                case '?':
                    printf("\nCOMMANDS:\n");
                    printf("SENSORS: r=read, c=continuous, t=stress test, s=status, d=data, i=i2c scan, v=validate\n");
                    printf("NETWORK: w=wifi, h=http test, H=http stress, n=network diag\n");
                    printf("SYSTEM: y=health, m=memory, u=performance, x=reset counters\n");
                    printf("? = help\n\n");
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

// Helper function implementations - add these AFTER the debug_control_task function:

static void print_sensor_data_compact(void) {
    sensor_data_t sht31_data, bmp280_data;
    esp_err_t ret = sensor_manager_get_latest_data(&sht31_data, &bmp280_data);
    
    if (ret == ESP_OK) {
        sensor_status_t status;
        sensor_manager_get_status(&status);
        
        if (status.sht31_found && status.bmp280_found) {
            printf("SHT31: %.1f°C %.1f%% | BMP280: %.1f°C %.0fPa\n", 
                   sht31_data.temperature, sht31_data.humidity,
                   bmp280_data.temperature, bmp280_data.pressure);
        } else if (status.sht31_found) {
            printf("SHT31: %.1f°C %.1f%%\n", sht31_data.temperature, sht31_data.humidity);
        } else if (status.bmp280_found) {
            printf("BMP280: %.1f°C %.0fPa\n", bmp280_data.temperature, bmp280_data.pressure);
        }
    }
}

static void print_sensor_status_enhanced(void) {
    printf("\n=== ENHANCED SENSOR STATUS ===\n");
    sensor_status_t status;
    esp_err_t ret = sensor_manager_get_status(&status);
    
    if (ret == ESP_OK) {
        printf("Total sensors: %d\n", status.total_sensors);
        printf("Success rate: %.1f%% (%lu successful, %lu errors)\n",
               status.read_count > 0 ? (float)(status.read_count - status.error_count) / status.read_count * 100 : 0,
               status.read_count - status.error_count, status.error_count);
        
        printf("\nSHT31 (Temperature/Humidity):\n");
        printf("  Status: %s", status.sht31_found ? "✓ FOUND" : "✗ NOT FOUND");
        if (status.sht31_found) printf(" at 0x%02X", status.sht31_address);
        printf("\n");
        
        printf("BMP280 (Pressure/Temperature):\n");
        printf("  Status: %s", status.bmp280_found ? "✓ FOUND" : "✗ NOT FOUND");
        if (status.bmp280_found) printf(" at 0x%02X", status.bmp280_address);
        printf("\n");
        
        if (status.last_read_time > 0) {
            uint32_t seconds_ago = (esp_timer_get_time() / 1000 - status.last_read_time) / 1000;
            printf("Last successful read: %lu seconds ago\n", seconds_ago);
        }
    } else {
        printf("Failed to get sensor status: %s\n", esp_err_to_name(ret));
    }
    printf("=============================\n\n");
}

static void validate_sensor_readings(void) {
    printf("\n=== SENSOR VALIDATION ===\n");
    sensor_data_t sht31_data, bmp280_data;
    esp_err_t ret = sensor_manager_get_latest_data(&sht31_data, &bmp280_data);
    
    if (ret != ESP_OK) {
        printf("Cannot validate - no sensor data available\n");
        return;
    }
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    bool all_valid = true;
    
    if (status.sht31_found) {
        printf("SHT31 Validation:\n");
        if (sht31_data.temperature >= -40.0 && sht31_data.temperature <= 125.0) {
            printf("  Temperature: ✓ %.2f°C (valid range)\n", sht31_data.temperature);
        } else {
            printf("  Temperature: ✗ %.2f°C (OUT OF RANGE! Expected: -40 to 125°C)\n", sht31_data.temperature);
            all_valid = false;
        }
        
        if (sht31_data.humidity >= 0.0 && sht31_data.humidity <= 100.0) {
            printf("  Humidity: ✓ %.1f%% (valid range)\n", sht31_data.humidity);
        } else {
            printf("  Humidity: ✗ %.1f%% (OUT OF RANGE! Expected: 0-100%%)\n", sht31_data.humidity);
            all_valid = false;
        }
    }
    
    if (status.bmp280_found) {
        printf("BMP280 Validation:\n");
        if (bmp280_data.temperature >= -40.0 && bmp280_data.temperature <= 85.0) {
            printf("  Temperature: ✓ %.2f°C (valid range)\n", bmp280_data.temperature);
        } else {
            printf("  Temperature: ✗ %.2f°C (OUT OF RANGE! Expected: -40 to 85°C)\n", bmp280_data.temperature);
            all_valid = false;
        }
        
        if (bmp280_data.pressure >= 30000.0 && bmp280_data.pressure <= 110000.0) {
            printf("  Pressure: ✓ %.0f Pa (valid range)\n", bmp280_data.pressure);
        } else {
            printf("  Pressure: ✗ %.0f Pa (OUT OF RANGE! Expected: 30000-110000 Pa)\n", bmp280_data.pressure);
            all_valid = false;
        }
    }
    
    printf("\nOverall validation: %s\n", all_valid ? "✓ ALL READINGS VALID" : "✗ SOME READINGS INVALID");
    printf("========================\n\n");
}

static void print_wifi_diagnostics(void) {
    printf("\n=== WiFi DIAGNOSTICS ===\n");
    printf("Connection: %s\n", sys_status.wifi_connected ? "✓ CONNECTED" : "✗ DISCONNECTED");
    
    if (sys_status.wifi_connected) {
        printf("Network: %s\n", WIFI_SSID);
        printf("Status: Ready for HTTP requests\n");
    }
    printf("=======================\n\n");
}

static void print_network_diagnostics(void) {
    printf("\n=== NETWORK DIAGNOSTICS ===\n");
    printf("WiFi: %s\n", sys_status.wifi_connected ? "Connected" : "Disconnected");
    printf("Server: %s:%d\n", SERVER_IP, SERVER_PORT);
    printf("Endpoint: %s\n", SERVER_ENDPOINT);
    printf("HTTP Success: %lu\n", sys_status.http_success_count);
    printf("HTTP Failures: %lu\n", sys_status.http_fail_count);
    
    if (strlen(sys_status.last_error) > 0) {
        printf("Last Error: %s\n", sys_status.last_error);
    }
    printf("=========================\n\n");
}

static void print_system_health(void) {
    printf("\n=== SYSTEM HEALTH ===\n");
    printf("Uptime: %.2f minutes\n", esp_timer_get_time() / 60000000.0);
    printf("WiFi: %s\n", sys_status.wifi_connected ? "✓ Connected" : "✗ Disconnected");
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    printf("Sensors: %d found\n", status.total_sensors);
    
    if (status.read_count > 0) {
        float success_rate = (float)(status.read_count - status.error_count) / status.read_count * 100;
        printf("Sensor Success Rate: %.1f%%\n", success_rate);
    }
    
    printf("Free Heap: %u bytes\n", (unsigned int)esp_get_free_heap_size());
    printf("Min Free Heap: %u bytes\n", (unsigned int)esp_get_minimum_free_heap_size());
    printf("=====================\n\n");
}

static void print_memory_usage(void) {
    printf("\n=== MEMORY USAGE ===\n");
    printf("Free Heap: %u bytes\n", (unsigned int)esp_get_free_heap_size());
    printf("Minimum Free Heap: %u bytes\n", (unsigned int)esp_get_minimum_free_heap_size());
    printf("Largest Free Block: %u bytes\n", (unsigned int)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    printf("===================\n\n");
}

static void print_performance_stats(void) {
    printf("\n=== PERFORMANCE STATS ===\n");
    printf("System Uptime: %.2f minutes\n", esp_timer_get_time() / 60000000.0);
    printf("CPU Frequency: 160 MHz\n");  // Fixed value instead of function call
    
    sensor_status_t status;
    sensor_manager_get_status(&status);
    printf("Total Sensor Reads: %lu\n", status.read_count);
    printf("Sensor Errors: %lu\n", status.error_count);
    printf("HTTP Requests: %lu successful, %lu failed\n", 
           sys_status.http_success_count, sys_status.http_fail_count);
    printf("========================\n\n");
}
#endif // DEBUG_MODE