#include "sensor_manager.h"
#include "sht31.h"
#include "bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SENSOR_MGR";

// Sea level pressure for altitude calculation (Pa)
#define SEA_LEVEL_PRESSURE 101325.0

// Manager state
static struct {
    bool initialized;
    sensor_manager_config_t config;
    sensor_status_t status;
    sensor_data_t sht31_data;
    sensor_data_t bmp280_data;
    sht31_t sht31_dev;
    bmp280_t bmp280_dev;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t read_task_handle;
} manager = {0};

// I2C initialization
static esp_err_t init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = manager.config.sda_pin,
        .scl_io_num = manager.config.scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };
    
    esp_err_t ret = i2c_param_config(manager.config.i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(manager.config.i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized on port %d (SDA:%d, SCL:%d)", 
             manager.config.i2c_port, manager.config.sda_pin, manager.config.scl_pin);
    return ESP_OK;
}

// Auto-read task
static void sensor_read_task(void *pvParameters) {
    ESP_LOGI(TAG, "Auto-read task started");
    
    while (manager.config.auto_read_enabled) {
        sensor_manager_read_all();
        vTaskDelay(pdMS_TO_TICKS(manager.config.read_interval_ms));
    }
    
    ESP_LOGI(TAG, "Auto-read task stopped");
    manager.read_task_handle = NULL;
    vTaskDelete(NULL);
}

// Initialize sensor manager
esp_err_t sensor_manager_init(const sensor_manager_config_t *config) {
    if (manager.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy configuration
    memcpy(&manager.config, config, sizeof(sensor_manager_config_t));
    
    // Create mutex
    manager.data_mutex = xSemaphoreCreateMutex();
    if (manager.data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize I2C
    esp_err_t ret = init_i2c();
    if (ret != ESP_OK) {
        vSemaphoreDelete(manager.data_mutex);
        return ret;
    }
    
    manager.initialized = true;
    ESP_LOGI(TAG, "Sensor manager initialized");
    
    // Scan for sensors
    sensor_manager_scan();
    
    return ESP_OK;
}

// Deinitialize sensor manager
esp_err_t sensor_manager_deinit(void) {
    if (!manager.initialized) {
        return ESP_OK;
    }
    
    // Stop auto-read if running
    sensor_manager_stop_auto_read();
    
    // Deinitialize sensors
    if (manager.status.bmp280_found) {
        bmp280_deinit(&manager.bmp280_dev);
    }
    
    // Delete I2C driver
    i2c_driver_delete(manager.config.i2c_port);
    
    // Delete mutex
    if (manager.data_mutex != NULL) {
        vSemaphoreDelete(manager.data_mutex);
        manager.data_mutex = NULL;
    }
    
    manager.initialized = false;
    ESP_LOGI(TAG, "Sensor manager deinitialized");
    
    return ESP_OK;
}

// Scan for sensors
esp_err_t sensor_manager_scan(void) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Scanning for sensors...");
    
    // Reset status
    manager.status.sht31_found = false;
    manager.status.bmp280_found = false;
    manager.status.total_sensors = 0;
    
    // Try to initialize SHT31
    esp_err_t ret = sht31_init(&manager.sht31_dev);
    if (ret == ESP_OK) {
        manager.status.sht31_found = true;
        manager.status.sht31_address = manager.sht31_dev.i2c_addr;
        manager.status.total_sensors++;
        ESP_LOGI(TAG, "SHT31 found at address 0x%02X", manager.status.sht31_address);
    } else {
        ESP_LOGW(TAG, "SHT31 not found");
    }
    
    // Try to initialize BMP280
    ret = bmp280_init_default(&manager.bmp280_dev, manager.config.i2c_port);
    if (ret == ESP_OK) {
        manager.status.bmp280_found = true;
        manager.status.bmp280_address = manager.bmp280_dev.i2c_addr;
        manager.status.total_sensors++;
        ESP_LOGI(TAG, "BMP280 found at address 0x%02X", manager.status.bmp280_address);
    } else {
        ESP_LOGW(TAG, "BMP280 not found");
    }
    
    ESP_LOGI(TAG, "Scan complete: %d sensor(s) found", manager.status.total_sensors);
    
    return (manager.status.total_sensors > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// Helper function to read SHT31 sensor
static esp_err_t read_sht31_sensor(uint32_t timestamp) {
    float temperature, humidity;
    esp_err_t ret = sht31_read_temperature_humidity(&manager.sht31_dev, &temperature, &humidity);
    if (ret == ESP_OK) {
        if (temperature < -40.0 || temperature > 125.0 ||
            humidity < 0.0 || humidity > 100.0) {
            ESP_LOGW(TAG, "SHT31 readings out of range: T=%.2f°C, H=%.1f%%",
                     temperature, humidity);
            manager.status.error_count++;
            return ESP_ERR_INVALID_RESPONSE;
        }
        
        manager.sht31_data.temperature = temperature;
        manager.sht31_data.humidity = humidity;
        manager.sht31_data.dew_point = sensor_manager_calculate_dew_point(
            temperature, humidity);
        manager.sht31_data.has_temperature = true;
        manager.sht31_data.has_humidity = true;
        manager.sht31_data.has_pressure = false;
        manager.sht31_data.timestamp = timestamp;
        
        ESP_LOGI(TAG, "SHT31: T=%.2f°C, H=%.1f%%, DP=%.1f°C", 
                 manager.sht31_data.temperature,
                 manager.sht31_data.humidity,
                 manager.sht31_data.dew_point);
    } else {
        ESP_LOGE(TAG, "Failed to read SHT31: %s", esp_err_to_name(ret));
        manager.status.error_count++;
    }
    return ret;
}

// Helper function to read BMP280 sensor
static esp_err_t read_bmp280_sensor(uint32_t timestamp) {
    float temperature, pressure;
    esp_err_t ret = bmp280_read_float(&manager.bmp280_dev, &temperature, &pressure);
    if (ret == ESP_OK) {
        if (temperature < -40.0 || temperature > 85.0 ||
            pressure < 30000.0 || pressure > 110000.0) {
            ESP_LOGW(TAG, "BMP280 readings out of range: T=%.2f°C, P=%.0fPa",
                     temperature, pressure);
            manager.status.error_count++;
            return ESP_ERR_INVALID_RESPONSE;
        }
        
        manager.bmp280_data.temperature = temperature;
        manager.bmp280_data.pressure = pressure;
        manager.bmp280_data.altitude = sensor_manager_calculate_altitude(pressure);
        manager.bmp280_data.has_temperature = true;
        manager.bmp280_data.has_humidity = false;
        manager.bmp280_data.has_pressure = true;
        manager.bmp280_data.timestamp = timestamp;
        
        ESP_LOGI(TAG, "BMP280: T=%.2f°C, P=%.0fPa, Alt=%.1fm", 
                 manager.bmp280_data.temperature,
                 manager.bmp280_data.pressure,
                 manager.bmp280_data.altitude);
    } else {
        ESP_LOGE(TAG, "Failed to read BMP280: %s", esp_err_to_name(ret));
        manager.status.error_count++;
    }
    return ret;
}

esp_err_t sensor_manager_read_all(void) {
    if (!manager.initialized) {
        ESP_LOGE(TAG, "Sensor manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    esp_err_t final_ret = ESP_OK;
    
    if (xSemaphoreTake(manager.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex - sensor read timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    uint32_t timestamp = esp_timer_get_time() / 1000; // Convert to ms
    
    // Read SHT31
    if (manager.status.sht31_found) {
        ret = read_sht31_sensor(timestamp);
        if (ret != ESP_OK) final_ret = ret;
    }
    
    // Read BMP280
    if (manager.status.bmp280_found) {
        ret = read_bmp280_sensor(timestamp);
        if (ret != ESP_OK) final_ret = ret;
    }
    
    // Update status even if there were errors
    manager.status.last_read_time = timestamp;
    manager.status.read_count++;
    
    xSemaphoreGive(manager.data_mutex);
    
    if (final_ret != ESP_OK) {
        ESP_LOGW(TAG, "One or more sensors failed to read");
    }
    
    return final_ret;
}
    


// Get sensor data by type
esp_err_t sensor_manager_get_sensor_data(sensor_type_t sensor, sensor_data_t *data) {
    if (!manager.initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(manager.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    switch (sensor) {
        case SENSOR_TYPE_SHT31:
            if (manager.status.sht31_found) {
                memcpy(data, &manager.sht31_data, sizeof(sensor_data_t));
                xSemaphoreGive(manager.data_mutex);
                return ESP_OK;
            }
            break;

        case SENSOR_TYPE_BMP280:
            if (manager.status.bmp280_found) {
                memcpy(data, &manager.bmp280_data, sizeof(sensor_data_t));
                xSemaphoreGive(manager.data_mutex);
                return ESP_OK;
            }
            break;

        default:
            break;
    }

    xSemaphoreGive(manager.data_mutex);
    return ESP_ERR_NOT_FOUND;
}

// Get latest data from all sensors
esp_err_t sensor_manager_get_latest_data(sensor_data_t *sht31_data, sensor_data_t *bmp280_data) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(manager.data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (sht31_data != NULL && manager.status.sht31_found) {
        memcpy(sht31_data, &manager.sht31_data, sizeof(sensor_data_t));
    }
    
    if (bmp280_data != NULL && manager.status.bmp280_found) {
        memcpy(bmp280_data, &manager.bmp280_data, sizeof(sensor_data_t));
    }
    
    xSemaphoreGive(manager.data_mutex);
    
    return ESP_OK;
}

// Get status
esp_err_t sensor_manager_get_status(sensor_status_t *status) {
    if (!manager.initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(status, &manager.status, sizeof(sensor_status_t));
    return ESP_OK;
}

// Start auto-read
esp_err_t sensor_manager_start_auto_read(void) {
    if (!manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (manager.config.auto_read_enabled) {
        ESP_LOGW(TAG, "Auto-read already enabled");
        return ESP_OK;
    }
    
    manager.config.auto_read_enabled = true;
    
    if (xTaskCreate(sensor_read_task, "sensor_read", 4096, NULL, 5, &manager.read_task_handle) != pdPASS) {
        manager.config.auto_read_enabled = false;
        ESP_LOGE(TAG, "Failed to create read task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Auto-read started (interval: %lums)", manager.config.read_interval_ms);
    return ESP_OK;
}

// Stop auto-read
esp_err_t sensor_manager_stop_auto_read(void) {
    if (!manager.config.auto_read_enabled) {
        return ESP_OK;
    }
    
    manager.config.auto_read_enabled = false;
    
    // Wait for task to finish
    if (manager.read_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(manager.config.read_interval_ms + 100));
    }
    
    ESP_LOGI(TAG, "Auto-read stopped");
    return ESP_OK;
}

// Calculate altitude from pressure
float sensor_manager_calculate_altitude(float pressure) {
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Calculate dew point
float sensor_manager_calculate_dew_point(float temperature, float humidity) {
    if (humidity <= 0) return temperature;
    float a = 17.27;
    float b = 237.7;
    float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

// Print status
void sensor_manager_print_status(void) {
    printf("\n========== Sensor Status ==========\n");
    printf("Initialized: %s\n", manager.initialized ? "Yes" : "No");
    printf("SHT31: %s", manager.status.sht31_found ? "Found" : "Not found");
    if (manager.status.sht31_found) {
        printf(" (0x%02X)", manager.status.sht31_address);
    }
    printf("\n");
    printf("BMP280: %s", manager.status.bmp280_found ? "Found" : "Not found");
    if (manager.status.bmp280_found) {
        printf(" (0x%02X)", manager.status.bmp280_address);
    }
    printf("\n");
    printf("Total sensors: %d\n", manager.status.total_sensors);
    printf("Read count: %lu\n", manager.status.read_count);
    printf("Error count: %lu\n", manager.status.error_count);
    printf("Auto-read: %s\n", manager.config.auto_read_enabled ? "Enabled" : "Disabled");
    printf("===================================\n\n");
}

// Get sensor data
esp_err_t sensor_manager_get_data(sensor_type_t sensor_type, sensor_data_t *data) {
    if (!manager.initialized || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (pdTRUE != xSemaphoreTake(manager.data_mutex, pdMS_TO_TICKS(1000))) {
        ESP_LOGE(TAG, "Failed to take data mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = ESP_OK;
    switch (sensor_type) {
        case SENSOR_TYPE_SHT31:
            if (!manager.status.sht31_found) {
                ret = ESP_ERR_NOT_FOUND;
            } else {
                memcpy(data, &manager.sht31_data, sizeof(sensor_data_t));
            }
            break;
            
        case SENSOR_TYPE_BMP280:
            if (!manager.status.bmp280_found) {
                ret = ESP_ERR_NOT_FOUND;
            } else {
                memcpy(data, &manager.bmp280_data, sizeof(sensor_data_t));
            }
            break;
            
        default:
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    xSemaphoreGive(manager.data_mutex);
    return ret;
}

// Print sensor data
void sensor_manager_print_data(void) {
    printf("\n========== Sensor Data ==========\n");
    
    if (manager.status.sht31_found) {
        printf("SHT31:\n");
        printf("  Temperature: %.2f°C\n", manager.sht31_data.temperature);
        printf("  Humidity: %.1f%%\n", manager.sht31_data.humidity);
        printf("  Dew Point: %.1f°C\n", manager.sht31_data.dew_point);
    }
    
    if (manager.status.bmp280_found) {
        printf("BMP280:\n");
        printf("  Temperature: %.2f°C\n", manager.bmp280_data.temperature);
        printf("  Pressure: %.0f Pa (%.2f hPa)\n", 
               manager.bmp280_data.pressure, manager.bmp280_data.pressure / 100.0);
        printf("  Altitude: %.1f m\n", manager.bmp280_data.altitude);
    }
    
    printf("=================================\n\n");
}