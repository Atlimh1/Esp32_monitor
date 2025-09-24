#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>

// Sensor types
typedef enum {
    SENSOR_TYPE_SHT31,
    SENSOR_TYPE_BMP280,
    SENSOR_TYPE_UNKNOWN
} sensor_type_t;

// Sensor data structure
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    float altitude;
    float dew_point;
    bool has_temperature;
    bool has_humidity;
    bool has_pressure;
    uint32_t timestamp;
} sensor_data_t;

// Sensor status
typedef struct {
    bool sht31_found;
    bool bmp280_found;
    uint8_t sht31_address;
    uint8_t bmp280_address;
    int total_sensors;
    uint32_t last_read_time;
    uint32_t read_count;
    uint32_t error_count;
} sensor_status_t;

// Configuration
typedef struct {
    int i2c_port;
    int sda_pin;
    int scl_pin;
    uint32_t read_interval_ms;
    bool auto_read_enabled;
} sensor_manager_config_t;

// Function prototypes
esp_err_t sensor_manager_init(const sensor_manager_config_t *config);
esp_err_t sensor_manager_deinit(void);
esp_err_t sensor_manager_scan(void);
esp_err_t sensor_manager_read_all(void);
esp_err_t sensor_manager_get_data(sensor_type_t sensor, sensor_data_t *data);
esp_err_t sensor_manager_get_latest_data(sensor_data_t *sht31_data, sensor_data_t *bmp280_data);
esp_err_t sensor_manager_get_status(sensor_status_t *status);
esp_err_t sensor_manager_start_auto_read(void);
esp_err_t sensor_manager_stop_auto_read(void);
void sensor_manager_print_status(void);
void sensor_manager_print_data(void);

// Utility functions
float sensor_manager_calculate_altitude(float pressure);
float sensor_manager_calculate_dew_point(float temperature, float humidity);

#endif // SENSOR_MANAGER_H