#include "bmp280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BMP280";

// Helper functions for I2C communication
static esp_err_t bmp280_read_register(bmp280_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bmp280_write_register(bmp280_t *dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read calibration data from sensor
static esp_err_t bmp280_read_calibration(bmp280_t *dev) {
    uint8_t calib[24];
    esp_err_t ret = bmp280_read_register(dev, BMP280_REG_CALIB00, calib, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }
    
    dev->calib.dig_T1 = (calib[1] << 8) | calib[0];
    dev->calib.dig_T2 = (calib[3] << 8) | calib[2];
    dev->calib.dig_T3 = (calib[5] << 8) | calib[4];
    dev->calib.dig_P1 = (calib[7] << 8) | calib[6];
    dev->calib.dig_P2 = (calib[9] << 8) | calib[8];
    dev->calib.dig_P3 = (calib[11] << 8) | calib[10];
    dev->calib.dig_P4 = (calib[13] << 8) | calib[12];
    dev->calib.dig_P5 = (calib[15] << 8) | calib[14];
    dev->calib.dig_P6 = (calib[17] << 8) | calib[16];
    dev->calib.dig_P7 = (calib[19] << 8) | calib[18];
    dev->calib.dig_P8 = (calib[21] << 8) | calib[20];
    dev->calib.dig_P9 = (calib[23] << 8) | calib[22];
    
    ESP_LOGI(TAG, "Calibration data loaded");
    return ESP_OK;
}

// Temperature compensation
static int32_t bmp280_compensate_temperature(bmp280_t *dev, int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * 
            ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * 
             ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * 
            ((int32_t)dev->calib.dig_T3)) >> 14;
    
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T; // Temperature in 0.01°C
}

// Pressure compensation
static uint32_t bmp280_compensate_pressure(bmp280_t *dev, int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + 
           ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
    
    return (uint32_t)p; // Pressure in Pa * 256
}

// Initialize BMP280
esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t i2c_port, uint8_t addr) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(dev, 0, sizeof(bmp280_t));
    dev->i2c_port = i2c_port;
    dev->i2c_addr = addr;
    
    // Read and verify chip ID
    uint8_t chip_id;
    esp_err_t ret = bmp280_read_register(dev, BMP280_REG_CHIPID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    
    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X", chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    dev->chip_id = chip_id;
    ESP_LOGI(TAG, "%s detected (chip_id=0x%02X)", 
             chip_id == BMP280_CHIP_ID ? "BMP280" : "BME280", chip_id);
    
    // Soft reset
    ret = bmp280_soft_reset(dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read calibration data
    ret = bmp280_read_calibration(dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set default configuration
    dev->config.mode = BMP280_MODE_NORMAL;
    dev->config.temp_oversampling = BMP280_OVERSAMPLE_X2;
    dev->config.press_oversampling = BMP280_OVERSAMPLE_X16;
    dev->config.filter = BMP280_FILTER_X16;
    dev->config.standby = BMP280_STANDBY_500;
    
    ret = bmp280_set_config(dev, &dev->config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    dev->initialized = true;
    ESP_LOGI(TAG, "BMP280 initialized successfully");
    return ESP_OK;
}

// Initialize with default I2C address
esp_err_t bmp280_init_default(bmp280_t *dev, i2c_port_t i2c_port) {
    // Try primary address first
    esp_err_t ret = bmp280_init(dev, i2c_port, BMP280_I2C_ADDR_PRIMARY);
    if (ret != ESP_OK) {
        // Try secondary address
        ret = bmp280_init(dev, i2c_port, BMP280_I2C_ADDR_SECONDARY);
    }
    return ret;
}

// Deinitialize
esp_err_t bmp280_deinit(bmp280_t *dev) {
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Put device to sleep
    dev->config.mode = BMP280_MODE_SLEEP;
    esp_err_t ret = bmp280_set_config(dev, &dev->config);
    
    dev->initialized = false;
    return ret;
}

// Soft reset
esp_err_t bmp280_soft_reset(bmp280_t *dev) {
    esp_err_t ret = bmp280_write_register(dev, BMP280_REG_SOFTRESET, 0xB6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

// Get configuration
esp_err_t bmp280_get_config(bmp280_t *dev, bmp280_config_t *config) {
    if (!dev->initialized || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(config, &dev->config, sizeof(bmp280_config_t));
    return ESP_OK;
}

// Set configuration
esp_err_t bmp280_set_config(bmp280_t *dev, const bmp280_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set to sleep mode first
    esp_err_t ret = bmp280_write_register(dev, BMP280_REG_CTRL_MEAS, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure filter and standby time
    uint8_t config_reg = (config->standby << 5) | (config->filter << 2);
    ret = bmp280_write_register(dev, BMP280_REG_CONFIG, config_reg);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure oversampling and mode
    uint8_t ctrl_meas = (config->temp_oversampling << 5) | 
                        (config->press_oversampling << 2) | 
                        config->mode;
    ret = bmp280_write_register(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
    if (ret != ESP_OK) {
        return ret;
    }
    
    memcpy(&dev->config, config, sizeof(bmp280_config_t));
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for first measurement
    
    return ESP_OK;
}

// Read raw sensor data
esp_err_t bmp280_read_raw(bmp280_t *dev, int32_t *temp_raw, int32_t *press_raw) {
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[6];
    esp_err_t ret = bmp280_read_register(dev, BMP280_REG_PRESS_DATA, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *press_raw = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
    *temp_raw = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4;
    
    return ESP_OK;
}

// Read compensated values as float
esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature, float *pressure) {
    int32_t temp_raw, press_raw;
    esp_err_t ret = bmp280_read_raw(dev, &temp_raw, &press_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int32_t temp_fixed = bmp280_compensate_temperature(dev, temp_raw);
    uint32_t press_fixed = bmp280_compensate_pressure(dev, press_raw);
    
    *temperature = temp_fixed / 100.0f;
    *pressure = press_fixed / 256.0f;
    
    return ESP_OK;
}

// Read compensated values as fixed point
esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure) {
    int32_t temp_raw, press_raw;
    esp_err_t ret = bmp280_read_raw(dev, &temp_raw, &press_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *temperature = bmp280_compensate_temperature(dev, temp_raw);
    *pressure = bmp280_compensate_pressure(dev, press_raw);
    
    return ESP_OK;
}

// Check if sensor is measuring
bool bmp280_is_measuring(bmp280_t *dev) {
    if (!dev->initialized) {
        return false;
    }
    
    uint8_t status;
    esp_err_t ret = bmp280_read_register(dev, BMP280_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return false;
    }
    
    return (status & 0x08) != 0;
}

// Force measurement (for forced mode)
esp_err_t bmp280_force_measurement(bmp280_t *dev) {
    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (dev->config.mode != BMP280_MODE_FORCED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t ctrl_meas = (dev->config.temp_oversampling << 5) | 
                        (dev->config.press_oversampling << 2) | 
                        BMP280_MODE_FORCED;
    
    return bmp280_write_register(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
}