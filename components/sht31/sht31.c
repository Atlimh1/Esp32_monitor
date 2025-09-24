#include "sht31.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SHT31";

// CRC-8 calculation for SHT31
static uint8_t sht31_crc8(const uint8_t *data, size_t len) {
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;
    
    for (size_t j = len; j; --j) {
        crc ^= *data++;
        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

// Write command to SHT31
static esp_err_t sht31_write_command(sht31_t *dev, uint16_t cmd) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd >> 8, true);     // MSB
    i2c_master_write_byte(cmd_handle, cmd & 0xFF, true);   // LSB
    i2c_master_stop(cmd_handle);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd_handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

// Read data from SHT31
static esp_err_t sht31_read_data(sht31_t *dev, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd_handle);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd_handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

// Initialize SHT31 device descriptor
esp_err_t sht31_init_desc(sht31_t *dev, i2c_port_t i2c_port, uint8_t addr) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    
    dev->i2c_port = i2c_port;
    dev->i2c_addr = addr;
    dev->repeatability = SHT31_REPEATABILITY_HIGH;
    dev->initialized = false;
    
    return ESP_OK;
}

// Initialize SHT31 device
esp_err_t sht31_init(sht31_t *dev) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = sht31_soft_reset(dev);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Wait for reset
        dev->initialized = true;
    }
    return ret;
}

// Soft reset
esp_err_t sht31_soft_reset(sht31_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = sht31_write_command(dev, SHT31_CMD_SOFT_RESET);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(15)); // Wait for reset to complete
    }
    return ret;
}

// Read temperature and humidity
esp_err_t sht31_read_temperature_humidity(sht31_t *dev, float *temperature, float *humidity) {
    if (!dev || !dev->initialized || !temperature || !humidity) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Select measurement command based on repeatability
    uint16_t cmd;
    int delay_ms;
    
    switch (dev->repeatability) {
        case SHT31_REPEATABILITY_LOW:
            cmd = SHT31_CMD_MEASURE_LOW_REP;
            delay_ms = 5;
            break;
        case SHT31_REPEATABILITY_MEDIUM:
            cmd = SHT31_CMD_MEASURE_MED_REP;
            delay_ms = 7;
            break;
        case SHT31_REPEATABILITY_HIGH:
        default:
            cmd = SHT31_CMD_MEASURE_HIGH_REP;
            delay_ms = 16;
            break;
    }
    
    // Send measurement command
    esp_err_t ret = sht31_write_command(dev, cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command");
        return ret;
    }
    
    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Read 6 bytes: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC
    uint8_t data[6];
    ret = sht31_read_data(dev, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    // Verify temperature CRC
    uint8_t temp_crc = sht31_crc8(data, 2);
    if (temp_crc != data[2]) {
        ESP_LOGE(TAG, "Temperature CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Verify humidity CRC
    uint8_t hum_crc = sht31_crc8(data + 3, 2);
    if (hum_crc != data[5]) {
        ESP_LOGE(TAG, "Humidity CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw values
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];
    
    // Calculate actual values
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = 100.0f * ((float)hum_raw / 65535.0f);
    
    // Clamp humidity to valid range
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;
    
    ESP_LOGD(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", *temperature, *humidity);
    
    return ESP_OK;
}

// Read status register
esp_err_t sht31_read_status(sht31_t *dev, uint16_t *status) {
    if (dev == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = sht31_write_command(dev, SHT31_CMD_READ_STATUS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    uint8_t data[3];
    ret = sht31_read_data(dev, data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Verify CRC
    uint8_t crc = sht31_crc8(data, 2);
    if (crc != data[2]) {
        ESP_LOGE(TAG, "Status CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    *status = (data[0] << 8) | data[1];
    return ESP_OK;
}

// Clear status register
esp_err_t sht31_clear_status(sht31_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sht31_write_command(dev, SHT31_CMD_CLEAR_STATUS);
}

// Enable/disable heater
esp_err_t sht31_heater_enable(sht31_t *dev, bool enable) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t cmd = enable ? SHT31_CMD_HEATER_ENABLE : SHT31_CMD_HEATER_DISABLE;
    return sht31_write_command(dev, cmd);
}

// Deinitialize device
esp_err_t sht31_deinit(sht31_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dev->initialized = false;
    return ESP_OK;
}
