#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// BMP280 I2C addresses
#define BMP280_I2C_ADDR_PRIMARY   0x77  // SDO connected to VDDIO
#define BMP280_I2C_ADDR_SECONDARY  0x76  // SDO connected to GND

// BMP280 Register addresses
#define BMP280_REG_CHIPID       0xD0
#define BMP280_REG_SOFTRESET    0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_DATA   0xF7
#define BMP280_REG_TEMP_DATA    0xFA
#define BMP280_REG_CALIB00      0x88

// Chip IDs
#define BMP280_CHIP_ID          0x58
#define BME280_CHIP_ID          0x60

// Operating modes
typedef enum {
    BMP280_MODE_SLEEP = 0x00,
    BMP280_MODE_FORCED = 0x01,
    BMP280_MODE_NORMAL = 0x03
} bmp280_mode_t;

// Oversampling settings
typedef enum {
    BMP280_OVERSAMPLE_SKIP = 0x00,
    BMP280_OVERSAMPLE_X1 = 0x01,
    BMP280_OVERSAMPLE_X2 = 0x02,
    BMP280_OVERSAMPLE_X4 = 0x03,
    BMP280_OVERSAMPLE_X8 = 0x04,
    BMP280_OVERSAMPLE_X16 = 0x05
} bmp280_oversampling_t;

// Filter settings
typedef enum {
    BMP280_FILTER_OFF = 0x00,
    BMP280_FILTER_X2 = 0x01,
    BMP280_FILTER_X4 = 0x02,
    BMP280_FILTER_X8 = 0x03,
    BMP280_FILTER_X16 = 0x04
} bmp280_filter_t;

// Standby time (ms)
typedef enum {
    BMP280_STANDBY_0_5 = 0x00,   // 0.5 ms
    BMP280_STANDBY_62_5 = 0x01,  // 62.5 ms
    BMP280_STANDBY_125 = 0x02,   // 125 ms
    BMP280_STANDBY_250 = 0x03,   // 250 ms
    BMP280_STANDBY_500 = 0x04,   // 500 ms
    BMP280_STANDBY_1000 = 0x05,  // 1000 ms
    BMP280_STANDBY_2000 = 0x06,  // 2000 ms
    BMP280_STANDBY_4000 = 0x07   // 4000 ms
} bmp280_standby_t;

// Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

// Configuration structure
typedef struct {
    bmp280_mode_t mode;
    bmp280_oversampling_t temp_oversampling;
    bmp280_oversampling_t press_oversampling;
    bmp280_filter_t filter;
    bmp280_standby_t standby;
} bmp280_config_t;

// Device structure
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    uint8_t chip_id;
    bmp280_config_t config;
    bmp280_calib_data_t calib;
    int32_t t_fine;  // Temperature compensation value
    bool initialized;
} bmp280_t;

// Function prototypes
esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t i2c_port, uint8_t addr);
esp_err_t bmp280_init_default(bmp280_t *dev, i2c_port_t i2c_port);
esp_err_t bmp280_deinit(bmp280_t *dev);
esp_err_t bmp280_soft_reset(bmp280_t *dev);
esp_err_t bmp280_get_config(bmp280_t *dev, bmp280_config_t *config);
esp_err_t bmp280_set_config(bmp280_t *dev, const bmp280_config_t *config);
esp_err_t bmp280_read_raw(bmp280_t *dev, int32_t *temp_raw, int32_t *press_raw);
esp_err_t bmp280_read_float(bmp280_t *dev, float *temperature, float *pressure);
esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure);
bool bmp280_is_measuring(bmp280_t *dev);
esp_err_t bmp280_force_measurement(bmp280_t *dev);

#endif // BMP280_H